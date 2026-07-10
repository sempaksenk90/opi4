/* usb_transport.c — Combined configfs gadget setup + FunctionFS daemon
 *
 * Single binary that handles the full USB gadget lifecycle:
 *   1. Creates/destroys the gadget via configfs (writing to sysfs files)
 *   2. Mounts functionfs via mount() syscall (no BusyBox dependency)
 *   3. Writes descriptors + strings to ep0
 *   4. Binds UDC once descriptors are ready
 *   5. Runs ep0 event loop and bulk data forwarding
 *
 * Platform: Allwinner A733 / dwc3 USB3 OTG (gadget-only mode)
 * Kernel:   6.6.x
 *
 * Endpoint layout (declaration order determines FFS numbering):
 *   ep0  control (FFS internal)
 *   ep1  BULK OUT  0x01  Host -> Device
 *   ep2  BULK IN   0x81  Device -> Host
 *
 * Transfer model:
 *   Raw bulk forwarding — reads from ep1 (OUT) are written to ep2 (IN).
 *   No command/status protocol is interpreted at this layer.
 */

#include "usb_transport.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <endian.h>
#include <signal.h>
#include <sys/mount.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <pthread.h>
#include <linux/usb/ch9.h>
#include <linux/usb/functionfs.h>

/* -- Constant-expression LE helpers (for static initializers) -- */
#if __BYTE_ORDER == __LITTLE_ENDIAN
#define cpu_to_le16(x) ((uint16_t)(x))
#define cpu_to_le32(x) ((uint32_t)(x))
#else
#define cpu_to_le16(x) ((((x) >> 8) & 0xffu) | (((x) & 0xffu) << 8))
#define cpu_to_le32(x) \
    ((((x) & 0xff000000u) >> 24) | (((x) & 0x00ff0000u) >>  8) | \
     (((x) & 0x0000ff00u) <<  8) | (((x) & 0x000000ffu) << 24))
#endif

/* -- Tunables -------------------------------------------------- */
#define DEV_VID         0x1209
#define DEV_PID         0x0001
#define DEV_BCDUSB      0x0300
#define DEV_BCDEVICE    0x0100
#define CONFIG_MAXPOWER 900

#define STR_MANUFACTURER "RYD76"
#define STR_PRODUCT      "RYD-Prog V1"
#define STR_SERIAL       "RYDP-A73F2B1"
#define STR_CONFIG       "RYD Bulk Config"

#define UDC_NAME        "6a00000.xhci2-controller"

#define GADGET_ROOT     "/sys/kernel/config/usb_gadget/g1"
#define FFS_MOUNT       "/dev/usb-ffs/bulk_ep"
#define FFS_DIR         FFS_MOUNT

/* -- Static const descriptors (flat struct avoids ARM64 padding) -- */

static const struct {
    struct usb_functionfs_descs_head_v2 header;
    uint32_t fs_count;
    uint32_t hs_count;
    uint32_t ss_count;
    uint32_t os_count;

    /* FS: iface + 2 eps */
    struct usb_interface_descriptor fs_iface;
    struct usb_endpoint_descriptor_no_audio fs_out;
    struct usb_endpoint_descriptor_no_audio fs_in;

    /* HS: iface + 2 eps */
    struct usb_interface_descriptor hs_iface;
    struct usb_endpoint_descriptor_no_audio hs_out;
    struct usb_endpoint_descriptor_no_audio hs_in;

    /* SS: iface + 2 eps + 2 comp */
    struct usb_interface_descriptor ss_iface;
    struct usb_endpoint_descriptor_no_audio ss_out;
    struct usb_ss_ep_comp_descriptor     ss_out_comp;
    struct usb_endpoint_descriptor_no_audio ss_in;
    struct usb_ss_ep_comp_descriptor     ss_in_comp;

    /* MS OS 2.0 header + 1 Extended Compat ID (interface 0 → WINUSB) */
    struct usb_os_desc_header   os_hdr;
    struct usb_ext_compat_desc  os_compat;
} __attribute__((packed)) ffs_descriptors = {
    .header = {
        .magic  = cpu_to_le32(FUNCTIONFS_DESCRIPTORS_MAGIC_V2),
        .length = cpu_to_le32(sizeof(ffs_descriptors)),
        .flags  = cpu_to_le32(FUNCTIONFS_HAS_FS_DESC |
                          FUNCTIONFS_HAS_HS_DESC |
                          FUNCTIONFS_HAS_SS_DESC |
                          FUNCTIONFS_HAS_MS_OS_DESC),
    },
    .fs_count = cpu_to_le32(3),
    .hs_count = cpu_to_le32(3),
    .ss_count = cpu_to_le32(5),
    .os_count = cpu_to_le32(1),

    /* -- Full Speed -- */
    .fs_iface = {
        .bLength            = sizeof(struct usb_interface_descriptor),
        .bDescriptorType    = USB_DT_INTERFACE,
        .bNumEndpoints      = 2,
        .bInterfaceClass    = USB_CLASS_VENDOR_SPEC,
        .iInterface         = 1,
    },
    .fs_out = {
        .bLength            = sizeof(struct usb_endpoint_descriptor_no_audio),
        .bDescriptorType    = USB_DT_ENDPOINT,
        .bEndpointAddress   = 0x01,
        .bmAttributes       = USB_ENDPOINT_XFER_BULK,
        .wMaxPacketSize     = cpu_to_le16(64),
    },
    .fs_in = {
        .bLength            = sizeof(struct usb_endpoint_descriptor_no_audio),
        .bDescriptorType    = USB_DT_ENDPOINT,
        .bEndpointAddress   = 0x81,
        .bmAttributes       = USB_ENDPOINT_XFER_BULK,
        .wMaxPacketSize     = cpu_to_le16(64),
    },

    /* -- High Speed -- */
    .hs_iface = {
        .bLength            = sizeof(struct usb_interface_descriptor),
        .bDescriptorType    = USB_DT_INTERFACE,
        .bNumEndpoints      = 2,
        .bInterfaceClass    = USB_CLASS_VENDOR_SPEC,
        .iInterface         = 1,
    },
    .hs_out = {
        .bLength            = sizeof(struct usb_endpoint_descriptor_no_audio),
        .bDescriptorType    = USB_DT_ENDPOINT,
        .bEndpointAddress   = 0x01,
        .bmAttributes       = USB_ENDPOINT_XFER_BULK,
        .wMaxPacketSize     = cpu_to_le16(512),
    },
    .hs_in = {
        .bLength            = sizeof(struct usb_endpoint_descriptor_no_audio),
        .bDescriptorType    = USB_DT_ENDPOINT,
        .bEndpointAddress   = 0x81,
        .bmAttributes       = USB_ENDPOINT_XFER_BULK,
        .wMaxPacketSize     = cpu_to_le16(512),
    },

    /* -- Super Speed (2 endpoints, matching FS/HS) -- */
    .ss_iface = {
        .bLength            = sizeof(struct usb_interface_descriptor),
        .bDescriptorType    = USB_DT_INTERFACE,
        .bNumEndpoints      = 2,
        .bInterfaceClass    = USB_CLASS_VENDOR_SPEC,
        .iInterface         = 1,
    },
    .ss_out = {
        .bLength            = sizeof(struct usb_endpoint_descriptor_no_audio),
        .bDescriptorType    = USB_DT_ENDPOINT,
        .bEndpointAddress   = 0x01,
        .bmAttributes       = USB_ENDPOINT_XFER_BULK,
        .wMaxPacketSize     = cpu_to_le16(1024),
    },
    .ss_out_comp = {
        .bLength            = USB_DT_SS_EP_COMP_SIZE,
        .bDescriptorType    = USB_DT_SS_ENDPOINT_COMP,
        .bMaxBurst          = 15,
    },
    .ss_in = {
        .bLength            = sizeof(struct usb_endpoint_descriptor_no_audio),
        .bDescriptorType    = USB_DT_ENDPOINT,
        .bEndpointAddress   = 0x81,
        .bmAttributes       = USB_ENDPOINT_XFER_BULK,
        .wMaxPacketSize     = cpu_to_le16(1024),
    },
    .ss_in_comp = {
        .bLength            = USB_DT_SS_EP_COMP_SIZE,
        .bDescriptorType    = USB_DT_SS_ENDPOINT_COMP,
        .bMaxBurst          = 15,
    },

    /* -- MS OS 2.0 Extended Compat ID (interface 0 → WINUSB) -- */
    .os_hdr = {
        .interface          = 0,
        .dwLength           = cpu_to_le32(sizeof(struct usb_os_desc_header) +
                                          sizeof(struct usb_ext_compat_desc)),
        .bcdVersion         = cpu_to_le16(0x0100),
        .wIndex             = cpu_to_le16(0x0004), /* Extended Compat ID */
        .wCount             = cpu_to_le16(1),
    },
    .os_compat = {
        .bFirstInterfaceNumber = 0,
        .Reserved1             = 1,
        .CompatibleID          = "WINUSB",
        .SubCompatibleID       = {0},
        .Reserved2             = {0},
    },
};

/* -- Static const strings -------------------------------------- */
#define STR_COUNT 1u
#define LANG_COUNT 1u

static const struct {
    struct usb_functionfs_strings_head header;
    struct {
        uint16_t code;
        char str[sizeof(STR_CONFIG)];
    } __attribute__((packed)) lang;
} __attribute__((packed)) ffs_strings = {
    .header = {
        .magic      = cpu_to_le32(FUNCTIONFS_STRINGS_MAGIC),
        .length     = cpu_to_le32(sizeof(ffs_strings)),
        .str_count  = cpu_to_le32(STR_COUNT),
        .lang_count = cpu_to_le32(LANG_COUNT),
    },
    .lang = {
        .code = cpu_to_le16(0x0409),
        .str  = STR_CONFIG,
    },
};

/* -- Graceful shutdown ---------------------------------------- */
static volatile sig_atomic_t g_stop = 0;

static void on_stop_signal(int signo)
{
    (void)signo;
    g_stop = 1;
}

/* ==============================================================
 * ConfigFS helpers
 * ============================================================== */

static int write_sysfs(const char *path, const char *value)
{
    int fd = open(path, O_WRONLY);
    if (fd < 0) return -1;
    size_t len = strlen(value);
    ssize_t w = write(fd, value, len);
    int saved = errno;
    close(fd);
    errno = saved;
    return (w == (ssize_t)len) ? 0 : -1;
}

static int mkdir_parent(const char *path)
{
    /* Create parent directory of a path. Path must be writable. */
    char buf[256];
    size_t len = strlen(path);
    if (len >= sizeof(buf)) { errno = ENAMETOOLONG; return -1; }
    memcpy(buf, path, len + 1);
    char *slash = strrchr(buf, '/');
    if (!slash || slash == buf) return 0;  /* no parent or root */
    *slash = '\0';
    if (mkdir(buf, 0755) < 0 && errno != EEXIST) return -1;
    return 0;
}

/* ==============================================================
 * Gadget lifecycle — teardown (must follow strict ConfigFS order)
 * ============================================================== */

static void teardown_gadget(void)
{
    /* Unbind UDC first */
    write_sysfs(GADGET_ROOT "/UDC", "");
    usleep(300000);

    /* Remove os_desc symlink + reset */
    char buf[256];
    snprintf(buf, sizeof(buf), GADGET_ROOT "/os_desc/c.1");
    unlink(buf);
    write_sysfs(GADGET_ROOT "/os_desc/use", "0");

    /* Remove symlink from config */
    snprintf(buf, sizeof(buf), GADGET_ROOT "/configs/c.1/ffs.usb0");
    unlink(buf);

    /* Remove config strings + config */
    snprintf(buf, sizeof(buf), GADGET_ROOT "/configs/c.1/strings/0x409");
    rmdir(buf);
    snprintf(buf, sizeof(buf), GADGET_ROOT "/configs/c.1");
    rmdir(buf);

    /* Unmount functionfs (ignore errors — may not be mounted) */
    umount(FFS_MOUNT);
    rmdir(FFS_DIR);

    /* Remove function + gadget */
    snprintf(buf, sizeof(buf), GADGET_ROOT "/functions/ffs.usb0");
    rmdir(buf);
    snprintf(buf, sizeof(buf), GADGET_ROOT "/strings/0x409");
    rmdir(buf);
    rmdir(GADGET_ROOT);
}

/* ==============================================================
 * Gadget setup (creates configfs entries)
 * ============================================================== */

static int setup_gadget(void)
{
    /* Mount configfs if not already */
    struct stat st;
    if (stat("/sys/kernel/config/usb_gadget", &st) < 0) {
        if (mount("none", "/sys/kernel/config", "configfs", 0, NULL) < 0) {
            perror("mount configfs");
            return -1;
        }
    }

    /* Create gadget directory */
    if (mkdir(GADGET_ROOT, 0755) < 0 && errno != EEXIST) {
        perror("mkdir " GADGET_ROOT);
        return -1;
    }

    /* Device descriptors */
    char vid_str[16], pid_str[16];
    snprintf(vid_str, sizeof(vid_str), "0x%04x", DEV_VID);
    snprintf(pid_str, sizeof(pid_str), "0x%04x", DEV_PID);
    if (write_sysfs(GADGET_ROOT "/idVendor", vid_str) < 0 ||
        write_sysfs(GADGET_ROOT "/idProduct", pid_str) < 0 ||
        write_sysfs(GADGET_ROOT "/bcdUSB", "0x0300") < 0 ||
        write_sysfs(GADGET_ROOT "/bcdDevice", "0x0100") < 0) {
        perror("write device descriptor");
        return -1;
    }

    /* Strings */
    char path[256];
    snprintf(path, sizeof(path), GADGET_ROOT "/strings/0x409");
    if (mkdir(path, 0755) < 0 && errno != EEXIST) {
        perror("mkdir strings");
        return -1;
    }
    if (write_sysfs(path, "") < 0) { /* ensure dir exists */ }
    snprintf(path, sizeof(path), GADGET_ROOT "/strings/0x409/manufacturer");
    write_sysfs(path, STR_MANUFACTURER);
    snprintf(path, sizeof(path), GADGET_ROOT "/strings/0x409/product");
    write_sysfs(path, STR_PRODUCT);
    snprintf(path, sizeof(path), GADGET_ROOT "/strings/0x409/serialnumber");
    write_sysfs(path, STR_SERIAL);

    /* Function */
    snprintf(path, sizeof(path), GADGET_ROOT "/functions/ffs.usb0");
    if (mkdir(path, 0755) < 0 && errno != EEXIST) {
        perror("mkdir function");
        return -1;
    }

    /* Configuration */
    snprintf(path, sizeof(path), GADGET_ROOT "/configs/c.1");
    if (mkdir(path, 0755) < 0 && errno != EEXIST) {
        perror("mkdir config");
        return -1;
    }
    snprintf(path, sizeof(path), GADGET_ROOT "/configs/c.1/strings/0x409");
    if (mkdir(path, 0755) < 0 && errno != EEXIST) {
        perror("mkdir config strings");
        return -1;
    }
    snprintf(path, sizeof(path), GADGET_ROOT "/configs/c.1/strings/0x409/configuration");
    write_sysfs(path, STR_CONFIG);
    snprintf(path, sizeof(path), GADGET_ROOT "/configs/c.1/MaxPower");
    write_sysfs(path, "900");

    /* Symlink function into config */
    snprintf(path, sizeof(path), GADGET_ROOT "/configs/c.1/ffs.usb0");
    if (access(path, F_OK) != 0) {
        char target[256];
        snprintf(target, sizeof(target), GADGET_ROOT "/functions/ffs.usb0");
        if (symlink(target, path) < 0) {
            perror("symlink ffs.usb0");
            return -1;
        }
    }

    /* Microsoft OS 2.0 descriptors — Windows WinUSB auto-install */
    snprintf(path, sizeof(path), GADGET_ROOT "/os_desc/use");
    write_sysfs(path, "1");
    snprintf(path, sizeof(path), GADGET_ROOT "/os_desc/b_vendor_code");
    write_sysfs(path, "0xcd");
    snprintf(path, sizeof(path), GADGET_ROOT "/os_desc/qw_sign");
    write_sysfs(path, "MSFT100");
    snprintf(path, sizeof(path), GADGET_ROOT "/os_desc/c.1");
    if (access(path, F_OK) != 0) {
        char target[256];
        snprintf(target, sizeof(target), GADGET_ROOT "/configs/c.1");
        if (symlink(target, path) < 0)
            fprintf(stderr, "[USB] WARN: os_desc symlink: %s\n", strerror(errno));
    }

    return 0;
}

/* ==============================================================
 * FunctionFS helpers — descriptors + strings
 * ============================================================== */

static int write_all(int fd, const void *buf, size_t len)
{
    const uint8_t *p = (const uint8_t *)buf;
    while (len > 0) {
        ssize_t w = write(fd, p, len);
        if (w < 0) {
            if (errno == EINTR) continue;
            return -1;
        }
        p   += (size_t)w;
        len -= (size_t)w;
    }
    return 0;
}

static int ffs_write_descriptors(int ep0)
{
    if (write_all(ep0, &ffs_descriptors, sizeof(ffs_descriptors)) < 0) {
        fprintf(stderr, "[USB] descriptor write: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}

static int ffs_write_strings(int ep0)
{
    if (write_all(ep0, &ffs_strings, sizeof(ffs_strings)) < 0) {
        fprintf(stderr, "[USB] string write: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}

/* ==============================================================
 * ep0 event loop
 * ============================================================== */

static int wait_for_enable(int ep0)
{
    struct pollfd pfd = { .fd = ep0, .events = POLLIN };
    for (;;) {
        if (g_stop) return -1;
        int rc = poll(&pfd, 1, -1);
        if (rc < 0) {
            if (errno == EINTR) continue;
            perror("[USB] poll ep0");
            return -1;
        }

        struct usb_functionfs_event ev;
        ssize_t n = read(ep0, &ev, sizeof(ev));
        if (n < (ssize_t)sizeof(ev)) continue;

        switch (ev.type) {
        case FUNCTIONFS_ENABLE:
            printf("[USB] Interface enabled\n");
            return 0;
        case FUNCTIONFS_DISABLE:
            printf("[USB] Host disconnected, waiting for reconnect...\n");
            return 1;  /* signal reconnect */
        case FUNCTIONFS_BIND:
            printf("[USB] Gadget bound to UDC\n");
            break;
        case FUNCTIONFS_UNBIND:
            printf("[USB] Gadget unbound\n");
            return -1;
        case FUNCTIONFS_SETUP:
            if (!(ev.u.setup.bRequestType & USB_DIR_IN)) {
                ssize_t _ign = write(ep0, "", 0);
                (void)_ign;
            }
            break;
        default:
            break;
        }
    }
}

/* ==============================================================
 * Bulk I/O loop — raw data forwarding
 *
 * Per Documentation/driver-api/usb/dwc3.rst "Known Limitations ->
 * OUT Transfer Size Requirements": every OUT TRB's *size* field
 * must be an integer multiple of the endpoint's wMaxPacketSize
 * (1024 SuperSpeed / 512 HighSpeed / 64 FullSpeed), or the OUT
 * transfer will not start. dwc3 will auto-chain a throwaway TRB
 * to cope with a non-multiple length, but we don't want to rely
 * on that — we query the *actual negotiated* wMaxPacketSize via
 * FUNCTIONFS_ENDPOINT_DESC and only ever post reads that are an
 * exact multiple of it.
 *
 * Just as important: the original implementation blocked on
 * write(ep_in) before re-posting read(ep_out). That means the OUT
 * endpoint has no TRB queued while we're blocked writing IN, so
 * the host stalls/NAKs on every buffer boundary — this is the
 * usual real-world symptom people report as "OUT transfers don't
 * start/resume" even when the length itself is a valid multiple.
 * We fix that by decoupling OUT reads from IN writes with a small
 * ring buffer serviced by two threads, so ep_out always has a
 * pending request.
 * ============================================================== */

#define USB_MAX_PACKET   16384u   /* multiple of 64 / 512 / 1024 */
#define RING_SLOTS       4        /* outstanding buffers in flight;
                                    * well under the 256-TRB/EP ring
                                    * limit (1 TRB per slot here) */

typedef struct {
    uint8_t buf[USB_MAX_PACKET];
    size_t  len;
} ring_slot_t;

typedef struct {
    ring_slot_t     slot[RING_SLOTS];
    size_t          head;   /* next slot to fill (OUT reader) */
    size_t          tail;   /* next slot to drain (IN writer) */
    size_t          count;
    pthread_mutex_t lock;
    pthread_cond_t  not_empty;
    pthread_cond_t  not_full;
    volatile int    error;  /* set by either thread on fatal error */
    volatile int    eof;    /* set by reader on ESHUTDOWN/ECONNRESET */
} ring_t;

static void ring_init(ring_t *r)
{
    memset(r, 0, sizeof(*r));
    pthread_mutex_init(&r->lock, NULL);
    pthread_cond_init(&r->not_empty, NULL);
    pthread_cond_init(&r->not_full, NULL);
}

static void ring_destroy(ring_t *r)
{
    pthread_mutex_destroy(&r->lock);
    pthread_cond_destroy(&r->not_empty);
    pthread_cond_destroy(&r->not_full);
}

/* Query the endpoint's negotiated wMaxPacketSize (speed-dependent).
 * Falls back to 512 (HighSpeed) if the ioctl isn't supported, which
 * is still a safe divisor of our fixed 16384-byte buffer. */
static unsigned get_out_max_packet_size(int ep_out)
{
    struct usb_endpoint_descriptor desc;
    memset(&desc, 0, sizeof(desc));
    if (ioctl(ep_out, FUNCTIONFS_ENDPOINT_DESC, &desc) < 0) {
        fprintf(stderr, "[USB] WARN: FUNCTIONFS_ENDPOINT_DESC ep_out: %s "
                "(assuming 512)\n", strerror(errno));
        return 512;
    }
    unsigned mps = le16toh(desc.wMaxPacketSize) & 0x07ff;
    if (mps == 0) mps = 512;
    return mps;
}

typedef struct {
    int          ep_out;
    unsigned     mps;
    ring_t      *ring;
} reader_args_t;

/* OUT reader thread: keeps a request continuously posted on ep_out.
 * Each read() length is USB_MAX_PACKET, which is a guaranteed
 * multiple of mps (64/512/1024 all divide 16384) — satisfying the
 * dwc3 OUT-transfer-size requirement explicitly rather than by
 * accident. A short/zero actual completion is normal bulk OUT
 * behavior (end of host write), not an error. */
static void *out_reader_thread(void *arg)
{
    reader_args_t *a = (reader_args_t *)arg;
    ring_t *r = a->ring;

    /* Sanity: our fixed buffer must be a multiple of mps. If a
     * future mps value doesn't divide it (shouldn't happen for
     * FS/HS/SS bulk), fail loudly instead of silently violating
     * the OUT transfer size rule. */
    if (USB_MAX_PACKET % a->mps != 0) {
        fprintf(stderr, "[USB] FATAL: USB_MAX_PACKET (%u) not a "
                "multiple of wMaxPacketSize (%u)\n", USB_MAX_PACKET, a->mps);
        pthread_mutex_lock(&r->lock);
        r->error = 1;
        pthread_cond_broadcast(&r->not_empty);
        pthread_mutex_unlock(&r->lock);
        return NULL;
    }

    for (;;) {
        if (g_stop) break;

        pthread_mutex_lock(&r->lock);
        while (r->count == RING_SLOTS && !g_stop && !r->error)
            pthread_cond_wait(&r->not_full, &r->lock);
        if (g_stop || r->error) { pthread_mutex_unlock(&r->lock); break; }
        pthread_mutex_unlock(&r->lock);

        ring_slot_t *s = &r->slot[r->head];

        /* CRITICAL: never post a read for more than the sender can
         * be relied on to terminate. A bulk OUT transfer completes
         * only when (a) the requested length is fully received, or
         * (b) a short/zero-length packet arrives. If the host issues
         * a WriteFile() whose length is an *exact* multiple of
         * wMaxPacketSize with no trailing ZLP (very common — e.g.
         * WinUSB does not auto-append one), and we'd posted a read
         * for more than that, dwc3 has no way to know the logical
         * transfer ended: it just keeps the TRB armed waiting for
         * more data that never comes, and the host's WriteFile hangs
         * until its USB timeout fires (WinError 121), often leaving
         * the endpoint wedged for the next transfer (WinError 22).
         *
         * Fix: post exactly one wMaxPacketSize chunk per read(). A
         * request whose length equals what's actually sent always
         * satisfies completion condition (a) on its own — we never
         * depend on the host sending a short packet or ZLP. We loop,
         * accumulating consecutive full packets into one forwarded
         * slot, and only stop accumulating on a short/zero packet
         * (natural end of a variable-length message) or a full slot. */
        size_t got = 0;
        for (;;) {
            size_t chunk = USB_MAX_PACKET - got;
            if (chunk > a->mps) chunk = a->mps;
            if (chunk == 0) break;  /* slot full */

            ssize_t n = read(a->ep_out, s->buf + got, chunk);
            if (n < 0) {
                if (errno == EINTR) continue;
                if (errno == ESHUTDOWN || errno == ECONNRESET) {
                    pthread_mutex_lock(&r->lock);
                    r->eof = 1;
                    pthread_cond_broadcast(&r->not_empty);
                    pthread_mutex_unlock(&r->lock);
                    goto reader_done;
                }
                perror("[USB] read ep_out");
                pthread_mutex_lock(&r->lock);
                r->error = 1;
                pthread_cond_broadcast(&r->not_empty);
                pthread_mutex_unlock(&r->lock);
                goto reader_done;
            }
            if (n == 0) break;         /* ZLP: end of message */
            got += (size_t)n;
            if ((size_t)n < chunk) break; /* short packet: end of message */
        }
        if (got == 0) continue;        /* nothing to forward, re-post */

        pthread_mutex_lock(&r->lock);
        s->len = got;
        r->head = (r->head + 1) % RING_SLOTS;
        r->count++;
        pthread_cond_broadcast(&r->not_empty);
        pthread_mutex_unlock(&r->lock);
    }
reader_done:
    return NULL;
}

/* IN writer: drains the ring and writes to ep_in. Runs independently
 * of the OUT reader, so a slow/blocked host-side IN read never
 * starves ep_out of a posted TRB. */
static int bulk_loop(int ep_in, int ep_out)
{
    ring_t ring;
    ring_init(&ring);

    reader_args_t rargs = {
        .ep_out = ep_out,
        .mps    = get_out_max_packet_size(ep_out),
        .ring   = &ring,
    };
    printf("[USB] Ready — forwarding bulk data (OUT mps=%u, buf=%u, "
           "ring=%d)\n", rargs.mps, USB_MAX_PACKET, RING_SLOTS);

    pthread_t reader;
    if (pthread_create(&reader, NULL, out_reader_thread, &rargs) != 0) {
        perror("[USB] pthread_create out_reader");
        ring_destroy(&ring);
        return -1;
    }

    int ret = 0;
    for (;;) {
        if (g_stop) { ret = -1; break; }

        pthread_mutex_lock(&ring.lock);
        while (ring.count == 0 && !ring.eof && !ring.error && !g_stop)
            pthread_cond_wait(&ring.not_empty, &ring.lock);

        if (ring.count == 0) {
            int eof = ring.eof, err = ring.error;
            pthread_mutex_unlock(&ring.lock);
            if (g_stop) { ret = -1; break; }
            if (eof)   { ret = 0;  break; }   /* clean disconnect */
            if (err)   { ret = -1; break; }
            continue;
        }

        ring_slot_t local = ring.slot[ring.tail];
        ring.tail = (ring.tail + 1) % RING_SLOTS;
        ring.count--;
        pthread_cond_broadcast(&ring.not_full);
        pthread_mutex_unlock(&ring.lock);

        size_t off = 0;
        while (off < local.len) {
            ssize_t w = write(ep_in, local.buf + off, local.len - off);
            if (w < 0) {
                if (errno == EINTR) continue;
                perror("[USB] write ep_in");
                ret = -1;
                goto done;
            }
            off += (size_t)w;
        }
    }

done:
    g_stop = g_stop ? g_stop : 0; /* no-op, keeps intent explicit */
    pthread_mutex_lock(&ring.lock);
    ring.error = 1; /* wake reader if it's waiting on not_full */
    pthread_cond_broadcast(&ring.not_full);
    pthread_mutex_unlock(&ring.lock);
    pthread_join(reader, NULL);
    ring_destroy(&ring);
    return ret;
}

/* -- Power/control paths for preventing DWC3 runtime PM suspend -- */
static void prevent_runtime_pm(void)
{
    /* The UDC child device (xhci2-controller under DWC3) */
    char pw_path[256];
    snprintf(pw_path, sizeof(pw_path),
             "/sys/class/udc/%s/device/power/control", UDC_NAME);
    if (write_sysfs(pw_path, "on") < 0) {
        fprintf(stderr, "[USB] WARN: power/control on %s: %s\n", pw_path, strerror(errno));
    }

    /* The DWC3 parent device — the genpd (pd_usb2_test) is attached here.
     * Writing "on" to only the child (xhci2-controller) does NOT prevent
     * the parent from runtime-suspending and taking down the whole domain. */
    static const char *extra_paths[] = {
        "/sys/devices/platform/soc/6a00000.usb/power/control",
        "/sys/devices/platform/6a00000.usb/power/control",
        "/sys/devices/platform/soc/soc:pd_usb2_test/power/control",
        "/sys/devices/platform/soc:pd_usb2_test/power/control",
    };
    for (size_t i = 0; i < sizeof(extra_paths) / sizeof(extra_paths[0]); i++) {
        if (write_sysfs(extra_paths[i], "on") < 0) continue;
        printf("[USB] power/control on %s\n", extra_paths[i]);
    }
}

/* ==============================================================
 * Entry point
 * ============================================================== */

int main(void)
{
    int ep0    = -1;
    int ep_out = -1;
    int ep_in  = -1;
    int ret    = 1;

    /* -- Signal handlers --------------------------------------- */
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = on_stop_signal;
    sigaction(SIGINT,  &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    printf("[USB] RYD-Prog V1 (VID 0x1209 PID 0x0001)\n");

    /* -- Phase 0: Prevent runtime PM suspend for DWC3 genpd ---- */
    prevent_runtime_pm();

    /* -- Phase 1: Cleanup any previous instance ---------------- */
    teardown_gadget();

    /* -- Phase 2: Full setup + bind — retry forever if genpd
     *    hasn't resumed yet ------------------------------------ */
    for (int attempt = 1; ; attempt++) {
        if (g_stop) { ret = 1; goto out; }

        /* Configfs gadget setup */
                if (setup_gadget() < 0) {
            fprintf(stderr, "[USB] gadget setup failed (attempt %d)\n", attempt);
            sleep(3);
            continue;
        }

        /* Mount functionfs (source must match the ffs. instance name) */
        mkdir_parent(FFS_DIR);
        mkdir(FFS_DIR, 0755);
        if (mount("usb0", FFS_DIR, "functionfs", 0, NULL) < 0) {
            perror("[USB] mount functionfs");
            teardown_gadget();
            sleep(3);
            continue;
        }

        /* Open ep0 */
        ep0 = open(FFS_DIR "/ep0", O_RDWR);
        if (ep0 < 0) {
            perror("[USB] open ep0");
            teardown_gadget();
            sleep(3);
            continue;
        }

        /* Write descriptors and strings */
        if (ffs_write_descriptors(ep0) < 0 ||
            ffs_write_strings(ep0)     < 0) {
            close(ep0); ep0 = -1;
            teardown_gadget();
            sleep(3);
            continue;
        }

        /* Keep UDC power domain alive */
        prevent_runtime_pm();

        /* Bind UDC immediately — this forces the kernel to create ep1/ep2 */
        printf("[USB] Binding UDC (attempt %d)...\n", attempt);
        int bound = 0;
        for (int r = 0; r < 10; r++) {
            if (write_sysfs(GADGET_ROOT "/UDC", UDC_NAME) == 0) {
                bound = 1;
                break;
            }
            fprintf(stderr, "[USB] bind UDC retry %d/10\n", r + 1);
            sleep(1);
        }
        if (!bound) {
            perror("[USB] bind UDC");
            close(ep0); ep0 = -1;
            teardown_gadget();
            sleep(3);
            continue;
        }

        printf("[USB] UDC bound to %s\n", UDC_NAME);
        break;  /* initialization success, moving to event loop */
    }

    /* -- Event loop — handle connect/disconnect/reconnect ------ */
    for (;;) {
        if (g_stop) {
            printf("[USB] Stop requested\n");
            break;
        }

        int wfe = wait_for_enable(ep0);
        if (wfe < 0) {
            /* FUNCTIONFS_UNBIND — domain likely suspended.
             * Teardown, keep trying to re-establish forever. */
            printf("[USB] Unbind/lost UDC, reconnecting forever...\n");
            close(ep_out); ep_out = -1;
            close(ep_in);  ep_in  = -1;
            close(ep0);    ep0    = -1;
            teardown_gadget();

            /* Reconnect: retry forever until success or SIGTERM */
            for (;;) {
                if (g_stop) { ret = 1; goto out; }

                sleep(3);
                prevent_runtime_pm();

        if (setup_gadget() < 0) {
                    fprintf(stderr, "[USB] reconnect: gadget setup failed\n");
                    continue;
                }
                mkdir_parent(FFS_DIR);
                mkdir(FFS_DIR, 0755);
                if (mount("usb0", FFS_DIR, "functionfs", 0, NULL) < 0) {
                    perror("[USB] reconnect: mount functionfs");
                    teardown_gadget();
                    continue;
                }
                ep0 = open(FFS_DIR "/ep0", O_RDWR);
                if (ep0 < 0) {
                    perror("[USB] reconnect: open ep0");
                    teardown_gadget();
                    continue;
                }
                if (ffs_write_descriptors(ep0) < 0 ||
                    ffs_write_strings(ep0)     < 0) {
                    close(ep0); ep0 = -1;
                    teardown_gadget();
                    continue;
                }

                prevent_runtime_pm();

                int bound = 0;
                for (int r = 0; r < 10; r++) {
                    if (write_sysfs(GADGET_ROOT "/UDC", UDC_NAME) == 0) {
                        bound = 1;
                        break;
                    }
                    sleep(1);
                }
                if (!bound) {
                    fprintf(stderr, "[USB] reconnect: bind UDC failed, retrying...\n");
                    close(ep0); ep0 = -1;
                    teardown_gadget();
                    continue;
                }
                printf("[USB] Reconnect successful — UDC bound\n");
                break;  /* back to event loop */
            }
            continue;
        }
        if (wfe == 1) continue;     /* DISABLE -> wait for ENABLE */

        /* ENABLE received — open data endpoints */
        if (ep_out >= 0) { close(ep_out); ep_out = -1; }
        if (ep_in  >= 0) { close(ep_in);  ep_in  = -1; }

        ep_out = open(FFS_DIR "/ep1", O_RDWR);
        ep_in  = open(FFS_DIR "/ep2", O_RDWR);

        if (ep_out < 0 || ep_in < 0) {
            fprintf(stderr, "[USB] endpoint open: "
                    "ep1=%d ep2=%d — %s\n",
                    ep_out, ep_in, strerror(errno));
            break;
        }
        printf("[USB] Endpoints ready\n");

        if (bulk_loop(ep_in, ep_out) < 0) break;
    }

    ret = 0;
out:
    if (ep_out >= 0) close(ep_out);
    if (ep_in  >= 0) close(ep_in);
    if (ep0    >= 0) close(ep0);

    teardown_gadget();
    printf("[USB] Shutdown complete\n");
    return ret;
}
