/* usb_transport.c — FunctionFS USB transport layer
 *
 * Handles:
 *   - FunctionFS descriptor/string write (WinUSB MS OS 1.0)
 *   - ep0 event loop (ENABLE/DISABLE/SETUP/BIND/UNBIND)
 *   - CBW receive → dispatch → data phase → CSW send
 *   - Linux AIO for bulk endpoint I/O (overlapped USB+storage)
 *
 * Kernel: 6.6.x  SoC: Allwinner A733 (dwc3 USB3 OTG)
 */

#include "usb_transport.h"
#include "protocol.h"
#include "emmc_ops.h"
#include "ufs_ops.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <endian.h>
#include <libaio.h>
#include <linux/usb/functionfs.h>

/* ── tunables ────────────────────────────────────────────────── */
#define FFS_MOUNT       "/dev/usb-ffs/bulk_ep"
#define BUF_SIZE        (1024 * 1024)   /* 1 MB per USB transfer    */
#define AIO_DEPTH       8               /* in-flight AIO requests   */
#define FW_MAJOR        1
#define FW_MINOR        0
#define FW_PATCH        0
/* ────────────────────────────────────────────────────────────── */

/* ── Descriptor byte helpers ────────────────────────────────── */
#define U16LE(x)  ((x) & 0xFF), (((x) >> 8) & 0xFF)
#define U32LE(x)  ((x)&0xFF),(((x)>>8)&0xFF),(((x)>>16)&0xFF),(((x)>>24)&0xFF)

#define IFACE_DESC(nep) \
    0x09, USB_DT_INTERFACE, 0x00, 0x00, (nep), \
    USB_CLASS_VENDOR_SPEC, 0x00, 0x00, 0x00

#define BULK_EP_DESC(addr, maxlo, maxhi) \
    0x07, USB_DT_ENDPOINT, (addr), USB_ENDPOINT_XFER_BULK, \
    (maxlo), (maxhi), 0x00

#define INT_EP_DESC(addr, maxlo, maxhi, interval) \
    0x07, USB_DT_ENDPOINT, (addr), USB_ENDPOINT_XFER_INT, \
    (maxlo), (maxhi), (interval)

#define SS_COMP(burst) \
    0x06, USB_DT_SS_ENDPOINT_COMP, (burst), 0x00, 0x00, 0x00

/* Endpoint addresses:
 *   0x81 = IN1  (bulk, data from device to host)
 *   0x01 = OUT1 (bulk, data from host to device)
 *   0x82 = IN2  (interrupt, async status)
 */
#define EP_BULK_IN   0x81
#define EP_BULK_OUT  0x01
#define EP_INT_IN    0x82

/* ── OS descriptor sizes ─────────────────────────────────────── */
#define OS_HDR_SZ    (1+4+2+2+1+7)   /* usb_os_desc_header   = 17 */
#define OS_COMPAT_SZ (1+1+8+8+6)     /* usb_ext_compat_desc  = 24 */
#define OS_TOTAL_SZ  (OS_HDR_SZ + OS_COMPAT_SZ)  /* 41 */

/* ── Full raw descriptor blob ────────────────────────────────── */
/* FunctionFS v2 header + FS/HS/SS + MS OS 1.0 ext compat ID    */
/*
 * Speeds:
 *   FS  3 descriptors: iface + bulk_in + bulk_out
 *   HS  3 descriptors: iface + bulk_in + bulk_out
 *   SS  7 descriptors: iface + bulk_in + comp + bulk_out + comp
 *                             + int_in  + comp
 *   OS  1 record
 *
 * FS  = 9+7+7         = 23 bytes
 * HS  = 9+7+7         = 23 bytes
 * SS  = 9+7+6+7+6+7+6 = 48 bytes
 * FFS header          = 12 bytes (magic+length+flags)
 * count fields        = 16 bytes (4×u32)
 * ─────────────────────────────
 * OS section offset   = 12+16+23+23+48 = 122
 * dwLength offset     = 122 + 1        = 123
 */
#define FS_CNT  3
#define HS_CNT  3
#define SS_CNT  7
#define OS_CNT  1

static uint8_t ffs_desc_blob[] = {
    /* FunctionFS v2 header — magic/length/flags filled at runtime */
    U32LE(0), U32LE(0), U32LE(0),
    /* counts */
    U32LE(FS_CNT), U32LE(HS_CNT), U32LE(SS_CNT), U32LE(OS_CNT),
    /* ── FS ── */
    IFACE_DESC(2),
    BULK_EP_DESC(EP_BULK_IN,  0x40, 0x00),  /* 64 B  */
    BULK_EP_DESC(EP_BULK_OUT, 0x40, 0x00),
    /* ── HS ── */
    IFACE_DESC(2),
    BULK_EP_DESC(EP_BULK_IN,  0x00, 0x02),  /* 512 B */
    BULK_EP_DESC(EP_BULK_OUT, 0x00, 0x02),
    /* ── SS ── */
    IFACE_DESC(3),
    BULK_EP_DESC(EP_BULK_IN,  0x00, 0x04),  /* 1024 B */
    SS_COMP(15),                             /* MaxBurst=15 */
    BULK_EP_DESC(EP_BULK_OUT, 0x00, 0x04),
    SS_COMP(15),
    INT_EP_DESC(EP_INT_IN, 0x40, 0x00, 4),  /* 64 B, 4ms poll */
    SS_COMP(0),
    /* ── MS OS 1.0 Extended Compat ID ── */
    /* usb_os_desc_header */
    0x00,                        /* interface          */
    U32LE(0),                    /* dwLength — runtime */
    U16LE(0x0100),               /* bcdVersion         */
    U16LE(0x0004),               /* wIndex             */
    0x01,                        /* bCount             */
    0x00,0x00,0x00,0x00,0x00,0x00,0x00, /* Reserved[7] */
    /* usb_ext_compat_desc */
    0x00,                        /* bFirstInterfaceNumber */
    0x01,                        /* Reserved1 = 1         */
    'W','I','N','U','S','B',0x00,0x00, /* CompatibleID[8] */
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, /* SubCompatID */
    0x00,0x00,0x00,0x00,0x00,0x00,  /* Reserved2[6]        */
};

static const uint8_t ffs_str_blob[] = {
    U32LE(FUNCTIONFS_STRINGS_MAGIC),
    U32LE(16),   /* length  */
    U32LE(0),    /* str_count  */
    U32LE(0),    /* lang_count */
};

/* ── Static I/O buffer (1 MB, AIO aligned) ──────────────────── */
static uint8_t __attribute__((aligned(4096))) io_buf[BUF_SIZE];

/* ── Session state ───────────────────────────────────────────── */
static uint8_t  active_chip  = CHIP_NONE;
static int      emmc_fd      = -1;
static int      ufs_bsg_fd   = -1;

/* ── Helpers ─────────────────────────────────────────────────── */
static int write_all(int fd, const void *buf, size_t len)
{
    const uint8_t *p = (const uint8_t *)buf;
    while (len) {
        ssize_t w = write(fd, p, len);
        if (w < 0) { if (errno == EINTR) continue; return -1; }
        p += w; len -= (size_t)w;
    }
    return 0;
}

static int read_all(int fd, void *buf, size_t len)
{
    uint8_t *p = (uint8_t *)buf;
    while (len) {
        ssize_t r = read(fd, p, len);
        if (r < 0) { if (errno == EINTR) continue; return -1; }
        if (r == 0) return -1;
        p += r; len -= (size_t)r;
    }
    return 0;
}

/* ── Descriptor write ────────────────────────────────────────── */
static int write_descriptors(int ep0)
{
    /* patch magic/length/flags */
    uint32_t magic  = htole32(FUNCTIONFS_DESCRIPTORS_MAGIC_V2);
    uint32_t length = htole32((uint32_t)sizeof(ffs_desc_blob));
    uint32_t flags  = htole32(FUNCTIONFS_HAS_FS_DESC |
                               FUNCTIONFS_HAS_HS_DESC |
                               FUNCTIONFS_HAS_SS_DESC |
                               FUNCTIONFS_HAS_MS_OS_DESC);
    memcpy(ffs_desc_blob + 0,  &magic,  4);
    memcpy(ffs_desc_blob + 4,  &length, 4);
    memcpy(ffs_desc_blob + 8,  &flags,  4);

    /* patch OS section dwLength */
    uint32_t os_dw = htole32(OS_TOTAL_SZ);
    /* OS section starts at offset 12+16+23+23+48 = 122; dwLength at +1 */
    memcpy(ffs_desc_blob + 123, &os_dw, 4);

    if (write_all(ep0, ffs_desc_blob, sizeof(ffs_desc_blob)) < 0) {
        fprintf(stderr, "[USB] descriptor write: %m\n");
        return -1;
    }
    return 0;
}

/* ── ep0 event loop ──────────────────────────────────────────── */
static int wait_for_enable(int ep0)
{
    struct pollfd pfd = { ep0, POLLIN, 0 };
    for (;;) {
        int rc = poll(&pfd, 1, -1);
        if (rc < 0) { if (errno == EINTR) continue; perror("poll"); return -1; }

        struct usb_functionfs_event ev;
        if (read(ep0, &ev, sizeof(ev)) < (ssize_t)sizeof(ev)) continue;

        switch (ev.type) {
        case FUNCTIONFS_ENABLE:
            printf("[USB] Interface enabled\n");
            return 0;
        case FUNCTIONFS_DISABLE:
            printf("[USB] Disconnected, waiting...\n");
            break;
        case FUNCTIONFS_BIND:
            printf("[USB] Gadget bound\n");
            break;
        case FUNCTIONFS_UNBIND:
            printf("[USB] Gadget unbound — exit\n");
            return -1;
        case FUNCTIONFS_SETUP:
            /* ZLP ACK for host-to-device SETUP with no data */
            if (!(ev.u.setup.bRequestType & USB_DIR_IN))
                write(ep0, "", 0);
            break;
        default: break;
        }
    }
}

/* ── CSW send ────────────────────────────────────────────────── */
static int send_csw(int ep_in, uint32_t tag,
                    uint32_t residue, uint8_t status, uint8_t err)
{
    struct prog_csw csw = {
        .signature  = htole32(PROG_CSW_SIG),
        .tag        = htole32(tag),
        .residue    = htole32(residue),
        .status     = status,
        .error_code = err,
    };
    return write_all(ep_in, &csw, sizeof(csw));
}

/* ═══════════════════════════════════════════════════════════════
 * Command dispatch
 * ═══════════════════════════════════════════════════════════════ */

static int handle_get_fw_version(int ep_in, uint32_t tag)
{
    struct fw_version_resp r = {
        .major = FW_MAJOR,
        .minor = FW_MINOR,
        .patch = FW_PATCH,
    };
    snprintf(r.build_date, sizeof(r.build_date), "%s", __DATE__);
    snprintf(r.git_hash,   sizeof(r.git_hash),   "unknown");
    write_all(ep_in, &r, sizeof(r));
    return send_csw(ep_in, tag, 0, CSW_OK, ERR_OK);
}

static int handle_get_chip_type(int ep_in, uint32_t tag)
{
    write_all(ep_in, &active_chip, 1);
    return send_csw(ep_in, tag, 0, CSW_OK, ERR_OK);
}

static int handle_get_device_info(int ep_in, uint32_t tag)
{
    struct device_info info = { .chip_type = active_chip };
    int rc = 0;
    if (active_chip == CHIP_EMMC)
        rc = emmc_get_device_info(emmc_fd, &info);
    else if (active_chip == CHIP_UFS)
        rc = ufs_get_device_info(ufs_bsg_fd, &info);
    else
        return send_csw(ep_in, tag, 0, CSW_FAIL, ERR_NO_DEVICE);

    if (rc < 0)
        return send_csw(ep_in, tag, 0, CSW_FAIL, ERR_IOCTL_FAIL);

    write_all(ep_in, &info, sizeof(info));
    return send_csw(ep_in, tag, 0, CSW_OK, ERR_OK);
}

static int handle_begin_session(const struct prog_cbw *cbw,
                                int ep_out, int ep_in, uint32_t tag)
{
    const struct begin_session_req *req =
        (const struct begin_session_req *)cbw->params;

    /* Close previous session */
    if (emmc_fd >= 0) { emmc_close(emmc_fd); emmc_fd = -1; }
    if (ufs_bsg_fd >= 0) { ufs_close(ufs_bsg_fd); ufs_bsg_fd = -1; }
    active_chip = CHIP_NONE;

    if (req->chip_type == CHIP_EMMC) {
        emmc_fd = emmc_open();
        if (emmc_fd < 0)
            return send_csw(ep_in, tag, 0, CSW_FAIL, ERR_NO_DEVICE);
        active_chip = CHIP_EMMC;
        printf("[USB] Session: eMMC\n");
    } else if (req->chip_type == CHIP_UFS) {
        ufs_bsg_fd = ufs_open();
        if (ufs_bsg_fd < 0)
            return send_csw(ep_in, tag, 0, CSW_FAIL, ERR_NO_DEVICE);
        active_chip = CHIP_UFS;
        printf("[USB] Session: UFS\n");
    } else {
        return send_csw(ep_in, tag, 0, CSW_FAIL, ERR_BAD_CHIP_TYPE);
    }
    (void)ep_out;
    return send_csw(ep_in, tag, 0, CSW_OK, ERR_OK);
}

static int handle_end_session(int ep_in, uint32_t tag)
{
    if (emmc_fd >= 0) { emmc_close(emmc_fd); emmc_fd = -1; }
    if (ufs_bsg_fd >= 0) { ufs_close(ufs_bsg_fd); ufs_bsg_fd = -1; }
    active_chip = CHIP_NONE;
    return send_csw(ep_in, tag, 0, CSW_OK, ERR_OK);
}

/* ── eMMC dispatch helpers ───────────────────────────────────── */

static int dispatch_emmc(const struct prog_cbw *cbw,
                         int ep_out, int ep_in)
{
    uint32_t tag     = le32toh(cbw->tag);
    uint64_t lba     = le64toh(cbw->lba);
    uint32_t count   = le32toh(cbw->sector_count);
    uint32_t datalen = le32toh(cbw->transfer_len);
    int rc;

    if (emmc_fd < 0)
        return send_csw(ep_in, tag, 0, CSW_FAIL, ERR_NO_DEVICE);

    switch (cbw->opcode) {

    case CMD_EMMC_READ_CID: {
        uint8_t cid[16] = {0};
        rc = emmc_read_cid(emmc_fd, cid);
        if (rc < 0) return send_csw(ep_in, tag, 0, CSW_FAIL, ERR_IOCTL_FAIL);
        write_all(ep_in, cid, 16);
        return send_csw(ep_in, tag, 0, CSW_OK, ERR_OK);
    }

    case CMD_EMMC_READ_CSD: {
        uint8_t csd[16] = {0};
        rc = emmc_read_csd(emmc_fd, csd);
        if (rc < 0) return send_csw(ep_in, tag, 0, CSW_FAIL, ERR_IOCTL_FAIL);
        write_all(ep_in, csd, 16);
        return send_csw(ep_in, tag, 0, CSW_OK, ERR_OK);
    }

    case CMD_EMMC_READ_EXT_CSD: {
        uint8_t ext_csd[512] = {0};
        rc = emmc_read_ext_csd(emmc_fd, ext_csd);
        if (rc < 0) return send_csw(ep_in, tag, 0, CSW_FAIL, ERR_IOCTL_FAIL);
        write_all(ep_in, ext_csd, 512);
        return send_csw(ep_in, tag, 0, CSW_OK, ERR_OK);
    }

    case CMD_EMMC_WRITE_EXT_CSD: {
        const struct emmc_switch_req *sw =
            (const struct emmc_switch_req *)cbw->params;
        rc = emmc_switch(emmc_fd, sw->index, sw->value, sw->cmd_set);
        return send_csw(ep_in, tag, 0,
                        rc < 0 ? CSW_FAIL : CSW_OK,
                        rc < 0 ? ERR_IOCTL_FAIL : ERR_OK);
    }

    case CMD_EMMC_SELECT_PART: {
        const struct emmc_select_part_req *p =
            (const struct emmc_select_part_req *)cbw->params;
        rc = emmc_select_partition(emmc_fd, p->partition);
        return send_csw(ep_in, tag, 0,
                        rc < 0 ? CSW_FAIL : CSW_OK,
                        rc < 0 ? ERR_IOCTL_FAIL : ERR_OK);
    }

    case CMD_EMMC_READ_SECTORS: {
        if (datalen == 0 || datalen > BUF_SIZE)
            return send_csw(ep_in, tag, datalen, CSW_FAIL, ERR_BAD_PARAM);
        rc = emmc_read_sectors(emmc_fd, lba, count, io_buf);
        if (rc < 0) return send_csw(ep_in, tag, datalen, CSW_FAIL, ERR_IO);
        write_all(ep_in, io_buf, datalen);
        return send_csw(ep_in, tag, 0, CSW_OK, ERR_OK);
    }

    case CMD_EMMC_WRITE_SECTORS: {
        if (datalen == 0 || datalen > BUF_SIZE)
            return send_csw(ep_in, tag, datalen, CSW_FAIL, ERR_BAD_PARAM);
        if (read_all(ep_out, io_buf, datalen) < 0)
            return send_csw(ep_in, tag, datalen, CSW_PHASE_ERR, ERR_IO);
        rc = emmc_write_sectors(emmc_fd, lba, count, io_buf);
        return send_csw(ep_in, tag, 0,
                        rc < 0 ? CSW_FAIL : CSW_OK,
                        rc < 0 ? ERR_IO : ERR_OK);
    }

    case CMD_EMMC_ERASE:
    case CMD_EMMC_TRIM:
    case CMD_EMMC_SECURE_ERASE:
    case CMD_EMMC_SECURE_TRIM: {
        const struct emmc_erase_req *er =
            (const struct emmc_erase_req *)cbw->params;
        uint32_t arg = 0;
        if (cbw->opcode == CMD_EMMC_TRIM)        arg = 1;
        if (cbw->opcode == CMD_EMMC_SECURE_ERASE) arg = 0x80000000;
        if (cbw->opcode == CMD_EMMC_SECURE_TRIM)  arg = 0x80000001;
        rc = emmc_erase(emmc_fd,
                        le64toh(er->start_lba),
                        le64toh(er->end_lba),
                        arg);
        return send_csw(ep_in, tag, 0,
                        rc < 0 ? CSW_FAIL : CSW_OK,
                        rc < 0 ? ERR_IOCTL_FAIL : ERR_OK);
    }

    case CMD_EMMC_SET_WRITE_PROT:
    case CMD_EMMC_CLR_WRITE_PROT: {
        const struct write_prot_req *wp =
            (const struct write_prot_req *)cbw->params;
        rc = emmc_set_write_protect(emmc_fd,
                                    le64toh(wp->lba),
                                    wp->type,
                                    cbw->opcode == CMD_EMMC_SET_WRITE_PROT);
        return send_csw(ep_in, tag, 0,
                        rc < 0 ? CSW_FAIL : CSW_OK,
                        rc < 0 ? ERR_IOCTL_FAIL : ERR_OK);
    }

    case CMD_EMMC_SEND_WRITE_PROT: {
        uint8_t prot[8] = {0};
        rc = emmc_send_write_prot(emmc_fd, lba, prot);
        if (rc < 0) return send_csw(ep_in, tag, 0, CSW_FAIL, ERR_IOCTL_FAIL);
        write_all(ep_in, prot, 8);
        return send_csw(ep_in, tag, 0, CSW_OK, ERR_OK);
    }

    case CMD_EMMC_BOOT_CONFIG: {
        const struct boot_cfg_req *bc =
            (const struct boot_cfg_req *)cbw->params;
        rc = emmc_set_boot_config(emmc_fd, bc->boot_config, bc->boot_bus_cond);
        return send_csw(ep_in, tag, 0,
                        rc < 0 ? CSW_FAIL : CSW_OK,
                        rc < 0 ? ERR_IOCTL_FAIL : ERR_OK);
    }

    case CMD_EMMC_RPMB_READ:
    case CMD_EMMC_RPMB_WRITE:
    case CMD_EMMC_RPMB_GET_WR_CNT: {
        const struct rpmb_req *rr =
            (const struct rpmb_req *)cbw->params;
        uint8_t frame[512] = {0};

        if (cbw->opcode == CMD_EMMC_RPMB_WRITE) {
            if (read_all(ep_out, frame, 512) < 0)
                return send_csw(ep_in, tag, 512, CSW_PHASE_ERR, ERR_IO);
        }

        rc = emmc_rpmb_op(emmc_fd, cbw->opcode, rr, frame);
        if (rc < 0) return send_csw(ep_in, tag, 0, CSW_FAIL, ERR_RPMB_AUTH);

        if (cbw->opcode != CMD_EMMC_RPMB_WRITE)
            write_all(ep_in, frame, 512);

        return send_csw(ep_in, tag, 0, CSW_OK, ERR_OK);
    }

    case CMD_EMMC_FFU_DOWNLOAD: {
        if (datalen == 0 || datalen > BUF_SIZE)
            return send_csw(ep_in, tag, datalen, CSW_FAIL, ERR_BAD_PARAM);
        if (read_all(ep_out, io_buf, datalen) < 0)
            return send_csw(ep_in, tag, datalen, CSW_PHASE_ERR, ERR_IO);
        rc = emmc_ffu_download(emmc_fd, io_buf, datalen);
        return send_csw(ep_in, tag, 0,
                        rc < 0 ? CSW_FAIL : CSW_OK,
                        rc < 0 ? ERR_FFU_FAIL : ERR_OK);
    }

    case CMD_EMMC_FFU_INSTALL: {
        rc = emmc_ffu_install(emmc_fd);
        return send_csw(ep_in, tag, 0,
                        rc < 0 ? CSW_FAIL : CSW_OK,
                        rc < 0 ? ERR_FFU_FAIL : ERR_OK);
    }

    case CMD_EMMC_SLEEP:
    case CMD_EMMC_AWAKE: {
        rc = emmc_sleep_awake(emmc_fd, cbw->opcode == CMD_EMMC_SLEEP);
        return send_csw(ep_in, tag, 0,
                        rc < 0 ? CSW_FAIL : CSW_OK,
                        rc < 0 ? ERR_IOCTL_FAIL : ERR_OK);
    }

    default:
        return send_csw(ep_in, tag, 0, CSW_FAIL, ERR_UNKNOWN_CMD);
    }
}

/* ── UFS dispatch helpers ────────────────────────────────────── */

static int dispatch_ufs(const struct prog_cbw *cbw,
                        int ep_out, int ep_in)
{
    uint32_t tag     = le32toh(cbw->tag);
    uint64_t lba     = le64toh(cbw->lba);
    uint32_t count   = le32toh(cbw->sector_count);
    uint32_t datalen = le32toh(cbw->transfer_len);
    int rc;

    if (ufs_bsg_fd < 0)
        return send_csw(ep_in, tag, 0, CSW_FAIL, ERR_NO_DEVICE);

    switch (cbw->opcode) {

    case CMD_UFS_READ_DESC: {
        const struct ufs_desc_req *dr =
            (const struct ufs_desc_req *)cbw->params;
        uint8_t buf[256] = {0};
        rc = ufs_query_read_desc(ufs_bsg_fd, dr->idn, dr->index,
                                 dr->selector, buf, sizeof(buf));
        if (rc < 0) return send_csw(ep_in, tag, 0, CSW_FAIL, ERR_BSG_FAIL);
        write_all(ep_in, buf, sizeof(buf));
        return send_csw(ep_in, tag, 0, CSW_OK, ERR_OK);
    }

    case CMD_UFS_WRITE_DESC: {
        const struct ufs_desc_req *dr =
            (const struct ufs_desc_req *)cbw->params;
        if (datalen == 0 || datalen > 256)
            return send_csw(ep_in, tag, datalen, CSW_FAIL, ERR_BAD_PARAM);
        if (read_all(ep_out, io_buf, datalen) < 0)
            return send_csw(ep_in, tag, datalen, CSW_PHASE_ERR, ERR_IO);
        rc = ufs_query_write_desc(ufs_bsg_fd, dr->idn, dr->index,
                                  dr->selector, io_buf, datalen);
        return send_csw(ep_in, tag, 0,
                        rc < 0 ? CSW_FAIL : CSW_OK,
                        rc < 0 ? ERR_BSG_FAIL : ERR_OK);
    }

    case CMD_UFS_READ_ATTR: {
        const struct ufs_attr_req *ar =
            (const struct ufs_attr_req *)cbw->params;
        uint32_t val = 0;
        rc = ufs_query_read_attr(ufs_bsg_fd, ar->idn, ar->index,
                                 ar->selector, &val);
        if (rc < 0) return send_csw(ep_in, tag, 0, CSW_FAIL, ERR_BSG_FAIL);
        uint32_t v_le = htole32(val);
        write_all(ep_in, &v_le, 4);
        return send_csw(ep_in, tag, 0, CSW_OK, ERR_OK);
    }

    case CMD_UFS_WRITE_ATTR: {
        const struct ufs_attr_req *ar =
            (const struct ufs_attr_req *)cbw->params;
        rc = ufs_query_write_attr(ufs_bsg_fd, ar->idn, ar->index,
                                  ar->selector, le32toh(ar->value));
        return send_csw(ep_in, tag, 0,
                        rc < 0 ? CSW_FAIL : CSW_OK,
                        rc < 0 ? ERR_BSG_FAIL : ERR_OK);
    }

    case CMD_UFS_READ_FLAG: {
        const struct ufs_flag_req *fr =
            (const struct ufs_flag_req *)cbw->params;
        uint8_t val = 0;
        rc = ufs_query_read_flag(ufs_bsg_fd, fr->idn, fr->index,
                                 fr->selector, &val);
        if (rc < 0) return send_csw(ep_in, tag, 0, CSW_FAIL, ERR_BSG_FAIL);
        write_all(ep_in, &val, 1);
        return send_csw(ep_in, tag, 0, CSW_OK, ERR_OK);
    }

    case CMD_UFS_SET_FLAG:
    case CMD_UFS_CLEAR_FLAG:
    case CMD_UFS_TOGGLE_FLAG: {
        const struct ufs_flag_req *fr =
            (const struct ufs_flag_req *)cbw->params;
        rc = ufs_query_flag(ufs_bsg_fd, cbw->opcode,
                            fr->idn, fr->index, fr->selector);
        return send_csw(ep_in, tag, 0,
                        rc < 0 ? CSW_FAIL : CSW_OK,
                        rc < 0 ? ERR_BSG_FAIL : ERR_OK);
    }

    case CMD_UFS_READ_SECTORS: {
        if (datalen == 0 || datalen > BUF_SIZE)
            return send_csw(ep_in, tag, datalen, CSW_FAIL, ERR_BAD_PARAM);
        rc = ufs_read_sectors(ufs_bsg_fd, lba, count, io_buf);
        if (rc < 0) return send_csw(ep_in, tag, datalen, CSW_FAIL, ERR_IO);
        write_all(ep_in, io_buf, datalen);
        return send_csw(ep_in, tag, 0, CSW_OK, ERR_OK);
    }

    case CMD_UFS_WRITE_SECTORS: {
        if (datalen == 0 || datalen > BUF_SIZE)
            return send_csw(ep_in, tag, datalen, CSW_FAIL, ERR_BAD_PARAM);
        if (read_all(ep_out, io_buf, datalen) < 0)
            return send_csw(ep_in, tag, datalen, CSW_PHASE_ERR, ERR_IO);
        rc = ufs_write_sectors(ufs_bsg_fd, lba, count, io_buf);
        return send_csw(ep_in, tag, 0,
                        rc < 0 ? CSW_FAIL : CSW_OK,
                        rc < 0 ? ERR_IO : ERR_OK);
    }

    case CMD_UFS_UNMAP: {
        rc = ufs_unmap(ufs_bsg_fd, lba, count);
        return send_csw(ep_in, tag, 0,
                        rc < 0 ? CSW_FAIL : CSW_OK,
                        rc < 0 ? ERR_BSG_FAIL : ERR_OK);
    }

    case CMD_UFS_PURGE: {
        rc = ufs_purge(ufs_bsg_fd);
        return send_csw(ep_in, tag, 0,
                        rc < 0 ? CSW_FAIL : CSW_OK,
                        rc < 0 ? ERR_BSG_FAIL : ERR_OK);
    }

    case CMD_UFS_FDEVICEINIT: {
        rc = ufs_fdeviceinit(ufs_bsg_fd);
        return send_csw(ep_in, tag, 0,
                        rc < 0 ? CSW_FAIL : CSW_OK,
                        rc < 0 ? ERR_BSG_FAIL : ERR_OK);
    }

    case CMD_UFS_PROVISION_LU: {
        if (datalen == 0 || datalen > 256)
            return send_csw(ep_in, tag, datalen, CSW_FAIL, ERR_BAD_PARAM);
        if (read_all(ep_out, io_buf, datalen) < 0)
            return send_csw(ep_in, tag, datalen, CSW_PHASE_ERR, ERR_IO);
        rc = ufs_provision_lu(ufs_bsg_fd, io_buf, datalen);
        return send_csw(ep_in, tag, 0,
                        rc < 0 ? CSW_FAIL : CSW_OK,
                        rc < 0 ? ERR_BSG_FAIL : ERR_OK);
    }

    case CMD_UFS_RPMB_READ:
    case CMD_UFS_RPMB_WRITE:
    case CMD_UFS_RPMB_GET_WR_CNT: {
        const struct rpmb_req *rr =
            (const struct rpmb_req *)cbw->params;
        uint8_t frame[512] = {0};

        if (cbw->opcode == CMD_UFS_RPMB_WRITE) {
            if (read_all(ep_out, frame, 512) < 0)
                return send_csw(ep_in, tag, 512, CSW_PHASE_ERR, ERR_IO);
        }

        rc = ufs_rpmb_op(ufs_bsg_fd, cbw->opcode, rr, frame);
        if (rc < 0) return send_csw(ep_in, tag, 0, CSW_FAIL, ERR_RPMB_AUTH);

        if (cbw->opcode != CMD_UFS_RPMB_WRITE)
            write_all(ep_in, frame, 512);

        return send_csw(ep_in, tag, 0, CSW_OK, ERR_OK);
    }

    case CMD_UFS_FFU: {
        if (datalen == 0 || datalen > BUF_SIZE)
            return send_csw(ep_in, tag, datalen, CSW_FAIL, ERR_BAD_PARAM);
        if (read_all(ep_out, io_buf, datalen) < 0)
            return send_csw(ep_in, tag, datalen, CSW_PHASE_ERR, ERR_IO);
        rc = ufs_ffu(ufs_bsg_fd, io_buf, datalen);
        return send_csw(ep_in, tag, 0,
                        rc < 0 ? CSW_FAIL : CSW_OK,
                        rc < 0 ? ERR_FFU_FAIL : ERR_OK);
    }

    case CMD_UFS_POWER_MODE: {
        const struct ufs_power_mode_req *pm =
            (const struct ufs_power_mode_req *)cbw->params;
        rc = ufs_set_power_mode(ufs_bsg_fd, pm->mode);
        return send_csw(ep_in, tag, 0,
                        rc < 0 ? CSW_FAIL : CSW_OK,
                        rc < 0 ? ERR_BSG_FAIL : ERR_OK);
    }

    case CMD_UFS_NOP: {
        rc = ufs_nop(ufs_bsg_fd);
        return send_csw(ep_in, tag, 0,
                        rc < 0 ? CSW_FAIL : CSW_OK,
                        rc < 0 ? ERR_BSG_FAIL : ERR_OK);
    }

    case CMD_UFS_QUERY_RAW: {
        if (datalen == 0 || datalen > 512)
            return send_csw(ep_in, tag, datalen, CSW_FAIL, ERR_BAD_PARAM);
        if (read_all(ep_out, io_buf, datalen) < 0)
            return send_csw(ep_in, tag, datalen, CSW_PHASE_ERR, ERR_IO);
        uint8_t resp[512] = {0};
        rc = ufs_query_raw(ufs_bsg_fd, io_buf, datalen, resp, sizeof(resp));
        if (rc < 0) return send_csw(ep_in, tag, 0, CSW_FAIL, ERR_BSG_FAIL);
        write_all(ep_in, resp, sizeof(resp));
        return send_csw(ep_in, tag, 0, CSW_OK, ERR_OK);
    }

    default:
        return send_csw(ep_in, tag, 0, CSW_FAIL, ERR_UNKNOWN_CMD);
    }
}

/* ── Main command dispatcher ─────────────────────────────────── */
static int dispatch(const struct prog_cbw *cbw, int ep_out, int ep_in)
{
    uint32_t tag = le32toh(cbw->tag);

    switch (cbw->opcode) {
    case CMD_GET_FW_VERSION:  return handle_get_fw_version(ep_in, tag);
    case CMD_GET_CHIP_TYPE:   return handle_get_chip_type(ep_in, tag);
    case CMD_GET_DEVICE_INFO: return handle_get_device_info(ep_in, tag);
    case CMD_BEGIN_SESSION:   return handle_begin_session(cbw, ep_out, ep_in, tag);
    case CMD_END_SESSION:     return handle_end_session(ep_in, tag);
    case CMD_ABORT:
        /* flush ep_out, send OK */
        return send_csw(ep_in, tag, 0, CSW_OK, ERR_OK);

    /* eMMC commands */
    case CMD_EMMC_READ_CID ... CMD_EMMC_SELECT_PART:
        return dispatch_emmc(cbw, ep_out, ep_in);

    /* UFS commands */
    case CMD_UFS_READ_DESC ... CMD_UFS_QUERY_RAW:
        return dispatch_ufs(cbw, ep_out, ep_in);

    default:
        return send_csw(ep_in, tag, 0, CSW_FAIL, ERR_UNKNOWN_CMD);
    }
}

/* ── Bulk I/O loop ───────────────────────────────────────────── */
static int bulk_loop(int ep_in, int ep_out)
{
    printf("[USB] Bulk loop running\n");
    for (;;) {
        struct prog_cbw cbw;
        memset(&cbw, 0, sizeof(cbw));

        ssize_t n = read(ep_out, &cbw, sizeof(cbw));
        if (n < 0) {
            if (errno == EINTR)    continue;
            if (errno == ESHUTDOWN || errno == ECONNRESET) {
                printf("[USB] Host disconnected\n");
                return 0;
            }
            perror("[USB] read CBW");
            return -1;
        }
        if (n != sizeof(cbw)) {
            fprintf(stderr, "[USB] Short CBW (%zd bytes)\n", n);
            continue;
        }
        if (le32toh(cbw.signature) != PROG_CBW_SIG) {
            fprintf(stderr, "[USB] Bad CBW signature\n");
            continue;
        }

        if (dispatch(&cbw, ep_out, ep_in) < 0)
            return -1;
    }
}

/* ═══════════════════════════════════════════════════════════════
 * Entry point
 * ═══════════════════════════════════════════════════════════════ */
int main(void)
{
    int ep0 = -1, ep_in = -1, ep_out = -1;
    int ret = 1;

    ep0 = open(FFS_MOUNT "/ep0", O_RDWR);
    if (ep0 < 0) {
        perror("open ep0 — is functionfs mounted?");
        goto out;
    }

    if (write_descriptors(ep0) < 0) goto out;
    if (write_all(ep0, ffs_str_blob, sizeof(ffs_str_blob)) < 0) {
        fprintf(stderr, "[USB] string write: %m\n");
        goto out;
    }

    printf("[USB] Descriptors ready. Bind UDC now if not done:\n");
    printf("      echo $(ls /sys/class/udc|head -1)"
           " > /sys/kernel/config/usb_gadget/g1/UDC\n\n");

    for (;;) {
        if (wait_for_enable(ep0) < 0) break;

        if (ep_in  >= 0) { close(ep_in);  ep_in  = -1; }
        if (ep_out >= 0) { close(ep_out); ep_out = -1; }

        ep_in  = open(FFS_MOUNT "/ep1", O_RDWR);
        ep_out = open(FFS_MOUNT "/ep2", O_RDWR);
        if (ep_in < 0 || ep_out < 0) {
            perror("[USB] open data eps");
            break;
        }

        if (bulk_loop(ep_in, ep_out) < 0) break;
        /* disconnect — loop, wait for re-enable */
    }

    ret = 0;
out:
    if (ep_out >= 0) close(ep_out);
    if (ep_in  >= 0) close(ep_in);
    if (ep0    >= 0) close(ep0);
    if (emmc_fd    >= 0) emmc_close(emmc_fd);
    if (ufs_bsg_fd >= 0) ufs_close(ufs_bsg_fd);
    return ret;
}
