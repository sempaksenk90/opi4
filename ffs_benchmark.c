/*
 * ffs_benchmark.c - FunctionFS bulk loopback bandwidth benchmark (host side).
 *
 * Device under test: VID 1d6b / PID 0104 (dwc3 Bulk/Int/Iso/Ctrl FFS Gadget)
 *   ep1 -> 0x01  bulk OUT  (host -> device)
 *   ep2 -> 0x81  bulk IN   (device -> host, echo of OUT)
 *
 * Uses libusb-1.0 in ASYNC/pipelined mode: it keeps `qlen` bulk-OUT and
 * `qlen` bulk-IN transfers in flight at all times, draining IN as soon as the
 * device echoes. This removes the host's synchronous per-chunk round-trip
 * latency so the measured throughput reflects the device daemon's real echo
 * rate (the daemon still processes one 1024-B chunk at a time, but it is
 * never starved and IN is never back-pressured by the host).
 *
 * Build:  gcc -Wall -O2 -o ffs_benchmark ffs_benchmark.c -lusb-1.0
 * Run:    sudo ./ffs_benchmark [-s size] [-c chunk] [-q qlen] [-f file]
 *   -f file : use FILE contents as the payload (real image copy/verify).
 *             total size defaults to the file size unless -s overrides.
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <time.h>
#include <unistd.h>
#include <libusb-1.0/libusb.h>

#ifdef _WIN32
#include <windows.h>
#endif

/* Portable monotonic clock (seconds). Windows/MinGW lacks clock_gettime(). */
static double now_sec(void)
{
#ifdef _WIN32
    LARGE_INTEGER freq, cnt;
    QueryPerformanceFrequency(&freq);
    QueryPerformanceCounter(&cnt);
    return (double)cnt.QuadPart / (double)freq.QuadPart;
#else
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec / 1e9;
#endif
}

#define VID 0x1d6b
#define PID 0x0104
#define EP_OUT 0x01
#define EP_IN  0x81
#define IFACE  0

static libusb_device_handle *devh;
static unsigned char *payload;
static size_t total_size = 8 * 1024 * 1024;
static size_t chunk = 16384;
static unsigned qlen = 16;
static const char *g_file = NULL;

static size_t out_next;     /* next OUT chunk index to submit            */
static size_t in_next;      /* next expected IN chunk index (in-order)   */
static size_t recv_total;   /* bytes received so far                     */
static int done;
static int error_flag;
static int g_err_status;    /* last failing transfer status              */
static int g_err_ep;        /* 0=OUT, 1=IN                               */

static uint32_t checksum;   /* FNV-1a over received stream, in order     */

static size_t chunk_len(size_t idx)
{
    size_t end = (idx + 1) * chunk;
    return end <= total_size ? chunk : total_size - idx * chunk;
}

static uint32_t fnv1a(const unsigned char *p, size_t n, uint32_t h)
{
    for (size_t i = 0; i < n; i++) {
        h ^= p[i];
        h *= 16777619u;
    }
    return h;
}

static void out_cb(struct libusb_transfer *t)
{
    if (t->status != LIBUSB_TRANSFER_COMPLETED) {
        fprintf(stderr, "[OUT] transfer error status=%d\n", t->status);
        g_err_status = t->status; g_err_ep = 0;
        error_flag = 1;
        return;
    }
    if (out_next < (total_size + chunk - 1) / chunk) {
        size_t idx = out_next++;
        size_t len = chunk_len(idx);
        memcpy(t->buffer, payload + idx * chunk, len);
        libusb_fill_bulk_transfer(t, devh, EP_OUT, t->buffer,
                                  (int)len, out_cb, NULL, 0);
        if (libusb_submit_transfer(t) != 0) {
            error_flag = 1;
            free(t->buffer);
            libusb_free_transfer(t);
        }
    } else {
        free(t->buffer);
        libusb_free_transfer(t);
    }
}

static void in_cb(struct libusb_transfer *t)
{
    if (t->status != LIBUSB_TRANSFER_COMPLETED) {
        fprintf(stderr, "[IN] transfer error status=%d\n", t->status);
        g_err_status = t->status; g_err_ep = 1;
        error_flag = 1;
        return;
    }
    int n = t->actual_length;
    size_t idx = in_next;
    size_t exp = chunk_len(idx);

    if ((size_t)n != exp ||
        memcmp(t->buffer, payload + idx * chunk, n) != 0) {
        fprintf(stderr, "[IN] echo mismatch at chunk %zu (got %d, expected %zu)\n",
                idx, n, exp);
        error_flag = 1;
        return;
    }
    checksum = fnv1a(t->buffer, n, checksum);
    in_next++;
    recv_total += (size_t)n;

    if (recv_total < total_size) {
        libusb_fill_bulk_transfer(t, devh, EP_IN, t->buffer,
                                  (int)chunk, in_cb, NULL, 0);
        if (libusb_submit_transfer(t) != 0) {
            error_flag = 1;
            free(t->buffer);
            libusb_free_transfer(t);
        }
    } else {
        done = 1;
        free(t->buffer);
        libusb_free_transfer(t);
    }
}

int main(int argc, char **argv)
{
    int c;
    while ((c = getopt(argc, argv, "s:c:q:f:")) != -1) {
        switch (c) {
        case 's': total_size = (size_t)atoll(optarg); break;
        case 'c': chunk     = (size_t)atoll(optarg); break;
        case 'q': qlen      = (unsigned)atoi(optarg); break;
        case 'f': g_file    = optarg; break;
        default:
            fprintf(stderr, "usage: %s [-s size] [-c chunk] [-q qlen] [-f file]\n", argv[0]);
            return 2;
        }
    }
    if (chunk < 1 || qlen < 1 || total_size < 1) {
        fprintf(stderr, "invalid args\n");
        return 2;
    }

    /* Host usbfs caps a single bulk URB at 128 KiB; larger chunks fail the
     * transfer (0 bytes / mismatch). Clamp so the benchmark stays valid. */
    if (chunk > 131072) {
        fprintf(stderr, "WARNING: host usbfs caps one URB at 128 KiB; "
                        "clamping chunk %zu -> 131072\n", chunk);
        chunk = 131072;
    }

    if (g_file) {
        FILE *f = fopen(g_file, "rb");
        if (!f) { perror("fopen input"); return 1; }
        fseek(f, 0, SEEK_END);
        long fsz = ftell(f);
        fseek(f, 0, SEEK_SET);
        if (fsz <= 0) { fprintf(stderr, "bad input file\n"); fclose(f); return 1; }
        if (total_size == 8 * 1024 * 1024)   /* -s not given: use file size */
            total_size = (size_t)fsz;
        else if ((long)total_size > fsz) {
            fprintf(stderr, "file too small: need %zu have %ld\n",
                    total_size, fsz);
            fclose(f); return 1;
        }
        payload = malloc(total_size);
        if (!payload) { perror("malloc"); fclose(f); return 1; }
        size_t rd = fread(payload, 1, total_size, f);
        fclose(f);
        if (rd != total_size) {
            fprintf(stderr, "short read %zu/%zu from file\n", rd, total_size);
            free(payload); return 1;
        }
    } else {
        payload = malloc(total_size);
        if (!payload) { perror("malloc"); return 1; }
        for (size_t i = 0; i < total_size; i++)
            payload[i] = (unsigned char)((i * 7 + 13) & 0xFF);
    }

    uint32_t expected = fnv1a(payload, total_size, 2166136261u);

    libusb_context *ctx = NULL;
    if (libusb_init(&ctx) != 0) {
        fprintf(stderr, "libusb_init failed\n");
        return 1;
    }

    devh = libusb_open_device_with_vid_pid(ctx, VID, PID);
    if (!devh) {
        fprintf(stderr, "device 1d6b:0104 not found\n");
        libusb_exit(ctx);
        return 1;
    }
    if (libusb_claim_interface(devh, IFACE) != 0) {
        fprintf(stderr, "claim interface failed\n");
        libusb_close(devh);
        libusb_exit(ctx);
        return 1;
    }

    out_next = 0;
    in_next = 0;
    recv_total = 0;
    done = 0;
    error_flag = 0;
    checksum = 2166136261u;

    printf("Found 1d6b:0104  size=%zu B  chunk=%zu B  qlen=%u\n",
           total_size, chunk, qlen);
    printf("Running async pipelined loopback...\n");

    unsigned nchunks = (unsigned)((total_size + chunk - 1) / chunk);
    for (unsigned i = 0; i < qlen; i++) {
        /* OUT */
        if (out_next < nchunks) {
            struct libusb_transfer *t = libusb_alloc_transfer(0);
            unsigned char *buf = malloc(chunk);
            size_t idx = out_next++;
            size_t len = chunk_len(idx);
            memcpy(buf, payload + idx * chunk, len);
            libusb_fill_bulk_transfer(t, devh, EP_OUT, buf,
                                      (int)len, out_cb, NULL, 0);
            if (libusb_submit_transfer(t) != 0) {
                error_flag = 1; free(buf); libusb_free_transfer(t);
            }
        }
        /* IN */
        {
            struct libusb_transfer *t = libusb_alloc_transfer(0);
            unsigned char *buf = malloc(chunk);
            libusb_fill_bulk_transfer(t, devh, EP_IN, buf,
                                      (int)chunk, in_cb, NULL, 0);
            if (libusb_submit_transfer(t) != 0) {
                error_flag = 1; free(buf); libusb_free_transfer(t);
            }
        }
    }

    double t0 = now_sec();

    struct timeval tv;
    tv.tv_sec = 0; tv.tv_usec = 200000;
    while (!done && !error_flag)
        libusb_handle_events_timeout_completed(ctx, &tv, NULL);

    double t1 = now_sec();
    double elapsed = t1 - t0;

    libusb_release_interface(devh, IFACE);
    libusb_close(devh);
    libusb_exit(ctx);

    double mbps = (total_size / (1024.0 * 1024.0)) / (elapsed > 0 ? elapsed : 1e-9);
    if (error_flag || recv_total != total_size || checksum != expected) {
        const char *ep = (g_err_ep == 0) ? "OUT" : "IN";
        const char *why = (recv_total == 0 && !error_flag)
                          ? "0 bytes received" : "transfer error";
        printf("FAILED: %s (%s ep=%s status=%d), transferred %zu/%zu B\n",
               why, (error_flag ? "err" : "short"), ep, g_err_status,
               recv_total, total_size);
        free(payload);
        return 1;
    }
    printf("Transferred %zu B in %.3f s -> %.2f MB/s\n",
           recv_total, elapsed, mbps);
    printf("Checksum: got=0x%08x expected=0x%08x  OK\n", checksum, expected);

    free(payload);
    return 0;
}
