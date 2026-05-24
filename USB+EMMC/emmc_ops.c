/* emmc_ops.c — JEDEC JESD84 eMMC command implementation
 *
 * All commands go through the Linux MMC ioctl interface:
 *   MMC_IOC_CMD       — single command
 *   MMC_IOC_MULTI_CMD — atomic command sequence
 *
 * Kernel: 6.6.x  (MMC ioctl stable since 4.x)
 *
 * eMMC device node: /dev/mmcblk0  (or /dev/mmcblk1 for external socket)
 * RPMB partition:   /dev/mmcblk0rpmb
 *
 * ISP / socket wiring notes:
 *   The A733 SoC exposes its eMMC host controller via the SDIO/MMC pads.
 *   For ISP (In-System Programming), connect the socket/cable to the
 *   eMMC host controller pads, then the kernel will enumerate the chip
 *   as /dev/mmcblk1 (bus 1) or /dev/mmcblk0 depending on your DTS.
 *   Use CMD_BEGIN_SESSION from the host to open the right device.
 */

#include "emmc_ops.h"
#include "protocol.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/mmc/ioctl.h>

/* ── Device nodes ────────────────────────────────────────────── */
#define EMMC_DEV        "/dev/mmcblk0"
#define EMMC_RPMB_DEV   "/dev/mmcblk0rpmb"
#define EMMC_BOOT1_DEV  "/dev/mmcblk0boot0"
#define EMMC_BOOT2_DEV  "/dev/mmcblk0boot1"

/* ── JEDEC eMMC 5.1 opcode mnemonics ────────────────────────── */
#ifndef MMC_GO_IDLE_STATE
#define MMC_GO_IDLE_STATE        0
#define MMC_SEND_OP_COND         1
#define MMC_ALL_SEND_CID         2
#define MMC_SET_RELATIVE_ADDR    3
#define MMC_SLEEP_AWAKE          5
#define MMC_SWITCH               6
#define MMC_SELECT_CARD          7
#define MMC_SEND_EXT_CSD         8
#define MMC_SEND_CSD             9
#define MMC_SEND_CID             10
#define MMC_STOP_TRANSMISSION    12
#define MMC_SEND_STATUS          13
#define MMC_SET_BLOCKLEN         16
#define MMC_READ_SINGLE_BLOCK    17
#define MMC_READ_MULTIPLE_BLOCK  18
#define MMC_SEND_TUNING_BLOCK    19
#define MMC_WRITE_BLOCK          24
#define MMC_WRITE_MULTIPLE_BLOCK 25
#define MMC_PROGRAM_CSD          27
#define MMC_SET_WRITE_PROT       28
#define MMC_CLR_WRITE_PROT       29
#define MMC_SEND_WRITE_PROT      30
#define MMC_SEND_WRITE_PROT_TYPE 31
#define MMC_ERASE_GROUP_START    35
#define MMC_ERASE_GROUP_END      36
#define MMC_ERASE                38
#define MMC_FAST_IO              39
#define MMC_GO_IRQ_STATE         40
#define MMC_LOCK_UNLOCK          42
#define MMC_APP_CMD              55
#define MMC_GEN_CMD              56
#endif

/* EXT_CSD byte indices (JEDEC JESD84-B51 Table 16) */
#define EXT_CSD_FFU_STATUS           26
#define EXT_CSD_MODE_OPERATION_CODES 29
#define EXT_CSD_MODE_CONFIG          30
#define EXT_CSD_FLUSH_CACHE          32
#define EXT_CSD_CACHE_CTRL           33
#define EXT_CSD_POWER_OFF_NOTIFICATION 34
#define EXT_CSD_PACKED_FAILURE_INDEX 35
#define EXT_CSD_PACKED_CMD_STATUS    36
#define EXT_CSD_EXP_EVENTS_STATUS    54
#define EXT_CSD_EXP_EVENTS_CTRL      56
#define EXT_CSD_DATA_SECTOR_SIZE     61
#define EXT_CSD_GP_SIZE_MULT         143  /* [143:154] */
#define EXT_CSD_PARTITION_SETTING    155
#define EXT_CSD_PARTITIONS_ATTRIBUTE 156
#define EXT_CSD_MAX_ENH_SIZE_MULT    157
#define EXT_CSD_PARTITION_SUPPORT    160
#define EXT_CSD_HPI_MGMT             161
#define EXT_CSD_RST_N_FUNCTION       162
#define EXT_CSD_BKOPS_EN             163
#define EXT_CSD_BKOPS_START          164
#define EXT_CSD_SANITIZE_START       165
#define EXT_CSD_WR_REL_PARAM         166
#define EXT_CSD_RPMB_MULT            168
#define EXT_CSD_FW_CONFIG            169
#define EXT_CSD_USER_WP              171
#define EXT_CSD_BOOT_WP              173
#define EXT_CSD_BOOT_WP_STATUS       174
#define EXT_CSD_ERASE_GROUP_DEF      175
#define EXT_CSD_BOOT_BUS_CONDITIONS  177
#define EXT_CSD_BOOT_CONFIG_PROT     178
#define EXT_CSD_PART_CONFIG          179  /* BOOT_CONFIG */
#define EXT_CSD_SLEEP_NOTIFICATION   216
#define EXT_CSD_SLEEP_AWAKE_TIMEOUT  217
#define EXT_CSD_PRODUCTION_STATE_AWARENESS 302
#define EXT_CSD_SECURE_REMOVAL_TYPE  16
#define EXT_CSD_CMDQ_MODE_EN         15

/* MMC response flags */
#ifndef MMC_RSP_PRESENT
#define MMC_RSP_PRESENT  (1 << 0)
#define MMC_RSP_136      (1 << 1)
#define MMC_RSP_CRC      (1 << 2)
#define MMC_RSP_BUSY     (1 << 3)
#define MMC_RSP_OPCODE   (1 << 4)
#define MMC_CMD_MASK     (3 << 5)
#define MMC_CMD_AC       (0 << 5)
#define MMC_CMD_ADTC     (1 << 5)
#define MMC_CMD_BC       (2 << 5)
#define MMC_CMD_BCR      (3 << 5)
#define MMC_RSP_R1       (MMC_RSP_PRESENT|MMC_RSP_CRC|MMC_RSP_OPCODE)
#define MMC_RSP_R1B      (MMC_RSP_PRESENT|MMC_RSP_CRC|MMC_RSP_OPCODE|MMC_RSP_BUSY)
#define MMC_RSP_R2       (MMC_RSP_PRESENT|MMC_RSP_136|MMC_RSP_CRC)
#define MMC_RSP_R3       (MMC_RSP_PRESENT)
#define MMC_RSP_R4       (MMC_RSP_PRESENT)
#define MMC_RSP_R5       (MMC_RSP_PRESENT|MMC_RSP_CRC|MMC_RSP_OPCODE)
#define MMC_RSP_NONE     0
#endif

#ifndef MMC_SWITCH_MODE_WRITE_BYTE
#define MMC_SWITCH_MODE_WRITE_BYTE  0x03
#define MMC_SWITCH_MODE_SET_BITS    0x01
#define MMC_SWITCH_MODE_CLEAR_BITS  0x02
#endif

#define EXT_CSD_CMD_SET_NORMAL  0

/* RPMB request/response types */
#define RPMB_REQ_KEY_WRITE       0x0001
#define RPMB_REQ_WRITE_CNT_READ  0x0002
#define RPMB_REQ_DATA_WRITE      0x0003
#define RPMB_REQ_DATA_READ       0x0004
#define RPMB_REQ_RESULT_READ     0x0005

/* ── Helper macro to fill mmc_ioc_cmd ───────────────────────── */
#define MMC_IOC_CMD_INIT(c, op, _arg, _flags, _blksz, _blocks, _write, _ptr) \
    do {                                          \
        memset(&(c), 0, sizeof(c));               \
        (c).opcode     = (op);                    \
        (c).arg        = (_arg);                  \
        (c).flags      = (_flags);                \
        (c).blksz      = (_blksz);                \
        (c).blocks     = (_blocks);               \
        (c).write_flag = (_write);                \
        (c).data_ptr   = (uint64_t)(uintptr_t)(_ptr); \
    } while (0)

/* ── Open / close ────────────────────────────────────────────── */
int emmc_open(void)
{
    int fd = open(EMMC_DEV, O_RDWR);
    if (fd < 0)
        fprintf(stderr, "[eMMC] open %s: %m\n", EMMC_DEV);
    return fd;
}

void emmc_close(int fd)
{
    if (fd >= 0) close(fd);
}

/* ── CID (CMD10 via ioctl) ───────────────────────────────────── */
int emmc_read_cid(int fd, uint8_t cid[16])
{
    struct mmc_ioc_cmd cmd;
    MMC_IOC_CMD_INIT(cmd, MMC_SEND_CID, 0,
                     MMC_RSP_R2 | MMC_CMD_BCR, 0, 0, 0, NULL);
    /* CID comes back in response[] */
    if (ioctl(fd, MMC_IOC_CMD, &cmd) < 0) {
        fprintf(stderr, "[eMMC] READ_CID: %m\n");
        return -1;
    }
    /* response[0..3] = CID[127:0] in 32-bit words, big-endian */
    for (int i = 0; i < 4; i++) {
        uint32_t w = __builtin_bswap32(cmd.response[i]);
        memcpy(cid + i * 4, &w, 4);
    }
    return 0;
}

/* ── CSD (CMD9) ──────────────────────────────────────────────── */
int emmc_read_csd(int fd, uint8_t csd[16])
{
    struct mmc_ioc_cmd cmd;
    MMC_IOC_CMD_INIT(cmd, MMC_SEND_CSD, 0,
                     MMC_RSP_R2 | MMC_CMD_BCR, 0, 0, 0, NULL);
    if (ioctl(fd, MMC_IOC_CMD, &cmd) < 0) {
        fprintf(stderr, "[eMMC] READ_CSD: %m\n");
        return -1;
    }
    for (int i = 0; i < 4; i++) {
        uint32_t w = __builtin_bswap32(cmd.response[i]);
        memcpy(csd + i * 4, &w, 4);
    }
    return 0;
}

/* ── EXT_CSD (CMD8) ──────────────────────────────────────────── */
int emmc_read_ext_csd(int fd, uint8_t ext_csd[512])
{
    struct mmc_ioc_cmd cmd;
    MMC_IOC_CMD_INIT(cmd, MMC_SEND_EXT_CSD, 0,
                     MMC_RSP_R1 | MMC_CMD_ADTC,
                     512, 1, 0, ext_csd);
    if (ioctl(fd, MMC_IOC_CMD, &cmd) < 0) {
        fprintf(stderr, "[eMMC] READ_EXT_CSD: %m\n");
        return -1;
    }
    return 0;
}

/* ── CMD6 SWITCH ─────────────────────────────────────────────── */
int emmc_switch(int fd, uint8_t index, uint8_t value, uint8_t cmd_set)
{
    struct mmc_ioc_cmd cmd;
    uint32_t arg = ((uint32_t)MMC_SWITCH_MODE_WRITE_BYTE << 24) |
                   ((uint32_t)index << 16) |
                   ((uint32_t)value << 8)  |
                   cmd_set;
    MMC_IOC_CMD_INIT(cmd, MMC_SWITCH, arg,
                     MMC_RSP_R1B | MMC_CMD_AC, 0, 0, 0, NULL);
    if (ioctl(fd, MMC_IOC_CMD, &cmd) < 0) {
        fprintf(stderr, "[eMMC] SWITCH idx=%u val=0x%02x: %m\n", index, value);
        return -1;
    }
    return 0;
}

/* ── Select hardware partition ───────────────────────────────── */
int emmc_select_partition(int fd, uint8_t partition)
{
    /* EXT_CSD[179] PARTITION_CONFIG bits [2:0] = PARTITION_ACCESS */
    uint8_t ext_csd[512];
    if (emmc_read_ext_csd(fd, ext_csd) < 0) return -1;

    uint8_t part_config = ext_csd[EXT_CSD_PART_CONFIG];
    part_config = (part_config & 0xF8) | (partition & 0x07);
    return emmc_switch(fd, EXT_CSD_PART_CONFIG, part_config,
                       EXT_CSD_CMD_SET_NORMAL);
}

/* ── Sector read (CMD18) ─────────────────────────────────────── */
int emmc_read_sectors(int fd, uint64_t lba, uint32_t count, uint8_t *buf)
{
    struct mmc_ioc_cmd cmd;
    MMC_IOC_CMD_INIT(cmd, MMC_READ_MULTIPLE_BLOCK, (uint32_t)lba,
                     MMC_RSP_R1 | MMC_CMD_ADTC,
                     512, count, 0, buf);
    cmd.is_acmd = 0;
    if (ioctl(fd, MMC_IOC_CMD, &cmd) < 0) {
        fprintf(stderr, "[eMMC] READ lba=%llu cnt=%u: %m\n",
                (unsigned long long)lba, count);
        return -1;
    }
    return 0;
}

/* ── Sector write (CMD25) ────────────────────────────────────── */
int emmc_write_sectors(int fd, uint64_t lba, uint32_t count,
                       const uint8_t *buf)
{
    struct mmc_ioc_cmd cmd;
    MMC_IOC_CMD_INIT(cmd, MMC_WRITE_MULTIPLE_BLOCK, (uint32_t)lba,
                     MMC_RSP_R1 | MMC_CMD_ADTC,
                     512, count, 1, (void *)buf);
    if (ioctl(fd, MMC_IOC_CMD, &cmd) < 0) {
        fprintf(stderr, "[eMMC] WRITE lba=%llu cnt=%u: %m\n",
                (unsigned long long)lba, count);
        return -1;
    }
    return 0;
}

/* ── Erase (CMD35 / CMD36 / CMD38) ──────────────────────────── */
int emmc_erase(int fd, uint64_t start_lba, uint64_t end_lba, uint32_t arg)
{
    struct mmc_ioc_multi_cmd *mc;
    mc = calloc(1, sizeof(*mc) + 3 * sizeof(struct mmc_ioc_cmd));
    if (!mc) return -1;
    mc->num_of_cmds = 3;

    /* CMD35 — ERASE_GROUP_START */
    MMC_IOC_CMD_INIT(mc->cmds[0], MMC_ERASE_GROUP_START, (uint32_t)start_lba,
                     MMC_RSP_R1 | MMC_CMD_AC, 0, 0, 0, NULL);
    /* CMD36 — ERASE_GROUP_END */
    MMC_IOC_CMD_INIT(mc->cmds[1], MMC_ERASE_GROUP_END, (uint32_t)end_lba,
                     MMC_RSP_R1 | MMC_CMD_AC, 0, 0, 0, NULL);
    /* CMD38 — ERASE */
    MMC_IOC_CMD_INIT(mc->cmds[2], MMC_ERASE, arg,
                     MMC_RSP_R1B | MMC_CMD_AC, 0, 0, 0, NULL);

    int rc = ioctl(fd, MMC_IOC_MULTI_CMD, mc);
    free(mc);
    if (rc < 0) {
        fprintf(stderr, "[eMMC] ERASE [%llu-%llu] arg=0x%x: %m\n",
                (unsigned long long)start_lba,
                (unsigned long long)end_lba, arg);
        return -1;
    }
    return 0;
}

/* ── Write protect (CMD28 / CMD29) ──────────────────────────── */
int emmc_set_write_protect(int fd, uint64_t lba, uint8_t type, int set)
{
    /* type: 0=temporary, 1=power-on, 2=permanent
     * For temp/permanent: CMD28(set) / CMD29(clear)
     * For power-on: set via EXT_CSD[173] BOOT_WP
     */
    if (type == 0 || type == 2) {
        struct mmc_ioc_cmd cmd;
        int opcode = set ? MMC_SET_WRITE_PROT : MMC_CLR_WRITE_PROT;
        MMC_IOC_CMD_INIT(cmd, opcode, (uint32_t)lba,
                         MMC_RSP_R1B | MMC_CMD_AC, 0, 0, 0, NULL);
        if (ioctl(fd, MMC_IOC_CMD, &cmd) < 0) {
            fprintf(stderr, "[eMMC] WRITE_PROT: %m\n");
            return -1;
        }
        if (type == 2 && set) {
            /* Permanent WP: also set EXT_CSD[171] USER_WP[2] = 1 */
            return emmc_switch(fd, EXT_CSD_USER_WP, 0x04,
                               EXT_CSD_CMD_SET_NORMAL);
        }
    } else if (type == 1) {
        /* Power-on write protect via BOOT_WP */
        uint8_t val = set ? 0x01 : 0x00;
        return emmc_switch(fd, EXT_CSD_BOOT_WP, val,
                           EXT_CSD_CMD_SET_NORMAL);
    }
    return 0;
}

/* ── Send write protect type (CMD31) ─────────────────────────── */
int emmc_send_write_prot(int fd, uint64_t lba, uint8_t prot[8])
{
    struct mmc_ioc_cmd cmd;
    MMC_IOC_CMD_INIT(cmd, MMC_SEND_WRITE_PROT_TYPE, (uint32_t)lba,
                     MMC_RSP_R1 | MMC_CMD_ADTC,
                     8, 1, 0, prot);
    if (ioctl(fd, MMC_IOC_CMD, &cmd) < 0) {
        fprintf(stderr, "[eMMC] SEND_WRITE_PROT: %m\n");
        return -1;
    }
    return 0;
}

/* ── Boot config (EXT_CSD[179] + EXT_CSD[177]) ───────────────── */
int emmc_set_boot_config(int fd, uint8_t boot_config, uint8_t boot_bus_cond)
{
    if (emmc_switch(fd, EXT_CSD_PART_CONFIG, boot_config,
                    EXT_CSD_CMD_SET_NORMAL) < 0)
        return -1;
    if (emmc_switch(fd, EXT_CSD_BOOT_BUS_CONDITIONS, boot_bus_cond,
                    EXT_CSD_CMD_SET_NORMAL) < 0)
        return -1;
    return 0;
}

/* ── Sleep / Awake (CMD5) ────────────────────────────────────── */
int emmc_sleep_awake(int fd, int sleep)
{
    struct mmc_ioc_cmd cmd;
    /* CMD5 arg: bit 15=1 for sleep, 0 for awake; bits [31:16]=RCA */
    uint32_t arg = sleep ? (1u << 15) : 0;
    MMC_IOC_CMD_INIT(cmd, MMC_SLEEP_AWAKE, arg,
                     MMC_RSP_R1B | MMC_CMD_AC, 0, 0, 0, NULL);
    if (ioctl(fd, MMC_IOC_CMD, &cmd) < 0) {
        fprintf(stderr, "[eMMC] SLEEP_AWAKE: %m\n");
        return -1;
    }
    return 0;
}

/* ── RPMB operations ─────────────────────────────────────────── */
/*
 * RPMB uses a dedicated partition accessed via /dev/mmcblk0rpmb.
 * Each RPMB frame is exactly 512 bytes.
 * Authentication uses HMAC-SHA256 with a 32-byte key.
 *
 * The kernel provides MMC_IOC_MULTI_CMD for atomic RPMB sequences:
 *   Write:  CMD23(reliable) + CMD25 + CMD18(result)
 *   Read:   CMD23 + CMD25(nonce frame) + CMD18(data)
 *   WrCnt:  CMD23 + CMD25(nonce frame) + CMD18(counter frame)
 */
int emmc_rpmb_op(int fd, uint8_t opcode, const struct rpmb_req *req,
                 uint8_t frame[512])
{
    int rpmb_fd = open(EMMC_RPMB_DEV, O_RDWR);
    if (rpmb_fd < 0) {
        fprintf(stderr, "[eMMC] open RPMB: %m\n");
        return -1;
    }

    struct mmc_ioc_multi_cmd *mc;
    mc = calloc(1, sizeof(*mc) + 3 * sizeof(struct mmc_ioc_cmd));
    if (!mc) { close(rpmb_fd); return -1; }

    uint8_t result_frame[512] = {0};

    switch (opcode) {
    case CMD_EMMC_RPMB_GET_WR_CNT:
    case CMD_EMMC_RPMB_READ: {
        /* Step 1: send read request frame (nonce + address) */
        mc->num_of_cmds = 3;
        /* CMD23: set block count = 1, reliable write NOT set */
        MMC_IOC_CMD_INIT(mc->cmds[0], 23, 1,
                         MMC_RSP_R1 | MMC_CMD_AC, 0, 0, 0, NULL);
        /* CMD25: write request frame to RPMB */
        MMC_IOC_CMD_INIT(mc->cmds[1], MMC_WRITE_MULTIPLE_BLOCK, 0,
                         MMC_RSP_R1 | MMC_CMD_ADTC,
                         512, 1, 1, frame);
        /* CMD18: read response frame */
        MMC_IOC_CMD_INIT(mc->cmds[2], MMC_READ_MULTIPLE_BLOCK, 0,
                         MMC_RSP_R1 | MMC_CMD_ADTC,
                         512, 1, 0, frame);
        break;
    }

    case CMD_EMMC_RPMB_WRITE: {
        /* Step 1: authenticated write, Step 2: read result */
        mc->num_of_cmds = 3;
        /* CMD23: reliable write flag (bit 31) + block count */
        uint32_t cnt = le16toh(req->block_count);
        if (cnt == 0) cnt = 1;
        MMC_IOC_CMD_INIT(mc->cmds[0], 23, (1u << 31) | cnt,
                         MMC_RSP_R1 | MMC_CMD_AC, 0, 0, 0, NULL);
        /* CMD25: write data frame(s) */
        MMC_IOC_CMD_INIT(mc->cmds[1], MMC_WRITE_MULTIPLE_BLOCK, 0,
                         MMC_RSP_R1 | MMC_CMD_ADTC,
                         512, cnt, 1, frame);
        /* CMD23 + CMD18: read result frame */
        /* We use a 2-step approach for result: re-use cmds[2] */
        MMC_IOC_CMD_INIT(mc->cmds[2], 23, 1,
                         MMC_RSP_R1 | MMC_CMD_AC, 0, 0, 0, NULL);
        /* Need 4-cmd sequence for write; rebuild */
        free(mc);
        mc = calloc(1, sizeof(*mc) + 4 * sizeof(struct mmc_ioc_cmd));
        if (!mc) { close(rpmb_fd); return -1; }
        mc->num_of_cmds = 4;
        MMC_IOC_CMD_INIT(mc->cmds[0], 23, (1u << 31) | cnt,
                         MMC_RSP_R1 | MMC_CMD_AC, 0, 0, 0, NULL);
        MMC_IOC_CMD_INIT(mc->cmds[1], MMC_WRITE_MULTIPLE_BLOCK, 0,
                         MMC_RSP_R1 | MMC_CMD_ADTC,
                         512, cnt, 1, frame);
        MMC_IOC_CMD_INIT(mc->cmds[2], 23, 1,
                         MMC_RSP_R1 | MMC_CMD_AC, 0, 0, 0, NULL);
        MMC_IOC_CMD_INIT(mc->cmds[3], MMC_READ_MULTIPLE_BLOCK, 0,
                         MMC_RSP_R1 | MMC_CMD_ADTC,
                         512, 1, 0, result_frame);
        break;
    }

    default:
        free(mc);
        close(rpmb_fd);
        return -1;
    }

    int rc = ioctl(rpmb_fd, MMC_IOC_MULTI_CMD, mc);
    free(mc);
    close(rpmb_fd);

    if (rc < 0) {
        fprintf(stderr, "[eMMC] RPMB op 0x%02x: %m\n", opcode);
        return -1;
    }

    /* Copy result frame back for write ops */
    if (opcode == CMD_EMMC_RPMB_WRITE)
        memcpy(frame, result_frame, 512);

    return 0;
}

/* ── FFU — Firmware Update (JEDEC eMMC 5.0+) ────────────────── */
/*
 * FFU flow (JESD84-B50, section 6.6.22):
 *   1. CMD6: set MODE_CONFIG[0]=1 (FFU mode)
 *   2. CMD25 with arg=0xFFFFFFFA (FFU download address)
 *      Write firmware image in 512-byte chunks
 *   3. CMD6: set MODE_OPERATION_CODES=1 (install FFU)
 *   4. CMD6: MODE_CONFIG[0]=0 (normal mode)
 *   5. Read EXT_CSD[26] FFU_STATUS — must be 0
 */
int emmc_ffu_download(int fd, const uint8_t *fw_data, size_t fw_len)
{
    /* Verify FFU support */
    uint8_t ext_csd[512];
    if (emmc_read_ext_csd(fd, ext_csd) < 0) return -1;
    if (!(ext_csd[EXT_CSD_FW_CONFIG] & 0x01)) {
        fprintf(stderr, "[eMMC] FFU not supported (FW_CONFIG[0]=0)\n");
        return -1;
    }

    /* Enter FFU mode */
    if (emmc_switch(fd, EXT_CSD_MODE_CONFIG, 0x01,
                    EXT_CSD_CMD_SET_NORMAL) < 0) return -1;

    /* Write firmware in 512B sectors */
    uint32_t sector_count = (uint32_t)(fw_len / 512);
    struct mmc_ioc_cmd cmd;
    MMC_IOC_CMD_INIT(cmd, MMC_WRITE_MULTIPLE_BLOCK, 0xFFFFFFFA,
                     MMC_RSP_R1 | MMC_CMD_ADTC,
                     512, sector_count, 1, (void *)fw_data);
    if (ioctl(fd, MMC_IOC_CMD, &cmd) < 0) {
        fprintf(stderr, "[eMMC] FFU download: %m\n");
        emmc_switch(fd, EXT_CSD_MODE_CONFIG, 0x00, EXT_CSD_CMD_SET_NORMAL);
        return -1;
    }

    /* Exit FFU mode */
    emmc_switch(fd, EXT_CSD_MODE_CONFIG, 0x00, EXT_CSD_CMD_SET_NORMAL);
    printf("[eMMC] FFU download complete (%zu bytes)\n", fw_len);
    return 0;
}

int emmc_ffu_install(int fd)
{
    uint8_t ext_csd[512];

    /* Trigger install */
    if (emmc_switch(fd, EXT_CSD_MODE_OPERATION_CODES, 0x01,
                    EXT_CSD_CMD_SET_NORMAL) < 0) return -1;

    /* Read FFU_STATUS */
    if (emmc_read_ext_csd(fd, ext_csd) < 0) return -1;
    if (ext_csd[EXT_CSD_FFU_STATUS] != 0) {
        fprintf(stderr, "[eMMC] FFU install failed: status=0x%02x\n",
                ext_csd[EXT_CSD_FFU_STATUS]);
        return -1;
    }
    printf("[eMMC] FFU install succeeded\n");
    return 0;
}

/* ── Device info aggregate ───────────────────────────────────── */
int emmc_get_device_info(int fd, struct device_info *info)
{
    uint8_t ext_csd[512];

    info->chip_type = CHIP_EMMC;

    if (emmc_read_cid(fd, info->emmc_cid)   < 0) return -1;
    if (emmc_read_csd(fd, info->emmc_csd)   < 0) return -1;
    if (emmc_read_ext_csd(fd, ext_csd)      < 0) return -1;

    /* Copy interesting EXT_CSD slice [196–227]: capacity, HS timing, etc */
    memcpy(info->emmc_ext_csd_slice, ext_csd + 196, 32);

    /* Sector count from EXT_CSD[215:212] */
    uint32_t sc;
    memcpy(&sc, ext_csd + 212, 4);
    info->emmc_sector_count = le32toh(sc);
    info->emmc_sector_size  = 9; /* log2(512) */

    return 0;
}
