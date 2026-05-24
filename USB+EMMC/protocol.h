/* protocol.h — USB programmer wire protocol
 *
 * Shared between device (ffs_app) and future Windows host app.
 * All multi-byte fields are LITTLE-ENDIAN on the wire.
 *
 * Transfer flow:
 *   1. Host sends prog_cbw  (64 bytes, always OUT)
 *   2. Data phase           (direction set by cbw.flags, length = cbw.transfer_len)
 *   3. Device sends prog_csw (16 bytes, always IN)
 *
 * For commands with no data phase, transfer_len = 0.
 */

#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

/* ── Signatures ─────────────────────────────────────────────── */
#define PROG_CBW_SIG    0x50524F47u   /* "PROG" */
#define PROG_CSW_SIG    0x53544154u   /* "STAT" */

/* ── Chip type returned by CMD_GET_CHIP_TYPE ────────────────── */
#define CHIP_NONE       0x00
#define CHIP_EMMC       0x01
#define CHIP_UFS        0x02

/* ── CBW flags ──────────────────────────────────────────────── */
#define CBW_FLAG_IN     0x80   /* Device → Host (read)  */
#define CBW_FLAG_OUT    0x00   /* Host → Device (write) */

/* ── CSW status ─────────────────────────────────────────────── */
#define CSW_OK          0x00
#define CSW_FAIL        0x01
#define CSW_PHASE_ERR   0x02

/* ── Application error codes (csw.error_code) ───────────────── */
#define ERR_OK              0x00
#define ERR_UNKNOWN_CMD     0x01
#define ERR_BAD_CHIP_TYPE   0x02
#define ERR_NO_DEVICE       0x03
#define ERR_IOCTL_FAIL      0x04
#define ERR_BSG_FAIL        0x05
#define ERR_BAD_PARAM       0x06
#define ERR_TIMEOUT         0x07
#define ERR_RPMB_AUTH       0x08
#define ERR_FFU_FAIL        0x09
#define ERR_ALLOC           0x0A
#define ERR_IO              0x0B

/* ═══════════════════════════════════════════════════════════════
 * Command opcodes
 * ═══════════════════════════════════════════════════════════════ */

/* ── Session / common ───────────────────────────────────────── */
#define CMD_GET_FW_VERSION      0x01  /* IN  → fw_version_resp */
#define CMD_GET_CHIP_TYPE       0x02  /* IN  → uint8 CHIP_*    */
#define CMD_GET_DEVICE_INFO     0x03  /* IN  → device_info     */
#define CMD_BEGIN_SESSION       0x04  /* OUT ← begin_session_req; selects chip */
#define CMD_END_SESSION         0x05  /* no data                */
#define CMD_ABORT               0x06  /* no data                */
#define CMD_SET_SPEED           0x07  /* OUT ← set_speed_req   */

/* ── eMMC ───────────────────────────────────────────────────── */
#define CMD_EMMC_READ_CID       0x10  /* IN  → 16 bytes raw CID        */
#define CMD_EMMC_READ_CSD       0x11  /* IN  → 16 bytes raw CSD        */
#define CMD_EMMC_READ_EXT_CSD   0x12  /* IN  → 512 bytes EXT_CSD       */
#define CMD_EMMC_WRITE_EXT_CSD  0x13  /* OUT ← emmc_switch_req (CMD6)  */
#define CMD_EMMC_READ_SECTORS   0x14  /* IN  → sector data             */
#define CMD_EMMC_WRITE_SECTORS  0x15  /* OUT ← sector data             */
#define CMD_EMMC_ERASE          0x16  /* OUT ← erase_req (CMD35/36/38) */
#define CMD_EMMC_TRIM           0x17  /* OUT ← erase_req with TRIM arg */
#define CMD_EMMC_SECURE_ERASE   0x18  /* OUT ← erase_req               */
#define CMD_EMMC_SECURE_TRIM    0x19  /* OUT ← erase_req               */
#define CMD_EMMC_SET_WRITE_PROT 0x1A  /* OUT ← write_prot_req          */
#define CMD_EMMC_CLR_WRITE_PROT 0x1B
#define CMD_EMMC_SEND_WRITE_PROT 0x1C /* IN  → 8 bytes prot status      */
#define CMD_EMMC_BOOT_CONFIG    0x1D  /* OUT ← boot_cfg_req             */
#define CMD_EMMC_RPMB_READ      0x1E  /* OUT ← rpmb_req; IN → 512B frame */
#define CMD_EMMC_RPMB_WRITE     0x1F  /* OUT ← rpmb_req + 512B frame    */
#define CMD_EMMC_RPMB_GET_WR_CNT 0x20 /* IN  → rpmb_write_counter       */
#define CMD_EMMC_FFU_DOWNLOAD   0x21  /* OUT ← firmware image data      */
#define CMD_EMMC_FFU_INSTALL    0x22  /* no data                        */
#define CMD_EMMC_SLEEP          0x23  /* no data (CMD5)                 */
#define CMD_EMMC_AWAKE          0x24  /* no data (CMD5)                 */
#define CMD_EMMC_SELECT_PART    0x25  /* OUT ← uint8 partition (0-6)    */

/* ── UFS ────────────────────────────────────────────────────── */
#define CMD_UFS_READ_DESC       0x30  /* OUT ← ufs_desc_req; IN → desc data */
#define CMD_UFS_WRITE_DESC      0x31  /* OUT ← ufs_desc_req + data          */
#define CMD_UFS_READ_ATTR       0x32  /* OUT ← ufs_attr_req; IN → uint32    */
#define CMD_UFS_WRITE_ATTR      0x33  /* OUT ← ufs_attr_req (includes value)*/
#define CMD_UFS_READ_FLAG       0x34  /* OUT ← ufs_flag_req; IN → uint8     */
#define CMD_UFS_SET_FLAG        0x35  /* OUT ← ufs_flag_req                 */
#define CMD_UFS_CLEAR_FLAG      0x36
#define CMD_UFS_TOGGLE_FLAG     0x37
#define CMD_UFS_READ_SECTORS    0x38  /* IN  → sector data                  */
#define CMD_UFS_WRITE_SECTORS   0x39  /* OUT ← sector data                  */
#define CMD_UFS_UNMAP           0x3A  /* SCSI UNMAP (trim)                  */
#define CMD_UFS_PURGE           0x3B  /* set bPurgeEnable attr              */
#define CMD_UFS_FDEVICEINIT     0x3C  /* set fDeviceInit flag               */
#define CMD_UFS_PROVISION_LU    0x3D  /* OUT ← ufs_provision_req            */
#define CMD_UFS_RPMB_READ       0x3E  /* OUT ← rpmb_req; IN → 512B frame    */
#define CMD_UFS_RPMB_WRITE      0x3F  /* OUT ← rpmb_req + 512B frame        */
#define CMD_UFS_RPMB_GET_WR_CNT 0x40
#define CMD_UFS_FFU             0x41  /* OUT ← firmware image (WRITE BUFFER)*/
#define CMD_UFS_POWER_MODE      0x42  /* OUT ← uint8 mode                   */
#define CMD_UFS_NOP             0x43  /* NOP UPIU ping                      */
#define CMD_UFS_QUERY_RAW       0x44  /* OUT ← raw UPIU bytes; IN → response*/

/* ═══════════════════════════════════════════════════════════════
 * Wire structures
 * All __attribute__((packed)), all LE.
 * ═══════════════════════════════════════════════════════════════ */

/* ── Command Block Wrapper: Host → Device, always 64 bytes ──── */
struct prog_cbw {
    uint32_t signature;      /* PROG_CBW_SIG                          */
    uint32_t tag;            /* echoed in CSW                         */
    uint32_t transfer_len;   /* bytes in data phase (0 if none)       */
    uint8_t  flags;          /* CBW_FLAG_IN / CBW_FLAG_OUT            */
    uint8_t  opcode;         /* CMD_*                                 */
    uint8_t  chip;           /* CHIP_EMMC / CHIP_UFS (for data cmds)  */
    uint8_t  reserved;
    uint64_t lba;            /* sector address                        */
    uint32_t sector_count;   /* number of 512-byte sectors            */
    uint8_t  params[36];     /* opcode-specific parameters            */
} __attribute__((packed));   /* exactly 64 bytes */

/* ── Command Status Wrapper: Device → Host, always 16 bytes ─── */
struct prog_csw {
    uint32_t signature;      /* PROG_CSW_SIG  */
    uint32_t tag;            /* matches CBW   */
    uint32_t residue;        /* untransferred */
    uint8_t  status;         /* CSW_*         */
    uint8_t  error_code;     /* ERR_*         */
    uint8_t  reserved[2];
} __attribute__((packed));   /* exactly 16 bytes */

/* ── Inline param structs (overlaid on cbw.params[]) ────────── */

/* CMD_BEGIN_SESSION */
struct begin_session_req {
    uint8_t  chip_type;      /* CHIP_EMMC or CHIP_UFS */
    uint8_t  reserved[35];
} __attribute__((packed));

/* CMD_SET_SPEED */
struct set_speed_req {
    uint8_t  mode;           /* eMMC: 0=HS52, 1=HS200, 2=HS400
                              * UFS:  0=G1L1, 1=G2L1, 2=G3L2, 3=G4L2 */
    uint8_t  reserved[35];
} __attribute__((packed));

/* CMD_EMMC_WRITE_EXT_CSD — single CMD6 SWITCH */
struct emmc_switch_req {
    uint8_t  index;          /* EXT_CSD byte index (0–511)     */
    uint8_t  value;          /* new value                       */
    uint8_t  cmd_set;        /* 0 = CMD_SET_NORMAL              */
    uint8_t  reserved[33];
} __attribute__((packed));

/* CMD_EMMC_ERASE / TRIM / SECURE_* */
struct emmc_erase_req {
    uint64_t start_lba;
    uint64_t end_lba;
    uint8_t  arg;            /* 0=normal, 1=trim, 0x80000001=secure */
    uint8_t  reserved[15];
} __attribute__((packed));

/* CMD_EMMC_SET/CLR_WRITE_PROT */
struct write_prot_req {
    uint64_t lba;
    uint8_t  type;           /* 0=temp, 1=power-on, 2=permanent */
    uint8_t  reserved[27];
} __attribute__((packed));

/* CMD_EMMC_BOOT_CONFIG */
struct boot_cfg_req {
    uint8_t  boot_config;    /* EXT_CSD[179] value              */
    uint8_t  boot_bus_cond;  /* EXT_CSD[177] value              */
    uint8_t  reserved[34];
} __attribute__((packed));

/* CMD_EMMC_SELECT_PART */
struct emmc_select_part_req {
    uint8_t  partition;      /* 0=UDA, 1=boot1, 2=boot2, 3=RPMB */
    uint8_t  reserved[35];
} __attribute__((packed));

/* CMD_EMMC_RPMB_* / CMD_UFS_RPMB_* */
struct rpmb_req {
    uint16_t req_resp;       /* RPMB request/response type       */
    uint16_t block_count;    /* frames to read/write             */
    uint8_t  key[32];        /* HMAC-SHA256 auth key (write ops) */
    uint8_t  reserved[0];    /* params[] is 36 bytes total       */
} __attribute__((packed));

/* CMD_UFS_READ_DESC / CMD_UFS_WRITE_DESC */
struct ufs_desc_req {
    uint8_t  idn;            /* QUERY_DESC_IDN_*                 */
    uint8_t  index;
    uint8_t  selector;
    uint8_t  reserved[33];
} __attribute__((packed));

/* CMD_UFS_READ_ATTR / CMD_UFS_WRITE_ATTR */
struct ufs_attr_req {
    uint8_t  idn;            /* QUERY_ATTR_IDN_*                 */
    uint8_t  index;
    uint8_t  selector;
    uint8_t  reserved[1];
    uint32_t value;          /* used for WRITE_ATTR              */
    uint8_t  pad[28];
} __attribute__((packed));

/* CMD_UFS_READ_FLAG / SET / CLEAR / TOGGLE */
struct ufs_flag_req {
    uint8_t  idn;            /* QUERY_FLAG_IDN_*                 */
    uint8_t  index;
    uint8_t  selector;
    uint8_t  reserved[33];
} __attribute__((packed));

/* CMD_UFS_PROVISION_LU */
struct ufs_provision_req {
    uint8_t  lu_count;       /* number of LUs to configure       */
    uint8_t  config_desc[255]; /* raw Configuration Descriptor   */
    /* Note: full config desc is 144 bytes; rest of data phase
     * carries the complete blob if > 36 bytes needed            */
} __attribute__((packed));

/* CMD_UFS_POWER_MODE */
struct ufs_power_mode_req {
    uint8_t  mode;           /* 0=active, 1=sleep, 2=powerdown   */
    uint8_t  reserved[35];
} __attribute__((packed));

/* ── Response structs (IN data phase payloads) ──────────────── */

struct fw_version_resp {
    uint8_t  major;
    uint8_t  minor;
    uint8_t  patch;
    char     build_date[16];  /* "YYYY-MM-DD\0"  */
    char     git_hash[12];
} __attribute__((packed));

struct device_info {
    uint8_t  chip_type;       /* CHIP_*          */
    /* eMMC fields */
    uint8_t  emmc_cid[16];
    uint8_t  emmc_csd[16];
    uint8_t  emmc_ext_csd_slice[32]; /* bytes 196–227: capacity/timing */
    uint32_t emmc_sector_count;
    uint8_t  emmc_sector_size; /* always 9 = log2(512)             */
    /* UFS fields */
    uint8_t  ufs_manuf_id[2]; /* wManufacturerID                  */
    uint8_t  ufs_product_id[16];
    uint8_t  ufs_product_rev[4];
    uint8_t  ufs_serial[16];
    uint64_t ufs_capacity_sectors;
    uint8_t  ufs_spec_version[2];
    uint8_t  reserved[6];
} __attribute__((packed));

struct rpmb_write_counter {
    uint32_t counter;
    uint8_t  nonce[16];
    uint8_t  mac[32];
} __attribute__((packed));

#endif /* PROTOCOL_H */
