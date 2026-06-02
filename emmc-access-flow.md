# eMMC Access Flow — Orange Pi 4 Pro (Allwinner A733)

## Overview

```
USB Host (WinUSB)
    │
    ▼  CBW/CSW protocol (bulk endpoints)
ffs_app  (userspace daemon)
    │
    ▼  MMC_IOC_CMD / MMC_IOC_MULTI_CMD ioctls
Linux MMC block layer  (/dev/mmcblk0 or mmcblk1)
    │
    ▼  mmc_request()
Allwinner SMHC driver  (drivers/mmc/host/sunxi-mmc.c)
    │
    ▼  SMHC3 registers @ 0x04023000
eMMC device (JEDEC JESD84-B51)
```

---

## 1. USB Gadget Layer

### Initialization (`/etc/init.d/S99ffs`)

1. Mount configfs, create gadget `g1`
2. Set VID=`0x1209`, PID=`0x0001`, serial=`RYD76-001`
3. Create FunctionFS function `ffs.usb0`
4. Mount functionfs at `/dev/usb-ffs/bulk_ep`
5. Start `ffs_app` (background)
6. Bind UDC `6a00000.xhci2-controller` (DWC3 in peripheral mode)

### Wire Protocol (CBW/CSW, modelled after USB SCSI BOT)

All transfers are 64-byte command block + optional data + 16-byte status:

**CBW** (Host → Device, 64 bytes):

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 0 | 4 | signature | `0x50524F47` ("PROG") |
| 4 | 4 | tag | echoed in CSW |
| 8 | 4 | transfer_len | data phase length (0 = none) |
| 12 | 1 | flags | `0x80` = device-to-host (IN), `0x00` = host-to-device (OUT) |
| 13 | 1 | opcode | operation (listed below) |
| 14 | 1 | chip | `0x01` = eMMC, `0x02` = UFS |
| 15 | 1 | reserved | |
| 16 | 8 | lba | sector address (little-endian) |
| 24 | 4 | sector_count | number of 512-byte sectors |
| 28 | 36 | params | opcode-specific parameters |

**CSW** (Device → Host, 16 bytes):

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 0 | 4 | signature | `0x53544154` ("STAT") |
| 4 | 4 | tag | matches CBW |
| 8 | 4 | residue | untransferred bytes |
| 12 | 1 | status | `0x00`=OK, `0x01`=FAIL, `0x02`=PHASE_ERR |
| 13 | 1 | error_code | specific error code |

### Protocol Opcodes

| Opcode | Name | Description |
|--------|------|-------------|
| `0x01` | CMD_GET_FW_VERSION | Read ffs_app firmware version |
| `0x02` | CMD_GET_CHIP_TYPE | Read storage type (eMMC/UFS) |
| `0x03` | CMD_GET_DEVICE_INFO | Query device info |
| `0x04` | CMD_BEGIN_SESSION | Start session |
| `0x05` | CMD_END_SESSION | End session |
| `0x07` | CMD_SET_SPEED | Set operating speed |
| `0x10` | CMD_EMMC_READ_CID | Read CID register (16 bytes) |
| `0x11` | CMD_EMMC_READ_CSD | Read CSD register (16 bytes) |
| `0x12` | CMD_EMMC_READ_EXT_CSD | Read EXT_CSD (512 bytes) |
| `0x13` | CMD_EMMC_WRITE_EXT_CSD | Write EXT_CSD byte |
| `0x14` | CMD_EMMC_READ_SECTORS | Read N sectors |
| `0x15` | CMD_EMMC_WRITE_SECTORS | Write N sectors |
| `0x16` | CMD_EMMC_ERASE | Erase range (CMD35+CMD36+CMD38) |
| `0x25` | CMD_EMMC_SELECT_PART | Switch to boot partition / user area |

---

## 2. ffs_app Dispatch

```
ffs_app (usb_transport.c)
  epoll loop on bulk endpoints
    read(ep_out, &cbw, 64)        → receive CBW
    dispatch(&cbw)                 → switch(chip, opcode)
      ├─ CHIP_EMMC:
      │    emmc_read_cid()         → open("/dev/mmcblk0"), ioctl(MMC_IOC_CMD, CMD2)
      │    emmc_read_csd()         → open("/dev/mmcblk0"), ioctl(MMC_IOC_CMD, CMD9)
      │    emmc_read_ext_csd()     → open("/dev/mmcblk0"), ioctl(MMC_IOC_CMD, CMD8)
      │    emmc_write_ext_csd()    → open("/dev/mmcblk0"), ioctl(MMC_IOC_CMD, CMD6)
      │    emmc_read_sectors()     → open("/dev/mmcblk0"), pread()  (direct block read)
      │    emmc_write_sectors()    → open("/dev/mmcblk0"), pwrite() (direct block write)
      │    emmc_erase()            → open("/dev/mmcblk0"), ioctl(MMC_IOC_MULTI_CMD, CMD35+CMD36+CMD38)
      │    emmc_select_part()      → ioctl(MMC_IOC_CMD, CMD6, EXT_CSD[179]) or BLKRRPART
      └─ ...
    data_phase(ep_in, buf, len)   → write_all(ep_in, buf, len)
    send_csw(ep_in, tag, 0, 0, 0) → write(ep_in, &csw, 16)
```

### eMMC ioctl Structures

```c
#include <linux/mmc/ioctl.h>

/* Single command */
struct mmc_ioc_cmd {
    int          write_flag;      /* 0=read, 1=write; bit31=reliable write */
    int          is_acmd;         /* 1=precede with CMD55 (ACMD) */
    __u32        opcode;          /* JEDEC command number */
    __u32        arg;             /* command argument */
    __u32        response[4];     /* R1/R1B → response[0]; R2 → all 4 words */
    unsigned int flags;           /* MMC_RSP_* | MMC_CMD_* */
    unsigned int blksz;           /* data block size */
    unsigned int blocks;          /* number of blocks */
    unsigned int postsleep_min_us;
    unsigned int postsleep_max_us;
    unsigned int data_timeout_ns;
    unsigned int cmd_timeout_ms;
    __u64        data_ptr;        /* userspace buffer pointer */
};

/* Multi-command atomic sequence */
struct mmc_ioc_multi_cmd {
    __u64              num_of_cmds;   /* max 255 */
    struct mmc_ioc_cmd cmds[];        /* flexible array */
};

/* Ioctl numbers */
#define MMC_IOC_CMD       _IOWR(MMC_BLOCK_MAJOR, 0, struct mmc_ioc_cmd)
#define MMC_IOC_MULTI_CMD _IOWR(MMC_BLOCK_MAJOR, 1, struct mmc_ioc_multi_cmd)
```

### Response Type Flags

```c
#define MMC_RSP_PRESENT   (1 << 0)
#define MMC_RSP_136       (1 << 1)     /* 136-bit response (R2) */
#define MMC_RSP_CRC       (1 << 2)     /* expect valid CRC */
#define MMC_RSP_BUSY      (1 << 3)     /* card may busy (R1B) */
#define MMC_RSP_OPCODE    (1 << 4)     /* response contains opcode */

#define MMC_RSP_NONE      (0)
#define MMC_RSP_R1        (MMC_RSP_PRESENT | MMC_RSP_CRC | MMC_RSP_OPCODE)
#define MMC_RSP_R1B       (MMC_RSP_PRESENT | MMC_RSP_CRC | MMC_RSP_OPCODE | MMC_RSP_BUSY)
#define MMC_RSP_R2        (MMC_RSP_PRESENT | MMC_RSP_136 | MMC_RSP_CRC)
#define MMC_RSP_R3        (MMC_RSP_PRESENT)
#define MMC_RSP_R4        (MMC_RSP_PRESENT)

#define MMC_CMD_AC        (0)          /* no data */
#define MMC_CMD_ADTC      (1 << 5)     /* data transfer */
#define MMC_CMD_BC        (2 << 5)     /* broadcast */
#define MMC_CMD_BCR       (3 << 5)     /* broadcast with response */
```

---

## 3. eMMC SMHC3 — Hardware Connection

### DTS Node (SoC `sun60iw2p1.dtsi` line 4419)

```dts
sdc3: sdmmc@4023000 {
    compatible = "allwinner,sunxi-mmc-v5p6x";   /* v5p6x, NOT v4p6x */
    reg = <0x0 0x04023000 0x0 0x1000>;
    interrupts = <GIC_SPI 164 IRQ_TYPE_LEVEL_HIGH>;
    clocks = <&sys24M>,
             <&ccu CLK_PLL_PERI1_800M>,
             <&ccu CLK_PLL_PERI1_600M>,
             <&ccu CLK_SMHC3>,
             <&ccu CLK_BUS_SMHC3>,
             <&ccu CLK_MBUS_STORE_GATE>,
             <&ccu CLK_MSI_LITE1>;
    clock-names = "osc24m","pll_periph","pll_periph_2",
                  "mmc","ahb","mmc_mbus","mmc_msi_lite";
    resets = <&ccu RST_BUS_SMHC3>;
};
```

### DTS Override (board `sun60i-a733-orangepi-4-pro.dts` line 1460)

```dts
&sdc3 {
    non-removable;
    bus-width = <8>;
    mmc-ddr-1_8v;
    mmc-hs200-1_8v;
    mmc-hs400-1_8v;
    mmc-hs400-enhanced-strobe;
    no-sdio;
    no-sd;
    ctl-spec-caps = <0x328>;
    cap-mmc-highspeed;
    sunxi-power-save-mode;
    sunxi-dis-signal-vol-sw;
    mmc-bootpart-noacc;
    cqe-on;
    ctl-cmdq-md = <0x2>;
    max-frequency = <200000000>;
    status = "okay";
};
```

### Pin Configuration

| Pin Group | Pins | Function | Drive Strength | Bias |
|-----------|------|----------|---------------|------|
| `sdc3_pins_a` (active) | PK0 (clk), K3-K6 (cmd+data0-3), K8-K11 (data4-7) | `sdc3` | — | pull-up |
| `sdc3_pins_b` (sleep) | PK0,PK3-6,PK8-11 | `gpio_in` | — | — |
| `sdc3_pins_c` | PK1 (rst) | `sdc3` | — | pull-up |
| `sdc3_pins_d` | PK2 (data strobe) | `sdc3` | — | pull-up |

### Key Differences from SMHC2

| Property | SMHC2 (sdc2) | SMHC3 (sdc3) |
|----------|-------------|-------------|
| Base address | `0x04022000` | `0x04023000` |
| IRQ | GIC SPI 163 | GIC SPI 164 |
| Compatible | `sunxi-mmc-v4p6x` | `sunxi-mmc-v5p6x` |
| Clock gate | `CLK_SMHC2` | `CLK_SMHC3` |
| Bus gate | `CLK_BUS_SMHC2` | `CLK_BUS_SMHC3` |
| Reset | `RST_BUS_SMHC2` | `RST_BUS_SMHC3` |
| Store gate | `CLK_STORE_AHB_GATE` | (not present) |

---

## 4. eMMC Speed Modes — Clock, Bus Width, and Voltage

### SMHC3 Clock Tree

```
PLL_PERI1_800M                   800 MHz fixed
    │
    ▼  CCU divider (configurable)
CLK_SMHC3                        52–200 MHz (clk_set_rate)
    │
    ▼  SMHC internal divider (REG_CLKCR[7:0])
Card clock                        400 kHz – 200 MHz
    │
    ▼  DDR mode doubles internal clock
SDXC_2X_TIMING_MODE (REG_SD_NTSR)
```

- DDR modes (DDR52, HS400) set `div=2` in `sunxi_mmc_clk_set_rate()` (line 790–791), doubling the module clock before the internal divider.
- The internal divider (`REG_CLKCR[7:0]`) is always set to `div-1`.
- `clk_round_rate()` and `clk_set_rate()` on `CLK_SMHC3` find the nearest rate from PLL_PERI1_800M.
- Non-DDR modes run the module clock at the card clock frequency (no doubling).

### Supported Speed Modes

| Mode | Timing Macro | Max Freq | Bus Width | Volt | DDR | Register DDR bit | Notes |
|------|-------------|----------|-----------|------|-----|-----------------|-------|
| DS26 (Legacy) | `MMC_TIMING_LEGACY` | 26 MHz | 1, 4, 8 | 3.3/1.8V | No | `GCTRL[10]=0` | CMD0–CMD3 init |
| HS-SDR (SDR25) | `MMC_TIMING_MMC_HS` | 52 MHz | 4, 8 | 3.3/1.8V | No | `GCTRL[10]=0` | CMD6 EXT_CSD[183]=1 |
| DDR52 | `MMC_TIMING_MMC_DDR52` | 52 MHz | 4, 8 | 1.8V | **Yes** | `GCTRL[10]=1` | CMD6 EXT_CSD[185]=5/6 |
| HS200 | `MMC_TIMING_MMC_HS200` | 200 MHz | 4, 8 | 1.8V | No | `GCTRL[10]=0` | CMD6 EXT_CSD[183]=0x10, CMD21 tune |
| HS400 | `MMC_TIMING_MMC_HS400` | 200 MHz | **8** | 1.8V | **Yes** | `GCTRL[10]=1` | CMD6 EXT_CSD[183]=0x30, DS strobe |

### Frequency Points

| Point | Frequency | Divider from 800M | PLL source | Used by |
|-------|-----------|------------------|------------|---------|
| f0 | 400 kHz | 2000 | PLL_PERI1_800M | Card init |
| f1 | 25 MHz | 32 | PLL_PERI1_800M | DS26 |
| f2 | 50 MHz | 16 | PLL_PERI1_800M | HS-SDR, DDR52 |
| f3 | 100 MHz | 8 | PLL_PERI1_800M | Transition |
| f4 | 150 MHz | 5.33 (non-int) | PLL_PERI1_600M | Transition |
| f5 | 200 MHz | 4 | PLL_PERI1_800M | HS200, HS400 |

### Per-Mode Register Settings

#### DS26 / Legacy (26 MHz, 1.8V or 3.3V)

```
CLK_SMHC3 = 26 MHz          clk_set_rate(26000000)
REG_CLKCR = 0x00            divider = 1 ÷ (0+1) = 1×
REG_WIDTH = 0x00/02/03      1/4/8 bit
GCTRL[10] = 0               DDR OFF
REG_SD_NTSR[0] = 1          2× timing mode ON
REG_SAMP_DL_REG = calibrate
REG_DS_DL_REG = unused
```

#### HS-SDR (52 MHz, 1.8V or 3.3V)

```
CLK_SMHC3 = 52 MHz          clk_set_rate(52000000)
REG_CLKCR = 0x00            divider = 1
REG_WIDTH = 0x02/03         4/8 bit
GCTRL[10] = 0               DDR OFF
REG_SD_NTSR[0] = 1
REG_SAMP_DL_REG = calibrate
```

#### DDR52 (52 MHz DDR, 1.8V only)

```
CLK_SMHC3 = 104 MHz         clk_set_rate(104000000)  (doubled for DDR)
REG_CLKCR = 0x01            divider = 2, card = 104/2 = 52 MHz DDR
REG_WIDTH = 0x02/03         4/8 bit
GCTRL[10] = 1               DDR ON
REG_SD_NTSR[0] = 1
REG_SAMP_DL_REG = calibrate
```

8-bit DDR52 uses timing table `SDXC_CLK_50M_DDR_8BIT` (output=90, sample=180 on eMMC ctl).

#### HS200 (200 MHz SDR, 1.8V only)

```
CLK_SMHC3 = 200 MHz         clk_set_rate(200000000)
REG_CLKCR = 0x00            divider = 1
REG_WIDTH = 0x02/03         4/8 bit
GCTRL[10] = 0               DDR OFF
REG_SD_NTSR[0] = 1
REG_SAMP_DL_REG = tune via CMD21    ← HS200 tuning required
```

#### HS400 (200 MHz DDR + Data Strobe, 1.8V only, 8-bit mandatory)

```
CLK_SMHC3 = 400 MHz         clk_set_rate(400000000)  (doubled for DDR)
REG_CLKCR = 0x01            divider = 2, card = 400/2 = 200 MHz DDR
REG_WIDTH = 0x03            8 bit ONLY
GCTRL[10] = 1               DDR ON
REG_SD_NTSR[0] = 1
REG_SAMP_DL_REG = calibrate for switch
REG_DS_DL_REG  = 0x16       data strobe delay (sunxi-ds-dly)
```

**HS400 switch sequence (JEDEC):**
1. Switch to HS200: `CMD6(EXT_CSD[183] = 0x10)`
2. Tune HS200: `CMD21` (SEND_TUNING_BLOCK)
3. Switch to HS400: `CMD6(EXT_CSD[183] = 0x30)`
4. Enable Enhanced Strobe: `CMD6(EXT_CSD[181] = 0x01)`

**NOTE:** The SMHC driver masks out `MMC_CAP2_HS400` (`sunxi-mmc.c:1461`, line: `mmc->caps2 &= ~MMC_CAP2_HS400`). HS400 is a known TODO. The DTS declares it but the driver does not yet implement it.

### CQE (Command Queue Engine)

```dts
cqe-on;              /* enabled */
ctl-cmdq-md = <0x2>; /* queue depth = 2 */
```

When CQE is enabled, the eMMC's queue interface (CQHCI) is used for I/O. Commands are submitted through the CQHCI register block rather than the legacy `SDXC_REG_CMDR`. The device must have `QUEUE_EN` set in `EXT_CSD[15]`.

---

## 5. SMHC3 Host Controller Registers

Base: `0x04023000`

| Offset | Register | Description |
|--------|----------|-------------|
| 0x00 | `GCTRL` | Global control. `bit10=1` → DDR mode |
| 0x04 | `CLKCR` | Clock control. `bits[7:0]` = divider - 1 |
| 0x08 | `TMOUT` | Timeout control |
| 0x0C | `WIDTH` | Bus width: `0x00`=1bit, `0x02`=4bit, `0x03`=8bit |
| 0x10 | `BLKSZ` | Block size |
| 0x14 | `BCNTR` | Byte count |
| 0x18 | `CMDR` | Command: opcode in bits[13:8], flags in bits[21:16] |
| 0x1C | `CARG` | Command argument |
| 0x20 | `RESPR0` | Response bits 31:0 |
| 0x24 | `RESPR1` | Response bits 63:32 (R2 high word) |
| 0x28 | `RESPR2` | Response bits 95:64 (R2) |
| 0x2C | `RESPR3` | Response bits 127:96 (R2) |
| 0x30 | `IMASK` | Interrupt mask |
| 0x34 | `MISTA` | Masked interrupt status |
| 0x38 | `RINT` | Raw interrupt |
| 0x3C | `STS` | Status |
| 0x4C | `FIFO` | Data FIFO (read/write 32-bit) |
| 0x5C | `SD_NTSR` | New timing set: `bit0=1` → 2× timing mode |
| 0x60 | `SAMP_DL_REG` | Sample delay calibration |
| 0x64 | `DS_DL_REG` | Data strobe delay (HS400 only) |
| 0xD0 | `A12` | Auto CMD12 |

### Clock Divider Calculation

```
target  = requested card clock
div     = 1 for SDR, 2 for DDR
module  = target × div              (DDR: 400 MHz for 200 MHz card)
rate    = clk_set_rate(SMHC3, module)
actual  = rate / div                (200 MHz = 400/2)
REG_CLKCR &= ~0xff
REG_CLKCR |= (div - 1)              (0 for SDR, 1 for DDR)
```

Example — HS400 (200 MHz DDR):
```
target = 200000000
div    = 2
module = 400000000
rate   = clk_set_rate(CLK_SMHC3, 400000000)  → ~400 MHz from PLL_PERI1_800M/2
actual = 400 / 2 = 200 MHz
REG_CLKCR = 0x01
```

---

## 6. eMMC Key JEDEC Commands

| CMD | Name | Arg | Resp | Data | Description |
|-----|------|-----|------|------|-------------|
| 0 | GO_IDLE_STATE | 0 | None | — | Software reset |
| 1 | SEND_OP_COND | OCR | R3 | — | Init/voltage negotiation |
| 2 | ALL_SEND_CID | 0 | R2 | — | Read CID (128-bit) |
| 3 | SET_RELATIVE_ADDR | RCA | R1 | — | Set RCA (eMMC: CMD3 used for sleep/awake too) |
| 6 | SWITCH | `(3<<24)\|idx<<16\|val<<8` | R1B | — | Write EXT_CSD byte |
| 7 | SELECT_DESELECT | RCA | R1 | — | Select card |
| 8 | SEND_EXT_CSD | 0 | R1 | 512B read | Read EXT_CSD |
| 9 | SEND_CSD | RCA | R2 | — | Read CSD (128-bit) |
| 12 | STOP_TRANSMISSION | 0 | R1B | — | Abort multi-block |
| 13 | SEND_STATUS | RCA | R1 | — | Read card status |
| 16 | SET_BLOCKLEN | len | R1 | — | Set block length |
| 17 | READ_SINGLE_BLOCK | addr | R1 | 1 block | Single block read |
| 18 | READ_MULTIPLE_BLOCK | addr | R1 | N blocks | Multi-block read |
| 21 | SEND_TUNING_BLOCK | 0 | R1 | 1 block | HS200 tuning pattern |
| 23 | SET_BLOCK_COUNT | count | R1 | — | Pre-set block count (for CMD18/25) |
| 24 | WRITE_BLOCK | addr | R1 | 1 block | Single block write |
| 25 | WRITE_MULTIPLE_BLOCK | addr | R1 | N blocks | Multi-block write |
| 35 | ERASE_GROUP_START | addr | R1 | — | Erase start address |
| 36 | ERASE_GROUP_END | addr | R1 | — | Erase end address |
| 38 | ERASE | 0=erase, 1=trim | R1B | — | Execute erase |

### ACMDs (precede with CMD55)

| ACMD | Name | Arg | Resp | Description |
|------|------|-----|------|-------------|
| 6 | SET_BUS_WIDTH | 0/1/2 | R1 | Bus width: 0=1bit, 1=4bit, 2=8bit |

### Setting Bus Width

eMMC bus width is set via **CMD6 (SWITCH) to EXT_CSD byte 185**:

```
CMD6  arg = (6 << 24) | (3 << 24) | (185 << 16) | (value << 8)
```

| Value | Bus Width | Mode | Voltage |
|-------|-----------|------|---------|
| `0x00` | 1-bit | SDR | 3.3V / 1.8V |
| `0x01` | 4-bit | SDR | 3.3V / 1.8V |
| `0x02` | 8-bit | SDR | 3.3V / 1.8V |
| `0x05` | 4-bit | DDR | 1.8V only |
| `0x06` | 8-bit | DDR | 1.8V only |

After CMD6, the kernel also writes the corresponding `REG_WIDTH` register:
- `MMC_BUS_WIDTH_1` → `REG_WIDTH = 0x00`
- `MMC_BUS_WIDTH_4` → `REG_WIDTH = 0x02`
- `MMC_BUS_WIDTH_8` → `REG_WIDTH = 0x03`

Note: ACMD6 (SET_BUS_WIDTH) is for SD cards. eMMC always uses CMD6 to EXT_CSD[185].

### CMD6 SWITCH Argument Encoding

```
 31  30  28  27  26   24  23   16  15     8  7      0
┌────┬───────┬────────┬────────┬──────────┬──────────┐
│ cmd6 │       │ access │ index  │  value   │  unused  │
│  6   │       │ [26:24]│ [23:16]│  [15:8]  │          │
└────┴───────┴────────┴────────┴──────────┴──────────┘
```

- `access = 3` → Write
- `access = 1` → Set bits
- `access = 2` → Clear bits

Example: switch to HS200:
```
arg = (6 << 24) | (3 << 24) | (183 << 16) | (0x10 << 8)
    = 0x0C37B100
```

### EXT_CSD Byte Indexes

| Index | Name | Usage | Values |
|-------|------|-------|--------|
| 15 | QUEUE_EN | Enable CQ | `0x00`=off, `0x01`=on |
| 179 | PARTITION_CONFIG | Partition switch | `boot_part<<3` \| `boot_ack` \| `part_access` |
| 181 | ENH_STROBE | Enhanced Strobe (HS400) | `0x01`=enable |
| 183 | HS_TIMING | Timing interface | `0x00`=DS, `0x01`=HS, `0x10`=HS200, `0x30`=HS400 |
| 185 | BUS_WIDTH | Bus width + mode | `0x00`=1bit, `0x01`=4bitSDR, `0x02`=8bitSDR, `0x05`=4bitDDR, `0x06`=8bitDDR |
| 214 | BOOT_BUS_CONDITIONS | Boot bus config | `boot_bus_width<<3\|boot_mode<<1\|reset` |

---

## 7. Complete Initialization Sequence

### At kernel boot (mmc core + sunxi-mmc driver on SMHC3):

```
Phase 1 — Card Detection & Legacy Init

  CLK_SMHC3 = 400 kHz, 1-bit
  CMD0   (GO_IDLE_STATE)
  CMD1   (SEND_OP_COND, negotiate 3.3V/1.8V)
  CMD2   (ALL_SEND_CID)        → 128-bit CID stored in mmc->cid
  CMD3   (SET_RELATIVE_ADDR)   → assign RCA = 1
  CMD9   (SEND_CSD)            → read CSD (capacity, block size, etc.)
  CMD7   (SELECT, RCA=1)       → card selected

Phase 2 — High Speed Switch (if supported)

  CMD6   (EXT_CSD[185] = 0x02) → 8-bit bus width
  sunxi_mmc_set_bus_width(8)
  CMD6   (EXT_CSD[183] = 0x01) → HS-SDR timing
  CLK_SMHC3 = 52 MHz

Phase 3 — HS200 (if supported)

  CMD6   (EXT_CSD[183] = 0x10) → HS200 timing
  CLK_SMHC3 = 200 MHz
  CMD21  (SEND_TUNING_BLOCK)   → tune REG_SAMP_DL_REG
  ← HS200 ready

Phase 4 — HS400 (if supported, currently masked in driver)

  CMD6   (EXT_CSD[185] = 0x06) → 8-bit DDR
  CMD6   (EXT_CSD[183] = 0x30) → HS400 timing
  calibrate REG_DS_DL_REG
  CMD6   (EXT_CSD[181] = 0x01) → Enhanced Strobe
  ← HS400 ready

Phase 5 — Block Device Registration

  /dev/mmcblk0        → user area (if no SD card inserted)
  /dev/mmcblk1        → user area (if SD card = mmcblk0)
  /dev/mmcblk0boot0   → boot partition 1
  /dev/mmcblk0boot1   → boot partition 2
  /dev/mmcblk0rpmb    → RPMB
```

**Boot setup** (`extlinux.conf`):
```
LABEL orangepi4pro
KERNEL /boot/Image
FDT /boot/sun60i-a733-orangepi-4-pro.dtb
APPEND root=/dev/mmcblk1p1 rootwait console=ttyS0,115200
```

`root=/dev/mmcblk1p1` — eMMC is `mmcblk1` when SD card (`mmcblk0`) is present.

---

## 8. Sequence Diagram: Complete eMMC Access

### Example: Read CID Register

```
USB Host                    ffs_app                    Linux MMC Core            SMHC3 Driver          eMMC Device
  │                           │                           │                        │                      │
  │  CBW (CMD_EMMC_READ_CID)  │                           │                        │                      │
  │──────────────────────────►│                           │                        │                      │
  │                           │  open("/dev/mmcblk0")     │                        │                      │
  │                           │──────────────────────────►│                        │                      │
  │                           │  ioctl(MMC_IOC_CMD)       │                        │                      │
  │                           │──────────────────────────►│                        │                      │
  │                           │  cmd.opcode = 2 (CID)     │                        │                      │
  │                           │  cmd.flags  = R2|BCR      │                        │                      │
  │                           │  cmd.blocks = 0           │                        │                      │
  │                           │                           │  mmc_wait_for_cmd()    │                      │
  │                           │                           │───────────────────────►│                      │
  │                           │                           │                        │  writel(CARG, 0)     │
  │                           │                           │                        │  writel(CMDR, CMD2)  │
  │                           │                           │                        │─────────────────────►│
  │                           │                           │                        │  CMD2 (ALL_SEND_CID) │
  │                           │                           │                        │◄─────────────────────│
  │                           │                           │                        │  readl(RESP0-3)      │
  │                           │                           │◄───────────────────────│  ← 128-bit CID       │
  │                           │◄──────────────────────────│  response[4]           │                      │
  │                           │  close(fd)                │                        │                      │
  │  Data (16 bytes CID)      │                           │                        │                      │
  │◄──────────────────────────│                           │                        │                      │
  │  CSW (status=OK)          │                           │                        │                      │
  │◄──────────────────────────│                           │                        │                      │
```

### Example: EXT_CSD Write (Switch Mode to HS200)

```
USB Host                    ffs_app                    eMMC Device
  │                           │                           │
  │  CBW (CMD_EMMC_WRITE_EXT_CSD, index=183, value=0x10)
  │──────────────────────────►│                           │
  │                           │  ioctl(MMC_IOC_CMD)       │
  │                           │  opcode = 6              │
  │                           │  arg = (3<<24)|(183<<16)|(0x10<<8)
  │                           │  flags = R1B|AC          │
  │                           │──────────────────────────►│
  │                           │  CMD6 (SWITCH)           │
  │                           │◄──────────────────────────│
  │                           │  kernel re-inits SMHC3   │
  │                           │  clk_set_rate(200 MHz)   │
  │                           │  REG_SAMP_DL tuning      │
  │  CSW (status=OK)          │                           │
  │◄──────────────────────────│                           │
```

### Example: Read 32 Sectors

```
USB Host                    ffs_app                    eMMC Device
  │                           │                           │
  │  CBW (CMD_EMMC_READ_SECTORS, lba=1000, count=32)
  │──────────────────────────►│                           │
  │                           │  open("/dev/mmcblk0")     │
  │                           │  pread(fd, buf, 16384, 512000)
  │                           │──────────────────────────►│
  │                           │  CMD23 (SET_BLOCK_COUNT=32)
  │                           │  CMD18 (READ_MULTIPLE, addr=1000)
  │                           │  ◄─── 32 blocks ─────────│
  │  Data (16384 bytes)       │                           │
  │◄──────────────────────────│                           │
  │  CSW (status=OK)          │                           │
  │◄──────────────────────────│                           │
```

---

## 9. ffs_app USB Protocol — C Source Examples

### Host: Send CBW + Read CID

```c
struct prog_cbw cbw = {
    .signature     = 0x50524F47,           /* "PROG" */
    .tag           = 1,
    .transfer_len  = 16,                   /* CID is 16 bytes */
    .flags         = 0x80,                 /* IN (device→host) */
    .opcode        = 0x10,                 /* CMD_EMMC_READ_CID */
    .chip          = 0x01,                 /* eMMC */
};
write(bulk_out_ep, &cbw, sizeof(cbw));

uint8_t cid[16];
read(bulk_in_ep, cid, 16);

struct prog_csw csw;
read(bulk_in_ep, &csw, sizeof(csw));
assert(csw.status == 0);
```

### Device (ffs_app): Read EXT_CSD

```c
static int emmc_read_ext_csd(uint8_t *ext_csd)
{
    int fd = open("/dev/mmcblk0", O_RDWR);
    struct mmc_ioc_cmd cmd = {
        .opcode     = 8,                    /* MMC_SEND_EXT_CSD */
        .arg        = 0,
        .flags      = MMC_RSP_R1 | MMC_CMD_ADTC,
        .blksz      = 512,
        .blocks     = 1,
        .write_flag = 0,                    /* read */
        .data_ptr   = (__u64)(uintptr_t)ext_csd,
    };
    int ret = ioctl(fd, MMC_IOC_CMD, &cmd);
    close(fd);
    return ret;
}
```

### Device: Write EXT_CSD (Switch Mode)

```c
static int emmc_write_ext_csd(uint8_t index, uint8_t value)
{
    int fd = open("/dev/mmcblk0", O_RDWR);
    struct mmc_ioc_cmd cmd = {
        .opcode     = 6,                    /* MMC_SWITCH */
        .arg        = (3 << 24) | (index << 16) | (value << 8),
        .flags      = MMC_RSP_R1B | MMC_CMD_AC,
        .write_flag = 1,
    };
    int ret = ioctl(fd, MMC_IOC_CMD, &cmd);
    close(fd);
    return ret;
}
```

### Device: Erase (Multi-Cmd)

```c
static int emmc_erase(uint32_t start_lba, uint32_t end_lba)
{
    int fd = open("/dev/mmcblk0", O_RDWR);
    struct mmc_ioc_multi_cmd *mc = calloc(1,
        sizeof(*mc) + 3 * sizeof(struct mmc_ioc_cmd));
    mc->num_of_cmds = 3;
    mc->cmds[0] = (struct mmc_ioc_cmd){
        .opcode = 35, .arg = start_lba,     /* ERASE_GROUP_START */
        .flags  = MMC_RSP_R1 | MMC_CMD_AC,
    };
    mc->cmds[1] = (struct mmc_ioc_cmd){
        .opcode = 36, .arg = end_lba,       /* ERASE_GROUP_END */
        .flags  = MMC_RSP_R1 | MMC_CMD_AC,
    };
    mc->cmds[2] = (struct mmc_ioc_cmd){
        .opcode = 38, .arg = 0,             /* ERASE */
        .flags  = MMC_RSP_R1B | MMC_CMD_AC,
    };
    int ret = ioctl(fd, MMC_IOC_MULTI_CMD, mc);
    free(mc);
    close(fd);
    return ret;
}
```

---

## 10. Key Source Files

| Component | Path |
|-----------|------|
| USB gadget init | `board/orangepi/orangepi4pro/rootfs-overlay/etc/init.d/S99ffs` |
| ffs_app main loop | `ffs_app/usb_transport.c` |
| ffs_app eMMC ops | `ffs_app/emmc_ops.c` |
| Wire protocol | `ffs_app/protocol.h` |
| SMHC driver | `drivers/mmc/host/sunxi-mmc.c` |
| MMC ioctl uAPI | `include/uapi/linux/mmc/ioctl.h` |
| JEDEC EXT_CSD defs | `include/linux/mmc/mmc.h` |
| Board DTS (sdc3) | `sun60i-a733-orangepi-4-pro.dts` (line 1460) |
| SoC DTSI (sdc3) | `sun60iw2p1.dtsi` (line 4419) |
| Boot config | `rootfs-overlay/boot/extlinux/extlinux.conf` |
