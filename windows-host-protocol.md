# Windows Host — RYD 76 Programmer Protocol

## 1  Hardware Connection

Connect the Orange Pi 4 Pro USB‑OTG port (USB‑C, bottom edge) directly to a
Windows USB 3.x host port with a USB‑C **or** USB‑C‑to‑A cable.

The device side is the dwc3 OTG controller (`6a00000.xhci2-controller`), driven
in **device/gadget** mode.  No external power or adapter is needed.

---

## 2  Device Enumeration

### 2.1  USB Descriptors

| Field              | Value        |
|--------------------|--------------|
| `idVendor`         | `0x1209`     |
| `idProduct`        | `0x0001`     |
| `bcdUSB`           | `0x0210`     |
| `bcdDevice`        | `0x0100`     |
| Class/Sub/Protocol | 0xEF / 0x02 / 0x01 (MISC) |
| Manufacturer       | `RYD PROG`   |
| Product            | `RYD 76`     |
| Serial             | `RYD76-001`  |

### 2.2  Endpoints

| Direction | Address | Type    | MaxPacket | Notes              |
|-----------|---------|---------|-----------|--------------------|
| IN        | 0x81    | Bulk    | 1024 (SS) | Data → Host        |
| OUT       | 0x01    | Bulk    | 1024 (SS) | Data → Device      |
| IN        | 0x82    | Interrupt | 1024    | Status (unused)    |

### 2.3  WinUSB (Windows 8.1+ / 10+)

The device advertises an **MS OS 1.0 Extended Compat ID** descriptor with
`"WINUSB"` as the compatible ID.  Windows 8.1 and later will automatically
load the **WinUSB** driver—no `.inf` file or Zadig is required.

On earlier Windows versions, or if automatic installation does not trigger,
use **Zadig** to bind `WINUSB` to the device (interface 0).

---

## 3  Wire Protocol (CBW / CSW)

The protocol is a command‑response transaction modelled after USB Attached
SCSI (UAS):

```
Host → Device:  Command Block Wrapper    (CBW,   64 bytes,  OUT ep)
Host ↔ Device:  Data phase (optional)    (direction set by CBW.flags)
Device → Host:  Command Status Wrapper   (CSW,   16 bytes,  IN ep)
```

All multi‑byte integers are **little‑endian** on the wire.

### 3.1  Command Block Wrapper (CBW) — 64 bytes

```
Offset  Size  Field           Description
──────  ────  ─────────────── ──────────────────────────────────────────
 0       4    signature       0x50524F47  ("PROG")
 4       4    tag             Echoed in CSW; use monotonic counter
 8       4    transfer_len    Bytes in data phase (0 = no data)
12       1    flags           0x80 = IN (device→host), 0x00 = OUT (host→device)
13       1    opcode          CMD_* (see §4)
14       1    chip            0x01 = eMMC,  0x02 = UFS
15       1    reserved
16       8    lba             Sector address (little‑endian)
24       4    sector_count    Number of 512‑byte sectors
28      36    params          Opcode‑specific parameters (see §5)
```

### 3.2  Command Status Wrapper (CSW) — 16 bytes

```
Offset  Size  Field           Description
──────  ────  ─────────────── ──────────────────────────────────────────
 0       4    signature       0x53544154  ("STAT")
 4       4    tag             Matches CBW tag
 8       4    residue         Untransferred bytes (should be 0 on success)
12       1    status          0x00 = OK,  0x01 = FAIL,  0x02 = PHASE_ERR
13       1    error_code      ERR_* (see §5)
14       2    reserved
```

### 3.3  Session Lifecycle

```
 1. CMD_BEGIN_SESSION (chip_type)   ← select eMMC or UFS
 2. CMD_GET_VERSION                  ← optional
 3. CMD_GET_CHIP_TYPE                ← verify selected chip
 4. CMD_GET_DEVICE_INFO              ← capacity, CID, CSD, …
 5. CMD_EMMC_READ_SECTORS / …       ← I/O commands
 6. CMD_END_SESSION                  ← close handles
```

---

## 4  Command Opcodes

### 4.1  Session / Common

| Opcode | Name                | Data   | Description                    |
|--------|---------------------|--------|--------------------------------|
| 0x01   | `CMD_GET_FW_VERSION` | IN     | Firmware version string        |
| 0x02   | `CMD_GET_CHIP_TYPE`  | IN     | Current chip: 1=eMMC, 2=UFS   |
| 0x03   | `CMD_GET_DEVICE_INFO`| IN     | Device info struct             |
| 0x04   | `CMD_BEGIN_SESSION`  | OUT    | `begin_session_req` (chip_type)|
| 0x05   | `CMD_END_SESSION`    | —      | Close all handles              |
| 0x06   | `CMD_ABORT`          | —      | Cancel in‑flight operation     |
| 0x07   | `CMD_SET_SPEED`      | OUT    | `set_speed_req` (timing mode)  |

### 4.2  eMMC

| Opcode | Name                     | Data   | Params                         |
|--------|--------------------------|--------|--------------------------------|
| 0x10   | `CMD_EMMC_READ_CID`      | IN 16B | —                              |
| 0x11   | `CMD_EMMC_READ_CSD`      | IN 16B | —                              |
| 0x12   | `CMD_EMMC_READ_EXT_CSD`  | IN 512B| —                              |
| 0x13   | `CMD_EMMC_WRITE_EXT_CSD` | OUT    | `emmc_switch_req`              |
| 0x14   | `CMD_EMMC_READ_SECTORS`  | IN     | LBA, count                     |
| 0x15   | `CMD_EMMC_WRITE_SECTORS` | OUT    | LBA, count                     |
| 0x16   | `CMD_EMMC_ERASE`         | OUT    | `emmc_erase_req`               |
| 0x17   | `CMD_EMMC_TRIM`          | OUT    | `emmc_erase_req`               |
| 0x18   | `CMD_EMMC_SECURE_ERASE`  | OUT    | `emmc_erase_req`               |
| 0x19   | `CMD_EMMC_SECURE_TRIM`   | OUT    | `emmc_erase_req`               |
| 0x1A   | `CMD_EMMC_SET_WRITE_PROT`| OUT    | `write_prot_req`               |
| 0x1B   | `CMD_EMMC_CLR_WRITE_PROT`| OUT    | `write_prot_req`               |
| 0x1C   | `CMD_EMMC_SEND_WRITE_PROT`| IN 8B | LBA                            |
| 0x1D   | `CMD_EMMC_BOOT_CONFIG`   | OUT    | `boot_cfg_req`                 |
| 0x1E–20| `CMD_EMMC_RPMB_*`        | varies | `rpmb_req` + 512B frame        |
| 0x21   | `CMD_EMMC_FFU_DOWNLOAD`  | OUT    | firmware image                 |
| 0x22   | `CMD_EMMC_FFU_INSTALL`   | —      | —                              |
| 0x23   | `CMD_EMMC_SLEEP`         | —      | —                              |
| 0x24   | `CMD_EMMC_AWAKE`         | —      | —                              |
| 0x25   | `CMD_EMMC_SELECT_PART`   | OUT    | `emmc_select_part_req`         |

### 4.3  UFS

| Opcode | Name                      | Data   | Params            |
|--------|---------------------------|--------|-------------------|
| 0x30   | `CMD_UFS_READ_DESC`       | IN     | `ufs_desc_req`    |
| 0x31   | `CMD_UFS_WRITE_DESC`      | OUT    | `ufs_desc_req`    |
| 0x32   | `CMD_UFS_READ_ATTR`       | IN 4B  | `ufs_attr_req`    |
| 0x33   | `CMD_UFS_WRITE_ATTR`      | OUT    | `ufs_attr_req`    |
| 0x34   | `CMD_UFS_READ_FLAG`       | IN 1B  | `ufs_flag_req`    |
| 0x35–37| `CMD_UFS_SET/CLEAR/TOGGLE_FLAG` | OUT | `ufs_flag_req` |
| 0x38   | `CMD_UFS_READ_SECTORS`    | IN     | LBA, count        |
| 0x39   | `CMD_UFS_WRITE_SECTORS`   | OUT    | LBA, count        |
| 0x3A   | `CMD_UFS_UNMAP`           | OUT    | LBA, count        |
| 0x3B   | `CMD_UFS_PURGE`           | OUT    | —                 |
| 0x3C   | `CMD_UFS_FDEVICEINIT`     | OUT    | —                 |
| 0x3D   | `CMD_UFS_PROVISION_LU`    | OUT    | provisioning blob |
| 0x3E–40| `CMD_UFS_RPMB_*`          | varies | `rpmb_req` + frame|
| 0x41   | `CMD_UFS_FFU`             | OUT    | firmware image    |
| 0x42   | `CMD_UFS_POWER_MODE`      | OUT    | `ufs_power_mode_req` |
| 0x43   | `CMD_UFS_NOP`             | —      | —                 |
| 0x44   | `CMD_UFS_QUERY_RAW`       | OUT/IN | raw UPIU bytes    |

---

## 5  Error Codes

| Code | Name               | Meaning                      |
|------|--------------------|------------------------------|
| 0x00 | `ERR_OK`           | Success                      |
| 0x01 | `ERR_UNKNOWN_CMD`  | Unrecognised opcode          |
| 0x02 | `ERR_BAD_CHIP_TYPE`| Invalid chip type in session |
| 0x03 | `ERR_NO_DEVICE`    | No open device handle        |
| 0x04 | `ERR_IOCTL_FAIL`   | Linux ioctl/SG_IO error      |
| 0x05 | `ERR_BSG_FAIL`     | UFS BSG command failed       |
| 0x06 | `ERR_BAD_PARAM`    | transfer_len out of range    |
| 0x07 | `ERR_TIMEOUT`      | Command timed out            |
| 0x08 | `ERR_RPMB_AUTH`    | RPMB authentication failure  |
| 0x09 | `ERR_FFU_FAIL`     | Field‑Firmware‑Update failed |
| 0x0A | `ERR_ALLOC`        | Memory allocation failure    |
| 0x0B | `ERR_IO`           | General I/O error            |

---

## 6  Windows Host Implementation Guide

### 6.1  Obtaining a Device Handle

Using [`winusb.h`] / `WinUSB` API (no driver package needed on Win 8.1+):

```c
GUID guid;
GUID_DEVINTERFACE_USB_DEVICE;  // {A5DCBF10-6530-11D2-901F-00C04FB951ED}

HDEVINFO devs = SetupDiGetClassDevs(
    &guid, NULL, NULL,
    DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);

SP_DEVICE_INTERFACE_DATA ifc = { .cbSize = sizeof(ifc) };
for (DWORD i = 0; SetupDiEnumDeviceInterfaces(devs, NULL,
         &guid, i, &ifc); i++) {
    // get detail, open WinUSB handle
}
```

Or retrieve the `"WINUSB"`‑bound device via:

```
SetupDiGetClassDevs(&WINUSB_DEVICE_GUID, ...)
// where WINUSB_DEVICE_GUID is {D8B4A6D5-4F2E-4F8E-9E2E-6F5E7A8B9C0D}
```

### 6.2  Sending a Command

```c
// 1. Send CBW (64 bytes on OUT endpoint)
struct prog_cbw cbw = {
    .signature    = htole32(0x50524F47),    // "PROG"
    .tag          = htole32(tag++),
    .transfer_len = htole32(num_bytes),
    .flags        = direction,              // 0x80 = read, 0x00 = write
    .opcode       = opcode,
    .chip         = chip_type,
    .lba          = htole64(sector_address),
    .sector_count = htole32(sector_count),
};
WinUsb_WritePipe(winusb_handle, 0x01, &cbw, sizeof(cbw), &written, NULL);

// 2. Data phase (if transfer_len > 0)
if (direction == 0x80)   // read  → device→host
    WinUsb_ReadPipe(winusb_handle, 0x81, buf, transfer_len, &read, NULL);
else                     // write → host→device
    WinUsb_WritePipe(winusb_handle, 0x01, buf, transfer_len, &written, NULL);

// 3. Read CSW (16 bytes from IN endpoint)
struct prog_csw csw;
WinUsb_ReadPipe(winusb_handle, 0x81, &csw, sizeof(csw), &read, NULL);
if (csw.status != 0)  // handle error
```

### 6.3  Bulk Transfer Constraints

| Speed       | MaxPacket | Notes                                |
|-------------|-----------|--------------------------------------|
| FS (12 Mb/s)| 64 B      | Small sectors only — very slow       |
| HS (480 Mb/s)| 512 B    | Practical for boot‑image sized I/O   |
| SS (5 Gb/s) | 1024 B    | Preferred — 200 MB/s+ achievable     |

- Keep individual transfers ≤ 1 MB (firmware limit).
- The device endpoint buffer is 1 MB; larger transfers must be split.

### 6.4  Windows Driver Notes

- **Windows 8.1 / 10 / 11**: WinUSB loads automatically via MS OS 1.0
  descriptor.  No `*.inf` or Zadig is needed.
- **Windows 7**: Use [Zadig](https://zadig.akeo.ie/) to bind `WinUSB` to
  interface 0 (`VID 0x1209 / PID 0x0001`).
- **USB View** or `UsbTreeView` can confirm that the `WINUSB` driver
  is attached.

---

## 7  Example — Reading eMMC CID

```c
uint32_t tag = 1;

// CBW
struct prog_cbw cbw = {0};
cbw.signature    = htole32(0x50524F47);
cbw.tag          = htole32(tag);
cbw.transfer_len = htole32(16);       // expect 16 bytes back
cbw.flags        = 0x80;              // IN
cbw.opcode       = 0x10;              // CMD_EMMC_READ_CID
cbw.chip         = 0x01;              // CHIP_EMMC

// Send CBW
WinUsb_WritePipe(h, 0x01, &cbw, 64, &w, NULL);

// Read CID
uint8_t cid[16];
WinUsb_ReadPipe(h, 0x81, cid, 16, &r, NULL);

// Read CSW
struct prog_csw csw;
WinUsb_ReadPipe(h, 0x81, &csw, 16, &r, NULL);

printf("CID: ");
for (int i = 0; i < 16; i++) printf("%02x", cid[i]);
printf("  status=%u err=%u\n", csw.status, csw.error_code);
```

---

## 8  Troubleshooting

| Symptom                      | Likely Fix                                |
|------------------------------|-------------------------------------------|
| Device not enumerated        | Cable is charge‑only; try data cable.     |
| `ERR_NO_DEVICE` on session   | eMMC/UFS not accessible on the device.   |
| `ERR_BAD_PARAM` on I/O       | transfer_len larger than 1 MB.           |
| CSW timeout                  | Device busy; retry or send `CMD_ABORT`.  |
| WinUSB not loading           | Windows 7? Use Zadig.                     |

---

`protocol.h` in the `ffs_app` source tree is the authoritative reference for
all struct layouts, opcodes, and error codes.
