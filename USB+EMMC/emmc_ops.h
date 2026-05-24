/* emmc_ops.h */
#ifndef EMMC_OPS_H
#define EMMC_OPS_H

#include <stdint.h>
#include <stddef.h>
#include "protocol.h"

int  emmc_open(void);
void emmc_close(int fd);

int  emmc_read_cid(int fd, uint8_t cid[16]);
int  emmc_read_csd(int fd, uint8_t csd[16]);
int  emmc_read_ext_csd(int fd, uint8_t ext_csd[512]);
int  emmc_switch(int fd, uint8_t index, uint8_t value, uint8_t cmd_set);
int  emmc_select_partition(int fd, uint8_t partition);

int  emmc_read_sectors(int fd, uint64_t lba, uint32_t count, uint8_t *buf);
int  emmc_write_sectors(int fd, uint64_t lba, uint32_t count,
                        const uint8_t *buf);
int  emmc_erase(int fd, uint64_t start_lba, uint64_t end_lba, uint32_t arg);

int  emmc_set_write_protect(int fd, uint64_t lba, uint8_t type, int set);
int  emmc_send_write_prot(int fd, uint64_t lba, uint8_t prot[8]);
int  emmc_set_boot_config(int fd, uint8_t boot_config, uint8_t boot_bus_cond);
int  emmc_sleep_awake(int fd, int sleep);

int  emmc_rpmb_op(int fd, uint8_t opcode, const struct rpmb_req *req,
                  uint8_t frame[512]);

int  emmc_ffu_download(int fd, const uint8_t *fw_data, size_t fw_len);
int  emmc_ffu_install(int fd);

int  emmc_get_device_info(int fd, struct device_info *info);

#endif /* EMMC_OPS_H */
