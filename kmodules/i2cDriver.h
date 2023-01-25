#ifndef I2C_DRIVER
#define I2C_DRIVER

#include "bmpTypes.h"

// Helpers
#define CLEAR(x, y) (~y & x)
#define SET(x, y) (y | x)
#define PRINT_DEBUG 1

#define CM_PER_START 0x44E00000
#define CM_PER_I2C2_OFFSET 0x44
#define CM_PER_SIZE 0x400
#define CTRL_MODULE_START 0x44E10000
#define CTRL_MODULE_SIZE 0x2000

#define SDA_CTRL_OFFSET 0x978
#define SCL_CTRL_OFFSET 0x97C

#define I2C_SYSC            0x10
#define I2C_IRQSTATUS_RAW   0x24
#define I2C_IRQSTATUS       0x28
#define I2C_IRQENABLE_SET   0x2C
#define I2C_IRQENABLE_CLR   0x30
#define I2C_CNT             0x98
#define I2C_DATA            0x9C
#define I2C_CON             0xA4
#define I2C_OA              0xA8
#define I2C_SA              0xAC
#define I2C_PSC             0xB0
#define I2C_SCLL            0xB4
#define I2C_SCLH            0xB8

#define I2C_DISABLE(x) CLEAR(0x8000, x)
#define I2C_ENABLE(x) SET(0x8000, x)

// PIN Config
#define PULL_UP 0x33

// I2C Config
#define CM_PER_ON   0x02
#define PSC_VALUE   0x03
#define SCLL_VALUE  0x53
#define SCLH_VALUE  0x55
#define OWN_ADDRESS 0x36
#define SYSC_VALUE  0x00
#define FINAL_CLK   12000000

// IRQ Statuses
#define XRDY        0x10
#define RRDY        0x8
#define BF          0x100
#define ALL_IRQ     0x6FFF

// I2C_CON Statuses
#define CON_ENABLE      0x8000
#define CON_ALL         0xBFF3
#define CON_MASTER      0x400
#define CON_TRANSMITER  0x200
#define CON_STP         0x2
#define CON_START       0x1

// BMP280
#define BMP_ID              0x76    // Slave ID
#define BMP_R_ID            0xD0    // ID Register Address
#define BMP_R_CTRL_MEAS     0xF4    // ctrl_meas Register Address
#define BMP_R_CALIB_00      0x88    // calib00 Register Address
#define BMP_R_TEMP_MSB      0xFA    // temp_msb Register Address
#define BMP_MODE_NORMAL     0x03    // bits mode[1:0] for normal mode
#define BMP_MODE_FORCED     0x02    // bits mode[1:0] for forced mode
#define BMP_TEMP_HIGH_RES   0xE0    // bits osrs_t[2:0] high res temperature measure

// Driver
#define DEVICE_COUNT 0x01

// Device Tree Keys
#define CM_PER_OFFSET   "cm-per-off"
#define SDA_CTRL        "sda-ctrl"
#define SCL_CTRL        "scl-ctrl"
#define MODULE_CLK      "module-clock-freq"
#define TRANSFER_CLK    "clock-frequency"

#endif