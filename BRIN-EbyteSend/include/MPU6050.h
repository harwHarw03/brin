/*
my references : 
MadgwickAHRS Filter Algorithm : http://x-io.co.uk/open-source-imu-and-ahrs-algorithms/
Jeff Rowberg's MPU6050 : https://github.com/jrowberg/i2cdevlib
Davide Gironi's AVR atmega MPU6050 : http://davidegironi.blogspot.com/2013/02/avr-atmega-mpu6050-gyroscope-and.html#.W7zM7mgzaUk
*/
// #include <Arduino.h>
#ifndef MPU6050_H
#define MPU6050_H
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <stdint.h>

#include "delay_m.h"
#include "Scheduler/Scheduler.h"
#include "Scheduler/Task.h"

// MPU6050 Register Map from Jeff Rowberg library
#define MPU6050_ADDRESS_AD0_LOW     0x68 // address pin low (GND), default for InvenSense evaluation board
#define MPU6050_ADDRESS_AD0_HIGH    0x69 // address pin high (VCC)
#define MPU6050_DEFAULT_ADDRESS     MPU6050_ADDRESS_AD0_LOW


#define MPU6050_RA_XG_OFFS_TC       0x00 //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_YG_OFFS_TC       0x01 //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_ZG_OFFS_TC       0x02 //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_X_FINE_GAIN      0x03 //[7:0] X_FINE_GAIN
#define MPU6050_RA_Y_FINE_GAIN      0x04 //[7:0] Y_FINE_GAIN
#define MPU6050_RA_Z_FINE_GAIN      0x05 //[7:0] Z_FINE_GAIN
#define MPU6050_RA_XA_OFFS_H        0x06 //[15:0] XA_OFFS
#define MPU6050_RA_XA_OFFS_L_TC     0x07
#define MPU6050_RA_YA_OFFS_H        0x08 //[15:0] YA_OFFS
#define MPU6050_RA_YA_OFFS_L_TC     0x09
#define MPU6050_RA_ZA_OFFS_H        0x0A //[15:0] ZA_OFFS
#define MPU6050_RA_ZA_OFFS_L_TC     0x0B
#define MPU6050_RA_SELF_TEST_X      0x0D //[7:5] XA_TEST[4-2], [4:0] XG_TEST[4-0]
#define MPU6050_RA_SELF_TEST_Y      0x0E //[7:5] YA_TEST[4-2], [4:0] YG_TEST[4-0]
#define MPU6050_RA_SELF_TEST_Z      0x0F //[7:5] ZA_TEST[4-2], [4:0] ZG_TEST[4-0]
#define MPU6050_RA_SELF_TEST_A      0x10 //[5:4] XA_TEST[1-0], [3:2] YA_TEST[1-0], [1:0] ZA_TEST[1-0]
#define MPU6050_RA_XG_OFFS_USRH     0x13 //[15:0] XG_OFFS_USR
#define MPU6050_RA_XG_OFFS_USRL     0x14
#define MPU6050_RA_YG_OFFS_USRH     0x15 //[15:0] YG_OFFS_USR
#define MPU6050_RA_YG_OFFS_USRL     0x16
#define MPU6050_RA_ZG_OFFS_USRH     0x17 //[15:0] ZG_OFFS_USR
#define MPU6050_RA_ZG_OFFS_USRL     0x18
#define MPU6050_RA_SMPLRT_DIV       0x19
#define MPU6050_RA_CONFIG           0x1A
#define MPU6050_RA_GYRO_CONFIG      0x1B
#define MPU6050_RA_ACCEL_CONFIG     0x1C
#define MPU6050_RA_FF_THR           0x1D
#define MPU6050_RA_FF_DUR           0x1E
#define MPU6050_RA_MOT_THR          0x1F
#define MPU6050_RA_MOT_DUR          0x20
#define MPU6050_RA_ZRMOT_THR        0x21
#define MPU6050_RA_ZRMOT_DUR        0x22
#define MPU6050_RA_FIFO_EN          0x23
#define MPU6050_RA_I2C_MST_CTRL     0x24
#define MPU6050_RA_I2C_SLV0_ADDR    0x25
#define MPU6050_RA_I2C_SLV0_REG     0x26
#define MPU6050_RA_I2C_SLV0_CTRL    0x27
#define MPU6050_RA_I2C_SLV1_ADDR    0x28
#define MPU6050_RA_I2C_SLV1_REG     0x29
#define MPU6050_RA_I2C_SLV1_CTRL    0x2A
#define MPU6050_RA_I2C_SLV2_ADDR    0x2B
#define MPU6050_RA_I2C_SLV2_REG     0x2C
#define MPU6050_RA_I2C_SLV2_CTRL    0x2D
#define MPU6050_RA_I2C_SLV3_ADDR    0x2E
#define MPU6050_RA_I2C_SLV3_REG     0x2F
#define MPU6050_RA_I2C_SLV3_CTRL    0x30
#define MPU6050_RA_I2C_SLV4_ADDR    0x31
#define MPU6050_RA_I2C_SLV4_REG     0x32
#define MPU6050_RA_I2C_SLV4_DO      0x33
#define MPU6050_RA_I2C_SLV4_CTRL    0x34
#define MPU6050_RA_I2C_SLV4_DI      0x35
#define MPU6050_RA_I2C_MST_STATUS   0x36
#define MPU6050_RA_INT_PIN_CFG      0x37
#define MPU6050_RA_INT_ENABLE       0x38
#define MPU6050_RA_DMP_INT_STATUS   0x39
#define MPU6050_RA_INT_STATUS       0x3A
#define MPU6050_RA_ACCEL_XOUT_H     0x3B
#define MPU6050_RA_ACCEL_XOUT_L     0x3C
#define MPU6050_RA_ACCEL_YOUT_H     0x3D
#define MPU6050_RA_ACCEL_YOUT_L     0x3E
#define MPU6050_RA_ACCEL_ZOUT_H     0x3F
#define MPU6050_RA_ACCEL_ZOUT_L     0x40
#define MPU6050_RA_TEMP_OUT_H       0x41
#define MPU6050_RA_TEMP_OUT_L       0x42
#define MPU6050_RA_GYRO_XOUT_H      0x43
#define MPU6050_RA_GYRO_XOUT_L      0x44
#define MPU6050_RA_GYRO_YOUT_H      0x45
#define MPU6050_RA_GYRO_YOUT_L      0x46
#define MPU6050_RA_GYRO_ZOUT_H      0x47
#define MPU6050_RA_GYRO_ZOUT_L      0x48
#define MPU6050_RA_EXT_SENS_DATA_00 0x49
#define MPU6050_RA_EXT_SENS_DATA_01 0x4A
#define MPU6050_RA_EXT_SENS_DATA_02 0x4B
#define MPU6050_RA_EXT_SENS_DATA_03 0x4C
#define MPU6050_RA_EXT_SENS_DATA_04 0x4D
#define MPU6050_RA_EXT_SENS_DATA_05 0x4E
#define MPU6050_RA_EXT_SENS_DATA_06 0x4F
#define MPU6050_RA_EXT_SENS_DATA_07 0x50
#define MPU6050_RA_EXT_SENS_DATA_08 0x51
#define MPU6050_RA_EXT_SENS_DATA_09 0x52
#define MPU6050_RA_EXT_SENS_DATA_10 0x53
#define MPU6050_RA_EXT_SENS_DATA_11 0x54
#define MPU6050_RA_EXT_SENS_DATA_12 0x55
#define MPU6050_RA_EXT_SENS_DATA_13 0x56
#define MPU6050_RA_EXT_SENS_DATA_14 0x57
#define MPU6050_RA_EXT_SENS_DATA_15 0x58
#define MPU6050_RA_EXT_SENS_DATA_16 0x59
#define MPU6050_RA_EXT_SENS_DATA_17 0x5A
#define MPU6050_RA_EXT_SENS_DATA_18 0x5B
#define MPU6050_RA_EXT_SENS_DATA_19 0x5C
#define MPU6050_RA_EXT_SENS_DATA_20 0x5D
#define MPU6050_RA_EXT_SENS_DATA_21 0x5E
#define MPU6050_RA_EXT_SENS_DATA_22 0x5F
#define MPU6050_RA_EXT_SENS_DATA_23 0x60
#define MPU6050_RA_MOT_DETECT_STATUS    0x61
#define MPU6050_RA_I2C_SLV0_DO      0x63
#define MPU6050_RA_I2C_SLV1_DO      0x64
#define MPU6050_RA_I2C_SLV2_DO      0x65
#define MPU6050_RA_I2C_SLV3_DO      0x66
#define MPU6050_RA_I2C_MST_DELAY_CTRL   0x67
#define MPU6050_RA_SIGNAL_PATH_RESET    0x68
#define MPU6050_RA_MOT_DETECT_CTRL      0x69
#define MPU6050_RA_USER_CTRL        0x6A
#define MPU6050_RA_PWR_MGMT_1       0x6B
#define MPU6050_RA_PWR_MGMT_2       0x6C
#define MPU6050_RA_BANK_SEL         0x6D
#define MPU6050_RA_MEM_START_ADDR   0x6E
#define MPU6050_RA_MEM_R_W          0x6F
#define MPU6050_RA_DMP_CFG_1        0x70
#define MPU6050_RA_DMP_CFG_2        0x71
#define MPU6050_RA_FIFO_COUNTH      0x72
#define MPU6050_RA_FIFO_COUNTL      0x73
#define MPU6050_RA_FIFO_R_W         0x74
#define MPU6050_RA_WHO_AM_I         0x75

#define MPU6050_SELF_TEST_XA_1_BIT     0x07
#define MPU6050_SELF_TEST_XA_1_LENGTH  0x03
#define MPU6050_SELF_TEST_XA_2_BIT     0x05
#define MPU6050_SELF_TEST_XA_2_LENGTH  0x02
#define MPU6050_SELF_TEST_YA_1_BIT     0x07
#define MPU6050_SELF_TEST_YA_1_LENGTH  0x03
#define MPU6050_SELF_TEST_YA_2_BIT     0x03
#define MPU6050_SELF_TEST_YA_2_LENGTH  0x02
#define MPU6050_SELF_TEST_ZA_1_BIT     0x07
#define MPU6050_SELF_TEST_ZA_1_LENGTH  0x03
#define MPU6050_SELF_TEST_ZA_2_BIT     0x01
#define MPU6050_SELF_TEST_ZA_2_LENGTH  0x02

#define MPU6050_SELF_TEST_XG_1_BIT     0x04
#define MPU6050_SELF_TEST_XG_1_LENGTH  0x05
#define MPU6050_SELF_TEST_YG_1_BIT     0x04
#define MPU6050_SELF_TEST_YG_1_LENGTH  0x05
#define MPU6050_SELF_TEST_ZG_1_BIT     0x04
#define MPU6050_SELF_TEST_ZG_1_LENGTH  0x05

#define MPU6050_TC_PWR_MODE_BIT     7
#define MPU6050_TC_OFFSET_BIT       6
#define MPU6050_TC_OFFSET_LENGTH    6
#define MPU6050_TC_OTP_BNK_VLD_BIT  0

#define MPU6050_VDDIO_LEVEL_VLOGIC  0
#define MPU6050_VDDIO_LEVEL_VDD     1

#define MPU6050_CFG_EXT_SYNC_SET_BIT    5
#define MPU6050_CFG_EXT_SYNC_SET_LENGTH 3
#define MPU6050_CFG_DLPF_CFG_BIT    2
#define MPU6050_CFG_DLPF_CFG_LENGTH 3

#define MPU6050_EXT_SYNC_DISABLED       0x0
#define MPU6050_EXT_SYNC_TEMP_OUT_L     0x1
#define MPU6050_EXT_SYNC_GYRO_XOUT_L    0x2
#define MPU6050_EXT_SYNC_GYRO_YOUT_L    0x3
#define MPU6050_EXT_SYNC_GYRO_ZOUT_L    0x4
#define MPU6050_EXT_SYNC_ACCEL_XOUT_L   0x5
#define MPU6050_EXT_SYNC_ACCEL_YOUT_L   0x6
#define MPU6050_EXT_SYNC_ACCEL_ZOUT_L   0x7

#define MPU6050_DLPF_BW_256         0x00
#define MPU6050_DLPF_BW_188         0x01
#define MPU6050_DLPF_BW_98          0x02
#define MPU6050_DLPF_BW_42          0x03
#define MPU6050_DLPF_BW_20          0x04
#define MPU6050_DLPF_BW_10          0x05
#define MPU6050_DLPF_BW_5           0x06

#define MPU6050_GCONFIG_FS_SEL_BIT      4
#define MPU6050_GCONFIG_FS_SEL_LENGTH   2

#define MPU6050_GYRO_FS_250         0x00
#define MPU6050_GYRO_FS_500         0x01
#define MPU6050_GYRO_FS_1000        0x02
#define MPU6050_GYRO_FS_2000        0x03

#define MPU6050_ACONFIG_XA_ST_BIT           7
#define MPU6050_ACONFIG_YA_ST_BIT           6
#define MPU6050_ACONFIG_ZA_ST_BIT           5
#define MPU6050_ACONFIG_AFS_SEL_BIT         4
#define MPU6050_ACONFIG_AFS_SEL_LENGTH      2
#define MPU6050_ACONFIG_ACCEL_HPF_BIT       2
#define MPU6050_ACONFIG_ACCEL_HPF_LENGTH    3

#define MPU6050_ACCEL_FS_2          0x00
#define MPU6050_ACCEL_FS_4          0x01
#define MPU6050_ACCEL_FS_8          0x02
#define MPU6050_ACCEL_FS_16         0x03

#define MPU6050_DHPF_RESET          0x00
#define MPU6050_DHPF_5              0x01
#define MPU6050_DHPF_2P5            0x02
#define MPU6050_DHPF_1P25           0x03
#define MPU6050_DHPF_0P63           0x04
#define MPU6050_DHPF_HOLD           0x07

#define MPU6050_TEMP_FIFO_EN_BIT    7
#define MPU6050_XG_FIFO_EN_BIT      6
#define MPU6050_YG_FIFO_EN_BIT      5
#define MPU6050_ZG_FIFO_EN_BIT      4
#define MPU6050_ACCEL_FIFO_EN_BIT   3
#define MPU6050_SLV2_FIFO_EN_BIT    2
#define MPU6050_SLV1_FIFO_EN_BIT    1
#define MPU6050_SLV0_FIFO_EN_BIT    0

#define MPU6050_MULT_MST_EN_BIT     7
#define MPU6050_WAIT_FOR_ES_BIT     6
#define MPU6050_SLV_3_FIFO_EN_BIT   5
#define MPU6050_I2C_MST_P_NSR_BIT   4
#define MPU6050_I2C_MST_CLK_BIT     3
#define MPU6050_I2C_MST_CLK_LENGTH  4

#define MPU6050_CLOCK_DIV_348       0x0
#define MPU6050_CLOCK_DIV_333       0x1
#define MPU6050_CLOCK_DIV_320       0x2
#define MPU6050_CLOCK_DIV_308       0x3
#define MPU6050_CLOCK_DIV_296       0x4
#define MPU6050_CLOCK_DIV_286       0x5
#define MPU6050_CLOCK_DIV_276       0x6
#define MPU6050_CLOCK_DIV_267       0x7
#define MPU6050_CLOCK_DIV_258       0x8
#define MPU6050_CLOCK_DIV_500       0x9
#define MPU6050_CLOCK_DIV_471       0xA
#define MPU6050_CLOCK_DIV_444       0xB
#define MPU6050_CLOCK_DIV_421       0xC
#define MPU6050_CLOCK_DIV_400       0xD
#define MPU6050_CLOCK_DIV_381       0xE
#define MPU6050_CLOCK_DIV_364       0xF

#define MPU6050_I2C_SLV_RW_BIT      7
#define MPU6050_I2C_SLV_ADDR_BIT    6
#define MPU6050_I2C_SLV_ADDR_LENGTH 7
#define MPU6050_I2C_SLV_EN_BIT      7
#define MPU6050_I2C_SLV_BYTE_SW_BIT 6
#define MPU6050_I2C_SLV_REG_DIS_BIT 5
#define MPU6050_I2C_SLV_GRP_BIT     4
#define MPU6050_I2C_SLV_LEN_BIT     3
#define MPU6050_I2C_SLV_LEN_LENGTH  4

#define MPU6050_I2C_SLV4_RW_BIT         7
#define MPU6050_I2C_SLV4_ADDR_BIT       6
#define MPU6050_I2C_SLV4_ADDR_LENGTH    7
#define MPU6050_I2C_SLV4_EN_BIT         7
#define MPU6050_I2C_SLV4_INT_EN_BIT     6
#define MPU6050_I2C_SLV4_REG_DIS_BIT    5
#define MPU6050_I2C_SLV4_MST_DLY_BIT    4
#define MPU6050_I2C_SLV4_MST_DLY_LENGTH 5

#define MPU6050_MST_PASS_THROUGH_BIT    7
#define MPU6050_MST_I2C_SLV4_DONE_BIT   6
#define MPU6050_MST_I2C_LOST_ARB_BIT    5
#define MPU6050_MST_I2C_SLV4_NACK_BIT   4
#define MPU6050_MST_I2C_SLV3_NACK_BIT   3
#define MPU6050_MST_I2C_SLV2_NACK_BIT   2
#define MPU6050_MST_I2C_SLV1_NACK_BIT   1
#define MPU6050_MST_I2C_SLV0_NACK_BIT   0

#define MPU6050_INTCFG_INT_LEVEL_BIT        7
#define MPU6050_INTCFG_INT_OPEN_BIT         6
#define MPU6050_INTCFG_LATCH_INT_EN_BIT     5
#define MPU6050_INTCFG_INT_RD_CLEAR_BIT     4
#define MPU6050_INTCFG_FSYNC_INT_LEVEL_BIT  3
#define MPU6050_INTCFG_FSYNC_INT_EN_BIT     2
#define MPU6050_INTCFG_I2C_BYPASS_EN_BIT    1
#define MPU6050_INTCFG_CLKOUT_EN_BIT        0

#define MPU6050_INTMODE_ACTIVEHIGH  0x00
#define MPU6050_INTMODE_ACTIVELOW   0x01

#define MPU6050_INTDRV_PUSHPULL     0x00
#define MPU6050_INTDRV_OPENDRAIN    0x01

#define MPU6050_INTLATCH_50USPULSE  0x00
#define MPU6050_INTLATCH_WAITCLEAR  0x01

#define MPU6050_INTCLEAR_STATUSREAD 0x00
#define MPU6050_INTCLEAR_ANYREAD    0x01

#define MPU6050_INTERRUPT_FF_BIT            7
#define MPU6050_INTERRUPT_MOT_BIT           6
#define MPU6050_INTERRUPT_ZMOT_BIT          5
#define MPU6050_INTERRUPT_FIFO_OFLOW_BIT    4
#define MPU6050_INTERRUPT_I2C_MST_INT_BIT   3
#define MPU6050_INTERRUPT_PLL_RDY_INT_BIT   2
#define MPU6050_INTERRUPT_DMP_INT_BIT       1
#define MPU6050_INTERRUPT_DATA_RDY_BIT      0

// TODO: figure out what these actually do
// UMPL source code is not very obivous
#define MPU6050_DMPINT_5_BIT            5
#define MPU6050_DMPINT_4_BIT            4
#define MPU6050_DMPINT_3_BIT            3
#define MPU6050_DMPINT_2_BIT            2
#define MPU6050_DMPINT_1_BIT            1
#define MPU6050_DMPINT_0_BIT            0

#define MPU6050_MOTION_MOT_XNEG_BIT     7
#define MPU6050_MOTION_MOT_XPOS_BIT     6
#define MPU6050_MOTION_MOT_YNEG_BIT     5
#define MPU6050_MOTION_MOT_YPOS_BIT     4
#define MPU6050_MOTION_MOT_ZNEG_BIT     3
#define MPU6050_MOTION_MOT_ZPOS_BIT     2
#define MPU6050_MOTION_MOT_ZRMOT_BIT    0

#define MPU6050_DELAYCTRL_DELAY_ES_SHADOW_BIT   7
#define MPU6050_DELAYCTRL_I2C_SLV4_DLY_EN_BIT   4
#define MPU6050_DELAYCTRL_I2C_SLV3_DLY_EN_BIT   3
#define MPU6050_DELAYCTRL_I2C_SLV2_DLY_EN_BIT   2
#define MPU6050_DELAYCTRL_I2C_SLV1_DLY_EN_BIT   1
#define MPU6050_DELAYCTRL_I2C_SLV0_DLY_EN_BIT   0

#define MPU6050_PATHRESET_GYRO_RESET_BIT    2
#define MPU6050_PATHRESET_ACCEL_RESET_BIT   1
#define MPU6050_PATHRESET_TEMP_RESET_BIT    0

#define MPU6050_DETECT_ACCEL_ON_DELAY_BIT       5
#define MPU6050_DETECT_ACCEL_ON_DELAY_LENGTH    2
#define MPU6050_DETECT_FF_COUNT_BIT             3
#define MPU6050_DETECT_FF_COUNT_LENGTH          2
#define MPU6050_DETECT_MOT_COUNT_BIT            1
#define MPU6050_DETECT_MOT_COUNT_LENGTH         2

#define MPU6050_DETECT_DECREMENT_RESET  0x0
#define MPU6050_DETECT_DECREMENT_1      0x1
#define MPU6050_DETECT_DECREMENT_2      0x2
#define MPU6050_DETECT_DECREMENT_4      0x3

#define MPU6050_USERCTRL_DMP_EN_BIT             7
#define MPU6050_USERCTRL_FIFO_EN_BIT            6
#define MPU6050_USERCTRL_I2C_MST_EN_BIT         5
#define MPU6050_USERCTRL_I2C_IF_DIS_BIT         4
#define MPU6050_USERCTRL_DMP_RESET_BIT          3
#define MPU6050_USERCTRL_FIFO_RESET_BIT         2
#define MPU6050_USERCTRL_I2C_MST_RESET_BIT      1
#define MPU6050_USERCTRL_SIG_COND_RESET_BIT     0

#define MPU6050_PWR1_DEVICE_RESET_BIT   7
#define MPU6050_PWR1_SLEEP_BIT          6
#define MPU6050_PWR1_CYCLE_BIT          5
#define MPU6050_PWR1_TEMP_DIS_BIT       3
#define MPU6050_PWR1_CLKSEL_BIT         2
#define MPU6050_PWR1_CLKSEL_LENGTH      3

#define MPU6050_CLOCK_INTERNAL          0x00
#define MPU6050_CLOCK_PLL_XGYRO         0x01
#define MPU6050_CLOCK_PLL_YGYRO         0x02
#define MPU6050_CLOCK_PLL_ZGYRO         0x03
#define MPU6050_CLOCK_PLL_EXT32K        0x04
#define MPU6050_CLOCK_PLL_EXT19M        0x05
#define MPU6050_CLOCK_KEEP_RESET        0x07

#define MPU6050_PWR2_LP_WAKE_CTRL_BIT       7
#define MPU6050_PWR2_LP_WAKE_CTRL_LENGTH    2
#define MPU6050_PWR2_STBY_XA_BIT            5
#define MPU6050_PWR2_STBY_YA_BIT            4
#define MPU6050_PWR2_STBY_ZA_BIT            3
#define MPU6050_PWR2_STBY_XG_BIT            2
#define MPU6050_PWR2_STBY_YG_BIT            1
#define MPU6050_PWR2_STBY_ZG_BIT            0

#define MPU6050_WAKE_FREQ_1P25      0x0
#define MPU6050_WAKE_FREQ_2P5       0x1
#define MPU6050_WAKE_FREQ_5         0x2
#define MPU6050_WAKE_FREQ_10        0x3

#define MPU6050_BANKSEL_PRFTCH_EN_BIT       6
#define MPU6050_BANKSEL_CFG_USER_BANK_BIT   5
#define MPU6050_BANKSEL_MEM_SEL_BIT         4
#define MPU6050_BANKSEL_MEM_SEL_LENGTH      5

#define MPU6050_WHO_AM_I_BIT        6
#define MPU6050_WHO_AM_I_LENGTH     6

#define MPU6050_DMP_MEMORY_BANKS        8
#define MPU6050_DMP_MEMORY_BANK_SIZE    256
#define MPU6050_DMP_MEMORY_CHUNK_SIZE   16

// Quaternion function
#define twoKpDef  (2.0f * 0.5f) // 2 * proportional gain
#define twoKiDef  (2.0f * 0.0f) // 2 * integral gain

// accel & gyro data
//AcX = 205 | AcY = -230 | AcZ = -120 | 

#define MPU6050_AXOFFSET 205
#define MPU6050_AYOFFSET -230
#define MPU6050_AZOFFSET -120
//#define MPU6050_AXOFFSET 0
//#define MPU6050_AYOFFSET 0
//#define MPU6050_AZOFFSET 0
//#define MPU6050_AXGAIN 16384.0 // AFS_SEL = 0, +/-2g, MPU6050_ACCEL_FS_2
//#define MPU6050_AYGAIN 16384.0 // AFS_SEL = 0, +/-2g, MPU6050_ACCEL_FS_2
//#define MPU6050_AZGAIN 16384.0 // AFS_SEL = 0, +/-2g, MPU6050_ACCEL_FS_2
//#define MPU6050_AXGAIN 8192.0 // AFS_SEL = 1, +/-4g, MPU6050_ACCEL_FS_4
//#define MPU6050_AYGAIN 8192.0 // AFS_SEL = 1, +/-4g, MPU6050_ACCEL_FS_4
//#define MPU6050_AZGAIN 8192.0 // AFS_SEL = 1, +/-4g, MPU6050_ACCEL_FS_4
#define MPU6050_AXGAIN 4096.0 // AFS_SEL = 2, +/-8g, MPU6050_ACCEL_FS_8
#define MPU6050_AYGAIN 4096.0 // AFS_SEL = 2, +/-8g, MPU6050_ACCEL_FS_8
#define MPU6050_AZGAIN 4096.0 // AFS_SEL = 2, +/-8g, MPU6050_ACCEL_FS_8
//#define MPU6050_AXGAIN 2048.0 // AFS_SEL = 3, +/-16g, MPU6050_ACCEL_FS_16
//#define MPU6050_AYGAIN 2048.0 // AFS_SEL = 3, +/-16g, MPU6050_ACCEL_FS_16
//#define MPU6050_AZGAIN 2048.0 // AFS_SEL = 3, +/-16g, MPU6050_ACCEL_FS_16
////GyX = -72 | GyY = -32 | GyZ = -10
#define MPU6050_GXOFFSET -72
#define MPU6050_GYOFFSET -32
#define MPU6050_GZOFFSET -18
//#define MPU6050_GXOFFSET 0
//#define MPU6050_GYOFFSET 0
//#define MPU6050_GZOFFSET 0
//#define MPU6050_GXGAIN 131.072 // FS_SEL = 0, +/-250degree/s, MPU6050_GYRO_FS_250
//#define MPU6050_GYGAIN 131.072 // FS_SEL = 0, +/-250degree/s, MPU6050_GYRO_FS_250
//#define MPU6050_GZGAIN 131.072 // FS_SEL = 0, +/-250degree/s, MPU6050_GYRO_FS_250
//#define MPU6050_GXGAIN 65.536 // FS_SEL = 1, +/-500degree/s, MPU6050_GYRO_FS_500
//#define MPU6050_GYGAIN 65.536 // FS_SEL = 1, +/-500degree/s, MPU6050_GYRO_FS_500
//#define MPU6050_GZGAIN 65.536 // FS_SEL = 1, +/-500degree/s, MPU6050_GYRO_FS_500
//#define MPU6050_GXGAIN 32.768 // FS_SEL = 2, +/-1000degree/s, MPU6050_GYRO_FS_1000
//#define MPU6050_GYGAIN 32.768 // FS_SEL = 2, +/-1000degree/s, MPU6050_GYRO_FS_1000
//#define MPU6050_GZGAIN 32.768 // FS_SEL = 2, +/-1000degree/s, MPU6050_GYRO_FS_1000
#define MPU6050_GXGAIN 16.384 // FS_SEL = 3, +/-2000degree/s, MPU6050_GYRO_FS_2000
#define MPU6050_GYGAIN 16.384 // FS_SEL = 3, +/-2000degree/s, MPU6050_GYRO_FS_2000
#define MPU6050_GZGAIN 16.384 // FS_SEL = 3, +/-2000degree/s, MPU6050_GYRO_FS_2000

// Variables
volatile float twoKp = twoKpDef;    // 2 * proportional gain (Kp)
volatile float twoKi = twoKiDef;    // 2 * integral gain (Ki)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;  // quaternion of sensor frame relative to auxiliary frame
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f; // integral error terms scaled by Ki

long timer;
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
float axg, ayg, azg, gxrs, gyrs, gzrs;
float roll, pitch, yaw;

float SelfTest[6];

float gyroBias[3] = {-31, -19, -19}, accelBias[3] = {242, -55, -669}; // Bias corrections for gyro and accelerometer
// AcX = 242 | AcY = -55 | AcZ = -669 | GyX = -31 | GyY = -19 | GyZ = -19

float sampleFreq = 0.0f;                              // integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0;         // used to calculate integration interval
uint32_t Now = 0;                                 // used to calculate integration interval

void WriteByte(uint8_t address, uint8_t subAddress, uint8_t data){
    Wire.beginTransmission(address);
    Wire.write(subAddress);
    Wire.write(data);
    Wire.endTransmission();
}

uint8_t ReadByte(uint8_t address, uint8_t subAddress){
    uint8_t data;
    Wire.beginTransmission(address);
    Wire.write(subAddress);
    Wire.endTransmission(false);
    Wire.requestFrom(address, (uint8_t) 1);
    data = Wire.read();
    return data;
}

void ReadBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest){
    Wire.beginTransmission(address);
    Wire.write(subAddress);
    Wire.endTransmission(false);
    uint8_t i = 0;
    Wire.requestFrom(address, count);
    while(Wire.available()){
        dest[i++] = Wire.read();
    }
}

void MPU6050Init(){
    WriteByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, 0x00);
    WriteByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, 0x01);
    // WriteByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_SMPLRT_DIV, 0x00);    // MPU6050 Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV) for DMP
    WriteByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, 0x18);   //FS_SEL=3, +- 2000 ^/s
    WriteByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG, 0x10);  //AFS_SEL=2, +- 8g
    WriteByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_CONFIG, 0x00);    //Accel BW 260Hz, Delay 0ms / Gyro BW 256Hz, Delay 0.98ms, Fs 8KHz
}



void MPU6050GetData(){
    uint8_t data_raw[14];
    ReadBytes(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_XOUT_H, 14, &data_raw[0]);
    AcX = data_raw[0] << 8 | data_raw[1];
    AcY = data_raw[2] << 8 | data_raw[3];
    AcZ = data_raw[4] << 8 | data_raw[5];
    Tmp = data_raw[6] << 8 | data_raw[7];
    GyX = data_raw[8] << 8 | data_raw[9];
    GyY = data_raw[10] << 8 | data_raw[11];
    GyZ = data_raw[12] << 8 | data_raw[13];
}

void UpdateQuaternion(){
    axg = (float)(AcX - MPU6050_AXOFFSET) / MPU6050_AXGAIN;
    ayg = (float)(AcY - MPU6050_AYOFFSET) / MPU6050_AYGAIN;
    azg = (float)(AcZ - MPU6050_AZOFFSET) / MPU6050_AZGAIN;
    gxrs = (float)(GyX - MPU6050_GXOFFSET) / MPU6050_GXGAIN * 0.01745329;   //degree to radians
    gyrs = (float)(GyY - MPU6050_GYOFFSET) / MPU6050_GYGAIN * 0.01745329;   //degree to radians
    gzrs = (float)(GyZ - MPU6050_GZOFFSET) / MPU6050_GZGAIN * 0.01745329;   //degree to radians
}

void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az){
    float norm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    // Normalise accelerometer measurement
    norm = sqrt(ax * ax + ay * ay + az * az);
    ax /= norm;
    ay /= norm;
    az /= norm;

    // Estimated direction of gravity and vector perpendicular to magnetic flux
    halfvx = q1 * q3 - q0 * q2;
    halfvy = q0 * q1 + q2 * q3;
    halfvz = q0 * q0 - 0.5f + q3 * q3;
  
    // Error is sum of cross product between estimated and measured direction of gravity
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);

    // Compute and apply integral feedback if enabled
    if(twoKi > 0.0f) {
      integralFBx += twoKi * halfex * (1.0f / sampleFreq);  // integral error scaled by Ki
      integralFBy += twoKi * halfey * (1.0f / sampleFreq);
      integralFBz += twoKi * halfez * (1.0f / sampleFreq);
      gx += integralFBx;  // apply integral feedback
      gy += integralFBy;
      gz += integralFBz;
    }
    else {
      integralFBx = 0.0f; // prevent integral windup
      integralFBy = 0.0f;
      integralFBz = 0.0f;
    }

    // Apply proportional feedback
    gx += twoKp * halfex;
    gy += twoKp * halfey;
    gz += twoKp * halfez;
  }
  
  // Integrate rate of change of quaternion
  gx *= (0.5f * (1.0f / sampleFreq));   // pre-multiply common factors
  gy *= (0.5f * (1.0f / sampleFreq));
  gz *= (0.5f * (1.0f / sampleFreq));
  qa = q0;
  qb = q1;
  qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += (qa * gx + qc * gz - q3 * gy);
  q2 += (qa * gy - qb * gz + q3 * gx);
  q3 += (qa * gz + qb * gy - qc * gx); 
  
  // Normalise quaternion
  norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 /= norm;
  q1 /= norm;
  q2 /= norm;
  q3 /= norm;
}

void GetRollPitchYaw(){
    // yaw = -atan2(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.29577951;
    // pitch = -asin(2.0f * (q1 * q3 - q0 * q2)) * 57.29577951;
    // roll = atan2(2.0f * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * 57.29577951;
    roll = atan2((2.0f * (q0 * q1 + q2 * q3)), (1.0f - 2.0f * (q1 * q1 + q2 * q2))) * 57.29577951;
    pitch = (2.0f * atan2((sqrt(1 + 2 * (q0 * q2 - q1 * q3))), (sqrt(1.0f - 2.0f * (q0 * q2 - q1 * q3)))) - M_PI / 2.0f )* 57.29577951;
    yaw = atan2(( 2.0f * (q0 * q3 + q1 * q2)), (1.0f - 2.0f * (q2 * q2 + q3 * q3))) * 57.29577951;
}

void MPU6050SelfTest(float * destination){
    uint8_t rawData[4];
    uint8_t selfTest[6];
    float factoryTrim[6];
    
    // Configure the accelerometer for self-test
    WriteByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG, 0xF0); // Enable self test on all three axes and set accelerometer range to +/- 8 g
    WriteByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
    DelayMillis(250);  // Delay a while to let the device execute the self-test
    rawData[0] = ReadByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_SELF_TEST_X); // X-axis self-test results
    rawData[1] = ReadByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_SELF_TEST_Y); // Y-axis self-test results
    rawData[2] = ReadByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_SELF_TEST_Z); // Z-axis self-test results
    rawData[3] = ReadByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_SELF_TEST_A); // Mixed-axis self-test results
    // Extract the acceleration test results first
    selfTest[0] = (rawData[0] >> 3) | (rawData[3] & 0x30) >> 4 ; // XA_TEST result is a five-bit unsigned integer
    selfTest[1] = (rawData[1] >> 3) | (rawData[3] & 0x0C) >> 2 ; // YA_TEST result is a five-bit unsigned integer
    selfTest[2] = (rawData[2] >> 3) | (rawData[3] & 0x03) ; // ZA_TEST result is a five-bit unsigned integer
    // Extract the gyration test results first
    selfTest[3] = rawData[0]  & 0x1F ; // XG_TEST result is a five-bit unsigned integer
    selfTest[4] = rawData[1]  & 0x1F ; // YG_TEST result is a five-bit unsigned integer
    selfTest[5] = rawData[2]  & 0x1F ; // ZG_TEST result is a five-bit unsigned integer   
    // Process results to allow final comparison with factory set values
    factoryTrim[0] = (4096.0*0.34)*(pow( (0.92/0.34) , (((float)selfTest[0] - 1.0)/30.0))); // FT[Xa] factory trim calculation
    factoryTrim[1] = (4096.0*0.34)*(pow( (0.92/0.34) , (((float)selfTest[1] - 1.0)/30.0))); // FT[Ya] factory trim calculation
    factoryTrim[2] = (4096.0*0.34)*(pow( (0.92/0.34) , (((float)selfTest[2] - 1.0)/30.0))); // FT[Za] factory trim calculation
    factoryTrim[3] =  ( 25.0*131.0)*(pow( 1.046 , ((float)selfTest[3] - 1.0) ));             // FT[Xg] factory trim calculation
    factoryTrim[4] =  (-25.0*131.0)*(pow( 1.046 , ((float)selfTest[4] - 1.0) ));             // FT[Yg] factory trim calculation
    factoryTrim[5] =  ( 25.0*131.0)*(pow( 1.046 , ((float)selfTest[5] - 1.0) ));             // FT[Zg] factory trim calculation
    // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
 // To get to percent, must multiply by 100 and subtract result from 100
    for (int i = 0; i < 6; i++) {
        destination[i] = 100.0 + 100.0*((float)selfTest[i] - factoryTrim[i])/factoryTrim[i]; // Report percent differences
    }  
}

void CalibrateMPU6050(float * dest1, float * dest2){
    uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
    uint16_t ii, packet_count, fifo_count;
    int32_t gyro_bias[3] = {-31, -19, -19}, accel_bias[3] = {242, -55, -669};
    
    // reset device, reset all registers, clear gyro and accelerometer bias registers
    WriteByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
    DelayMillis(100);  
    
    // get stable time source
    // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
    WriteByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, 0x01);  
    WriteByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_2, 0x00); 
    DelayMillis(200);
    
    // Configure device for bias calculation
    WriteByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_INT_ENABLE, 0x00);   // Disable all interrupts
    WriteByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_FIFO_EN, 0x00);      // Disable FIFO
    WriteByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, 0x00);   // Turn on internal clock source
    WriteByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_I2C_MST_CTRL, 0x00); // Disable I2C master
    WriteByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
    WriteByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_USER_CTRL, 0x0C);    // Reset FIFO and DMP
    DelayMillis(15);
    
    // Configure MPU6050 gyro and accelerometer for bias calculation
    WriteByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_CONFIG, 0x01);      // Set low-pass filter to 188 Hz
    WriteByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
    WriteByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
    WriteByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity
    
    uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
    uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

    // Configure FIFO to capture accelerometer and gyro data for bias calculation
    WriteByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_USER_CTRL, 0x40);   // Enable FIFO  
    WriteByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 1024 bytes in MPU-6050)
    DelayMillis(80); // accumulate 80 samples in 80 milliseconds = 960 bytes

    // At end of sample accumulation, turn off FIFO sensor read
    WriteByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
    ReadBytes(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
    fifo_count = ((uint16_t)data[0] << 8) | data[1];
    packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging

    for (ii = 0; ii < packet_count; ii++) {
        int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
        ReadBytes(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_FIFO_R_W, 12, &data[0]); // read data for averaging
        accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
        accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
        accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;    
        gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
        gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
        gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;
        
        accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
        accel_bias[1] += (int32_t) accel_temp[1];
        accel_bias[2] += (int32_t) accel_temp[2];
        gyro_bias[0]  += (int32_t) gyro_temp[0];
        gyro_bias[1]  += (int32_t) gyro_temp[1];
        gyro_bias[2]  += (int32_t) gyro_temp[2];
                
    }
        accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
        accel_bias[1] /= (int32_t) packet_count;
        accel_bias[2] /= (int32_t) packet_count;
        gyro_bias[0]  /= (int32_t) packet_count;
        gyro_bias[1]  /= (int32_t) packet_count;
        gyro_bias[2]  /= (int32_t) packet_count;
        
    if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
    else {accel_bias[2] += (int32_t) accelsensitivity;}
    
    // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
    data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
    data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
    data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
    data[3] = (-gyro_bias[1]/4)       & 0xFF;
    data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
    data[5] = (-gyro_bias[2]/4)       & 0xFF;

    // Push gyro biases to hardware registers
    WriteByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_XG_OFFS_USRH, data[0]);// might not be supported in MPU6050
    WriteByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_XG_OFFS_USRL, data[1]);
    WriteByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_YG_OFFS_USRH, data[2]);
    WriteByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_YG_OFFS_USRL, data[3]);
    WriteByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ZG_OFFS_USRH, data[4]);
    WriteByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ZG_OFFS_USRL, data[5]);

    dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity; // construct gyro bias in deg/s for later manual subtraction
    dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
    dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

    // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
    // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
    // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
    // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
    // the accelerometer biases calculated above must be divided by 8.

    int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
    ReadBytes(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_XA_OFFS_H, 2, &data[0]); // Read factory accelerometer trim values
    accel_bias_reg[0] = (int16_t) ((int16_t)data[0] << 8) | data[1];
    ReadBytes(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_YA_OFFS_H, 2, &data[0]);
    accel_bias_reg[1] = (int16_t) ((int16_t)data[0] << 8) | data[1];
    ReadBytes(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ZA_OFFS_H, 2, &data[0]);
    accel_bias_reg[2] = (int16_t) ((int16_t)data[0] << 8) | data[1];
    
    uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
    uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis
    
    for(ii = 0; ii < 3; ii++) {
        if(accel_bias_reg[ii] & mask) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
    }

    // Construct total accelerometer bias, including calculated average accelerometer bias from above
    accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
    accel_bias_reg[1] -= (accel_bias[1]/8);
    accel_bias_reg[2] -= (accel_bias[2]/8);
    
    data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
    data[1] = (accel_bias_reg[0])      & 0xFF;
    data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
    data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
    data[3] = (accel_bias_reg[1])      & 0xFF;
    data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
    data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
    data[5] = (accel_bias_reg[2])      & 0xFF;
    data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

    // Push accelerometer biases to hardware registers
    WriteByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_XA_OFFS_H, data[0]); // might not be supported in MPU6050
    WriteByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_XA_OFFS_L_TC, data[1]);
    WriteByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_YA_OFFS_H, data[2]);
    WriteByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_YA_OFFS_L_TC, data[3]);  
    WriteByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ZA_OFFS_H, data[4]);
    WriteByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ZA_OFFS_L_TC, data[5]);

    // Output scaled accelerometer biases for manual subtraction in the main program
    dest2[0] = (float)accel_bias[0]/(float)accelsensitivity; 
    dest2[1] = (float)accel_bias[1]/(float)accelsensitivity;
    dest2[2] = (float)accel_bias[2]/(float)accelsensitivity;
}
        // void Init();
        // void WriteByte(uint8_t address, uint8_t subAddress, uint8_t data);
        // uint8_t ReadByte(uint8_t address, uint8_t subAddress);
        // void ReadBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest);
        // void GetData();
        // void UpdateQuaternion();
        // void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
        // void GetRollPitchYaw();
        // void SelfTest(float * destination);
        // void CalibrateMPU6050(float * dest1, float * dest2);
void imu_init()
{
    Wire.begin();
    Wire.setClock(400000);
    MPU6050Init();
}
void imu_update()
{
    MPU6050GetData();
    UpdateQuaternion();
    Now = micros();
    sampleFreq = (1000000.0f / (Now - lastUpdate)); // set integration time by time elapsed since last filter update
    lastUpdate = Now;
    MahonyAHRSupdateIMU(gxrs, gyrs, gzrs, axg, ayg, azg);
    GetRollPitchYaw();
    // _sensordata.roll = roll;
    // _sensordata.pitch = pitch;
}

// IMUTASK imu_task(100);

#endif