#ifndef _VVCAM_REGS
#define _VVCAM_REGS

#include "vvsensor.h"

static struct vvcam_sccb_data_s imx662_init_setting[] = {};

#define STANDBY_REG 0x3000
#define STANDBY_MASK 0x01
#define STANDBY_OFFSET 0

#define XMSTA_REG 0x3002
#define DIGITAL_GAIN_LOW_REG 0x3070
#define DIGITAL_GAIN_HIGH_REG 0x3071
#define VMAX_LOW_REG 0x3028
#define VMAX_MID_REG 0x3029
#define VMAX_HIGH_REG 0x302a
#define HMAX_LOW_REG 0x302c
#define HMAX_HIGH_REG 0x302d
#define SHR0_LOW_REG 0x3050
#define SHR0_MID_REG 0x3051
#define SHR0_HIGH_REG 0x3052
#define PULSE1_UP_LOW_REG 0x30b0
#define PULSE1_UP_MID_REG 0x30b1
#define PULSE1_UP_HIGH_REG 0x30b2
#define PULSE1_DN_LOW_REG 0x30b4
#define PULSE1_DN_MID_REG 0x30b5
#define PULSE1_DN_HIGH_REG 0x30b6

#endif

