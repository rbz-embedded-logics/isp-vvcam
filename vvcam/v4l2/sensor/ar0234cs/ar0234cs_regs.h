#ifndef _VVCAM_REGS
#define _VVCAM_REGS

#include "vvsensor.h"

static struct vvcam_sccb_data_s ar0234_init_setting[] = {};

#define CHIPID_REG			0x3000
#define RESET_REG		     	0x301a
#define Y_ADDR_END_REG               	0x3006
#define Y_ADDR_START_REG             	0x3002
#define X_ADDR_END_REG               	0x3008
#define X_ADDR_START_REG             	0x3004
#define COARSE_INTEGRATION_TIME_REG  	0x3012
#define DIGITAL_GAIN_REG             	0x305e
#define ANALOG_GAIN_REG              	0x3060
#define READ_MODE_REG                	0x3040
#define IMAGE_ORIENTATION_REG           0x301d

#define FRAME_LENGTH_LINE_REG		0x300A
#define LINE_LENGTH_PCK_REG		0x300c

#endif

