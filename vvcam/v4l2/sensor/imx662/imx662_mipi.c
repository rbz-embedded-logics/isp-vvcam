#include <linux/of_graph.h>
#include <linux/device.h>
#include <linux/of_device.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-fwnode.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include "vvsensor.h"

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/firmware.h>
#include <linux/v4l2-mediabus.h>
#include <linux/videodev2.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

#include "imx662_regs.h"

#define DRIVER_NAME "imx662"

#define IMX662_MAX_EXPOSURE_TIME	49840
#define IMX662_MIN_EXPOSURE_TIME	50
#define IMX662_MAX_GAIN			2197
#define IMX662_MIN_GAIN			150
#define IMX662_DEF_GAIN			150
#define IMX662_MAX_ANALOG_GAIN		15	
#define IMX662_MIN_ANALOG_GAIN		1	
#define IMX662_DEF_FRAME_RATE		20
#define IMX662_EXP_TIME_CORRECTION	0
#define IMX662_MAX_WIDTH		1928
#define IMX662_MAX_HEIGHT		1208

#define MICROSECS_TO_NSECS      1000
#define SECS_TO_NSECS           1000000000
#define T_EXP_ERROR             0
#define MIN_SHS                 4
#define COMMUNICATION_PERIOD    10

#define IMX662_TABLE_END		0xffff

#define IMX662_PIX_CLK			74250000

#define IMX662_OUT_RANGE (-1)

#define V4L2_CID_FLASH_TIME             (V4L2_CID_DV_CLASS_BASE + 0x1002)
#define V4L2_CID2_GAIN      		    (V4L2_CID_DV_CLASS_BASE + 0x1005)
#define V4L2_CID_MAX_GAIN      		    (V4L2_CID_DV_CLASS_BASE + 0x1006)
#define V4L2_CID_MIN_GAIN      		    (V4L2_CID_DV_CLASS_BASE + 0x1007)
#define V4L2_CID_MAX_EXPOSURE      		(V4L2_CID_DV_CLASS_BASE + 0x1008)
#define V4L2_CID_MIN_EXPOSURE      		(V4L2_CID_DV_CLASS_BASE + 0x1009)

#define SENSOR_CHANNEL_0    0
#define SENSOR_CHANNEL_1    1

#define SENSOR_MODEL_MONO   0
#define SENSOR_MODEL_COLOR  1

#define MASTER_MODE	    1
/*#define DEBUG*/

#ifdef CONFIG_HARDENED_USERCOPY
#define client_to_imx662(client)\
        container_of(i2c_get_clientdata(client), struct imx662, subdev)

#define USER_TO_KERNEL(TYPE) \
	        do {\
			TYPE tmp; \
			unsigned long copy_ret; \
			arg = (void *)(&tmp); \
			copy_ret = copy_from_user(arg, arg_user, sizeof(TYPE));\
		} while (0)

#define KERNEL_TO_USER(TYPE) \
	        do {\
			unsigned long copy_ret; \
			copy_ret = copy_to_user(arg_user, arg, sizeof(TYPE));\
		} while (0)
#else
#define USER_TO_KERNEL(TYPE)
#define KERNEL_TO_USER(TYPE)
#endif

/*#define DEBUG //Comment this line for disabling DEBUG traces*/

enum {
  VVSENSORIOC_S_MAX_GAIN = 0x200,
  VVSENSORIOC_S_MIN_GAIN,
  VVSENSORIOC_S_MAX_INT_TIME,
  VVSENSORIOC_S_MIN_INT_TIME,
  VVSENSORIOC_G_GAIN,
  VVSENSORIOC_G_EXP
};

/* PRIVATE FUNCTIONS */
static int imx662_s_stream(struct v4l2_subdev *sd, int on);

//end private functions

static const struct regmap_config imx662_regmap_config = {
  .reg_bits = 16,
  .val_bits = 8,
  .cache_type = REGCACHE_NONE,
};


struct imx662_ctrls {
  struct v4l2_ctrl_handler handler;
  struct v4l2_ctrl *gain;
  struct v4l2_ctrl *exposure;
  struct v4l2_ctrl *flash_time;
  struct v4l2_ctrl *max_gain;
  struct v4l2_ctrl *min_gain;
  struct v4l2_ctrl *max_exposure;
  struct v4l2_ctrl *min_exposure;
};

struct imx662_capture_properties {
  __u64 max_lane_frequency;
  __u64 max_pixel_frequency;
  __u64 max_data_rate;
};

struct stimx662 {
  struct v4l2_subdev sd;
  struct v4l2_rect crop;
  struct v4l2_mbus_framefmt format;
  struct v4l2_fract frame_interval;
  struct media_pad pad;
  struct i2c_client *client;
  struct regmap *regmap;
  struct imx662_ctrls ctrls;
  struct mutex lock;
  struct imx662_capture_properties ocp;
  struct clk *sensor_clk;
  u32 sensor_channel;
  u32 sensor_model;
  u32 x_start;
  u32 y_start;
  vvcam_mode_info_t cur_mode;
  int csi_id;
  struct reg_8 *imx662_init_config;
  unsigned int xclk_source;
  unsigned int pwn_gpio;
  unsigned int rst_gpio;
  vvcam_lens_t focus_lens;
};

struct reg_8 {
  uint16_t addr;
  uint8_t val;
  uint8_t mask;
  uint8_t offset;
};

/* Con reloj externo a 54 MHz */
static const struct reg_8 imx662_init_config[] = {
  {0x3000, 0x01, 0xFF, 0}, // STANDBY
  {0x3001, 0x01, 0xFF, 0}, // REGHOLD
  {0x3014, 0x00, 0xFF, 0}, // INCK_SEL[3:0] = 74.25MHz
  {0x3015, 0x03, 0xFF, 0}, // DATARATE_SEL[3:0] = 1440Mbps
  {0x3018, 0x00, 0xFF, 0}, // WINMODE[3:0] = All-pixel mode
  {0x3019, 0x00, 0xFF, 0}, // Color filter mode setting = RGB
  {0x301a, 0x00, 0xFF, 0}, // WDMODE HDR mode setting = Normal Mode
  {0x301b, 0x00, 0xFF, 0}, // ADDMODE[1:0] = Non-binning
  {0x301c, 0x00, 0xFF, 0}, // THIN_V_EN = Disable (Normal)
  {0x301e, 0x01, 0xFF, 0}, // VCMODE = Virtual Channel Mode
  {0x3020, 0x00, 0xFF, 0}, // HREVERSE = Normal
  {0x3021, 0x00, 0xFF, 0}, // VREVERSE = Normal
  {0x3022, 0x00, 0xFF, 0}, // ADBIT[0] = 10bit
  {0x3023, 0x00, 0xFF, 0}, // MDBIT[0] = 10bit
  {0x3028, 0x48, 0xFF, 0}, // VMAX
  {0x3029, 0x0d, 0xFF, 0}, // VMAX
  {0x302a, 0x00, 0xFF, 0}, // VMAX
  {0x302c, 0x9a, 0xFF, 0}, // HMAX
  {0x302d, 0x0b, 0xFF, 0}, // HMAX
  {0x3030, 0x00, 0xFF, 0}, // FDG_SEL0[1:0] = LCG Mode
  {0x3031, 0x00, 0xFF, 0}, // FDG_SEL1[1:0] = LCG Mode
  {0x3032, 0x00, 0xFF, 0}, // FDG_SEL2[1:0] = LCG Mode
  {0x303c, 0x00, 0xFF, 0}, // PIX_HST[7:0]
  {0x303d, 0x00, 0xFF, 0}, // PIX_HST[12:8]
  {0x303e, 0x00, 0xFF, 0}, // PIX_HWIDTH[7:0]
  {0x303f, 0x00, 0xFF, 0}, // PIX_HWIDTH[12:8]
  {0x3040, 0x01, 0xFF, 0}, // LANEMODE[2:0] = 2Lane
  {0x3044, 0x00, 0xFF, 0}, // PIX_VST[7:0]
  {0x3045, 0x00, 0xFF, 0}, // PIX_VST[11:8]
  {0x3046, 0x00, 0xFF, 0}, // PIX_VWIDTH[7:0]
  {0x3047, 0x00, 0xFF, 0}, // PIX_VWIDTH[11:8]
  {0x3050, 0x82, 0xFF, 0}, // SHR0[7:0]
  {0x3051, 0x00, 0xFF, 0}, // SHR0[15:8]
  {0x3052, 0x00, 0xFF, 0}, // SHR0[19:16]
  {0x3054, 0x0e, 0xFF, 0}, // SHR1[7:0]
  {0x3055, 0x00, 0xFF, 0}, // SHR1[15:8]
  {0x3056, 0x00, 0xFF, 0}, // SHR1[19:16]
  {0x3058, 0x8a, 0xFF, 0}, // SHR2[7:0]
  {0x3059, 0x01, 0xFF, 0}, // SHR2[15:8]
  {0x305a, 0x00, 0xFF, 0}, // SHR2[19:16]
  {0x3060, 0x16, 0xFF, 0}, // RHS1[7:0]
  {0x3061, 0x01, 0xFF, 0}, // RHS1[15:8]
  {0x3062, 0x00, 0xFF, 0}, // RHS1[19:16]
  {0x3064, 0xc4, 0xFF, 0}, // RHS2[7:0]
  {0x3065, 0x0c, 0xFF, 0}, // RHS2[15:8]
  {0x3066, 0x00, 0xFF, 0}, // RHS2[19:16]
  {0x3069, 0x00, 0xFF, 0}, // CHDR_GAIN_EN = Disable
  {0x3070, 0x00, 0xFF, 0}, // GAIN[7:0]
  {0x3071, 0x00, 0xFF, 0}, // GAIN[10:8]
  {0x3072, 0x00, 0xFF, 0}, // GAIN_1[7:0]
  {0x3073, 0x00, 0xFF, 0}, // GAIN_1[10:8]
  {0x3081, 0x00, 0xFF, 0}, // EXP_GAIN
  {0x308c, 0x00, 0xFF, 0}, // CHDR_DGAIN0_HG[7:0]
  {0x308d, 0x01, 0xFF, 0}, // CHDR_DGAIN0_HG[15:8]
  {0x3094, 0x00, 0xFF, 0}, // CHDR_AGAIN0_LG[7:0]
  {0x3095, 0x00, 0xFF, 0}, // CHDR_AGAIN0_LG[15:8]
  {0x3096, 0x00, 0xFF, 0}, // CHDR_AGAIN1[7:0]
  {0x3097, 0x00, 0xFF, 0}, // CHDR_AGAIN1[15:8]
  {0x309c, 0x00, 0xFF, 0}, // CHDR_AGAIN0_HG[7:0]
  {0x309d, 0x00, 0xFF, 0}, // CHDR_AGAIN0_HG[15:8]
  {0x30a4, 0xba, 0xFF, 0},  
  {0x30a6, 0x03, 0xFF, 0}, // XVS input; XHS output ////////////// 0x03
  {0x30ac, 0x01, 0xFF, 0}, 
  {0x30cc, 0x00, 0xFF, 0}, // XVSLNG[1:0] = 1H
  {0x30cd, 0x00, 0xFF, 0}, // XHSLNG[1:0] = 16clock
  {0x30ce, 0x01, 0xFF, 0}, // EXTMODE = External sync ////////// 0x01
  {0x30dc, 0x32, 0xFF, 0}, // BLKLEVEL[7:0]
  {0x30dd, 0x40, 0xFF, 0}, // BLKLEVEL[11:8]
  {0x3400, 0x01, 0xFF, 0}, // GAIN_PGC_FIDMD
  {0x3444, 0xac, 0xFF, 0}, // Set to 'AC'
  {0x3460, 0x21, 0xFF, 0}, // Normal Mode
  {0x3492, 0x08, 0xFF, 0}, // Set to '08'
  {0x3a50, 0x62, 0xFF, 0}, // Normal Mode AD10bit
  {0x3a51, 0x01, 0xFF, 0}, // Normal Mode AD10bit
  {0x3a52, 0x19, 0xFF, 0}, // AD conversion bit width setting = AD 10bit
  {0x3b00, 0x39, 0xFF, 0}, // Set to '39'
  {0x3b23, 0x2d, 0xFF, 0}, // Set to '2d'
  {0x3b45, 0x04, 0xFF, 0}, // Set to '04'
  {0x3c0a, 0x1f, 0xFF, 0}, // Set to '1f'
  {0x3c0b, 0x1e, 0xFF, 0}, // Set to '1e'
  {0x3c38, 0x21, 0xFF, 0}, // Set to '21'
  {0x3c40, 0x06, 0xFF, 0}, // Normal Mode
  {0x3c44, 0x00, 0xFF, 0}, // Set to '00'
  {0x3cb6, 0xd8, 0xFF, 0}, // Set to 'd8'
  {0x3cc4, 0xda, 0xFF, 0}, // Set to 'da'
  {0x3e24, 0x79, 0xFF, 0}, // Set to '79'
  {0x3e2c, 0x15, 0xFF, 0}, // Set to '15'
  {0x3edc, 0x2d, 0xFF, 0}, // Set to '2d'
  {0x4498, 0x05, 0xFF, 0}, // Set to '05'
  {0x449c, 0x19, 0xFF, 0}, // Set to '19'
  {0x449d, 0x00, 0xFF, 0}, // Set to '00'
  {0x449e, 0x32, 0xFF, 0}, // Set to '32'
  {0x449f, 0x01, 0xFF, 0}, // Set to '01' 
  {0x44a0, 0x92, 0xFF, 0}, // Set to '92'
  {0x44a2, 0x91, 0xFF, 0}, // Set to '91'
  {0x44a4, 0x8c, 0xFF, 0}, // Set to '8c'
  {0x44a6, 0x87, 0xFF, 0}, // Set to '87'
  {0x44a8, 0x82, 0xFF, 0}, // Set to '82'
  {0x44aa, 0x78, 0xFF, 0}, // Set to '78'
  {0x44ac, 0x6e, 0xFF, 0}, // Set to '6e'
  {0x44ae, 0x69, 0xFF, 0}, // Set to '69'
  {0x44b0, 0x92, 0xFF, 0}, // Set to '92'
  {0x44b2, 0x91, 0xFF, 0}, // Set to '91'
  {0x44b4, 0x8c, 0xFF, 0}, // Set to '8c'
  {0x44b6, 0x87, 0xFF, 0}, // Set to '87'
  {0x44b8, 0x82, 0xFF, 0}, // Set to '82'
  {0x44ba, 0x78, 0xFF, 0}, // Set to '78'
  {0x44bc, 0x6e, 0xFF, 0}, // Set to '6e'
  {0x44be, 0x69, 0xFF, 0}, // Set to '69'
  {0x44c0, 0x7f, 0xFF, 0}, // Set to '7f'
  {0x44c1, 0x01, 0xFF, 0}, // Set to '01'
  {0x44c2, 0x7f, 0xFF, 0}, // Set to '7f'
  {0x44c3, 0x01, 0xFF, 0}, // Set to '01'
  {0x44c4, 0x7a, 0xFF, 0}, // Set to '7a'
  {0x44c5, 0x01, 0xFF, 0}, // Set to '01'
  {0x44c6, 0x7a, 0xFF, 0}, // Set to '7a'
  {0x44c7, 0x01, 0xFF, 0}, // Set to '01'
  {0x44c8, 0x70, 0xFF, 0}, // Set to '70'
  {0x44c9, 0x01, 0xFF, 0}, // Set to '01'
  {0x44ca, 0x6b, 0xFF, 0}, // Set to '6b'
  {0x44cb, 0x01, 0xFF, 0}, // Set to '01'
  {0x44cc, 0x6b, 0xFF, 0}, // Set to '6b'
  {0x44cd, 0x01, 0xFF, 0}, // Set to '01'
  {0x44ce, 0x5c, 0xFF, 0}, // Set to '5c'
  {0x44cf, 0x01, 0xFF, 0}, // Set to '01'
  {0x44d0, 0x7f, 0xFF, 0}, // Set to '7f'
  {0x44d1, 0x01, 0xFF, 0}, // Set to '01'
  {0x44d2, 0x7f, 0xFF, 0}, // Set to '7f'
  {0x44d3, 0x01, 0xFF, 0}, // Set to '01'
  {0x44d4, 0x7a, 0xFF, 0}, // Set to '7a'
  {0x44d5, 0x01, 0xFF, 0}, // Set to '01'
  {0x44d6, 0x7a, 0xFF, 0}, // Set to '7a'
  {0x44d7, 0x01, 0xFF, 0}, // Set to '01'
  {0x44d8, 0x70, 0xFF, 0}, // Set to '70'
  {0x44d9, 0x01, 0xFF, 0}, // Set to '01'
  {0x44da, 0x6b, 0xFF, 0}, // Set to '6b'
  {0x44db, 0x01, 0xFF, 0}, // Set to '01'
  {0x44dc, 0x6b, 0xFF, 0}, // Set to '6b'
  {0x44dd, 0x01, 0xFF, 0}, // Set to '01'
  {0x44de, 0x5c, 0xFF, 0}, // Set to '5c'
  {0x44df, 0x01, 0xFF, 0}, // Set to '01'
  {0x4534, 0x1c, 0xFF, 0}, // Set to '1c'
  {0x4535, 0x03, 0xFF, 0}, // Set to '03'
  {0x4538, 0x1c, 0xFF, 0}, // Set to '1c'
  {0x4539, 0x1c, 0xFF, 0}, // Set to '1c'
  {0x453a, 0x1c, 0xFF, 0}, // Set to '1c'
  {0x453b, 0x1c, 0xFF, 0}, // Set to '1c'
  {0x453c, 0x1c, 0xFF, 0}, // Set to '1c'
  {0x453d, 0x1c, 0xFF, 0}, // Set to '1c'
  {0x453e, 0x1c, 0xFF, 0}, // Set to '1c'
  {0x453f, 0x1c, 0xFF, 0}, // Set to '1c'
  {0x4540, 0x1c, 0xFF, 0}, // Set to '1c'
  {0x4541, 0x03, 0xFF, 0}, // Set to '03'
  {0x4542, 0x03, 0xFF, 0}, // Set to '03'
  {0x4543, 0x03, 0xFF, 0}, // Set to '03'
  {0x4544, 0x03, 0xFF, 0}, // Set to '03'
  {0x4545, 0x03, 0xFF, 0}, // Set to '03'
  {0x4546, 0x03, 0xFF, 0}, // Set to '03'
  {0x4547, 0x03, 0xFF, 0}, // Set to '03'
  {0x4548, 0x03, 0xFF, 0}, // Set to '03'
  {0x3001, 0x00, 0xFF, 0}, // REGHOLD
  {0x4549, 0x03, 0xFF, 0}, // Set to '03'
  {IMX662_TABLE_END, 0x00, 0x00, 0} //end config
};

static struct vvcam_mode_info_s imx662_mode_info[] = {
  {
    .index          = 0,
    .size           = {
      .bounds_width  = 1920,
      .bounds_height = 1080,
      .top           = 0,
      .left          = 0,
      .width         = 1920,
      .height        = 1080,
    },
    .hdr_mode       = SENSOR_MODE_LINEAR,
    .bit_width      = 10,
    .data_compress  = {
      .enable = 0,
    },
    .bayer_pattern  = BAYER_RGGB,
    .ae_info = {
      .def_frm_len_lines     = 0x478,
      .curr_frm_len_lines    = 0x478,
      .one_line_exp_time_ns  = 1000,
      .max_integration_line  = 0x2918,
      .min_integration_line  = 1,
      .max_again             = IMX662_MIN_ANALOG_GAIN * 1024,
      .min_again             = IMX662_MIN_ANALOG_GAIN * 1024,
      .max_dgain             = IMX662_MAX_GAIN    * 1024,
      .min_dgain             = IMX662_MIN_GAIN    * 1024,
      .start_exposure        = 5000 * 1024,
      .cur_fps               = 20   * 1024,
      .max_fps               = 40   * 1024,
      .min_fps               = 10   * 1024,
      .min_afps              = 1   * 1024,
      .int_update_delay_frm  = 1,
      .gain_update_delay_frm = 1,
    },
    .mipi_info = {
      .mipi_lane = 2,},
    .preg_data      = imx662_init_setting,
    .reg_data_count = ARRAY_SIZE(imx662_init_setting),
  },
  // Config for being able to choose 2 different xml calib files.
  {
    .index          = 1,
    .size           = {
      .bounds_width  = 1920,
      .bounds_height = 1080,
      .top           = 0,
      .left          = 0,
      .width         = 1920,
      .height        = 1080,
    },
    .hdr_mode       = SENSOR_MODE_LINEAR,
    .bit_width      = 10,
    .data_compress  = {
      .enable = 0,
    },
    .bayer_pattern  = BAYER_RGGB,
    .ae_info = {
      .def_frm_len_lines     = 0x478,
      .curr_frm_len_lines    = 0x478,
      .one_line_exp_time_ns  = 1000,
      .max_integration_line  = 0x2918,
      .min_integration_line  = 1,
      .max_again             = IMX662_MIN_ANALOG_GAIN * 1024,
      .min_again             = IMX662_MIN_ANALOG_GAIN    * 1024,
      .max_dgain             = IMX662_MAX_GAIN    * 1024,
      .min_dgain             = IMX662_MIN_GAIN    * 1024,
      .start_exposure        = 5000 * 1024,
      .cur_fps               = 20   * 1024,
      .max_fps               = 40   * 1024,
      .min_fps               = 10   * 1024,
      .min_afps              = 1   * 1024,
      .int_update_delay_frm  = 1,
      .gain_update_delay_frm = 1,
    },
    .mipi_info = {
      .mipi_lane = 2,},
    .preg_data      = imx662_init_setting,
    .reg_data_count = ARRAY_SIZE(imx662_init_setting),
  },
  {
    .index          = 2,
    .size           = {
      .bounds_width  = 1400,
      .bounds_height = 1024,
      .top           = 0,
      .left          = 0,
      .width         = 1400,
      .height        = 1024,
    },
    .hdr_mode       = SENSOR_MODE_LINEAR,
    .bit_width      = 10,
    .data_compress  = {
      .enable = 0,
    },
    .bayer_pattern  = BAYER_RGGB,
    .ae_info = {
      .def_frm_len_lines     = 0x478,
      .curr_frm_len_lines    = 0x478,
      .one_line_exp_time_ns  = 1000,
      .max_integration_line  = 0x2918,
      .min_integration_line  = 1,
      .max_again             = IMX662_MIN_ANALOG_GAIN * 1024,
      .min_again             = IMX662_MIN_ANALOG_GAIN * 1024,
      .max_dgain             = IMX662_MAX_GAIN    * 1024,
      .min_dgain             = IMX662_MIN_GAIN    * 1024,
      .start_exposure        = 5000 * 1024,
      .cur_fps               = 20   * 1024,
      .max_fps               = 40   * 1024,
      .min_fps               = 10   * 1024,
      .min_afps              = 1   * 1024,
      .int_update_delay_frm  = 1,
      .gain_update_delay_frm = 1,
    },
    .mipi_info = {
      .mipi_lane = 2,},
    .preg_data      = imx662_init_setting,
    .reg_data_count = ARRAY_SIZE(imx662_init_setting),
  },
};

static inline struct v4l2_subdev *ctrl_to_sd(struct v4l2_ctrl *ctrl)
{
  return &container_of(ctrl->handler,
      struct stimx662, ctrls.handler)->sd;
}

static inline struct stimx662 *to_imx662(struct v4l2_subdev *sd)
{
  return container_of(sd, struct stimx662, sd);
}

static inline int imx662_read_reg(struct stimx662 *priv, u16 addr, u8 *val)
{
  int err;
  unsigned int temp;

  err = regmap_read(priv->regmap, addr, &temp);
  if (err) {
    dev_err(&priv->client->dev,
        "%s : i2c read failed, addr = %x\n", __func__, addr);
  } else {
    *val = temp;
    dev_dbg(&priv->client->dev,
        "%s : addr 0x%x, val=0x%x\n", __func__,
        addr, *val);
  }
  return err;
}

static int imx662_write_reg(struct stimx662 *imx662, u16 reg, u8 val)
{
  struct device *dev = &imx662->client->dev;
  u8 au8Buf[3] = { 0 };

  au8Buf[0] = reg >> 8;
  au8Buf[1] = reg & 0xff;
  au8Buf[2] = (val & 0xff);

  if (i2c_master_send(imx662->client, au8Buf, 3) < 0) {
    dev_err(dev, "Write reg error: reg=%x, val=%x\n", reg, val);
    return -1;
  }
  return 0;
}

static int imx662_get_clk(struct stimx662 *imx662, void *clk)
{
  struct vvcam_clk_s vvcam_clk;
  int ret = 0;
  vvcam_clk.sensor_mclk = 74250000;
  vvcam_clk.csi_max_pixel_clk = imx662->ocp.max_pixel_frequency;
  ret = copy_to_user(clk, &vvcam_clk, sizeof(struct vvcam_clk_s));
  if (ret != 0)
    ret = -EINVAL;
  return ret;
}

static int imx662_get_exposure(struct stimx662 *imx662, u32 *exp)
{
  *(exp) = imx662->ctrls.exposure->cur.val; 
  return 0;
}

static int imx662_calc_h_interval(int fps, int n_lines)
{
  int new_h_interval;

  new_h_interval = (SECS_TO_NSECS / fps) / n_lines;

  return new_h_interval;
}

static int imx662_calc_flash_time(int fps)
{
  int flash_time;

  /*flash_time = ((SECS_TO_NSECS / fps)) / 3;*/
  flash_time = ((SECS_TO_NSECS / fps) / 10000) * 3225;

  return flash_time;
}

static int imx662_microseconds_to_intervals(struct stimx662 *priv, int exposure_val, int n_lines)
{
  int exposure_val_nanosec = exposure_val * MICROSECS_TO_NSECS;
  int shs;
  int fps = priv->cur_mode.ae_info.cur_fps / 1024; 
  int h_interval = imx662_calc_h_interval(fps, n_lines);

  int h_intervals = (exposure_val_nanosec - T_EXP_ERROR) / h_interval;
  shs = n_lines - h_intervals;

  if(shs > (n_lines -1)) {
    shs = (n_lines - 1);
  } else if(shs < MIN_SHS) {
    shs = MIN_SHS;
  }
  printk("%s: fps=%d, h_interval=%d; h_intervals=%d; n_lines=%d, shs=%d\n", __func__, fps, h_interval, h_intervals, n_lines, shs);
  return shs;
}

static int imx662_set_flash_time(struct stimx662 *imx662)
{
  int ret =  0;
  u8 reg_val[3];
  uint32_t pulse_up, pulse_dn;
  uint32_t n_lines = 3400;
  int h_interval;
  uint32_t shs_val;
  uint32_t n_h_intervals;
  int fps = imx662->cur_mode.ae_info.cur_fps / 1024; 

  h_interval = imx662_calc_h_interval(fps, n_lines);
  n_h_intervals = imx662_calc_flash_time(fps) / h_interval;
  ret = imx662_read_reg(imx662, SHR0_LOW_REG, &reg_val[0]);
  if (ret != 0)
    return -1;
  
  ret = imx662_read_reg(imx662, SHR0_MID_REG, &reg_val[1]);
  if (ret != 0)
    return -1;

  ret = imx662_read_reg(imx662, SHR0_HIGH_REG, &reg_val[2]);
  if (ret != 0)
    return -1;
  shs_val = reg_val[0] | (reg_val[1] << 8) | (reg_val[2] << 16);
  pulse_up = shs_val + (uint32_t)COMMUNICATION_PERIOD;
  pulse_dn = shs_val + (uint32_t)COMMUNICATION_PERIOD + n_h_intervals;

  reg_val[0] = (pulse_up & 0xff);
  reg_val[1] = (pulse_up >> 8) & 0xff;
  reg_val[2] = (pulse_up >> 16) & 0xff;
  ret = imx662_write_reg(imx662, PULSE1_UP_LOW_REG, reg_val[0]);
  if (ret != 0)
    return -1;
  ret = imx662_write_reg(imx662, PULSE1_UP_MID_REG, reg_val[1]);
  if (ret != 0)
    return -1;
  ret = imx662_write_reg(imx662, PULSE1_UP_HIGH_REG, reg_val[2]);
  if (ret != 0)
    return -1;

  reg_val[0] = (pulse_dn & 0xff);
  reg_val[1] = (pulse_dn >> 8) & 0xff;
  reg_val[2] = (pulse_dn >> 16) & 0xff;
  ret = imx662_write_reg(imx662, PULSE1_DN_LOW_REG, reg_val[0]);
  if (ret != 0)
    return -1;
  ret = imx662_write_reg(imx662, PULSE1_DN_MID_REG, reg_val[1]);
  if (ret != 0)
    return -1;
  ret = imx662_write_reg(imx662, PULSE1_DN_HIGH_REG, reg_val[2]);
  if (ret != 0)
    return -1;
  printk("%s: h_interval: %d, n_h_intervals: %d, shs_val: %d, pulse_up: %d, pulse_dn: %d, fps: %d, n_lines: %d\n", __func__, h_interval, n_h_intervals, shs_val, pulse_up, pulse_dn, fps, n_lines);

  return 0;
}

static int imx662_set_exposure(struct stimx662 *imx662, u32 new_exp)
{
  int ret =  0;
  u16 h_intervals;
  int n_lines = 3400;
  u8 aux = 0;

#ifdef DEBUG
  printk("%s: Trying exposure value: %d \n", __func__, new_exp);
#endif	

  if(new_exp > imx662->cur_mode.ae_info.max_integration_line) 
  {
    printk("%s: ERROR. Exposure value out of range, setting exposure to maximum (%d)\n",__func__, imx662->cur_mode.ae_info.max_integration_line);
    new_exp = imx662->cur_mode.ae_info.max_integration_line;
  }        

  if (new_exp < imx662->cur_mode.ae_info.min_integration_line)
  {
    printk("%s: ERROR. Exposure value out of range, setting exposure to minimum (%d)\n",__func__, imx662->cur_mode.ae_info.min_integration_line);
    new_exp = imx662->cur_mode.ae_info.min_integration_line;
  }

  h_intervals = imx662_microseconds_to_intervals(imx662, new_exp, n_lines);

  aux = h_intervals & 0xff;
  ret = imx662_write_reg(imx662, SHR0_LOW_REG, aux);
  aux = (h_intervals >> 8) & 0xff;
  ret = imx662_write_reg(imx662, SHR0_MID_REG, aux);
  aux = (h_intervals >> 16) & 0xff;
  ret = imx662_write_reg(imx662, SHR0_HIGH_REG, aux);

  if((ret >= 0))
  {
    imx662->ctrls.exposure->val = new_exp;
    imx662->ctrls.exposure->cur.val = new_exp;
  }
  else
  {
    printk("%s: ERROR. Value failed to write value\n",__func__);
  }

  imx662_set_flash_time(imx662);
  return ret;
}

static int imx662_get_digital_gain(struct stimx662 *priv, int *val)
{
  *(val) = priv->ctrls.gain->cur.val;
  return 0;
}

static int imx662_set_digital_gain(struct stimx662 *priv, int val)
{
  u8 aux = 0;
  int res  = 0;

  /* NOTE: values coming from AE control are multiplied by 1024 */
  u32 gain = (val / 1024) - 150;

#ifdef DEBUG
  printk("%s: Trying to set %d gain %d\n", __func__, gain, val);
#endif
  
  if (gain > priv->ctrls.max_gain->cur.val)
  {
    gain = priv->ctrls.max_gain->cur.val;
    res = IMX662_OUT_RANGE;
  }

  if (gain < priv->ctrls.min_gain->cur.val)
  {
    gain = priv->ctrls.min_gain->cur.val;
    res = IMX662_OUT_RANGE;
  }

  aux = (gain & 0xFF);
  res = imx662_write_reg(priv, DIGITAL_GAIN_LOW_REG, aux);
  aux = (gain >> 8) & 0x07;
  res = imx662_write_reg(priv, DIGITAL_GAIN_HIGH_REG, aux);


  if (res <= -1)
  {
    printk("%s: ERROR. Value failed to write value\n", __func__);
  }

  if (res >= 0)
  {
    priv->ctrls.gain->val = gain;
    priv->ctrls.gain->cur.val = gain;
  }
  return res;
}

static int imx662_set_max_exposure(struct stimx662 *imx662, int in_val)
{
  int res = 0;
  int val = in_val;

#ifdef DEBUG
  printk("%s: Trying to set %d max exposure\n", __func__, val);
#endif

  if((val >= IMX662_MIN_EXPOSURE_TIME) && (val <= IMX662_MAX_EXPOSURE_TIME))
  {
    imx662->ctrls.max_exposure->val = val;
    imx662->ctrls.max_exposure->cur.val = val;
    imx662->cur_mode.ae_info.max_integration_line = val;
#ifdef DEBUG
    printk("%s: Set to %d \n", __func__, imx662->cur_mode.ae_info.max_integration_line);
#endif
  } 
  else
  {
    printk("%s: ERROR. Value failed to write set new value %d\n", __func__, val);
  }
  return res;
}

static int imx662_set_min_exposure(struct stimx662 *imx662, int in_val)
{
  int res = 0;
  int val = in_val;

#ifdef DEBUG
  printk("%s: Trying to set %d min exposure\n", __func__, val);
#endif

  if((val >= IMX662_MIN_EXPOSURE_TIME) && (val <= IMX662_MAX_EXPOSURE_TIME))
  {
    imx662->ctrls.min_exposure->val = val;
    imx662->ctrls.min_exposure->cur.val = val;
    imx662->cur_mode.ae_info.min_integration_line = val;
#ifdef DEBUG
    printk("%s: Set to %d \n", __func__, imx662->cur_mode.ae_info.min_integration_line);
#endif
  } 
  else
  {
    printk("%s: ERROR. Value failed to write set new value %d\n", __func__, val);
  }
  return res;
}

static int imx662_set_max_gain(struct stimx662 *imx662, int val)
{
  int res = 0;

#ifdef DEBUG
  printk("%s: Trying to set %d max gain\n", __func__, val);
#endif

  if((val >= IMX662_MIN_GAIN) && (val <= IMX662_MAX_GAIN))
  {
    val = val - 150;
    imx662->ctrls.max_gain->val = val;
    imx662->ctrls.max_gain->cur.val = val;
    imx662->cur_mode.ae_info.max_dgain = val * 1024;
#ifdef DEBUG
    printk("%s: Set to %d \n", __func__, imx662->cur_mode.ae_info.max_dgain);
#endif
  } 
  else
  {
    printk("%s: ERROR. Value failed to write set new value %d\n", __func__, val);
  }
  return res;
}

static int imx662_set_min_gain(struct stimx662 *imx662, int val)
{
  int res = 0;

#ifdef DEBUG
  printk("%s: Trying to set %d min gain\n", __func__, val);
#endif

  if((val >= IMX662_MIN_GAIN) && (val <= IMX662_MAX_GAIN))
  {
    val = val - 150;
    imx662->ctrls.min_gain->val = val;
    imx662->ctrls.min_gain->cur.val = val;
    imx662->cur_mode.ae_info.min_dgain = val * 1024;
#ifdef DEBUG
    printk("%s: Set to %d \n", __func__, imx662->cur_mode.ae_info.min_dgain);
#endif
  } 
  else
  {
    printk("%s: ERROR. Value failed to write set new value %d\n", __func__, val);
  }
  return res;
}

static int imx662_s_ctrl(struct v4l2_ctrl *ctrl)
{
  struct v4l2_subdev *sd = ctrl_to_sd(ctrl);                                                   
  struct stimx662 *imx662 = to_imx662(sd);
  int ret = -EINVAL;

  switch (ctrl->id) {
    case V4L2_CID_GAIN:
      ret = imx662_set_digital_gain(imx662, ctrl->val);
      break;
    case V4L2_CID_EXPOSURE:
      ret = imx662_set_exposure(imx662, ctrl->val);
      break;
    case V4L2_CID_FLASH_TIME:
      break;
    case V4L2_CID_MAX_GAIN:                                                           
      ret = imx662_set_max_gain(imx662, ctrl->val);                             
      break;
    case V4L2_CID_MIN_GAIN:                                                           
      ret = imx662_set_min_gain(imx662, ctrl->val);                             
      break;
    case V4L2_CID_MAX_EXPOSURE:
      ret = imx662_set_max_exposure(imx662, ctrl->val);
      break;
    case V4L2_CID_MIN_EXPOSURE:
      ret = imx662_set_min_exposure(imx662, ctrl->val);
      break;
  }

  return 0; 
}
   
static int imx662_get_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_state *state, struct v4l2_subdev_format *format) 
{ 
  struct stimx662 *imx662 = to_imx662(sd); 
  mutex_lock(&imx662->lock);
  format->format = imx662->format; 
  mutex_unlock(&imx662->lock);
  return 0; 
}

static int imx662_query_supports(struct stimx662 *imx662, void* parry)
{
  struct vvcam_mode_info_array_s *psensor_mode_arry = parry;
  struct vvcam_mode_info_s mode_info_aux[3];

  memcpy((void *)&mode_info_aux,(void *) &imx662_mode_info, sizeof(imx662_mode_info));

  mode_info_aux[0].ae_info.max_dgain = imx662->ctrls.max_gain->cur.val * 1024;
  mode_info_aux[0].ae_info.min_dgain = imx662->ctrls.min_gain->cur.val * 1024;
  mode_info_aux[1].ae_info.max_dgain = imx662->ctrls.max_gain->cur.val * 1024;
  mode_info_aux[1].ae_info.min_dgain = imx662->ctrls.min_gain->cur.val * 1024;
  mode_info_aux[0].ae_info.max_integration_line = imx662->ctrls.max_exposure->cur.val;
  mode_info_aux[0].ae_info.min_integration_line = imx662->ctrls.min_exposure->cur.val;
  mode_info_aux[1].ae_info.max_integration_line = imx662->ctrls.max_exposure->cur.val;
  mode_info_aux[1].ae_info.min_integration_line = imx662->ctrls.min_exposure->cur.val;

  psensor_mode_arry->count = ARRAY_SIZE(imx662_mode_info);
  memcpy((void *)&psensor_mode_arry->modes, (void *)mode_info_aux, sizeof(imx662_mode_info));
  return 0;
}

static int imx662_write_table(struct stimx662 *priv, const struct reg_8 table[])
{
  int ret = 0;
  struct regmap *regmap = priv->regmap;
  const struct reg_8 *reg;
  unsigned int read_reg = 0;

  for(reg = table;;reg++)
  {
    dev_dbg(&priv->client->dev, "%s : i2c table write, addr = 0x%02x value=0x%04x\n", __func__, reg->addr, reg->val);
    if(reg->addr != IMX662_TABLE_END)
    {
      ret = regmap_write_bits(regmap, reg->addr, reg->mask << reg->offset, reg->val << reg->offset);
    }
    else
    {
      break;
    }

    ret = regmap_read(priv->regmap, reg->addr, &read_reg);
    dev_dbg(&priv->client->dev, "%s : i2c table write, addr = 0x%02x value=0x%04x\n", __func__, reg->addr, read_reg);
    if (ret)
    {
      dev_err(&priv->client->dev, "%s : i2c table write failed, addr = 0x%02x\n", __func__, reg->addr);
      return ret;
    }
  }

  return ret;
}

static int imx662_set_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_state *state, struct v4l2_subdev_format *format) 
{ 
  struct v4l2_mbus_framefmt *fmt = &format->format; 
  struct stimx662 *imx662 = to_imx662(sd); 

  mutex_lock(&imx662->lock);

  imx662->cur_mode.size.bounds_width = fmt->width;
  imx662->cur_mode.size.bounds_height = fmt->height;
  imx662->cur_mode.size.width = fmt->width;
  imx662->cur_mode.size.height = fmt->height;
  fmt->field = V4L2_FIELD_NONE; 
  imx662->format = *fmt;
  mutex_unlock(&imx662->lock);

  return 0;
}

static int imx662_query_capabilities(struct stimx662 *imx662, void *arg)
{
  struct v4l2_capability *pcap = (struct v4l2_capability *)arg;

  strcpy((char *)pcap->driver, "imx662");
  sprintf((char *)pcap->bus_info, "csi%d", imx662->csi_id);

#ifdef DEBUG
  printk("%s: csi%d\n", __func__, imx662->csi_id);
#endif

  if(imx662->client->adapter) {
    pcap->bus_info[VVCAM_CAP_BUS_INFO_I2C_ADAPTER_NR_POS] =
      (__u8)imx662->client->adapter->nr;
  } else {
    pcap->bus_info[VVCAM_CAP_BUS_INFO_I2C_ADAPTER_NR_POS] = 0xFF;
  }

  return 0;
}

static int imx662_get_sensor_mode(struct stimx662 *imx662, void* pmode)
{
  int ret = 0;
#ifdef DEBUG
  printk("%s: csi%d\n", __func__, imx662->csi_id);
#endif
  imx662->cur_mode.ae_info.max_dgain = imx662->ctrls.max_gain->cur.val * 1024;
  imx662->cur_mode.ae_info.min_dgain = imx662->ctrls.min_gain->cur.val * 1024;
  imx662->cur_mode.ae_info.max_integration_line = imx662->ctrls.max_exposure->cur.val;
  imx662->cur_mode.ae_info.min_integration_line = imx662->ctrls.min_exposure->cur.val;
#ifdef DEBUG
  printk("%s: max_dgain = %d\n", __func__, imx662->cur_mode.ae_info.max_dgain);
  printk("%s: min_dgain = %d\n", __func__, imx662->cur_mode.ae_info.min_dgain);
  printk("%s: max_exposure = %d\n", __func__, imx662->cur_mode.ae_info.max_integration_line);
  printk("%s: min_exposure = %d\n", __func__, imx662->cur_mode.ae_info.min_integration_line);
#endif
  ret = copy_to_user(pmode, &imx662->cur_mode,
      sizeof(struct vvcam_mode_info_s));

  if (ret != 0)
    ret = -ENOMEM;
  return ret;
}

static int imx662_set_sensor_mode(struct stimx662 *imx662, void* pmode)
{
  int ret = 0;
  int i = 0;
  struct vvcam_mode_info_s sensor_mode;
  int width = imx662->cur_mode.size.width; 
  int height = imx662->cur_mode.size.height; 

#ifdef DEBUG
  printk("%s: csi%d\n", __func__, imx662->csi_id);
#endif
  ret = copy_from_user(&sensor_mode, pmode,
      sizeof(struct vvcam_mode_info_s));
  if (ret != 0)
    return -ENOMEM;
  for (i = 0; i < ARRAY_SIZE(imx662_mode_info); i++) {
    if (imx662_mode_info[i].index == sensor_mode.index) {
      memcpy(&imx662->cur_mode, &imx662_mode_info[i],
          sizeof(struct vvcam_mode_info_s));
      imx662->cur_mode.ae_info.max_dgain = imx662->ctrls.max_gain->cur.val * 1024;
      imx662->cur_mode.ae_info.min_dgain = imx662->ctrls.min_gain->cur.val * 1024;
      imx662->cur_mode.ae_info.max_integration_line = imx662->ctrls.max_exposure->cur.val;
      imx662->cur_mode.ae_info.min_integration_line = imx662->ctrls.min_exposure->cur.val;
      imx662->cur_mode.size.width = width;
      imx662->cur_mode.size.height = height;
      imx662->cur_mode.size.bounds_width = width;
      imx662->cur_mode.size.bounds_height = height;
      return 0;
    }
  }

  return -ENXIO;
}

static int imx662_get_sensor_id(struct stimx662 *imx662, u16 *pchipid)
{
  int ret = 0;
  u16 chip_id = 0xa5a5;

  ret = copy_to_user(pchipid, &chip_id, sizeof(u16));
  if (ret != 0)
    ret = -ENOMEM;
  return ret;
}

static int imx662_get_reserve_id(struct stimx662 *imx662, void* preserve_id)
{
  int ret = 0;
  u16 reserve_id = 0xa5a5;
  ret = copy_to_user(preserve_id, &reserve_id, sizeof(u16));
  if (ret != 0)
  {
    ret = -ENOMEM;
  }
  return ret;
}

static int imx662_get_fps(struct stimx662 *imx662, u32 *pfps)
{
  *pfps = imx662->cur_mode.ae_info.cur_fps;
  return 0;
}

static int imx662_set_fps(struct stimx662 *imx662, u32 fps)
{
  int ret = 0;
  uint32_t real_fps = 0;
#if MASTER_MODE == 1
  u32 reg_val = 0;
  u32 vmax = 3400;
  u8 aux = 0;
#endif

  printk("%s: FPS: %d\n", __func__, fps);
  if (fps > imx662->cur_mode.ae_info.max_fps) 
  {
    fps = imx662->cur_mode.ae_info.max_fps;
  }
  else if( fps < imx662->cur_mode.ae_info.min_fps)
  {
    fps = imx662->cur_mode.ae_info.min_fps;
  }

  imx662->cur_mode.ae_info.cur_fps = fps;
  real_fps = fps / 1024;

#if MASTER_MODE == 1
  // Computing of value to be written in sensor register
  reg_val = (IMX662_PIX_CLK / (vmax * real_fps));

#ifdef DEBUG
  printk("%s: Try to set %d fps\n", __func__, real_fps);
#endif

  aux = reg_val & 0xFF;
  ret = imx662_write_reg(imx662, HMAX_LOW_REG, aux);
  aux = (reg_val >> 8) & 0xFF;
  ret = imx662_write_reg(imx662, HMAX_HIGH_REG, aux);
  if (ret != 0)
  {
    printk("%s: Failed to write new frame rate value (%d fps) \n", __func__, real_fps);
    return -1;
  }

#endif
  /*// Adjusting exposure time (in microseconds) registers to new frame rate*/
  imx662_set_exposure(imx662, imx662->ctrls.exposure->cur.val);

  return ret;
}

static int imx662_get_lens(struct stimx662 *sensor, void * arg) 
{
  vvcam_lens_t *pfocus_lens = (vvcam_lens_t *)arg;
  printk("%s\n", __func__);

  if (!arg)
    return -ENOMEM;

  if (strlen(sensor->focus_lens.name) == 0)
    return -1;

  printk("%s: returning data\n", __func__);
  return copy_to_user(pfocus_lens, &sensor->focus_lens, sizeof(vvcam_lens_t));
}

static long imx662_priv_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg_user)
{
  struct stimx662 *imx662 = to_imx662(sd);
  long ret = 0;
  struct vvcam_sccb_data_s sensor_reg;
  void *arg = arg_user;

#ifdef DEBUG
  printk("%s cmd = 0x%x\n", __func__, cmd);
#endif	

  mutex_lock(&imx662->lock);
  switch(cmd)
  {
    case VVSENSORIOC_S_POWER:
      ret = 0;
      break;
    case VVSENSORIOC_S_CLK:
      ret = 0;
      break;
    case VVSENSORIOC_G_CLK:
      ret = imx662_get_clk(imx662, arg);
      break;
    case VVSENSORIOC_RESET:
      ret = 0;
      break;
    case VIDIOC_G_FMT:
      break;
    case VIDIOC_S_FMT:
      break;
    case VIDIOC_QUERYCAP:
      ret = imx662_query_capabilities(imx662, arg);
      break;
    case VVSENSORIOC_QUERY:
      USER_TO_KERNEL(struct vvcam_mode_info_array_s);
      ret = imx662_query_supports(imx662, arg);
      KERNEL_TO_USER(struct vvcam_mode_info_array_s);
      break;
    case VVSENSORIOC_WRITE_REG:
      ret = copy_from_user(&sensor_reg, arg,
          sizeof(struct vvcam_sccb_data_s));
      break;
    case VVSENSORIOC_READ_REG:
      ret = copy_from_user(&sensor_reg, arg,
          sizeof(struct vvcam_sccb_data_s));
      ret |= imx662_read_reg(imx662, sensor_reg.addr,
          (u8*)&sensor_reg.data);
      ret |= copy_to_user(arg, &sensor_reg,
          sizeof(struct vvcam_sccb_data_s));
      break;
    case VVSENSORIOC_G_CHIP_ID:
      ret = imx662_get_sensor_id(imx662, arg);
      break;
    case VVSENSORIOC_G_RESERVE_ID:
      ret = imx662_get_reserve_id(imx662, arg);
      break;
    case VVSENSORIOC_G_SENSOR_MODE:
      ret = imx662_get_sensor_mode(imx662, arg);
      break;
    case VVSENSORIOC_S_SENSOR_MODE:
      ret = imx662_set_sensor_mode(imx662, arg);
      break;
    case VVSENSORIOC_S_GAIN: 
      USER_TO_KERNEL(int);
      ret = imx662_set_digital_gain(imx662, *(int *)arg);
      ret = 0;
      break;
    case VVSENSORIOC_G_GAIN:
      ret = imx662_get_digital_gain(imx662, arg);
      break;
    case VVSENSORIOC_S_VSGAIN:
      ret = 0;
      break;
    case VVSENSORIOC_S_EXP:
      USER_TO_KERNEL(u32); 
      ret = imx662_set_exposure(imx662, *(int *)arg);
      ret = 0;
      break;
    case VVSENSORIOC_G_EXP:
      ret = imx662_get_exposure(imx662, arg);
      break;
    case VVSENSORIOC_S_VSEXP:
      ret = 0;
      break;
    case VVSENSORIOC_S_FPS:	
      USER_TO_KERNEL(int);
      ret = imx662_set_fps(imx662, *(int *)arg); 
      break;
    case VVSENSORIOC_G_FPS:
      ret = imx662_get_fps(imx662, arg);
      break;
    case VVSENSORIOC_S_STREAM:
      USER_TO_KERNEL(int);
      ret = imx662_s_stream(sd, *(int *)arg);
      break;
    case VVSENSORIOC_S_LONG_EXP:
      ret = 0;
      break;
    case VVSENSORIOC_S_LONG_GAIN:
      ret = 0;
      break;
    case VVSENSORIOC_S_HDR_RADIO:
      ret = 0;
      break;
    case VVSENSORIOC_S_BLC:
      ret = 0;
      break;
    case VVSENSORIOC_S_WB:
      ret = 0;
      break;
    case VVSENSORIOC_G_EXPAND_CURVE:
      ret = 0;
      break;
    case VVSENSORIOC_S_TEST_PATTERN:
      ret = 0;
      break;
    case VVSENSORIOC_S_MAX_GAIN:
      USER_TO_KERNEL(int);
      ret = imx662_set_max_gain(imx662, *(int *)arg);
      break;
    case VVSENSORIOC_S_MIN_GAIN:
      USER_TO_KERNEL(int);
      ret = imx662_set_min_gain(imx662, *(int *)arg);
      break;
    case VVSENSORIOC_S_MAX_INT_TIME:
      USER_TO_KERNEL(int);
      ret = imx662_set_max_exposure(imx662, *(int *)arg);
      break;
    case VVSENSORIOC_S_MIN_INT_TIME:
      USER_TO_KERNEL(int);
      ret = imx662_set_min_exposure(imx662, *(int *)arg);
      break;
    case VVSENSORIOC_G_LENS:
      ret = imx662_get_lens(imx662, arg);
      break;
    default:
      ret = -EINVAL;
      break;
  }

  mutex_unlock(&imx662->lock);
  return ret;
}

static int imx662_s_stream(struct v4l2_subdev *sd, int on) 
{
  struct stimx662 *imx662 = to_imx662(sd); 
  u8 reg = 0x00;
  int ret = 0; 

#ifdef DEBUG
  printk("%s : %s\n", __func__, on ? "Stream Start" : "Stream Stop");
#endif

  if(on) {
    /* start stream */
    ret = imx662_write_reg(imx662, STANDBY_REG, reg | ((0x00 << STANDBY_OFFSET) & STANDBY_MASK));
    if(ret){
      goto fail;
    }
#if MASTER_MODE == 1
    msleep(25);
    ret = imx662_write_reg(imx662, XMSTA_REG, 0x00);
#endif
    msleep(50); 
  }	
  else{
    ret = imx662_write_reg(imx662, STANDBY_REG, reg | ((0x01 << STANDBY_OFFSET) & STANDBY_MASK));
    if(ret){
      goto fail;
    }
#if MASTER_MODE == 1
    ret = imx662_write_reg(imx662, XMSTA_REG, 0x01);
#endif
  }
  return 0;

fail:
  return ret;
}

static int imx662_power_on(struct stimx662 *imx662)
{
  int ret = 0;

#ifdef DEBUG
  printk("%s: enter \n", __func__);
#endif

  ret = clk_prepare_enable(imx662->sensor_clk);

  return ret;
}

static int imx662_power_off(struct stimx662 *imx662)
{
#ifdef DEBUG
  printk("enter %s\n", __func__);
#endif
  clk_disable_unprepare(imx662->sensor_clk);

  return 0;
}

static int imx662_s_power(struct v4l2_subdev *sd, int on)
{
  struct stimx662 *imx662 = to_imx662(sd);
#ifdef DEBUG
  printk("%s: setting power to: %d\n",__func__, on);
#endif

  if(on)
    imx662_power_on(imx662);
  else
    imx662_power_off(imx662);

  return 0;
}

static int imx662_reset(struct stimx662 *imx662)
{
#ifdef DEBUG
  printk("enter %s\n",__func__);
#endif

  if (!gpio_is_valid(imx662->rst_gpio))
    return -1;

  gpio_set_value_cansleep(imx662->rst_gpio, 0);
  msleep(20);

  gpio_set_value_cansleep(imx662->rst_gpio, 1);
  msleep(20);

  return 0;
}

static int imx662_link_setup(struct media_entity *entity,
		struct media_pad const *local,
		struct media_pad const *remote,
		u32 flags)
{
  return 0;
}

static int imx662_get_format_code(struct stimx662 *imx662, u32 *code)
{
#ifdef DEBUG
  printk("%s\n", __func__);
#endif

  switch (imx662->cur_mode.bayer_pattern) {
    case BAYER_RGGB:
      if (imx662->cur_mode.bit_width == 8) {
        *code = MEDIA_BUS_FMT_SRGGB8_1X8;
      } else if (imx662->cur_mode.bit_width == 10) {
        *code = MEDIA_BUS_FMT_SRGGB10_1X10;
      } else {
        *code = MEDIA_BUS_FMT_SRGGB12_1X12;
      }
      break;
    case BAYER_GRBG:
      if (imx662->cur_mode.bit_width == 8) {
        *code = MEDIA_BUS_FMT_SGRBG8_1X8;
      } else if (imx662->cur_mode.bit_width == 10) {
        *code = MEDIA_BUS_FMT_SGRBG10_1X10;
      } else {
        *code = MEDIA_BUS_FMT_SGRBG12_1X12;
      }
      break;
    case BAYER_GBRG:
      if (imx662->cur_mode.bit_width == 8) {
        *code = MEDIA_BUS_FMT_SGBRG8_1X8;
      } else if (imx662->cur_mode.bit_width == 10) {
        *code = MEDIA_BUS_FMT_SGBRG10_1X10;
      } else {
        *code = MEDIA_BUS_FMT_SGBRG12_1X12;
      }
      break;
    case BAYER_BGGR:
      if (imx662->cur_mode.bit_width == 8) {
        *code = MEDIA_BUS_FMT_SBGGR8_1X8;
      } else if (imx662->cur_mode.bit_width == 10) {
        *code = MEDIA_BUS_FMT_SBGGR10_1X10;
      } else {
        *code = MEDIA_BUS_FMT_SBGGR12_1X12;
      }
      break;
    default:
      /*nothing need to do*/
      break;
  }
  return 0;
}

/*static int imx662_s_frame_interval(struct v4l2_subdev *sd, struct v4l2_subdev_frame_interval *fi)*/
/*{*/
/*struct stimx662 *imx662 = to_imx662(sd);*/
/*int ret;*/

/*ret = imx662_set_fps(imx662, (u32) (fi->interval.denominator/fi->interval.numerator));*/

/*return 0;*/
/*}*/

/*static int imx662_g_frame_interval(struct v4l2_subdev *sd, struct v4l2_subdev_frame_interval *fi)*/
/*{*/
/*struct stimx662 *imx662 = to_imx662(sd);*/
/*fi->interval = imx662->frame_interval;*/
/*return 0;*/
/*}*/

#if LINUX_VERSION_CODE > KERNEL_VERSION(5, 12, 0)
static int imx662_enum_mbus_code(struct v4l2_subdev *sd,
		struct v4l2_subdev_state *state,
		struct v4l2_subdev_mbus_code_enum *code)
#else
static int imx662_enum_mbus_code(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_mbus_code_enum *code)
#endif
{
  struct stimx662 *sensor = to_imx662(sd);

  u32 cur_code = MEDIA_BUS_FMT_SGRBG10_1X10;

#ifdef DEBUG
  printk("%s\n", __func__);
#endif
  imx662_get_format_code(sensor, &cur_code);
  code->code = cur_code;

  return 0;
}

static const struct v4l2_subdev_pad_ops imx662_subdev_pad_ops = {
  .enum_mbus_code = imx662_enum_mbus_code,
  .get_fmt = imx662_get_fmt,
  .set_fmt = imx662_set_fmt,
};

static const struct v4l2_subdev_video_ops imx662_subdev_video_ops = {
  .s_stream = imx662_s_stream,
  /*.s_frame_interval = imx662_s_frame_interval,*/
  /*.g_frame_interval = imx662_g_frame_interval,*/
};

static struct v4l2_subdev_core_ops imx662_subdev_core_ops = {
  .s_power = imx662_s_power,
  .ioctl = imx662_priv_ioctl,
};

static const struct v4l2_ctrl_ops imx662_ctrl_ops = {
  .s_ctrl = imx662_s_ctrl,
};

static struct v4l2_subdev_ops imx662_subdev_ops = {
  .core  = &imx662_subdev_core_ops,
  .video = &imx662_subdev_video_ops,
  .pad   = &imx662_subdev_pad_ops,
};

static const struct media_entity_operations imx662_entity_ops = {
  .link_setup = imx662_link_setup,
};

/* ----------------------------- custom ctrls ----------------------------- */
static const struct v4l2_ctrl_config imx662_ctrl_max_gain = {
  .ops = &imx662_ctrl_ops,
  .id = V4L2_CID_MAX_GAIN,
  .name = "Maximum gain",
  .type = V4L2_CTRL_TYPE_INTEGER,
  .min = IMX662_MIN_GAIN,
  .max = IMX662_MAX_GAIN,
  .step = 1,
  .def = IMX662_MAX_GAIN,
};

static const struct v4l2_ctrl_config imx662_ctrl_min_gain = {
  .ops = &imx662_ctrl_ops,
  .id = V4L2_CID_MIN_GAIN,
  .name = "Minimum gain",
  .type = V4L2_CTRL_TYPE_INTEGER,
  .min = IMX662_MIN_GAIN,
  .max = IMX662_MAX_GAIN,
  .step = 1,
  .def = IMX662_MIN_GAIN,
};

static const struct v4l2_ctrl_config imx662_ctrl_max_exposure = {
  .ops = &imx662_ctrl_ops,
  .id = V4L2_CID_MAX_EXPOSURE,
  .name = "Maximum exposure time",
  .type = V4L2_CTRL_TYPE_INTEGER,
  .min = IMX662_MIN_EXPOSURE_TIME,
  .max = IMX662_MAX_EXPOSURE_TIME,
  .step = 1,
  .def = IMX662_MAX_EXPOSURE_TIME,
};

static const struct v4l2_ctrl_config imx662_ctrl_min_exposure = {
  .ops = &imx662_ctrl_ops,
  .id = V4L2_CID_MIN_EXPOSURE,
  .name = "Minimum exposure time",
  .type = V4L2_CTRL_TYPE_INTEGER,
  .min = IMX662_MIN_EXPOSURE_TIME,
  .max = IMX662_MAX_EXPOSURE_TIME,
  .step = 1,
  .def = IMX662_MIN_EXPOSURE_TIME,
};

static const struct of_device_id imx662_of_id_table[] = {
  { .compatible = "sony,imx662" },
  {}
};
MODULE_DEVICE_TABLE(of, imx662_of_id_table);

static const struct i2c_device_id imx662_id[] = {
  { "imx662", 0 },
  { }
};
MODULE_DEVICE_TABLE(i2c, imx662_id);

static int imx662_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
  struct v4l2_subdev *sd = NULL;
  struct stimx662 *imx662 = NULL;
  int ret = 0;
  struct device_node *node;
  u32 val;
  struct device *dev = &client->dev;
  __u64 mpf = 0;
  struct device_node *ep;
  char lens_name[64]={0};

#ifdef DEBUG
  printk("%s: Starting IMX662 module with trigger :) ...\n", __func__);
#endif
  
  imx662 = devm_kzalloc(&client->dev, sizeof(*imx662), GFP_KERNEL);
  if(!imx662)
  {
    return -ENOMEM;
  }

  imx662->client = client;
  node = client->dev.of_node;
	
  imx662->rst_gpio = of_get_named_gpio(dev->of_node, "rst-gpios", 0);
  if (!gpio_is_valid(imx662->rst_gpio))
	  printk("No sensor reset pin available");
  else{
  	if(devm_gpio_request_one(dev, imx662->rst_gpio, GPIOF_OUT_INIT_HIGH, "imx662_mipi_reset")<0){		
	  printk("Failed to set reset_pin\n");
	}
  }
  
  if(of_property_read_u32(node, "assigned-clock-rates",(u32 *)&(imx662->xclk_source)))
  {
  	dev_err(&client->dev, "xclk_source missing o invalid\n");
  }


  if (of_property_read_u32(node, "sensor,channel", &val) == 0) {
    imx662->sensor_channel = val;
    printk("%s : imx662->sensor_channel -> %d\n", __func__, imx662->sensor_channel);
  }
  else {
    imx662->sensor_channel = SENSOR_CHANNEL_0;
  }

  /* Get sensor model (mono or color) */
  if (of_property_read_u32(node, "sensor,model", &val) == 0) {
    imx662->sensor_model = val;
    printk("%s : imx662->sensor_model -> %d\n", __func__, imx662->sensor_model);
  }
  else {
    imx662->sensor_model = SENSOR_MODEL_MONO;
  }
  imx662->csi_id = imx662->sensor_model;
  sprintf(lens_name, "optic%d", imx662->csi_id);
  memcpy(imx662->focus_lens.name, lens_name, strlen(lens_name));
  imx662->focus_lens.id = imx662->csi_id;

  imx662->sensor_clk = devm_clk_get(&client->dev, "xclk");
  clk_set_rate(imx662->sensor_clk, 74250000);

#ifdef DEBUG
  printk("%s: clk_set_rate done\n", __func__);
#endif

  if ( clk_prepare_enable(imx662->sensor_clk))
  	dev_err(&client->dev, "enable sensor clk fail");

#ifdef DEBUG
  printk("%s: clk_prepare_enable done\n", __func__);
#endif

  ep = of_graph_get_next_endpoint(dev->of_node, NULL);
  if (!ep) {
	  dev_err(dev, "missing endpoint node\n");
	  return -ENODEV;
  }

  ret = fwnode_property_read_u64(of_fwnode_handle(ep),
		  "max-pixel-frequency", &mpf);
  if (ret || mpf == 0) {
	  dev_dbg(dev, "no limit for max-pixel-frequency\n");
  }

  imx662->ocp.max_pixel_frequency = mpf;

  /* initialize format */
  imx662->format.code = MEDIA_BUS_FMT_SRGGB10_1X10; 
 
  if (imx662_power_on(imx662) < 0)
  {
  	printk("%s: sensor power on fail\n",__func__);
  }

  imx662_reset(imx662);

  /* Initialize regmap */
  imx662->regmap = devm_regmap_init_i2c(client, &imx662_regmap_config);
  if(IS_ERR(imx662->regmap))
  {
    dev_err(&client->dev, "regmap init failed: %ld\n", PTR_ERR(imx662->regmap));
    ret = -ENODEV;
    goto err_regmap;
  }

#ifdef DEBUG
  printk("%s: Regmap initialized... ",__func__);
#endif

  /* Default sensor configuration */
  ret = imx662_write_table(imx662, imx662_init_config);
  if(ret)
  {
    dev_err(&client->dev,
        "%s : imx662_write_table write failed\n", __func__);
    goto err_ctrls;
  }
  /* Initialize subdevice */
  sd = &imx662->sd;
  v4l2_i2c_subdev_init(sd, client, &imx662_subdev_ops);
#ifdef DEBUG
  printk("%s: subdev_init\n", __func__);
#endif
  /* initialize subdev media pad */
  sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
  sd->dev = &client->dev;
  sd->entity.ops = &imx662_entity_ops;
  sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
  imx662->pad.flags = MEDIA_PAD_FL_SOURCE;

  ret = media_entity_pads_init(&sd->entity, 1, &imx662->pad);

#ifdef DEBUG
  printk("%s: media_entity_pads_init\n", __func__);
#endif

  if (ret < 0) {
    dev_err(&client->dev,
        "%s : media entity init Failed %d\n", __func__, ret);
    goto err_regmap;
  }

  /* register subdevice */
  ret = v4l2_async_register_subdev(sd);

#ifdef DEBUG
  printk("%s: register_subdev\n", __func__);
#endif

  if (ret < 0) {
    dev_err(&client->dev,
        "%s : v4l2_async_register_subdev failed %d\n",
        __func__, ret);
    goto err_ctrls;
  }

  mutex_init(&imx662->lock);

  memcpy(&imx662->cur_mode, &imx662_mode_info[0],
		                          sizeof(struct vvcam_mode_info_s));

  /* initialize controls */
  ret = v4l2_ctrl_handler_init(&imx662->ctrls.handler, 2);
  if (ret < 0) {
    dev_err(&client->dev,
        "%s : ctrl handler init Failed\n", __func__);
    goto err_me;
  }

  imx662->ctrls.handler.lock = &imx662->lock;

  /* add new controls */
  imx662->ctrls.gain = v4l2_ctrl_new_std(
      &imx662->ctrls.handler,
      &imx662_ctrl_ops,
      V4L2_CID_GAIN, IMX662_MIN_GAIN,
      IMX662_MAX_GAIN, 5,
      IMX662_DEF_GAIN);
  imx662->ctrls.exposure = v4l2_ctrl_new_std(
      &imx662->ctrls.handler,
      &imx662_ctrl_ops,
      V4L2_CID_EXPOSURE, IMX662_MIN_EXPOSURE_TIME,
      IMX662_MAX_EXPOSURE_TIME, 1,
      IMX662_MIN_EXPOSURE_TIME);

  /* add user controls */
  imx662->ctrls.max_gain = v4l2_ctrl_new_custom(
      &imx662->ctrls.handler,
      &imx662_ctrl_max_gain,
      NULL);
  imx662->ctrls.min_gain = v4l2_ctrl_new_custom(
      &imx662->ctrls.handler,
      &imx662_ctrl_min_gain,
      NULL);

  imx662->ctrls.max_exposure = v4l2_ctrl_new_custom(
      &imx662->ctrls.handler,
      &imx662_ctrl_max_exposure,
      NULL);
  imx662->ctrls.min_exposure = v4l2_ctrl_new_custom(
      &imx662->ctrls.handler,
      &imx662_ctrl_min_exposure,
      NULL);

  imx662->sd.ctrl_handler = &imx662->ctrls.handler;
  if (imx662->ctrls.handler.error) {
    ret = imx662->ctrls.handler.error;
    goto err_ctrls;
  }

#ifdef DEBUG
  printk("%s: Controls set",__func__); 
#endif
  /* setup default controls */
  ret = v4l2_ctrl_handler_setup(&imx662->ctrls.handler);
  if (ret) {
    dev_err(&client->dev,
        "Error %d setup default controls\n", ret);
    goto err_ctrls;
  }
 
  dev_info(&client->dev, "IMX662 : Probe success !\n");
  return 0;
err_ctrls:
  printk("%s : err_ctrls\n", __func__);
  v4l2_ctrl_handler_free(&imx662->ctrls.handler);
err_me:
  media_entity_cleanup(&sd->entity);
err_regmap:
  mutex_destroy(&imx662->lock);
  return ret;
}

static void imx662_remove(struct i2c_client *client)
{
  struct v4l2_subdev *sd = i2c_get_clientdata(client);
  struct stimx662 *imx662 = to_imx662(sd);

  v4l2_async_unregister_subdev(sd);
  v4l2_ctrl_handler_free(&imx662->ctrls.handler);
  media_entity_cleanup(&sd->entity);
  mutex_destroy(&imx662->lock);        
}

static struct i2c_driver imx662_i2c_driver = {
  .driver = {
    .name   = DRIVER_NAME,
    .of_match_table = imx662_of_id_table,
  },
  .probe          = imx662_probe,
  .remove         = imx662_remove,
  .id_table       = imx662_id,
};

module_i2c_driver(imx662_i2c_driver);

MODULE_AUTHOR("Anthony Chica Zambrano <achica@rbz.es>");
MODULE_DESCRIPTION("IMX662 Image Sensor driver");
MODULE_LICENSE("GPL v2");

