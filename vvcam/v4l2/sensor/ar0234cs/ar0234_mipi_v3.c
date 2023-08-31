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

#include "ar0234cs_regs.h"

#define DRIVER_NAME "ar0234cs"

#define AR0234_SOFT_STBY_MODE 0x2058
#define AR0234_STREAM_MODE    0x205c
#define AR0234_TRIGGER_ENABLED_MODE 0x2958 //temp
#define AR0234_MAX_EXPOSURE_TIME	10000
#define AR0234_MIN_EXPOSURE_TIME	50
#define AR0234_MAX_GAIN			1023
#define AR0234_MIN_GAIN			150
#define AR0234_DEF_GAIN			150
#define AR0234_MAX_ANALOG_GAIN		15	
#define AR0234_MIN_ANALOG_GAIN		1	
#define AR0234_DEF_ANALOG_GAIN		1
#define AR0234_MAX_ANALOG_FINE_GAIN	4
#define AR0234_MIN_ANALOG_FINE_GAIN	1
#define AR0234_DEF_ANALOG_FINE_GAIN	1
#define AR0234_DEF_FRAME_RATE		20
#define AR0234_EXP_TIME_CORRECTION	0
#define AR0234_MAX_WIDTH		1928
#define AR0234_MAX_HEIGHT		1208

#define AR0234_TABLE_END		0xffff

#define AR0234_PIX_CLK			41666666

#define AR0234_EXP_TIME_OUT_RANGE (-1)

#define V4L2_CID_ROI_H_POSITION         (V4L2_CID_DV_CLASS_BASE + 0x1000)
#define V4L2_CID_ROI_V_POSITION         (V4L2_CID_DV_CLASS_BASE + 0x1001)
#define V4L2_CID_FLASH_TIME             (V4L2_CID_DV_CLASS_BASE + 0x1002)
#define V4L2_CID_ANALOG_GAIN            (V4L2_CID_DV_CLASS_BASE + 0x1003)
#define V4L2_CID_ANALOG_FINE_GAIN   	(V4L2_CID_DV_CLASS_BASE + 0x1004)
#define V4L2_CID2_GAIN      		(V4L2_CID_DV_CLASS_BASE + 0x1005)

#define SENSOR_CHANNEL_0    0
#define SENSOR_CHANNEL_1    1

#define SENSOR_MODEL_MONO   0
#define SENSOR_MODEL_COLOR  1

#define SLAVE_MODE	    1

#define DEBUG 1 //Comment this line for disabling DEBUG traces

#ifdef CONFIG_HARDENED_USERCOPY
#define client_to_ar0234(client)\
        container_of(i2c_get_clientdata(client), struct ar0234, subdev)

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


/* PRIVATE FUNCTIONS */
static int ar0234_s_stream(struct v4l2_subdev *sd, int on);

//end private functions




static const struct regmap_config ar0234_regmap_config = {
        .reg_bits = 16,
        .val_bits = 16,
        .cache_type = REGCACHE_RBTREE,
};


struct ar0234_ctrls {
  struct v4l2_ctrl_handler handler;
  struct v4l2_ctrl *gain;
  struct v4l2_ctrl *h_pos;
  struct v4l2_ctrl *v_pos;
  struct v4l2_ctrl *exposure;
  struct v4l2_ctrl *flash_time;
  struct v4l2_ctrl *analog_gain;
  struct v4l2_ctrl *analog_fine_gain; //a lo mejor se puede camuflar con la analog y juntarla
  struct v4l2_ctrl *vflip;
  struct v4l2_ctrl *hflip;
};

struct ar0234_capture_properties {
	__u64 max_lane_frequency;
	__u64 max_pixel_frequency;
	__u64 max_data_rate;
};

struct star0234 {
  struct v4l2_subdev sd;
  struct v4l2_rect crop;
  struct v4l2_mbus_framefmt format;
  struct v4l2_fract frame_interval;
  struct media_pad pad;
  struct i2c_client *client;
  struct regmap *regmap;
  struct ar0234_ctrls ctrls;
  struct mutex lock;
  struct ar0234_capture_properties ocp;
  struct clk *sensor_clk;
  u32 sensor_channel;
  u32 sensor_model;
  u32 x_start;
  u32 y_start;
  vvcam_mode_info_t cur_mode;
  int v_flip_init_done;
  int h_flip_init_done;
  int analog_gain;
  int analog_fine_gain;
  int csi_id;
  struct reg_8 *ar0234_init_config;
  unsigned int xclk_source;
  unsigned int pwn_gpio;
  unsigned int rst_gpio;
};

struct reg_8 {
	uint16_t addr;
	uint16_t val;
	uint16_t mask;
	uint8_t offset;
};

#if 0
//FROM AR0234CS-REV3.ini
//[hidden: MIPI4Lane_1920x1200@30fps_Pxlclk22.5MHz_Extclk27MHz]  
static const struct reg_8 ar0234_init_config[] = {
	{0x301a, 0x2058, 0xFFFF, 0}, 
	{0x3088, 0x81ba, 0xFFFF, 0}, 
	{0x3086, 0x3d02, 0xFFFF, 0},
	{0x30ba, 0x7626, 0xFFFF, 0}, 
	{0x31ae, 0x0202, 0xFFFF, 0},//MIPI 2-LANE
	{0x3002, 0x0008, 0xFFFF, 0},
	{0x3004, 0x0008, 0xFFFF, 0},
	{0x3006, 0x04b7, 0xFFFF, 0},
	{0x3008, 0x0787, 0xFFFF, 0},
	{0x300a, 0x04C4, 0xFFFF, 0},
	{0x300c, 0x0500, 0xFFFF, 0},
	{0x3012, 0x01f0, 0xFFFF, 0},
	{0x305e, 0x0080, 0xFFFF, 0},
	{0x31ac, 0x0a0a, 0xFFFF, 0},
	{0x306e, 0x9010, 0xFFFF, 0},
	{0x30a2, 0x0001, 0xFFFF, 0},
	{0x30a6, 0x0001, 0xFFFF, 0},
	{0x3082, 0x0003, 0xFFFF, 0},
	{0x3040, 0x0000, 0xFFFF, 0}, 
	{0x31d0, 0x0000, 0xFFFF, 0}, 
	{0x3030, 0x007d, 0xFFFF, 0}, //A partir de aqui cambios de PLL. Config para xclk = 50MHz
	{0x302e, 0x0009, 0xFFFF, 0},
	{0x302c, 0x0003, 0xFFFF, 0},
	{0x302a, 0x0003, 0xFFFF, 0},
	{0x3036, 0x0010, 0xFFFF, 0},
	{0x3038, 0x0002, 0xFFFF, 0},
//	{0x31b0, 0x002f, 0xFFFF, 0},
//	{0x31b2, 0x002C, 0xFFFF, 0},
//	{0x31b4, 0x1144, 0xFFFF, 0},
//	{0x31b6, 0x00c7, 0xFFFF, 0},
//	{0x31b8, 0x3047, 0xFFFF, 0},
//	{0x31ba, 0x0103, 0xFFFF, 0},
//	{0x31bc, 0x8583, 0xFFFF, 0},
	{AR0234_TABLE_END, 0x0000, 0x0000, 0} //end config

};
#endif

static const struct reg_8 ar0234_init_config[] = {
	{0x301a, 0x2058, 0xFFFF, 0}, 
	{0x302a, 0x0005, 0xFFFF, 0}, //VT_PIX_CLK_DIV
	{0x302c, 0x0001, 0xFFFF, 0}, //VT_SYS_CLK_DIV
	{0x302e, 0x0006, 0xFFFF, 0}, //PRE_PLL_CLK_DIV
	{0x3030, 0x0032, 0xFFFF, 0}, //PLL_MULTIPLIER
	{0x3036, 0x000A, 0xFFFF, 0}, //OP_PIX_CLK_DIV
	{0x3038, 0x0001, 0xFFFF, 0}, //OP_SYS_CLK_DIV
	{0x30b0, 0x0028, 0xFFFF, 0}, //DIGITAL_TEST
	
//	{0x3030, 0x007d, 0xFFFF, 0}, //A partir de aqui cambios de PLL. Config para xclk = 50MHz
//	{0x302e, 0x0009, 0xFFFF, 0},
//	{0x302c, 0x0001, 0xFFFF, 0},
//	{0x302a, 0x0009, 0xFFFF, 0},
//	{0x3036, 0x000A, 0xFFFF, 0},
//	{0x3038, 0x0001, 0xFFFF, 0},//FIN CAMBIOS PLL
	
	{0x305e, 0x00ff, 0xFFFF, 0},
	{0x31b0, 0x0082, 0xFFFF, 0},
	{0x31b2, 0x005c, 0xFFFF, 0},
	{0x31b4, 0x5248, 0xFFFF, 0},
	{0x31b6, 0x3298, 0xFFFF, 0},
	{0x31b8, 0x914b, 0xFFFF, 0},
	{0x31ba, 0x030c, 0xFFFF, 0},
	{0x31bc, 0x8e09, 0xFFFF, 0},
	{0x3354, 0x002b, 0xFFFF, 0},
	{0x31ae, 0x0202, 0xFFFF, 0},//MIPI 2-LANE
	{0x3002, 0x0008, 0xFFFF, 0},//Y_ADDR_START
	{0x3004, 0x0008, 0xFFFF, 0},//X_ADDR_START
	{0x3006, 0x043b, 0xFFFF, 0},//Y_ADDR_END
	{0x3008, 0x0787, 0xFFFF, 0},//X_ADDR_END
	{0x300a, 0x0448, 0xFFFF, 0},//FRAME_LENGTH_LINES
	{0x300c, 0x072c, 0xFFFF, 0},//LINE_LENGTH_PCK
	{0x3012, 0x0100, 0xFFFF, 0},//COARSE_INTEGRATION_TIME
	{0x31ac, 0x0a0a, 0xFFFF, 0},
	{0x306e, 0x9010, 0xFFFF, 0},
	{0x30a2, 0x0001, 0xFFFF, 0},
	{0x30a6, 0x0001, 0xFFFF, 0},
	{0x3082, 0x0003, 0xFFFF, 0},
	{0x3040, 0x0000, 0xFFFF, 0}, 
	{0x31d0, 0x0000, 0xFFFF, 0}, 
//	{0x3088, 0x8050, 0xFFFF, 0}, //SEQ_CONTROL_PORT
//	{0x3086, 0x9237, 0xFFFF, 0}, //SEQ_DATA_PORT
	{0x3096, 0x0280, 0xFFFF, 0}, //RESERVED
	{0x31e0, 0x0003, 0xFFFF, 0}, //PIX_DEF_ID
	{0x30b0, 0x0028, 0xFFFF, 0}, //DIGITAL_TEST
	{0x3f4c, 0x121f, 0xFFFF, 0}, //RESERVED
	{0x3f4e, 0x121f, 0xFFFF, 0}, //RESERVED
	{0x3f50, 0x0b81, 0xFFFF, 0}, //RESERVED
	{0x3088, 0x81ba, 0xFFFF, 0}, //SEQ_CONTROL_PORT
	{0x3086, 0x3d02, 0xFFFF, 0}, //SEQ_DATA_PORT
	{0x3ed2, 0xfa96, 0xFFFF, 0}, //RESERVED
	{0x3180, 0xfa96, 0xFFFF, 0}, //RESERVED
	{0x3ecc, 0x0d42, 0xFFFF, 0}, //RESERVED
	{0x30f0, 0x2283, 0xFFFF, 0}, //RESERVED
	{0x3102, 0x5000, 0xFFFF, 0}, //AE_LUMA_TARGET
	{0x3060, 0x0001, 0xFFFF, 0}, //ANALOG_GAIN
	{0x30ba, 0x7626, 0xFFFF, 0}, 
	{AR0234_TABLE_END, 0x0000, 0x0000, 0} //end config

};


static struct vvcam_mode_info_s ar0234_mode_info[] = {
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
                .bayer_pattern  = BAYER_GRBG,
                .ae_info = {
                        .def_frm_len_lines     = 0x478,
                        .curr_frm_len_lines    = 0x478,
                        .one_line_exp_time_ns  = 4385,
                        .max_integration_line  = 0x478 - 8,
                        .min_integration_line  = 1,
                        .max_again             = AR0234_MAX_ANALOG_GAIN * 1024,
                        .min_again             = AR0234_MIN_ANALOG_GAIN    * 1024,
                        .max_dgain             = AR0234_MAX_GAIN    * 1024,
                        .min_dgain             = AR0234_MIN_GAIN    * 1024,
                        .start_exposure        = 5000 * 1024,
                        .cur_fps               = 10   * 1024,
                        .max_fps               = 30   * 1024,
                        .min_fps               = 1   * 1024,
                        .min_afps              = 1   * 1024,
                        .int_update_delay_frm  = 1,
                        .gain_update_delay_frm = 1,
                },
                .mipi_info = {
                        .mipi_lane = 2,},
                .preg_data      = ar0234_init_setting,
                .reg_data_count = ARRAY_SIZE(ar0234_init_setting),
	},
};



static inline struct v4l2_subdev *ctrl_to_sd(struct v4l2_ctrl *ctrl)
{
        return &container_of(ctrl->handler,
                             struct star0234, ctrls.handler)->sd;
}

static inline struct star0234 *to_ar0234(struct v4l2_subdev *sd)
{
        return container_of(sd, struct star0234, sd);
}



static inline int ar0234_read_reg(struct star0234 *priv, u16 addr, u16 *val)
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

/*
   static inline int ar0234_write_reg(struct star0234 *priv, u16 addr, u16 val)
   {
   int err;
   err = regmap_write(priv->regmap, addr, val);
   if (err)
   dev_err(&priv->client->dev,
   "%s : i2c write failed, %x = %x\n", __func__,
   addr, val);
   else
   dev_dbg(&priv->client->dev,
   "%s : addr 0x%x, val=0x%x\n", __func__,
   addr, val);
   return err;
   }
   */

static int ar0234_write_reg(struct star0234 *ar0234, u16 reg, u16 val)
{
	struct device *dev = &ar0234->client->dev;
	u8 au8Buf[4] = { 0 };

	au8Buf[0] = reg >> 8;
	au8Buf[1] = reg & 0xff;
	au8Buf[2] = (val >> 8) & 0xff;
	au8Buf[3] = (val & 0xff);

	if (i2c_master_send(ar0234->client, au8Buf, 4) < 0) {
		dev_err(dev, "Write reg error: reg=%x, val=%x\n", reg, val);
		return -1;
	}
#ifdef DEBUG
	printk("Escritura done!\n");
#endif
	return 0;

}

static int ar0234_get_clk(struct star0234 *ar0234, void *clk)
{
	struct vvcam_clk_s vvcam_clk;
	int ret = 0;
	vvcam_clk.sensor_mclk = 50000000;
	vvcam_clk.csi_max_pixel_clk = ar0234->ocp.max_pixel_frequency;
	ret = copy_to_user(clk, &vvcam_clk, sizeof(struct vvcam_clk_s));
	if (ret != 0)
		ret = -EINVAL;
	return ret;
}

#if 0
static int ar0234_write_reg_arry(struct star0234 *ar0234,
		struct vvcam_sccb_data_s *reg_arry,
		u32 size)
{
	/*
	   int i = 0;
	   int ret = 0;
	   struct i2c_msg msg;
	   u8 *send_buf;
	   u32 send_buf_len = 0;
	   struct i2c_client *i2c_client = ar0234->client;

	   send_buf = (u8 *)kmalloc(size + 2, GFP_KERNEL);
	   if (!send_buf)
	   return -ENOMEM;

	   send_buf[send_buf_len++] = (reg_arry[0].addr >> 8) & 0xff;
	   send_buf[send_buf_len++] = reg_arry[0].addr & 0xff;
	   send_buf[send_buf_len++] = reg_arry[0].data & 0xff;
	   for (i=1; i < size; i++) {
	   if (reg_arry[i].addr == (reg_arry[i-1].addr + 1)){
	   send_buf[send_buf_len++] = reg_arry[i].data & 0xff;
	   } else {
	   msg.addr  = i2c_client->addr;
	   msg.flags = i2c_client->flags;
	   msg.buf   = send_buf;
	   msg.len   = send_buf_len;
	   ret = i2c_transfer(i2c_client->adapter, &msg, 1);
	   if (ret < 0) {
	   pr_err("%s:i2c transfer error\n",__func__);
	   kfree(send_buf);
	   return ret;
	   }
	   send_buf_len = 0;
	   send_buf[send_buf_len++] =
	   (reg_arry[i].addr >> 8) & 0xff;
	   send_buf[send_buf_len++] =
	   reg_arry[i].addr & 0xff;
	   send_buf[send_buf_len++] =
	   reg_arry[i].data & 0xff;
	   }
	   }

	   if (send_buf_len > 0) {
	   msg.addr  = i2c_client->addr;
	   msg.flags = i2c_client->flags;
	   msg.buf   = send_buf;
	   msg.len   = send_buf_len;
	   ret = i2c_transfer(i2c_client->adapter, &msg, 1);
	   if (ret < 0)
	   pr_err("%s:i2c transfer end meg error\n",__func__);
	   else
	   ret = 0;
	   }
	   kfree(send_buf);
	   return ret;*/
	return 0;
}

#endif


static int ar0234_set_exposure(struct star0234 *ar0234, u32 new_exp)
{
	int ret =  0;
	//u16 real_exposure = 0;
	u32 coarse_exp_time = 0;
	u16 llp = 0;
	u16 inv_row_time = 0;

	printk("%s: Trying exposure value: %d", __func__, new_exp);
	

	if(new_exp > AR0234_MAX_EXPOSURE_TIME)
	{
		printk("%s: ERROR. Exposure time out of range\n",__func__);
		ret = AR0234_EXP_TIME_OUT_RANGE;
	}
	else
	{
		// Computing inverse row_time (as row_time would be 0 because of integer stuff)
		ret = ar0234_read_reg(ar0234, LINE_LENGTH_PCK_REG, &llp);
		if (ret != 0)
		{
			return -1;
		}
		
		inv_row_time = AR0234_PIX_CLK/llp;
		
		// Computing COARSE INTEGRATION TIME register value. Time new_exp given in microseconds.
		coarse_exp_time = inv_row_time * new_exp / 1000000;
		
		if (coarse_exp_time > (ar0234->format.width + ar0234->x_start - 1) )
		{
			printk("%s: CIT value not valid for this resolution",__func__);
			return -1;
		}
			
		ret = ar0234_write_reg(ar0234, COARSE_INTEGRATION_TIME_REG, (u16) coarse_exp_time);
		if((ret >= 0))
		{
			ret = 0;
			ar0234->ctrls.exposure->val = new_exp;
			ar0234->ctrls.exposure->cur.val = new_exp;
		}
		else
		{
			printk("%s: ERROR. Value failed to write value\n",__func__);
		}
	}
	return ret;

}

static int ar0234_set_digital_gain(struct star0234 *priv,int val)
{
        int res  = 0;
        u16 gain = val / 1024;
	printk("%s: Trying to set %d gain\n", __func__, gain);
	
	if (gain < AR0234_MIN_GAIN || gain > AR0234_MAX_GAIN)
	{
		printk("%s: Gain value out of range, setting gain to maximum (1023)\n",__func__);
		gain = AR0234_MAX_GAIN;
	}
	res = ar0234_write_reg(priv, DIGITAL_GAIN_REG, gain);
	
	if (res <= -1)
	{
		printk("%s: ERROR. Value failed to write value\n", __func__);
	}
	
	return res;
}

static int ar0234_set_analog_gain(struct star0234 *ar0234, int val)
{
  int res = 0;
  ar0234->analog_gain = val;
  res = ar0234_write_reg(ar0234, ANALOG_GAIN_REG, val);
  if (res >= 0)
  {
	  res = 0;
  }
  else
  {
	  printk("%s: ERROR. Value failed to write value\n", __func__);
  }
  return res;
}

static int ar0234_set_analog_fine_gain(struct star0234 *ar0234, int val)
{
  int res = 0;
  ar0234->analog_fine_gain = val;
  // EN ESTE CASO ES EL MISMO REGISTRO QUE EL ANALOG, PERO TOCANDO LOS 4 BITS LSB
  //res = ar0234_write_reg(ar0234, ANALOG_GAIN_REG, val);
  if (res >= 0)
  {
	  res = 0;
  }
  else
  {
	  printk("%s: ERROR. Value failed to write value\n", __func__);
  }
  
  return res;
}

static int ar0234_set_h_pos(struct star0234 *ar0234, int new_h_pos)
{
  int res = 0;

  printk("%s: %d\n", __func__, new_h_pos);
  ar0234->x_start = new_h_pos;
  //ar0234_write_reg(ar0234, X_ADDR_START_REG, new_h_pos);
  //ar0234_write_reg(ar0234, X_ADDR_END_REG, ar0234->format.width + new_h_pos - 1);
  return res;
}

static int ar0234_set_v_pos(struct star0234 *ar0234, int new_v_pos)
{
  int res = 0;

  printk("%s: %d\n", __func__, new_v_pos);
  ar0234->y_start = new_v_pos;
  //ar0234_write_reg(ar0234, Y_ADDR_START_REG, new_v_pos);
  //ar0234_write_reg(ar0234, Y_ADDR_END_REG, ar0234->format.height + new_v_pos - 1);
  return res;
}

static int ar0234_set_v_flip(struct star0234 *ar0234, int v_flip)
{
  uint16_t reg_read_mode = 0;
  int res = 0;
  


  ar0234_read_reg(ar0234, READ_MODE_REG, &reg_read_mode);
  if(v_flip == 1)
  {
    	//ar0234_write_reg(ar0234, READ_MODE_REG, reg_read_mode | (v_flip << 15));
   	ar0234->v_flip_init_done = 1;
  }
  else if(v_flip == 0)
  {
    	//ar0234_write_reg(ar0234, READ_MODE_REG, reg_read_mode & ~(!v_flip << 15));
  	ar0234->v_flip_init_done = 1;
  }
  else
  {
    res = -1;
    printk("%s: invalid flip value\n", __func__);
  }


  return res;
}


static int ar0234_set_h_flip(struct star0234 *ar0234, int h_flip)
{
  uint16_t reg_read_mode = 0;
  int res = 0;

  ar0234_read_reg(ar0234, READ_MODE_REG, &reg_read_mode);
  if(h_flip == 1)
  {
    	//ar0234_write_reg(ar0234, READ_MODE_REG, reg_read_mode | (h_flip << 14));
    	ar0234->h_flip_init_done = 1;
  }
  else if(h_flip == 0)
  {
    	//ar0234_write_reg(ar0234, READ_MODE_REG, reg_read_mode & ~(!h_flip << 14));
    	ar0234->h_flip_init_done = 1;
  }
  else
  {
    res = -1;
    printk("%s: invalid flip value\n", __func__);
  }


  return res;
}




// DESPUES DE FUNCIONES DE CONTROL                                                                   
static int ar0234_s_ctrl(struct v4l2_ctrl *ctrl)                                                     
{
        struct v4l2_subdev *sd = ctrl_to_sd(ctrl);                                                   
        struct star0234 *ar0234 = to_ar0234(sd);
        int ret = -EINVAL;
        
        switch (ctrl->id) {                                                                          
                case V4L2_CID_GAIN:                                                                  
                        ret = ar0234_set_digital_gain(ar0234,ctrl->val);                             
                        break;                                                                       
                case V4L2_CID_ANALOG_FINE_GAIN: 
                        ret = ar0234_set_analog_fine_gain(ar0234, ctrl->val);                        
                        break;                                                                       
                case V4L2_CID_ANALOG_GAIN:                                                           
                        ret = ar0234_set_analog_gain(ar0234, ctrl->val);                             
                        break;                                                                       
                case V4L2_CID_ROI_H_POSITION:                                                        
                        ret = ar0234_set_h_pos(ar0234, ctrl->val);                                   
                        break;
                case V4L2_CID_ROI_V_POSITION:                                                        
                        ret = ar0234_set_v_pos(ar0234, ctrl->val);                                   
                        break;
                case V4L2_CID_EXPOSURE:                                                              
                        ret = ar0234_set_exposure(ar0234, ctrl->val);                                           
                        break;
                case V4L2_CID_FLASH_TIME:                                                            
                        break;
                case V4L2_CID_HFLIP:                                                                 
                        ret = ar0234_set_h_flip(ar0234, ctrl->val);                                  
                        break;
                case V4L2_CID_VFLIP:
                        ret = ar0234_set_v_flip(ar0234,ctrl->val);                                   
                        break;    
					
        }
        
        return 0; 
}
   
static int ar0234_get_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_state *state, struct v4l2_subdev_format *format) 
{ 
	struct star0234 *ar0234 = to_ar0234(sd); 
	mutex_lock(&ar0234->lock);
	format->format = ar0234->format; 
	mutex_unlock(&ar0234->lock);
	return 0; 
}

/*
static int __ar0234_change_compose(struct star0234 *ar0234, struct v4l2_subdev_pad_config *cfg, u32 which, u32 *width, u32 *height, u32 flags) 
{ 
	return 0; 
}
*/


static int ar0234_query_supports(struct star0234 *ar0234, void* parry)
{
	struct vvcam_mode_info_array_s *psensor_mode_arry = parry;

	psensor_mode_arry->count = ARRAY_SIZE(ar0234_mode_info);
	memcpy((void *)&psensor_mode_arry->modes, (void *)ar0234_mode_info, sizeof(ar0234_mode_info));

	return 0;
}

static int ar0234_write_table(struct star0234 *priv, const struct reg_8 table[])
{
	int ret = 0;
	struct regmap *regmap = priv->regmap;
	const struct reg_8 *reg;
	unsigned int read_reg = 0;

	for(reg = table;;reg++)
	{
		dev_dbg(&priv->client->dev, "%s : i2c table write, addr = 0x%02x value=0x%04x\n", __func__, reg->addr, reg->val);
		if(reg->addr != AR0234_TABLE_END)
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



static int ar0234_set_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_state *state, struct v4l2_subdev_format *format) 
{ 
	struct v4l2_mbus_framefmt *fmt = &format->format; 
	struct star0234 *ar0234 = to_ar0234(sd); 
	int ret = 0;

    	mutex_lock(&ar0234->lock);

	if ((fmt->width != ar0234->cur_mode.size.bounds_width) ||
			(fmt->height != ar0234->cur_mode.size.bounds_height)) {
		pr_err("%s:set sensor format %dx%d error\n",
				__func__,fmt->width,fmt->height);
		mutex_unlock(&ar0234->lock);
		return -EINVAL;
	}

    	fmt->field = V4L2_FIELD_NONE; 


	if((fmt->width + ar0234->x_start - 1) > AR0234_MAX_WIDTH) {
        	ar0234_write_reg(ar0234, X_ADDR_START_REG, 0);
            	ar0234->x_start = AR0234_MAX_WIDTH - fmt->width;
            	printk("La imagen se pasa de ancho, cambiar ROI\n");
    	}

    	if((fmt->height + ar0234->y_start - 1) > AR0234_MAX_HEIGHT) {
            	ar0234_write_reg(ar0234, Y_ADDR_START_REG, 0);
            	ar0234->y_start = AR0234_MAX_HEIGHT - fmt->height;
            	printk("La imagen se pasa de alto, cambiar ROI\n");
    	}
	
    	//ar0234_write_reg(ar0234, X_ADDR_END_REG, (fmt->width + ar0234->x_start -1));
    	//ar0234_write_reg(ar0234, Y_ADDR_END_REG,(fmt->height + ar0234->y_start - 1));

	//ret = ar0234_write_reg_arry(ar0234,
	//		(struct vvcam_sccb_data_s *)ar0234->cur_mode.preg_data,
	//		ar0234->cur_mode.reg_data_count);
	if (ret < 0) {
		pr_err("%s:ar0234_write_reg_arry error\n",__func__);
		mutex_unlock(&ar0234->lock);
		return -EINVAL;
	}

	printk("%s: end = %dx%d\n", __func__, (fmt->width + ar0234->x_start -1), (fmt->height + ar0234->y_start - 1));
//	fmt->code = BAYER_RGGB;
	fmt->field = V4L2_FIELD_NONE;
	ar0234->format = *fmt;
	mutex_unlock(&ar0234->lock);


    	return 0;
}

static int ar0234_query_capabilities(struct star0234 *ar0234, void *arg)
{
	struct v4l2_capability *pcap = (struct v4l2_capability *)arg;

	strcpy((char *)pcap->driver, "ar0234cs");
	sprintf((char *)pcap->bus_info, "csi%d",ar0234->csi_id);
	printk("%s: csi%d\n", __func__, ar0234->csi_id);
	//sprintf((char *)pcap->bus_info, "csi0");
	if(ar0234->client->adapter) {
		pcap->bus_info[VVCAM_CAP_BUS_INFO_I2C_ADAPTER_NR_POS] =
			(__u8)ar0234->client->adapter->nr;
	} else {
		pcap->bus_info[VVCAM_CAP_BUS_INFO_I2C_ADAPTER_NR_POS] = 0xFF;
	}

	return 0;

}


static int ar0234_get_sensor_mode(struct star0234 *ar0234, void* pmode)
{
	int ret = 0;
	ret = copy_to_user(pmode, &ar0234->cur_mode,
			sizeof(struct vvcam_mode_info_s));
	if (ret != 0)
		ret = -ENOMEM;
	return ret;
}

static int ar0234_set_sensor_mode(struct star0234 *ar0234, void* pmode)
{
	int ret = 0;
	int i = 0;
	struct vvcam_mode_info_s sensor_mode;
	ret = copy_from_user(&sensor_mode, pmode,
			sizeof(struct vvcam_mode_info_s));
	if (ret != 0)
		return -ENOMEM;
	for (i = 0; i < ARRAY_SIZE(ar0234_mode_info); i++) {
		if (ar0234_mode_info[i].index == sensor_mode.index) {
			memcpy(&ar0234->cur_mode, &ar0234_mode_info[i],
					sizeof(struct vvcam_mode_info_s));
			return 0;
		}
	}

	return -ENXIO;
}

static int ar0234_get_sensor_id(struct star0234 *ar0234, u16 *pchipid)
{
  int ret = 0;
  u16 chip_id = 0xa5a5;

  ret = copy_to_user(pchipid, &chip_id, sizeof(u16));
  if (ret != 0)
    ret = -ENOMEM;
  return ret;
	
}

static int ar0234_get_reserve_id(struct star0234 *ar0234, void* preserve_id)
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

static int ar0234_get_fps(struct star0234 *ar0234, u32 *pfps)
{
	*pfps = ar0234->cur_mode.ae_info.cur_fps;
	return 0;
}

static int ar0234_set_fps(struct star0234 *ar0234, u32 fps)
{
	int ret = 0;
	uint32_t real_fps = 0;
	u32 reg_val = 0;
	u16 fll = 0;

	if (fps > ar0234->cur_mode.ae_info.max_fps) 
	{
		fps = ar0234->cur_mode.ae_info.max_fps;
	}
	else if( fps < ar0234->cur_mode.ae_info.min_fps)
	{
		fps =  ar0234->cur_mode.ae_info.min_fps;
	}

	ar0234->cur_mode.ae_info.cur_fps = fps;
	real_fps = fps / 1024;
	
	// Obtaining frame lenght line from sensor register
	ret = ar0234_read_reg(ar0234, FRAME_LENGTH_LINE_REG, &fll);
	if (ret != 0)
	{
		return -1;
	}

	// Computing of value to be written in sensor register
	reg_val = AR0234_PIX_CLK / (fll * real_fps);

#ifdef DEBUG
	printk("%s: Try to set %d fps\n", __func__, real_fps);
#endif

	ret = ar0234_write_reg(ar0234, LINE_LENGTH_PCK_REG, (u16) reg_val);
	if (ret != 0)
	{
		printk("%s: Failed to write new frame rate value (%d fps)", __func__, real_fps);
		return -1;
	}

	// Adjusting exposure time (in microseconds) registers to new frame rate
	
	reg_val = (AR0234_PIX_CLK/reg_val)*ar0234->ctrls.exposure->cur.val/1000000;
#ifdef DEBUG
	printk("%s: COARSE_INTEGRATION_TIME gonna be: %d", __func__, reg_val);
#endif
	ret = ar0234_write_reg(ar0234, COARSE_INTEGRATION_TIME_REG, (u16) reg_val);
	
	return ret;
}

static long ar0234_priv_ioctl(struct v4l2_subdev *sd,	                  
    		unsigned int cmd,
		void *arg_user)
{
 	struct star0234 *ar0234 = to_ar0234(sd);
	//struct i2c_client *client = ar0234->client;
	long ret = 0;
	struct vvcam_sccb_data_s sensor_reg;
	void *arg = arg_user;

	printk("%s cmd = 0x%x\n", __func__, cmd);
	
	mutex_lock(&ar0234->lock);
	printk("%s: mutex locked\n", __func__);
	switch(cmd)
	{
		case VVSENSORIOC_S_POWER:
			ret = 0;
			break;
		case VVSENSORIOC_S_CLK:
			ret = 0;
			break;
		case VVSENSORIOC_G_CLK:
			ret = ar0234_get_clk(ar0234,arg);
			break;
		case VVSENSORIOC_RESET:
			ret = 0;
			break;
		case VIDIOC_QUERYCAP:
			printk("%s: Consulta capabilities", __func__);
			ret = ar0234_query_capabilities(ar0234, arg);
			break;
        	case VVSENSORIOC_QUERY:
			USER_TO_KERNEL(struct vvcam_mode_info_array_s);
			printk("%s: Consulta support",__func__);
			ret = ar0234_query_supports(ar0234, arg);
			KERNEL_TO_USER(struct vvcam_mode_info_array_s);
			break;
       		case VVSENSORIOC_WRITE_REG:
			ret = copy_from_user(&sensor_reg, arg,
					sizeof(struct vvcam_sccb_data_s));
			break;
		case VVSENSORIOC_READ_REG:
			ret = copy_from_user(&sensor_reg, arg,
					sizeof(struct vvcam_sccb_data_s));
			ret |= ar0234_read_reg(ar0234, sensor_reg.addr,
					(u16*)&sensor_reg.data);
			ret |= copy_to_user(arg, &sensor_reg,
					sizeof(struct vvcam_sccb_data_s));
			break;
		case VVSENSORIOC_G_CHIP_ID:
			ret = ar0234_get_sensor_id(ar0234, arg);
			break;
		case VVSENSORIOC_G_RESERVE_ID:
			ret = ar0234_get_reserve_id(ar0234, arg);
			break;
		case VVSENSORIOC_G_SENSOR_MODE:
			ret = ar0234_get_sensor_mode(ar0234, arg);
			break;
		case VVSENSORIOC_S_SENSOR_MODE:
			ret = ar0234_set_sensor_mode(ar0234, arg);
			break;
		case VVSENSORIOC_S_GAIN: //Warning. La ganancia ahora mismo deja una imagen rara.
			USER_TO_KERNEL(int);
			ret = ar0234_set_digital_gain(ar0234, *(int *)arg);
			ret = 0;
			break;
		case VVSENSORIOC_S_VSGAIN:
			ret = 0;
			break;
		case VVSENSORIOC_S_EXP:
			USER_TO_KERNEL(u32); 
			//ret = ar0234_write_reg(sensor, AR0234_COARSE_INTEGRATION_TIME, 0x01fc); 
			ret = ar0234_set_exposure(ar0234, *(int *)arg);
			ret = 0;
			break;
		case VVSENSORIOC_S_VSEXP:
			ret = 0;
			break;
		case VVSENSORIOC_S_FPS:	
			USER_TO_KERNEL(int);
			printk("%s: IOCTL VVSENSORIOC_S_FPS called successfully, arg_value: %d\n",__func__, *(int *)arg);
			ret = ar0234_set_fps(ar0234, *(int *)arg); 
			break;
		case VVSENSORIOC_G_FPS:
			ret = ar0234_get_fps(ar0234, arg);
			break;
		case VVSENSORIOC_S_STREAM:
			printk("%s: VVSENSORIOC_S_STREAM\n", __func__);
			USER_TO_KERNEL(int);
			ret = ar0234_s_stream(sd, *(int *)arg);
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
		default:
			ret = -EINVAL;
			break;

	}

	mutex_unlock(&ar0234->lock);
	printk("%s: mutex unlocked\n", __func__);
	return ret;
}



static int ar0234_s_stream(struct v4l2_subdev *sd, int on) { 
	
	struct star0234 *ar0234 = to_ar0234(sd); 
	int ret = 0; 

  	printk("%s : %s\n", __func__, on ? "Stream Start" : "Stream Stop");
  	//mutex_lock(&ar0234->lock);
	
	
  	if(on){
    		//A lo mejor meto un campo stream_status = enable, como con el sensor de omnivision de refrencia.
		ret = ar0234_write_reg(ar0234, RESET_REG, AR0234_TRIGGER_ENABLED_MODE);
    		if(ret){
      			goto fail;
    		}
  	}	
  	else{
   		ret = ar0234_write_reg(ar0234, RESET_REG, AR0234_SOFT_STBY_MODE);
   		if(ret){
    			goto fail;
   		}

  	}
  	//mutex_unlock(&ar0234->lock);
  	return 0;

	
  	fail:
    	return ret;
}

static int ar0234_power_on(struct star0234 *ar0234)
{
	int ret = 0;
	printk("%s: enter \n", __func__);
	/*
	if(gpio_is_valid(ar0234->pwn_gpio))
		gpio_set_value_cansleep(ar0234->pwn_gpio, 1);
	
	ret = clk_prepare_enable(ar0234->sensor_clk);
	if (ret < 0)
		printk("%s: enable sensor clk fail \n", __func__);
	*/
	return ret;
}

static int ar0234_power_off(struct star0234 *ar0234)
{
	printk("enter %s\n", __func__);
	/*
	if (gpio_is_valid(ar0234->pwn_gpio))
		gpio_set_value_cansleep(ar0234->pwn_gpio, 0);
	clk_disable_unprepare(ar0234->sensor_clk);
	*/
	return 0;
}

static int ar0234_s_power(struct v4l2_subdev *sd, int on)
{
	struct star0234 *ar0234 = to_ar0234(sd);

	printk("%s: setting power to: %d\n",__func__, on);
	if(on)
		ar0234_power_on(ar0234);
	else
		ar0234_power_off(ar0234);

	return 0;
}

static int ar0234_reset(struct star0234 *ar0234)
{
	printk("enter %s\n",__func__);
	if (!gpio_is_valid(ar0234->rst_gpio))
		return -1;

	gpio_set_value_cansleep(ar0234->rst_gpio, 0);
	msleep(20);

	gpio_set_value_cansleep(ar0234->rst_gpio, 1);
	msleep(20);

	return 0;
}


static int ar0234_link_setup(struct media_entity *entity,
		struct media_pad const *local,
		struct media_pad const *remote,
		u32 flags)
{

	return 0;
}

static int ar0234_get_format_code(struct star0234 *ar0234, u32 *code)
{
	printk("%s\n", __func__);
	switch (ar0234->cur_mode.bayer_pattern) {
		case BAYER_RGGB:
			if (ar0234->cur_mode.bit_width == 8) {
				*code = MEDIA_BUS_FMT_SRGGB8_1X8;
			} else if (ar0234->cur_mode.bit_width == 10) {
				*code = MEDIA_BUS_FMT_SRGGB10_1X10;
			} else {
				*code = MEDIA_BUS_FMT_SRGGB12_1X12;
			}
			break;
		case BAYER_GRBG:
			if (ar0234->cur_mode.bit_width == 8) {
				*code = MEDIA_BUS_FMT_SGRBG8_1X8;
			} else if (ar0234->cur_mode.bit_width == 10) {
				*code = MEDIA_BUS_FMT_SGRBG10_1X10;
			} else {
				*code = MEDIA_BUS_FMT_SGRBG12_1X12;
			}
			break;
		case BAYER_GBRG:
			if (ar0234->cur_mode.bit_width == 8) {
				*code = MEDIA_BUS_FMT_SGBRG8_1X8;
			} else if (ar0234->cur_mode.bit_width == 10) {
				*code = MEDIA_BUS_FMT_SGBRG10_1X10;
			} else {
				*code = MEDIA_BUS_FMT_SGBRG12_1X12;
			}
			break;
		case BAYER_BGGR:
			if (ar0234->cur_mode.bit_width == 8) {
				*code = MEDIA_BUS_FMT_SBGGR8_1X8;
			} else if (ar0234->cur_mode.bit_width == 10) {
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

static int ar0234_s_frame_interval(struct v4l2_subdev *sd, struct v4l2_subdev_frame_interval *fi)
{
	struct star0234 *ar0234 = to_ar0234(sd);
	int ret;
	
	ret = ar0234_set_fps(ar0234, (u32) (fi->interval.denominator/fi->interval.numerator));

	return 0;

}

static int ar0234_g_frame_interval(struct v4l2_subdev *sd, struct v4l2_subdev_frame_interval *fi)
{
	struct star0234 *ar0234 = to_ar0234(sd);
	fi->interval = ar0234->frame_interval;
	return 0;
}


#if LINUX_VERSION_CODE > KERNEL_VERSION(5, 12, 0)
static int ar0234_enum_mbus_code(struct v4l2_subdev *sd,
		struct v4l2_subdev_state *state,
		struct v4l2_subdev_mbus_code_enum *code)
#else
static int ar0234_enum_mbus_code(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_mbus_code_enum *code)
#endif
{
	struct star0234 *sensor = to_ar0234(sd);

	u32 cur_code = MEDIA_BUS_FMT_SGRBG10_1X10;

	printk("%s\n", __func__);

	/*if (code->index > 0)*/
	/*return -EINVAL;*/
	ar0234_get_format_code(sensor, &cur_code);
	code->code = cur_code;

	return 0;
}


static const struct v4l2_subdev_pad_ops ar0234_subdev_pad_ops = {
	.enum_mbus_code = ar0234_enum_mbus_code,
        .get_fmt = ar0234_get_fmt,
        .set_fmt = ar0234_set_fmt,
};

static const struct v4l2_subdev_video_ops ar0234_subdev_video_ops = {
        .s_stream = ar0234_s_stream,
	.s_frame_interval = ar0234_s_frame_interval,
	.g_frame_interval = ar0234_g_frame_interval,
};


static struct v4l2_subdev_core_ops ar0234_subdev_core_ops = {
	.s_power = ar0234_s_power,
	.ioctl = ar0234_priv_ioctl,
};

static const struct v4l2_ctrl_ops ar0234_ctrl_ops = {
        .s_ctrl = ar0234_s_ctrl,
};

static struct v4l2_subdev_ops ar0234_subdev_ops = {
	        .core  = &ar0234_subdev_core_ops,
		.video = &ar0234_subdev_video_ops,
		.pad   = &ar0234_subdev_pad_ops,
};


static const struct media_entity_operations ar0234_entity_ops = {
	.link_setup = ar0234_link_setup,
};


/* ----------------------------- custom ctrls ----------------------------- */
static const struct v4l2_ctrl_config ar0234_ctrl_roi_h_position = {
        .ops = &ar0234_ctrl_ops,
        .id = V4L2_CID_ROI_H_POSITION,
        .name = "Roi Horizontal Position",
        .type = V4L2_CTRL_TYPE_INTEGER,
        .min = 0,
        .max = 1920,
        .step = 1,
        .def = 8,
};

static const struct v4l2_ctrl_config ar0234_ctrl_roi_v_position = {
        .ops = &ar0234_ctrl_ops,
        .id = V4L2_CID_ROI_V_POSITION,
        .name = "Roi Vertical Position",
        .type = V4L2_CTRL_TYPE_INTEGER,
        .min = 0,
        .max = 1080,
        .step = 1,
        .def = 8,
};

/* Como no se como será el flash que acompanie al bicho, no hago nada todavía.
static const struct v4l2_ctrl_config ar0234_ctrl_flash_time = {
        .ops = &ar0234_ctrl_ops,
        .id = V4L2_CID_FLASH_TIME,
        .name = "Flash time",
        .type = V4L2_CTRL_TYPE_INTEGER,
        .min = 0,
        .max = 1000,
        .step = 1,
        .def = 0,
};
*/
static const struct v4l2_ctrl_config ar0234_ctrl_analog_gain = {
        .ops = &ar0234_ctrl_ops,
        .id = V4L2_CID_ANALOG_GAIN,
        .name = "Analog gain",
        .type = V4L2_CTRL_TYPE_INTEGER,
        .min = 0,
        .max = 127,
        .step = 1,
        .def = 0,
};

static const struct v4l2_ctrl_config ar0234_ctrl_analog_fine_gain = {
        .ops = &ar0234_ctrl_ops,
        .id = V4L2_CID_ANALOG_FINE_GAIN,
        .name = "Analog fine gain",
        .type = V4L2_CTRL_TYPE_INTEGER,
        .min = 0,
        .max = 15,
        .step = 1,
        .def = 0,
};




static const struct of_device_id ar0234_of_id_table[] = {
        { .compatible = "on,ar0234cs" },
	{}
};
MODULE_DEVICE_TABLE(of, ar0234_of_id_table);

static const struct i2c_device_id ar0234_id[] = {
        { "ar0234cs", 0 },
        { }
};
MODULE_DEVICE_TABLE(i2c, ar0234_id);



static int ar0234_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
  struct v4l2_subdev *sd = NULL;
  struct star0234 *ar0234 = NULL;
  int ret = 0;
  struct device_node *node;
  u32 val;
  struct device *dev = &client->dev;
  __u64 mpf = 0;
  struct device_node *ep;

#ifdef DEBUG
  printk("%s: Starting AR0234 module with trigger :) ...\n", __func__);
#endif
  
  ar0234 = devm_kzalloc(&client->dev, sizeof(*ar0234), GFP_KERNEL);
  if(!ar0234)
  {
    return -ENOMEM;
  }

  ar0234->client = client;
  node = client->dev.of_node;
	
  /*
  ar0234->pwn_gpio = of_get_named_gpio(dev->of_node, "pwn-gpios", 0);
  if (!gpio_is_valid(ar0234->pwn_gpio))
	  printk("No sensor pwdn pin available");
  else{
  	if(devm_gpio_request_one(dev, ar0234->pwn_gpio, GPIOF_OUT_INIT_HIGH, "ar0234_mipi_pdwn")<0)
	{
		printk("Failed to set power pin\n");
	}
  }
  */
  ar0234->rst_gpio = of_get_named_gpio(dev->of_node, "rst-gpios", 0);
  if (!gpio_is_valid(ar0234->rst_gpio))
	  printk("No sensor reset pin available");
  else{
  	if(devm_gpio_request_one(dev, ar0234->rst_gpio, GPIOF_OUT_INIT_HIGH, "ar0234_mipi_reset")<0){		
	printk("Failed to set reset_pin\n");
	}
  }
  
  if(of_property_read_u32(node, "assigned-clock-rates",(u32 *)&(ar0234->xclk_source)))
  {
  	dev_err(&client->dev, "xclk_source missing o invalid\n");
  }


  if (of_property_read_u32(node, "sensor,channel", &val) == 0) {
    ar0234->sensor_channel = val;
    printk("%s : ar0234->sensor_channel -> %d\n", __func__, ar0234->sensor_channel);
  }
  else {
    ar0234->sensor_channel = SENSOR_CHANNEL_0;
  }

  /* Get sensor model (mono or color) */
  if (of_property_read_u32(node, "sensor,model", &val) == 0) {
    ar0234->sensor_model = val;
    printk("%s : ar0234->sensor_model -> %d\n", __func__, ar0234->sensor_model);
  }
  else {
    ar0234->sensor_model = SENSOR_MODEL_MONO;
  }
  ar0234->csi_id = ar0234->sensor_model;

  ar0234->sensor_clk = devm_clk_get(&client->dev, "xclk");
  //ar0234->sensor_clk->mclk = 50000000;
  clk_set_rate(ar0234->sensor_clk, 50000000);

  printk("%s: clk_set_rate done\n", __func__);

  if ( clk_prepare_enable(ar0234->sensor_clk))
  {
  	dev_err(&client->dev, "enable sensor clk fail");
  }
  printk("%s: clk_prepare_enable done\n", __func__);

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

  ar0234->ocp.max_pixel_frequency = mpf;

  printk("%s: mpf\n", __func__);
  /* initialize format */
 // ar0234->crop.width = AR0234_MAX_WIDTH;
 // ar0234->crop.height = AR0234_MAX_HEIGHT;
 // ar0234->format.width = ar0234->crop.width;
 // ar0234->format.height = ar0234->crop.height;
 // ar0234->format.field = V4L2_FIELD_NONE;
 // ar0234->format.code = MEDIA_BUS_FMT_SRGGB10_1X10;
 // ar0234->format.colorspace = V4L2_COLORSPACE_SRGB;
 // ar0234->frame_interval.numerator = 1;
 // ar0234->frame_interval.denominator = AR0234_DEF_FRAME_RATE;
 // ar0234->v_flip_init_done = 0;
 // ar0234->h_flip_init_done = 0;
 // ar0234->x_start = 8;
 // ar0234->y_start = 8;
 // ar0234->analog_gain = AR0234_DEF_ANALOG_GAIN;
 // ar0234->analog_fine_gain = AR0234_DEF_ANALOG_FINE_GAIN; 
  
  if (ar0234_power_on(ar0234) < 0)
  {
  	printk("%s: sensor power on fail\n",__func__);
  }

  ar0234_reset(ar0234);

  /* Initialize regmap */
  ar0234->regmap = devm_regmap_init_i2c(client, &ar0234_regmap_config);
  if(IS_ERR(ar0234->regmap))
  {
    dev_err(&client->dev, "regmap init failed: %ld\n", PTR_ERR(ar0234->regmap));
    ret = -ENODEV;
    goto err_regmap;
  }
  printk("%s: Regmap initialized... ",__func__);

  /* Default sensor configuration */
  ret = ar0234_write_table(ar0234, ar0234_init_config);
  if(ret)
  {
    dev_err(&client->dev,
        "%s : ar0234_write_table write failed\n", __func__);
    goto err_ctrls;
  }
  /* Initialize subdevice */
  sd = &ar0234->sd;
  v4l2_i2c_subdev_init(sd, client, &ar0234_subdev_ops);
  printk("%s: subdev_init\n", __func__);

  /* initialize subdev media pad */
  //strlcpy(sd->name, DRIVER_NAME, sizeof(sd->name));
  sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
  sd->dev = &client->dev;
  sd->entity.ops = &ar0234_entity_ops;
  sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
  ar0234->pad.flags = MEDIA_PAD_FL_SOURCE;

  ret = media_entity_pads_init(&sd->entity, 1, &ar0234->pad);
  printk("%s: media_entity_pads_init\n", __func__);
  if (ret < 0) {
    dev_err(&client->dev,
        "%s : media entity init Failed %d\n", __func__, ret);
    goto err_regmap;
  }

  /* register subdevice */
  ret = v4l2_async_register_subdev(sd);
  printk("%s: register_subdev\n", __func__);
  if (ret < 0) {
    dev_err(&client->dev,
        "%s : v4l2_async_register_subdev failed %d\n",
        __func__, ret);
    goto err_ctrls;
  }

  mutex_init(&ar0234->lock);

  memcpy(&ar0234->cur_mode, &ar0234_mode_info[0],
		                          sizeof(struct vvcam_mode_info_s));

  /* initialize controls */
  ret = v4l2_ctrl_handler_init(&ar0234->ctrls.handler, 2);
  if (ret < 0) {
    dev_err(&client->dev,
        "%s : ctrl handler init Failed\n", __func__);
    goto err_me;
  }

  ar0234->ctrls.handler.lock = &ar0234->lock;

  /* add new controls */
  ar0234->ctrls.gain = v4l2_ctrl_new_std(
      &ar0234->ctrls.handler,
      &ar0234_ctrl_ops,
      V4L2_CID_GAIN, AR0234_MIN_GAIN,
      AR0234_MAX_GAIN, 5,
      AR0234_DEF_GAIN);
  ar0234->ctrls.exposure = v4l2_ctrl_new_std(
      &ar0234->ctrls.handler,
      &ar0234_ctrl_ops,
      V4L2_CID_EXPOSURE, AR0234_MIN_EXPOSURE_TIME,
      AR0234_MAX_EXPOSURE_TIME, 1,
      AR0234_MIN_EXPOSURE_TIME);
  ar0234->ctrls.hflip = v4l2_ctrl_new_std(
      &ar0234->ctrls.handler,
      &ar0234_ctrl_ops,
      V4L2_CID_HFLIP, 0, 1, 1, 0);
  ar0234->ctrls.vflip = v4l2_ctrl_new_std(
      &ar0234->ctrls.handler,
      &ar0234_ctrl_ops,
      V4L2_CID_VFLIP, 0, 1, 1, 0);

  /* add user controls */
  ar0234->ctrls.h_pos = v4l2_ctrl_new_custom(&ar0234->ctrls.handler,
      &ar0234_ctrl_roi_h_position, NULL);
  ar0234->ctrls.v_pos = v4l2_ctrl_new_custom(&ar0234->ctrls.handler,
      &ar0234_ctrl_roi_v_position, NULL);
  //ar0234->ctrls.flash_time = v4l2_ctrl_new_custom(&ar0234->ctrls.handler,
    //  &ar0234_ctrl_flash_time, NULL);
  ar0234->ctrls.analog_gain = v4l2_ctrl_new_custom(&ar0234->ctrls.handler,
      &ar0234_ctrl_analog_gain, NULL);
  ar0234->ctrls.analog_fine_gain = v4l2_ctrl_new_custom(&ar0234->ctrls.handler,
      &ar0234_ctrl_analog_fine_gain, NULL);

  ar0234->sd.ctrl_handler = &ar0234->ctrls.handler;
  if (ar0234->ctrls.handler.error) {
    ret = ar0234->ctrls.handler.error;
    goto err_ctrls;
  }

  printk("%s: Controls set",__func__); 

  /* setup default controls */
  ret = v4l2_ctrl_handler_setup(&ar0234->ctrls.handler);
  if (ret) {
    dev_err(&client->dev,
        "Error %d setup default controls\n", ret);
    goto err_ctrls;
  }

 
  dev_info(&client->dev, "AR0234 : Probe success !\n");
  return 0;
err_ctrls:
  printk("%s : err_ctrls\n", __func__);
  v4l2_ctrl_handler_free(&ar0234->ctrls.handler);
err_me:
  media_entity_cleanup(&sd->entity);
err_regmap:
  mutex_destroy(&ar0234->lock);
  return ret;
}


static void ar0234_remove(struct i2c_client *client)
{
        struct v4l2_subdev *sd = i2c_get_clientdata(client);
        struct star0234 *ar0234 = to_ar0234(sd);

        v4l2_async_unregister_subdev(sd);
        v4l2_ctrl_handler_free(&ar0234->ctrls.handler);
        media_entity_cleanup(&sd->entity);
        mutex_destroy(&ar0234->lock);
        
}

static struct i2c_driver ar0234_i2c_driver = {
        .driver = {
                .name   = DRIVER_NAME,
                .of_match_table = ar0234_of_id_table,
        },
        .probe          = ar0234_probe,
        .remove         = ar0234_remove,
        .id_table       = ar0234_id,
};



module_i2c_driver(ar0234_i2c_driver);

MODULE_AUTHOR("Anthony Chica Zambrano <achica@rbz.es>");
MODULE_DESCRIPTION("AR0234 Image Sensor driver");
MODULE_LICENSE("GPL v2");

