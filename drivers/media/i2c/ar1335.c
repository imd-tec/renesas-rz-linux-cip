// SPDX-License-Identifier: GPL-2.0
/*
 * AR1335 driver
 *
 * Copyright (C) 2024 Advanced Micro Devices, Inc.
 *
 * Contacts: Anil Kumar Mamidala
 *           Vishnu Vardhan Ravuri
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#define DEBUG
#include <linux/videodev2.h>
#include <media/v4l2-device.h>

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>
#define AR1335_NAME "ar1335"
#define AR1335_MAX_RATIO_MISMATCH 10
#define EXPOSURE_MAX 0xC4E
#define FRAME_LENGTH_LINE_MAX 0x0C4E
#define LINE_LENGTH_PCK_MAX 4656
/* External clock (extclk) frequencies */
#define AR1335_EXTCLK_MIN		(6 * 1000 * 1000)
#define AR1335_EXTCLK_MAX		(48 * 1000 * 1000)
/* PLL and PLL2 */
#define AR1335_PLL_MIN			(320 * 1000 * 1000)
#define AR1335_PLL_MAX			(1200 * 1000 * 1000)
#define MAX_FRAME_RATE 60
#define MIN_FRAME_RATE 30
#define AR1335_DEF_FRAME_RATE 30
#define REG_FRAME_RATE 0x0340

/* Effective pixel sample rate on the pixel array. */
#define AR1335_PIXEL_CLOCK_RATE        (220 * 1000 * 1000)
#define AR1335_PIXEL_CLOCK_MIN         (168 * 1000 * 1000)
#define AR1335_PIXEL_CLOCK_MAX         (414 * 1000 * 1000)

#define AR1335_MIN_X_ADDR_START        8u
#define AR1335_MIN_Y_ADDR_START        8u
#define AR1335_MAX_X_ADDR_END          4231u
#define AR1335_MAX_Y_ADDR_END          3143u

#define AR1335_WIDTH_MIN               0u
#define AR1335_WIDTH_MAX               4239u
#define AR1335_HEIGHT_MIN              0u
#define AR1335_HEIGHT_MAX              3151u

#define AR1335_WIDTH_BLANKING_MIN      240u
#define AR1335_HEIGHT_BLANKING_MIN     142u /* must be even */
#define AR1335_TOTAL_HEIGHT_MAX        65535u /* max_frame_length_lines */
#define AR1335_TOTAL_WIDTH_MAX         65532u /* max_line_length_pck */

#define AR1335_ANA_GAIN_MIN            0x00
#define AR1335_ANA_GAIN_MAX            0x3f
#define AR1335_ANA_GAIN_STEP           0x01
#define AR1335_ANA_GAIN_DEFAULT        0x00

/* AR1335 registers */
#define AR1335_REG_VT_PIX_CLK_DIV              0x0300
#define AR1335_REG_FRAME_LENGTH_LINES          0x0340

#define AR1335_REG_CHIP_ID                     0x0000
#define AR1335_REG_COARSE_INTEGRATION_TIME     0x3012
#define AR1335_REG_ROW_SPEED                   0x3016
#define AR1335_REG_EXTRA_DELAY                 0x3018
#define AR1335_REG_RESET                       0x301A
#define   AR1335_REG_RESET_DEFAULTS            0x0238
#define   AR1335_REG_RESET_GROUP_PARAM_HOLD    0x8000
#define   AR1335_REG_RESET_STREAM              BIT(2)
#define   AR1335_REG_RESET_RESTART             BIT(1)
#define   AR1335_REG_RESET_INIT                BIT(0)

#define AR1335_REG_ANA_GAIN_CODE_GLOBAL        0x3028

#define AR1335_REG_GREEN1_GAIN                 0x3056
#define AR1335_REG_BLUE_GAIN                   0x3058
#define AR1335_REG_RED_GAIN                    0x305A
#define AR1335_REG_GREEN2_GAIN                 0x305C
#define AR1335_REG_GLOBAL_GAIN                 0x305E

#define AR1335_REG_HISPI_TEST_MODE             0x3066
#define   AR1335_REG_HISPI_TEST_MODE_LP11      0x0004

#define AR1335_REG_TEST_PATTERN_MODE           0x3070

#define AR1335_REG_SERIAL_FORMAT               0x31AE
#define   AR1335_REG_SERIAL_FORMAT_MIPI        0x0200

#define AR1335_REG_HISPI_CONTROL_STATUS        0x31C6
#define   AR1335_REG_HISPI_CONTROL_STATUS_FRAMER_TEST_MODE_ENABLE 0x80

#define be                                     cpu_to_be16

static const char * const ar1335_supply_names[] = {
	"vdd_io",	/* I/O (1.8V) supply */
	"vdd",		/* Core, PLL and MIPI (1.2V) supply */
	"vaa",		/* Analog (2.7V) supply */
};
struct ar1335_reg {
	u16 addr;
	u16 val;
};

struct ar1335_res_struct {
	u16 width;
	u16 height;
	u16 out_fmt;
	u16 fps;
	struct ar1335_reg *ar1335_mode;
};

struct ar1335_context_res {
	s32 res_num;
	s32 cur_res;
	struct ar1335_res_struct *res_table;
};
static const s64 ar1335_link_frequencies[] = {
	184000000,
};

#define REGS_ENTRY(a)	{(a), ARRAY_SIZE(a)}
#define REGS(...)	REGS_ENTRY(((const __be16[]){__VA_ARGS__}))

struct initial_reg {
	const __be16 *data; /* data[0] is register address */
	unsigned int count;
};

struct set_bit_entry {
	u16 addr;
	u16 mask;
	u16 val;
};

static const struct initial_reg initial_regs[] = { // 1080P @30fps
    REGS(be(0x0300), be(0x0005)), // VT_PIX_CLK_DIV
    REGS(be(0x0302), be(0x0001)), // VT_SYS_CLK_DIV
    REGS(be(0x0304), be(0x0101)), // PRE_PLL_CLK_DIV
    REGS(be(0x0306), be(0x2E2E)), // PLL_MULTIPLIER
    REGS(be(0x0308), be(0x000A)), // OP_PIX_CLK_DIV
    REGS(be(0x030A), be(0x0001)), // OP_SYS_CLK_DIV
    REGS(be(0x0344), be(0x00C8)), // X_ADDR_START
    REGS(be(0x0348), be(0x0FC7)), // X_ADDR_END
    REGS(be(0x0346), be(0x01F0)), // Y_ADDR_START
    REGS(be(0x034A), be(0x0A5F)), // Y_ADDR_END
    REGS(be(0x034C), be(0x0780)), // X_OUTPUT_SIZE
    REGS(be(0x034E), be(0x0438)), // Y_OUTPUT_SIZE
    REGS(be(0x3040), be(0x0043)), // X_BIN, X_ODD_INC, Y_ODD_INC
    REGS(be(0x3172), be(0x0206)), // DIGBIN_ENABLE
    REGS(be(0x317A), be(0x516E)), // SF_BIN_ENABLE
    REGS(be(0x3F3C), be(0x0003)), // SF_BIN_ENABLE
    REGS(be(0x0400), be(0x0001)), // Scaling Enabling
    REGS(be(0x0404), be(0x0020)), // Scale_M
    REGS(be(0x0342), be(0x1200)), // LINE_LENGTH_PCK
    REGS(be(0x0340), be(0x0C7A)), // FRAME_LENGTH_LINES
    REGS(be(0x0202), be(0x0C5A)), // COARSE_INTEGRATION_TIME
    REGS(be(0x31B0), be(0x005C)), // Frame Preamble
    REGS(be(0x31B2), be(0x002D)), // Line Preamble
    REGS(be(0x31B4), be(0x2412)), // MIPI Timing 0
    REGS(be(0x31B6), be(0x142A)), // MIPI Timing 1
    REGS(be(0x31B8), be(0x2413)), // MIPI Timing 2
    REGS(be(0x31BA), be(0x1C70)), // MIPI Timing 3
    REGS(be(0x31BC), be(0x868B)), // MIPI Timing 4
    REGS(be(0x0112), be(0x0A0A)), // CCP_DATA_FORMAT
};

static int modify_i2c_register(struct i2c_client *client, u16 reg, u16 mask, u16 value);

struct ar1335_ctrls {
	struct v4l2_ctrl_handler handler;
	struct {
		struct v4l2_ctrl *gain;
		struct v4l2_ctrl *red_balance;
		struct v4l2_ctrl *blue_balance;
	};
	struct {
		struct v4l2_ctrl *hblank;
		struct v4l2_ctrl *vblank;
	};
	struct v4l2_ctrl *pixrate;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *test_pattern;
};

struct ar1335_dev {
	struct i2c_client *i2c_client;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct clk *extclk;
	u32 extclk_freq;
	struct v4l2_subdev subdev;
	struct v4l2_ctrl_handler ctrl_handler;

	struct regulator *supplies[ARRAY_SIZE(ar1335_supply_names)];
	struct gpio_desc *reset_gpio;

	/* lock to protect all members below */
	struct mutex lock;
	struct ar1335_res_struct *res_table;
	s32 cur_res;
	struct v4l2_fract frame_rate;
	struct v4l2_mbus_framefmt fmt;
	struct ar1335_ctrls ctrls;
	unsigned int lane_count;
	struct {
		u16 pre;
		u16 mult;
		u16 pre2;
		u16 mult2;
		u16 vt_pix;
	} pll;
};

static inline struct ar1335_dev *to_ar1335_dev(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ar1335_dev, sd);
}

static inline struct v4l2_subdev *ctrl_to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct ar1335_dev,
			     ctrls.handler)->sd;
}

static u32 div64_round(u64 v, u32 d)
{
	return div_u64(v + (d >> 1), d);
}

static u32 div64_round_up(u64 v, u32 d)
{
	return div_u64(v + d - 1, d);
}

static int ar1335_code_to_bpp(struct ar1335_dev *sensor)
{
	switch (sensor->fmt.code) {
	case MEDIA_BUS_FMT_SRGGB10_1X10:
		return 10;
	}

	return -EINVAL;
}


/* Data must be BE16, the first value is the register address */
static int ar1335_write_regs(struct ar1335_dev *sensor, const __be16 *data,
	unsigned int count)
{
	struct i2c_client *client = sensor->i2c_client;
	struct i2c_msg msg;
	int ret;
	unsigned int i;

	msg.addr = client->addr;
	msg.flags = client->flags;
	msg.buf = (u8 *)data;
	msg.len = count * sizeof(*data);

	dev_dbg(&client->dev, "Writing I2C data: ");
	for (i = 0; i < count; i++) {
		dev_dbg(&client->dev, "0x%04x ", be16_to_cpu(data[i]));
	}
	dev_dbg(&client->dev, "\n");

	ret = i2c_transfer(client->adapter, &msg, 1);

	if (ret < 0) {
		v4l2_err(&sensor->sd, "%s: I2C write error\n", __func__);
		return ret;
	}
	return 0;
}

/* Data must be BE16, the first value is the register address */
static int ar1335_read_write_modify(struct ar1335_dev *sensor, const __be16 *data,
	unsigned int count)
{
	struct i2c_client *client = sensor->i2c_client;
	struct i2c_msg msg;
	int ret;
	__be16 reg_addr, reg_mask, reg_value;
	__be16 reg_read_value, reg_modified_value;
	u16 reg_cpu_value;
	int shift_amount;

	if (count == 3) {
		/* Read-modify-write operation */
		reg_addr = data[0];
		reg_mask = data[1];
		reg_value = data[2];

		dev_dbg(&client->dev, "[RMW] Addr=0x%04x, Mask=0x%04x, NewValue=0x%04x\n",
				be16_to_cpu(reg_addr), be16_to_cpu(reg_mask), be16_to_cpu(reg_value));

		/* Read the register first */
		msg.addr = client->addr;
		msg.flags = I2C_M_RD;
		msg.buf = (u8 *)&reg_read_value;
		msg.len = sizeof(reg_read_value);

		ret = i2c_transfer(client->adapter, &msg, 1);
		if (ret < 0) {
			v4l2_err(&sensor->sd, "%s: I2C read error at 0x%04x\n",
					__func__, be16_to_cpu(reg_addr));
			return ret;
		}

		/* Convert read value to CPU endianness */
		reg_cpu_value = be16_to_cpu(reg_read_value);

		dev_dbg(&client->dev, "[READ] Addr=0x%04x, Data=0x%04x\n",
				be16_to_cpu(reg_addr), reg_cpu_value);

		/* Find shift amount (position of the lowest bit in reg_mask) */
		shift_amount = __builtin_ctz(be16_to_cpu(reg_mask));

		/* Shift reg_value to match the bit position of reg_mask */
		u16 shifted_value = (be16_to_cpu(reg_value) << shift_amount);

		/* Apply bitmask and modify value */
		reg_cpu_value = (reg_cpu_value & ~be16_to_cpu(reg_mask)) | 
						(shifted_value & be16_to_cpu(reg_mask));

		/* Convert back to big-endian */
		reg_modified_value = cpu_to_be16(reg_cpu_value);

		dev_dbg(&client->dev, "[MODIFY] Addr=0x%04x, Modified Data=0x%04x (Shift: %d)\n",
				be16_to_cpu(reg_addr), reg_cpu_value, shift_amount);

		/* Write the modified value */
		__be16 write_data[] = { reg_addr, reg_modified_value };
		msg.addr = client->addr;
		msg.flags = 0; /* Write operation */
		msg.buf = (u8 *)write_data;
		msg.len = sizeof(write_data);

		ret = i2c_transfer(client->adapter, &msg, 1);
		if (ret < 0) {
			v4l2_err(&sensor->sd, "%s: I2C write error at 0x%04x\n",
					__func__, be16_to_cpu(reg_addr));
			return ret;
		}

		dev_dbg(&client->dev, "[WRITE] Addr=0x%04x, Final Data=0x%04x\n",
				be16_to_cpu(reg_addr), reg_cpu_value);

		return 0;
	}

	/* Invalid count */
	v4l2_err(&sensor->sd, "%s: Invalid count=%d\n", __func__, count);
	return -EINVAL;
}


static int ar1335_write_reg(struct ar1335_dev *sensor, u16 reg, u16 val)
{
	__be16 buf[2] = {be(reg), be(val)};
	dev_dbg(&sensor->i2c_client->dev, "Writing to reg: 0x%04x, val: 0x%04x\n", reg, val);
	return ar1335_write_regs(sensor, buf, 2);
}

static int ar1335_set_geometry(struct ar1335_dev *sensor)
{
	/* Always use the full native sensor area for readout, output 1920x1080 for scaling */
	u16 x_start = AR1335_MIN_X_ADDR_START;
	u16 y_start = AR1335_MIN_Y_ADDR_START;
	u16 x_end = AR1335_MAX_X_ADDR_END;
	u16 y_end = AR1335_MAX_Y_ADDR_END;
	u16 out_width = 1920;
	u16 out_height = 1080;

	       /* All dimensions are unsigned 12-bit integers */
	__be16 regs[] = {
		be(AR1335_REG_FRAME_LENGTH_LINES),
		be(out_height + sensor->ctrls.vblank->val),
		be(out_width + sensor->ctrls.hblank->val),
		be(x_start),
		be(y_start),
		be(x_end),
		be(y_end),
		be(out_width),
		be(out_height)
	};
	return ar1335_write_regs(sensor, regs, ARRAY_SIZE(regs));
}
static int ar1335_set_gains(struct ar1335_dev *sensor)
{
	int green = sensor->ctrls.gain->val;
	int red = max(green + sensor->ctrls.red_balance->val, 0);
	int blue = max(green + sensor->ctrls.blue_balance->val, 0);
	unsigned int gain = min(red, min(green, blue));
	unsigned int analog = min(gain, 64u); /* range is 0 - 127 */
	__be16 regs[5];

	red   = min(red   - analog + 64, 511u);
	green = min(green - analog + 64, 511u);
	blue  = min(blue  - analog + 64, 511u);
	regs[0] = be(AR1335_REG_GREEN1_GAIN);
	regs[1] = be(green << 7 | analog);
	regs[2] = be(blue  << 7 | analog);
	regs[3] = be(red   << 7 | analog);
	regs[4] = be(green << 7 | analog);
	return ar1335_write_regs(sensor, regs, ARRAY_SIZE(regs));
}

static u32 calc_pll(struct ar1335_dev *sensor, u32 freq, u16 *pre_ptr, u16 *mult_ptr)
{
	u16 pre = 1, mult = 1, new_pre;
	u32 pll = AR1335_PLL_MAX + 1;
	dev_info(&sensor->i2c_client->dev, "Sensor is running at %u Hz input clock\n", sensor->extclk_freq);
	for (new_pre = 1; new_pre < 64; new_pre++) {
		u32 new_pll;
		u32 new_mult = div64_round_up((u64)freq * new_pre,
					      sensor->extclk_freq);

		if (new_mult < 32)
			continue; /* Minimum value */
		if (new_mult > 254)
			break; /* Maximum, larger pre won't work either */
		if (sensor->extclk_freq * (u64)new_mult < AR1335_PLL_MIN *
		    new_pre)
			continue;
		if (sensor->extclk_freq * (u64)new_mult > AR1335_PLL_MAX *
		    new_pre)
			break; /* Larger pre won't work either */
		new_pll = div64_round_up(sensor->extclk_freq * (u64)new_mult,
					 new_pre);
		if (new_pll < pll) {
			pll = new_pll;
			pre = new_pre;
			mult = new_mult;
		}
	}
	pll = div64_round(sensor->extclk_freq * (u64)mult, pre);
	*pre_ptr = pre;
	*mult_ptr = mult;
	return pll;
}

static void ar1335_calc_pll(struct ar1335_dev *sensor)
{
	unsigned int pixel_clock;
	u16 pre, mult;
	u32 vco;
	int bpp;

	dev_dbg(&sensor->i2c_client->dev, "Calculating pixel clock\n");
	pixel_clock = AR1335_PIXEL_CLOCK_RATE * 2 / sensor->lane_count;
	bpp = ar1335_code_to_bpp(sensor);
	if (bpp < 0) {
		dev_err(&sensor->i2c_client->dev, "Invalid bits per pixel\n");
		return;
	}
	dev_dbg(&sensor->i2c_client->dev, "Bits per pixel: %d\n", bpp);
	sensor->pll.vt_pix = bpp / 2;
	vco = pixel_clock * sensor->pll.vt_pix;

	dev_dbg(&sensor->i2c_client->dev, "Calculating PLL with vco=%u, pixel_clock=%u, vt_pix=%u\n", vco, pixel_clock, sensor->pll.vt_pix);
	calc_pll(sensor, vco, &pre, &mult);

	sensor->pll.pre = sensor->pll.pre2 = pre;
	sensor->pll.mult = sensor->pll.mult2 = mult;

	dev_dbg(&sensor->i2c_client->dev, "PLL calculated: pre=%u, mult=%u, vco=%u, pixel_clock=%u, vt_pix=%u\n", pre, mult, vco, pixel_clock, sensor->pll.vt_pix);
}

static int ar1335_pll_config(struct ar1335_dev *sensor)
{
	dev_dbg(&sensor->i2c_client->dev, "Calculating PLL values\n");
	ar1335_calc_pll(sensor);
	dev_dbg(&sensor->i2c_client->dev, "PLL values calculated: vt_pix_clk_div=%u, vt_sys_clk_div=1, pre=%u, mult=%u, pre2=%u, mult2=%u, op_pix_clk_div=%u, op_sys_clk_div=1\n",
		sensor->pll.vt_pix, sensor->pll.pre, sensor->pll.mult, sensor->pll.pre2, sensor->pll.mult2, sensor->pll.vt_pix * 2);

	__be16 pll_regs[] = {
		be(AR1335_REG_VT_PIX_CLK_DIV),
		/* 0x300 */ be(sensor->pll.vt_pix), /* vt_pix_clk_div = bpp / 2 */
		/* 0x302 */ be(1), /* vt_sys_clk_div */
		/* 0x304 */ be((sensor->pll.pre2 << 8) | sensor->pll.pre),
		/* 0x306 */ be((sensor->pll.mult2 << 8) | sensor->pll.mult),
		/* 0x308 */ be(sensor->pll.vt_pix * 2), /* op_pix_clk_div = 2 * vt_pix_clk_div */
		/* 0x30A */ be(1)  /* op_sys_clk_div */
	};

	int ret = ar1335_write_regs(sensor, pll_regs, ARRAY_SIZE(pll_regs));
	if (ret) {
		dev_err(&sensor->i2c_client->dev, "Failed to write PLL registers\n");
		return ret;
	}

	dev_dbg(&sensor->i2c_client->dev, "PLL registers written successfully\n");
	return 0;
}

static int ar1335_set_stream(struct ar1335_dev *sensor, bool on)
{
	int ret, cnt;
	if (on) {
		/* Stop streaming for just a moment */
		dev_dbg(&sensor->i2c_client->dev, "Stopping streaming temporarily\n");
		ret = ar1335_write_reg(sensor, AR1335_REG_RESET,
					   AR1335_REG_RESET_DEFAULTS);
		if (ret) {
			dev_err(&sensor->i2c_client->dev, "Failed to stop streaming\n");
			return ret;
		}

		// /* Set geometry */
		// dev_dbg(&sensor->i2c_client->dev, "Setting geometry\n");
		// ret = ar1335_set_geometry(sensor);
		// if (ret) {
		// 	dev_err(&sensor->i2c_client->dev, "Failed to set geometry\n");
		// 	return ret;
		// }

		/* Configure PLL */
		// dev_dbg(&sensor->i2c_client->dev, "Configuring PLL\n");
		// ret = ar1335_pll_config(sensor);
		// if (ret) {
		// 	dev_err(&sensor->i2c_client->dev, "Failed to configure PLL\n");
		// 	goto err;
		// }

		/* Setup control handler */
		dev_dbg(&sensor->i2c_client->dev, "Setting up control handler\n");
		ret = __v4l2_ctrl_handler_setup(&sensor->ctrls.handler);
		if (ret) {
			dev_err(&sensor->i2c_client->dev, "Failed to setup control handler\n");
			goto err;
		}

		// /* Exit LP-11 mode on clock and data lanes */
		dev_dbg(&sensor->i2c_client->dev, "Exiting LP-11 mode\n");
		ret = ar1335_write_reg(sensor, AR1335_REG_HISPI_CONTROL_STATUS, 0);
		if (ret) {
			dev_err(&sensor->i2c_client->dev, "Failed to exit LP-11 mode\n");
			goto err;
		}
		
		/* Start streaming */
		dev_dbg(&sensor->i2c_client->dev, "Starting streaming\n");
		ret = ar1335_write_reg(sensor, AR1335_REG_RESET,
					   AR1335_REG_RESET_DEFAULTS |
					   AR1335_REG_RESET_STREAM);
		mdelay(2000);
		if (ret) {
			dev_err(&sensor->i2c_client->dev, "Failed to start streaming\n");
			goto err;
		}

		dev_dbg(&sensor->i2c_client->dev, "Sensor is streaming\n");
		return 0;

err:
		dev_err(&sensor->i2c_client->dev, "Error occurred during streaming setup\n");
		return ret;
	} else {
		/*
		 * Reset gain, the sensor may produce all white pixels without
		 * this
		 */
		dev_dbg(&sensor->i2c_client->dev, "Resetting gain\n");
		ret = ar1335_write_reg(sensor, AR1335_REG_GLOBAL_GAIN, 0x2000);
		if (ret) {
			dev_err(&sensor->i2c_client->dev, "Failed to reset gain\n");
			return ret;
		}

		/* Stop streaming */
		dev_dbg(&sensor->i2c_client->dev, "Stopping streaming\n");
		ret = ar1335_write_reg(sensor, AR1335_REG_RESET,
					   AR1335_REG_RESET_DEFAULTS);
		if (ret) {
			dev_err(&sensor->i2c_client->dev, "Failed to stop streaming\n");
			return ret;
		}

		//pm_runtime_put(&sensor->i2c_client->dev);
		dev_dbg(&sensor->i2c_client->dev, "Streaming stopped successfully\n");
		return 0;
	}
}

static struct ar1335_res_struct ar1335_res_table[] = {
	{
		.width = 1920,
		.height = 1080,
	}
};


static int ar1335_match_resolution(struct v4l2_mbus_framefmt *fmt)
{
	// Only support 1080p
	fmt->width = 1920;
	fmt->height = 1080;
	return 0;
}

static s32 ar1335_try_mbus_fmt_locked(struct v4l2_subdev *sd,
				      struct v4l2_mbus_framefmt *fmt)
{
	// Only support 1080p (native resolution which is scaled down)
	fmt->width = 1920;
	fmt->height = 1080;
	return 0;
}


static void ar1335_adj_fmt(struct v4l2_mbus_framefmt *fmt)
{
	fmt->width = clamp(ALIGN(fmt->width, 4), AR1335_WIDTH_MIN,
			   AR1335_WIDTH_MAX);
	fmt->height = clamp(ALIGN(fmt->height, 4), AR1335_HEIGHT_MIN,
			    AR1335_HEIGHT_MAX);
	fmt->code = MEDIA_BUS_FMT_SGRBG10_1X10;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_SRGB;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_DEFAULT;
}

static int ar1335_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *fmt = &format->format;
	struct ar1335_dev *sensor = to_ar1335_dev(sd);
	if (format->pad)
		return -EINVAL;

	mutex_lock(&sensor->lock);
	fmt->width = sensor->fmt.width;
	fmt->height = sensor->fmt.height;
	fmt->code = sensor->fmt.code;
	fmt->field = sensor->fmt.field;
	mutex_unlock(&sensor->lock);
	return 0;
}

static int ar1335_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *format)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ar1335_dev *sensor = to_ar1335_dev(sd);
	struct v4l2_mbus_framefmt *fmt = &format->format;
	int max_vblank, max_hblank;
	s32 idx, ret = 0;

	dev_dbg(&client->dev, "Requested format: width=%d, height=%d, code=%d\n",
		fmt->width, fmt->height, fmt->code);

	mutex_lock(&sensor->lock);

	if (format->which == V4L2_SUBDEV_FORMAT_TRY) {
		dev_dbg(&client->dev, "Trying format\n");
		ar1335_try_mbus_fmt_locked(sd, fmt);
		#ifdef V4L2_SUBDEV_PAD_CONFIG_HAS_TRY_FMT
		cfg->try_fmt = *fmt;
		#endif

		mutex_unlock(&sensor->lock);
		return 0;
	}

	idx = ar1335_try_mbus_fmt_locked(sd, &format->format);
	sensor->cur_res = idx;
	ar1335_calc_pll(sensor);
	sensor->fmt.width = format->format.width;
	sensor->fmt.height = format->format.height;
	sensor->fmt.field = V4L2_FIELD_NONE;

	if (format->format.code == MEDIA_BUS_FMT_SRGGB10_1X10 ||
		format->format.code == MEDIA_BUS_FMT_SRGGB8_1X8) {
		dev_dbg(&client->dev, "Setting format code to %d\n", format->format.code);
		sensor->fmt.code = format->format.code;
	} else {
		dev_err(&client->dev, "Unsupported format code %d\n", format->format.code);
		ret = -EINVAL;
		goto unlock;
	}

	/* Update the exposure and blankings limits. Blankings are also reset
	 * to the minimum.
	 */
	int vblank = FRAME_LENGTH_LINE_MAX - fmt->height;
	int hblank = LINE_LENGTH_PCK_MAX - fmt->width;

	dev_dbg(&client->dev, "Calculated vblank=%d, hblank=%d\n", vblank, hblank);

	max_hblank = AR1335_TOTAL_WIDTH_MAX - sensor->fmt.width;
	ret = __v4l2_ctrl_modify_range(sensor->ctrls.hblank,
					   sensor->ctrls.hblank->minimum,
					   max_hblank, sensor->ctrls.hblank->step,
					   hblank);
	if (ret) {
		dev_err(&client->dev, "Failed to modify hblank range: %d\n", ret);
		goto unlock;
	}

	ret = __v4l2_ctrl_s_ctrl(sensor->ctrls.hblank, hblank);
	if (ret) {
		dev_err(&client->dev, "Failed to set hblank: %d\n", ret);
		goto unlock;
	}

	max_vblank = AR1335_TOTAL_HEIGHT_MAX - sensor->fmt.height;
	ret = __v4l2_ctrl_modify_range(sensor->ctrls.vblank,
					   sensor->ctrls.vblank->minimum,
					   max_vblank, sensor->ctrls.vblank->step,
					   vblank);
	if (ret) {
		dev_err(&client->dev, "Failed to modify vblank range: %d\n", ret);
		goto unlock;
	}

	ret = __v4l2_ctrl_s_ctrl(sensor->ctrls.vblank, vblank);
	if (ret) {
		dev_err(&client->dev, "Failed to set vblank: %d\n", ret);
		goto unlock;
	}

	ret = __v4l2_ctrl_modify_range(sensor->ctrls.exposure,
					   sensor->ctrls.exposure->minimum,
					   EXPOSURE_MAX,
					   sensor->ctrls.exposure->step,
					   sensor->ctrls.exposure->default_value);
	if (ret) {
		dev_err(&client->dev, "Failed to modify exposure range: %d\n", ret);
		goto unlock;
	}

	dev_dbg(&client->dev, "Format set: width=%d, height=%d, code=%d\n",
		sensor->fmt.width, sensor->fmt.height, sensor->fmt.code);

unlock:
	mutex_unlock(&sensor->lock);
	return ret;
}

static u16 ar1335_test_pattern_values[] = {
        0x0, // Normal pixel mode
        0x1, // Solid color
        0x2, // 100% color bar
        0x3, // fade to gray color
        0x100, // walking 1 (10bit)
        0x101, // walking 1 (8bit)
};

static int ar1335_test_pattern(struct v4l2_subdev *sd, s32 val)
{
        struct ar1335_dev *sensor = to_ar1335_dev(sd);
        return ar1335_write_reg(sensor, AR1335_REG_TEST_PATTERN_MODE,
                                    ar1335_test_pattern_values[val]);
}


static int ar1335_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = ctrl_to_sd(ctrl);
	struct ar1335_dev *sensor = to_ar1335_dev(sd);
	int exp_max;
	int ret;

	/* v4l2_ctrl_lock() locks our own mutex */

	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		exp_max = sensor->fmt.height + ctrl->val - 4;
		__v4l2_ctrl_modify_range(sensor->ctrls.exposure,
					 sensor->ctrls.exposure->minimum,
					 exp_max, sensor->ctrls.exposure->step,
					 sensor->ctrls.exposure->default_value);
		break;
	}
	switch (ctrl->id) {
	case V4L2_CID_HBLANK:
	case V4L2_CID_VBLANK:
		ret = ar1335_set_geometry(sensor);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		ret = ar1335_write_reg(sensor, AR1335_REG_ANA_GAIN_CODE_GLOBAL,
				       ctrl->val);
		break;
	case V4L2_CID_GAIN:
	case V4L2_CID_RED_BALANCE:
	case V4L2_CID_BLUE_BALANCE:
		ret = ar1335_set_gains(sensor);
		break;
	case V4L2_CID_EXPOSURE:
		ret = ar1335_write_reg(sensor,
				       AR1335_REG_COARSE_INTEGRATION_TIME,
				       ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN:
		 ret = ar1335_test_pattern(&sensor->sd,ctrl->val);
		break;
	default:
		dev_err(&sensor->i2c_client->dev,
			"Unsupported control %x\n", ctrl->id);
		ret = -EINVAL;
		break;
	}
	return ret;
}

static const struct v4l2_ctrl_ops ar1335_ctrl_ops = {
	.s_ctrl = ar1335_s_ctrl,
};

static const char * const test_pattern_menu[] = {
	"Normal pixel operation",
	"Solid color",
	"100% Color Bar",
	"Fade-to-Gray Color Bars",
	"Walking 1s (10-bit)",
	"Walking 1s (8-bit)",
};

static int ar1335_init_controls(struct ar1335_dev *sensor)
{
	const struct v4l2_ctrl_ops *ops = &ar1335_ctrl_ops;
	struct ar1335_ctrls *ctrls = &sensor->ctrls;
	struct v4l2_ctrl_handler *hdl = &ctrls->handler;
	int max_vblank, max_hblank;
	struct v4l2_ctrl *link_freq;
	int ret;

	v4l2_ctrl_handler_init(hdl, 32);

	/* We can use our own mutex for the ctrl lock */
	hdl->lock = &sensor->lock;

	/* Analog gain */
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_ANALOGUE_GAIN,
			  AR1335_ANA_GAIN_MIN, AR1335_ANA_GAIN_MAX,
			  AR1335_ANA_GAIN_STEP, AR1335_ANA_GAIN_DEFAULT);

	/* Manual gain */
	ctrls->gain = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_GAIN, 0, 511, 1, 40);
	ctrls->red_balance = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_RED_BALANCE,
					       -512, 511, 1, 0);
	ctrls->blue_balance = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_BLUE_BALANCE,
						-512, 511, 1, 0);
	v4l2_ctrl_cluster(3, &ctrls->gain);

	/* Initialize blanking limits using the default 2592x1944 format. */
	max_hblank = AR1335_TOTAL_WIDTH_MAX - AR1335_WIDTH_MAX;
	ctrls->hblank = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_HBLANK,
					  AR1335_WIDTH_BLANKING_MIN,
					  max_hblank, 1,
					  AR1335_WIDTH_BLANKING_MIN);

	max_vblank = AR1335_TOTAL_HEIGHT_MAX - AR1335_HEIGHT_MAX;
	ctrls->vblank = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_VBLANK,
					  AR1335_HEIGHT_BLANKING_MIN,
					  max_vblank, 2,
					  AR1335_HEIGHT_BLANKING_MIN);
	v4l2_ctrl_cluster(2, &ctrls->hblank);

	/* Read-only */
	ctrls->pixrate = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_PIXEL_RATE,
					   AR1335_PIXEL_CLOCK_MIN,
					   AR1335_PIXEL_CLOCK_MAX, 1,
					   AR1335_PIXEL_CLOCK_RATE);
	ctrls->exposure = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_EXPOSURE, 0,
					    EXPOSURE_MAX, 1, 0xC2E);

	link_freq = v4l2_ctrl_new_int_menu(hdl, ops, V4L2_CID_LINK_FREQ,
					ARRAY_SIZE(ar1335_link_frequencies) - 1,
					0, ar1335_link_frequencies);
	if (link_freq)
		link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	ctrls->test_pattern = v4l2_ctrl_new_std_menu_items(hdl, ops,
					V4L2_CID_TEST_PATTERN,
					ARRAY_SIZE(test_pattern_menu) - 1,
					0, 0, test_pattern_menu);

	if (hdl->error) {
		ret = hdl->error;
		goto free_ctrls;
	}

	sensor->sd.ctrl_handler = hdl;
	return 0;

free_ctrls:
	v4l2_ctrl_handler_free(hdl);
	return ret;
}


static int modify_i2c_register(struct i2c_client *client, u16 reg, u16 mask, u16 value) {
    int ret;
    u8 buf[4]; // 2 bytes for address, 2 bytes for data
    struct i2c_msg msgs[2];
    u16 current_value;
    
    // Step 1: Read current value from register
    buf[0] = reg >> 8;  // High byte of register address
    buf[1] = reg & 0xFF; // Low byte of register address
    
    msgs[0].addr  = client->addr;
    msgs[0].flags = 0;
    msgs[0].len   = 2;
    msgs[0].buf   = buf;
    
    msgs[1].addr  = client->addr;
    msgs[1].flags = I2C_M_RD;
    msgs[1].len   = 2;
    msgs[1].buf   = (u8 *)&current_value;
    
    ret = i2c_transfer(client->adapter, msgs, 2);
    if (ret < 0) {
        dev_err(&client->dev, "Failed to read register 0x%04X\n", reg);
        return ret;
    }

    current_value = be16_to_cpu(current_value); // Convert from big-endian

    // Step 2: Modify only the necessary bits
    current_value = (current_value & ~mask) | (value & mask);

    // Step 3: Write the modified value back to the register
    buf[2] = current_value >> 8;  // High byte of new value
    buf[3] = current_value & 0xFF; // Low byte of new value
    
    msgs[0].flags = 0;
    msgs[0].len   = 4;
    msgs[0].buf   = buf;

    ret = i2c_transfer(client->adapter, msgs, 1);
    if (ret < 0) {
        dev_err(&client->dev, "Failed to write register 0x%04X\n", reg);
        return ret;
    }

    return 0;
}


static int ar1335_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct ar1335_dev *sensor = to_ar1335_dev(sd);
	int i;
	dev_err(dev, "%s entry\n", __func__);
	clk_disable_unprepare(sensor->extclk);

	if (sensor->reset_gpio)
		gpiod_set_value_cansleep(sensor->reset_gpio, 1); /* assert RESET signal */

	for (i = ARRAY_SIZE(ar1335_supply_names) - 1; i >= 0; i--) {
		if (sensor->supplies[i])
			regulator_disable(sensor->supplies[i]);
	}
	dev_err(dev, "%s exit\n", __func__);
	return 0;
}

static int ar1335_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct ar1335_dev *sensor = to_ar1335_dev(sd);
	unsigned int cnt;
	int ret;
	dev_err(dev, "%s entry\n", __func__);

	if(sensor->reset_gpio)
	{
		dev_dbg(dev, "Resetting GPIO\n");
		gpiod_set_value_cansleep(sensor->reset_gpio, 0);
		mdelay(1);
		gpiod_set_value_cansleep(sensor->reset_gpio, 1);
		mdelay(1);
}
		for (cnt = 0; cnt < ARRAY_SIZE(initial_regs); cnt++) {
			
			if (initial_regs[cnt].count == 3)
			{
				// Support for read-modify-write operations with a bitmask
				ret = ar1335_read_write_modify(sensor, initial_regs[cnt].data,
						initial_regs[cnt].count);
			}
			else
			{
				ret = ar1335_write_regs(sensor, initial_regs[cnt].data,
						initial_regs[cnt].count);
			}
		
		}

	dev_dbg(dev, "Setting serial format\n");
	ret = ar1335_write_reg(sensor, AR1335_REG_SERIAL_FORMAT,
				   AR1335_REG_SERIAL_FORMAT_MIPI |
				   sensor->lane_count);
	if (ret) {
		dev_err(dev, "Failed to set serial format\n");
		goto off;
	}

	dev_dbg(dev, "Setting MIPI test mode\n");
	ret = ar1335_write_reg(sensor, AR1335_REG_HISPI_TEST_MODE,
				   ((0x40 << sensor->lane_count) - 0x40) |
				   AR1335_REG_HISPI_TEST_MODE_LP11);
	if (ret) {
		dev_err(dev, "Failed to set MIPI test mode\n");
		goto off;
	}

	dev_dbg(dev, "Setting row speed\n");
	ret = ar1335_write_reg(sensor, AR1335_REG_ROW_SPEED, 0x110 |
				   4 / sensor->lane_count);
	if (ret) {
		dev_err(dev, "Failed to set row speed\n");
		goto off;
	}

	dev_err(dev, "%s exit\n", __func__);
	return 0;

off:
	dev_err(dev, "Power on failed, powering off\n");
	ar1335_power_off(dev);
	return ret;
}

static int ar1335_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct ar1335_dev *sensor = to_ar1335_dev(sd);
	dev_err(sd->dev, "ar1335_enum_mbus_code: index = %x\n", code->index);


	if (code->index)
		return -EINVAL;

	code->code = sensor->fmt.code;
	return 0;
}

static int ar1335_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_frame_size_enum *fse)
{

	dev_dbg(sd->dev, "ar1335_enum_frame_size: index = %u", fse->index);
	dev_dbg(sd->dev, "ar1335_enum_frame_size: fse->code = %x", fse->code);

	if (fse->index)
		return -EINVAL;

	if (fse->code != MEDIA_BUS_FMT_SGRBG10_1X10)
		return -EINVAL;

	fse->min_width = AR1335_WIDTH_MIN;
	fse->max_width = AR1335_WIDTH_MAX;
	fse->min_height = AR1335_HEIGHT_MIN;
	fse->max_height = AR1335_HEIGHT_MAX;

	return 0;
}

// static int ar1335_pre_streamon(struct v4l2_subdev *sd, u32 flags)
// {
// 	struct ar1335_dev *sensor = to_ar1335_dev(sd);
// 	int ret;

// 	if (!(flags & V4L2_SUBDEV_PRE_STREAMON_FL_MANUAL_LP))
// 		return -EACCES;
// 	/* Set LP-11 on clock and data lanes */
// 	ret = ar1335_write_reg(sensor, AR1335_REG_HISPI_CONTROL_STATUS,
// 			AR1335_REG_HISPI_CONTROL_STATUS_FRAMER_TEST_MODE_ENABLE);
// 	if (ret)
// 		goto err;

// 	/* Start streaming LP-11 */
// 	ret = ar1335_write_reg(sensor, AR1335_REG_RESET,
// 			       AR1335_REG_RESET_DEFAULTS |
// 			       AR1335_REG_RESET_STREAM);
// 	if (ret)
// 		goto err;
// 	return 0;

// err:
// 	return ret;
// }

static int ar1335_set_frame_interval(struct v4l2_subdev *sd,
				     struct v4l2_subdev_pad_config *cfg,
				     struct v4l2_subdev_frame_interval *ival)
{
	struct ar1335_dev *sensor = to_ar1335_dev(sd);
	struct v4l2_fract *tpf = &ival->interval;

	if (tpf->numerator == 0 || tpf->denominator == 0 ||
		(tpf->denominator > tpf->numerator * MAX_FRAME_RATE)) {
			/* Reset to max frame rate */
			tpf->numerator = 1;
			tpf->denominator = MAX_FRAME_RATE;
	}

	sensor->frame_rate.numerator = tpf->numerator;

	if (tpf->numerator == 30) {
		ar1335_write_reg(sensor, 0x340, 0xC4E);
		ar1335_write_reg(sensor, 0x202, 0xC4E);
		sensor->frame_rate.denominator = tpf->denominator;
	} else if (tpf->numerator == 60) {
		ar1335_write_reg(sensor, 0x340, 0x626);
		ar1335_write_reg(sensor, 0x202, 0x5E8);
		sensor->frame_rate.denominator = tpf->denominator;
	} else {
		ar1335_write_reg(sensor, 0x340, 0xC4E);
		ar1335_write_reg(sensor, 0x202, 0xC4E);
		sensor->frame_rate.denominator = MIN_FRAME_RATE;
	}

	return 0;
}

static int ar1335_get_frame_interval(struct v4l2_subdev *sd,
				     struct v4l2_subdev_pad_config *cfg,
				     struct v4l2_subdev_frame_interval *interval)
{
	struct ar1335_dev *sensor = to_ar1335_dev(sd);

	mutex_lock(&sensor->lock);
	interval->interval.denominator = sensor->frame_rate.denominator;
	interval->interval.numerator = sensor->frame_rate.numerator;
	mutex_unlock(&sensor->lock);

	return 0;
}

static int ar1335_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct ar1335_dev *sensor = to_ar1335_dev(sd);
	int ret, cnt;

	// if (!(flags & V4L2_SUBDEV_PRE_STREAMON_FL_MANUAL_LP))
	// // 	return -EACCES;
	// /* Set LP-11 on clock and data lanes */
	// ret = ar1335_write_reg(sensor, AR1335_REG_HISPI_CONTROL_STATUS,
	// 		AR1335_REG_HISPI_CONTROL_STATUS_FRAMER_TEST_MODE_ENABLE);
	// if (ret)
	// 	goto err;

	// /* Start streaming LP-11 */
	// ret = ar1335_write_reg(sensor, AR1335_REG_RESET,
	// 		       AR1335_REG_RESET_DEFAULTS |
	// 		       AR1335_REG_RESET_STREAM);
	// if (ret)
	// 	goto err;
	// return 0;

	mutex_lock(&sensor->lock);
	ret = ar1335_set_stream(sensor, enable);
	mutex_unlock(&sensor->lock);

err:
	return ret;
}

static int ar1335_link_setup(struct media_entity *entity,
		  const struct media_pad *local,
		  const struct media_pad *remote, u32 flags)
{
	return 0;
}

static const struct media_entity_operations ar1335_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
	.link_setup = ar1335_link_setup
};

static const struct v4l2_subdev_core_ops ar1335_core_ops = {
	.log_status = v4l2_ctrl_subdev_log_status,
};

static const struct v4l2_subdev_video_ops ar1335_video_ops = {
	// .s_frame_interval = ar1335_set_frame_interval,
	// .g_frame_interval = ar1335_get_frame_interval,
	.s_stream = ar1335_s_stream,
	//.pre_streamon = ar1335_pre_streamon,
};

static const struct v4l2_subdev_pad_ops ar1335_pad_ops = {
	.enum_mbus_code = ar1335_enum_mbus_code,
	.enum_frame_size = ar1335_enum_frame_size,
//	.set_frame_interval = ar1335_set_frame_interval,
//	.get_frame_interval = ar1335_get_frame_interval,
	.get_fmt = ar1335_get_fmt,
	.set_fmt = ar1335_set_fmt,
};

static const struct v4l2_subdev_ops ar1335_subdev_ops = {
	.core = &ar1335_core_ops,
	.video = &ar1335_video_ops,
	.pad = &ar1335_pad_ops,
};

static int ar1335_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct v4l2_fwnode_endpoint ep = {
		.bus_type = V4L2_MBUS_CSI2_DPHY
	};
	struct device *dev = &client->dev;
	struct fwnode_handle *endpoint;
	struct ar1335_dev *sensor;
	unsigned int cnt;
	int ret;
	static int num_defer = 0;

	dev_info(dev, "Starting AR1335 probe\n");
	// if (num_defer<10)
	// {
	// 	num_defer++;
	// 	dev_err(dev, "ar1335: Deferring\n");
	// 	return -EPROBE_DEFER;
	// }

	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor) {
		dev_err(dev, "Failed to allocate memory for sensor\n");
		return -ENOMEM;
	}

	sensor->i2c_client = client;
	sensor->fmt.width = AR1335_WIDTH_MAX;
	sensor->fmt.height = AR1335_HEIGHT_MAX;
	sensor->frame_rate.numerator = 1;
	sensor->frame_rate.denominator = AR1335_DEF_FRAME_RATE;
	endpoint = fwnode_graph_get_endpoint_by_id(dev_fwnode(dev), 0, 0,
						   FWNODE_GRAPH_ENDPOINT_NEXT);
	if (!endpoint) {
		dev_err(dev, "Endpoint node not found\n");
		return -EINVAL;
	}

	ret = v4l2_fwnode_endpoint_parse(endpoint, &ep);
	fwnode_handle_put(endpoint);
	if (ret) {
		dev_err(dev, "Could not parse endpoint\n");
		return ret;
	}

	if (ep.bus_type != V4L2_MBUS_CSI2_DPHY) {
		dev_err(dev, "Invalid bus type, must be MIPI CSI2\n");
		return -EINVAL;
	}

	sensor->lane_count = ep.bus.mipi_csi2.num_data_lanes;
	switch (sensor->lane_count) {
	case 1:
	case 2:
	case 4:
		break;
	default:
		dev_err(dev, "Invalid number of MIPI data lanes\n");
		return -EINVAL;
	}
	/* Get master clock (extclk) */
	sensor->extclk = devm_clk_get(dev, "extclk");
	if (IS_ERR(sensor->extclk)) {
		dev_err(dev, "Failed to get extclk\n");
		return PTR_ERR(sensor->extclk);
	}

	sensor->extclk_freq = clk_get_rate(sensor->extclk);

	if (sensor->extclk_freq < AR1335_EXTCLK_MIN ||
		sensor->extclk_freq > AR1335_EXTCLK_MAX) {
		dev_err(dev, "Extclk frequency out of range: %u Hz\n",
			sensor->extclk_freq);
		return -EINVAL;
	}
	dev_err(&client->dev, "Sensor is running at %u Hz input clock\n", sensor->extclk_freq); 

	/* Request optional reset pin (usually active low) and assert it */
	sensor->reset_gpio = devm_gpiod_get_optional(dev, "reset",
							 GPIOD_OUT_HIGH);

	struct v4l2_subdev *sd = &sensor->sd;

	v4l2_i2c_subdev_init(&sensor->sd, client, &ar1335_subdev_ops);


	strscpy(sd->name, AR1335_NAME, sizeof(sd->name));
	strlcat(sd->name, ".", sizeof(sd->name));
	strlcat(sd->name, dev_name(dev), sizeof(sd->name));
	dev_err(dev, "name %s\n", sd->name);

	sensor->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;
	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;
	sensor->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	sensor->sd.entity.ops = &ar1335_media_ops;
	ret = media_entity_pads_init(&sensor->sd.entity, 1, &sensor->pad);
	if (ret) {
		dev_err(dev, "Failed to initialize media entity pads\n");
		return ret;
	}

	for (cnt = 0; cnt < ARRAY_SIZE(ar1335_supply_names); cnt++) {
		struct regulator *supply = devm_regulator_get(dev,
						ar1335_supply_names[cnt]);

		if (IS_ERR(supply)) {
			dev_info(dev, "No %s regulator found: %li\n",
				 ar1335_supply_names[cnt], PTR_ERR(supply));
			return PTR_ERR(supply);
		} 
		sensor->supplies[cnt] = supply;
	}

	mutex_init(&sensor->lock);

	ret = ar1335_init_controls(sensor);
	if (ret) {
		dev_err(dev, "Failed to initialize controls\n");
		goto entity_cleanup;
	}

	ar1335_adj_fmt(&sensor->fmt);

	ret = v4l2_async_register_subdev(&sensor->sd);
	if (ret) {
		dev_err(dev, "Failed to register subdev\n");
		goto free_ctrls;
	}
	ret = ar1335_power_on(&client->dev);
	if (ret) {
		dev_err(dev, "Failed to power on sensor\n");
		goto disable;
	}
	dev_info(&client->dev, "AR1335 probe completed successfully\n");
	return 0;

disable:
	v4l2_async_unregister_subdev(&sensor->sd);
	media_entity_cleanup(&sensor->sd.entity);
free_ctrls:
	v4l2_ctrl_handler_free(&sensor->ctrls.handler);
entity_cleanup:
	media_entity_cleanup(&sensor->sd.entity);
	mutex_destroy(&sensor->lock);
	return ret;
}

static int ar1335_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ar1335_dev *sensor = to_ar1335_dev(sd);

	v4l2_async_unregister_subdev(&sensor->sd);
	media_entity_cleanup(&sensor->sd.entity);
	v4l2_ctrl_handler_free(&sensor->ctrls.handler);
	mutex_destroy(&sensor->lock);
	return 0;
}

static const struct of_device_id ar1335_id[] = {
	{.compatible = "onnn,ar1335" },
	{}
};
MODULE_DEVICE_TABLE(of, ar1335_id);

static struct i2c_driver ar1335_driver = {
	.driver = {
		.name  = AR1335_NAME,
		.of_match_table = ar1335_id,
	},
	.probe = ar1335_probe,
	.remove = ar1335_remove,
};

module_i2c_driver(ar1335_driver);

MODULE_AUTHOR("Anil Kumar Mamidala, Vishnu Vardhan Ravuri");
MODULE_DESCRIPTION("V4L driver for camera sensor AR1335");
MODULE_LICENSE("GPL v2");

