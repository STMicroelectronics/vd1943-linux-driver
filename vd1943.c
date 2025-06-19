// SPDX-License-Identifier: GPL-2.0
/*
 * A V4L2 driver for ST VD1943 RGB-NIR Image Sensor.
 * Copyright (C) 2024, STMicroelectronics SA
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

/* Backward compatibility */
#include <linux/version.h>

#if KERNEL_VERSION(6, 12, 0) > LINUX_VERSION_CODE
#include <asm/unaligned.h>
#else
#include <linux/unaligned.h>
#endif

#if KERNEL_VERSION(6, 8, 0) > LINUX_VERSION_CODE
/*
 * Warning : CCI_REGxy_LE definitions doesn't fit exactly with v4l2-cci.h .
 * In fact endianness is managed directly in vd1943_read/write() functions.
 */
#include <linux/bitfield.h>
#define CCI_REG_ADDR_MASK		GENMASK(15, 0)
#define CCI_REG_WIDTH_SHIFT		16
#define CCI_REG_ADDR(x)			FIELD_GET(CCI_REG_ADDR_MASK, x)
#define CCI_REG8(x)			((1 << CCI_REG_WIDTH_SHIFT) | (x))
#define CCI_REG16_LE(x)			((2 << CCI_REG_WIDTH_SHIFT) | (x))
#define CCI_REG32_LE(x)			((4 << CCI_REG_WIDTH_SHIFT) | (x))
#else
#include <media/v4l2-cci.h>
#endif

#if KERNEL_VERSION(5, 18, 0) > LINUX_VERSION_CODE
#define MIPI_CSI2_DT_RAW8	0x2a
#define MIPI_CSI2_DT_RAW10	0x2b
#define MIPI_CSI2_DT_RAW12	0x2c
#else
#include <media/mipi-csi2.h>
#endif

#if KERNEL_VERSION(5, 15, 0) > LINUX_VERSION_CODE
#define HZ_PER_MHZ		1000000UL
#define MEGA			1000000UL
#else
#include <linux/units.h>
#endif

#if KERNEL_VERSION(6, 10, 0) > LINUX_VERSION_CODE
#define MEDIA_BUS_FMT_RGBNIR4X4_1X8			0x3021
#define MEDIA_BUS_FMT_RGBNIR4X4_1X10			0x3022
#define MEDIA_BUS_FMT_RGBNIR4X4_1X12			0x3023
#endif

/* Register Map */
#define VD1943_REG_MODEL_ID				CCI_REG32_LE(0x0000)
#define VD1943_MODEL_ID_1_3				0x53393430
#define VD1943_MODEL_ID_1_4				0x53393431
#define VD1943_REG_ROM_REVISION				CCI_REG16_LE(0x000c)
#define VD1943_ROM_1_3					0x400
#define VD1943_ROM_1_4					0x540
#define VD1943_REG_CFA_SELECTION			CCI_REG16_LE(0x000e)
#define VD1943_OPTICAL_RGBIR				0x00
#define VD1943_OPTICAL_MONO				0x01
#define VD1943_REG_SYSTEM_FSM				CCI_REG8(0x0044)
#define VD1943_SYSTEM_FSM_SYSTEM_UP			0x01
#define VD1943_SYSTEM_FSM_BOOT				0x02
#define VD1943_SYSTEM_FSM_SW_STBY			0x03
#define VD1943_SYSTEM_FSM_STREAMING			0x04
#define VD1943_REG_TEMPERATURE				CCI_REG16_LE(0x006a)
#define VD1943_REG_SYSTEM_UP				CCI_REG8(0x0514)
#define VD1943_CMD_ACK					0x00
#define VD1943_CMD_START_SENSOR				0x01
#define VD1943_REG_BOOT					CCI_REG8(0x0515)
#define VD1943_CMD_LOAD_CERTIFICATE			0x01
#define VD1943_CMD_LOAD_FWP				0x02
#define VD1943_CMD_END_BOOT				0x10
#define VD1943_REG_SW_STBY				CCI_REG8(0x0516)
#define VD1943_CMD_START_STREAMING			0x01
#define VD1943_CMD_THSENS_READ				0x02
#define VD1943_CMD_UPDATE_VT_RAM_START			0x03
#define VD1943_CMD_UPDATE_VT_RAM_END			0x04
#define VD1943_REG_STREAMING				CCI_REG8(0x0517)
#define VD1943_CMD_STOP_STREAM				0x01
#define VD1943_REG_EXT_CLOCK				CCI_REG32_LE(0x0734)
#define VD1943_REG_MIPI_DATA_RATE			CCI_REG32_LE(0x0738)
#define VD1943_REG_LANE_NB_SEL				CCI_REG8(0x0743)
#define VD1943_LANES_NB_4				0
#define VD1943_LANES_NB_2				1
#define VD1943_REG_ROI_WIDTH_OFFSET			CCI_REG16_LE(0x090c)
#define VD1943_REG_ROI_HEIGHT_OFFSET			CCI_REG16_LE(0x090e)
#define VD1943_REG_ROI_WIDTH				CCI_REG16_LE(0x0910)
#define VD1943_REG_ROI_HEIGHT				CCI_REG16_LE(0x0912)
#define VD1943_REG_ROI_DT				CCI_REG8(0x0914)
#define VD1943_REG_LINE_LENGTH				CCI_REG16_LE(0x0934)
#define VD1943_REG_ORIENTATION				CCI_REG8(0x0937)
#define VD1943_REG_PATGEN_CTRL				CCI_REG8(0x0938)
#define VD1943_REG_OIF_LANE_PHY_MAP			CCI_REG8(0x093a)
#define VD1943_REG_OIF_LANE_PHY_SWAP			CCI_REG8(0x093b)
#define VD1943_REG_VT_CTRL				CCI_REG8(0x0ac6)
#define VD1943_REG_OIF_ISL_ENABLE			CCI_REG8(0x0ac7)
#define VD1943_REG_GPIO_0_CTRL				CCI_REG8(0x0ad4)
#define VD1943_GPIOX_STROBE_MODE			0x00
#define VD1943_GPIOX_GPIO_IN				0x03
#define VD1943_GPIOX_FSYNC_IN				0x05
#define VD1943_REG_DARKCAL_ENABLE			CCI_REG8(0x0af3)
#define VD1943_REG_SENSOR_CONFIGURATION			CCI_REG32_LE(0x0b40)
#define VD1943_CONFIG_GS_NATIVE_RAW8			0x01
#define VD1943_CONFIG_GS_NATIVE_RAW10			0x02
#define VD1943_CONFIG_GS_RGB_RAW8			0x05
#define VD1943_CONFIG_GS_RGB_RAW10			0x06
#define VD1943_CONFIG_GS_IR_RAW8			0x0f
#define VD1943_CONFIG_GS_IR_RAW10			0x10
#define VD1943_CONFIG_RS_NATIVE_RAW8			0x1a
#define VD1943_CONFIG_RS_NATIVE_RAW10			0x1b
#define VD1943_CONFIG_RS_NATIVE_RAW12			0x1c
#define VD1943_CONFIG_RS_RGB_RAW8			0x1d
#define VD1943_CONFIG_RS_RGB_RAW10			0x1e
#define VD1943_CONFIG_RS_RGB_RAW12			0x1f
#define VD1943_REG_FRAME_LENGTH				CCI_REG16_LE(0x0b46)
#define VD1943_REG_GPIO_CTRL				CCI_REG8(0x0b49)
#define VD1943_REG_DARKCAL_PEDESTAL			CCI_REG16_LE(0x0b6e)
#define VD1943_REG_ANALOG_GAIN				CCI_REG8(0x0c79)
#define VD1943_REG_INTEGRATION_TIME_PRIMARY		CCI_REG16_LE(0x0c7a)
#define VD1943_REG_INTEGRATION_TIME_IR			CCI_REG16_LE(0x0c7c)
#define VD1943_REG_INTEGRATION_TIME_SHORT		CCI_REG16_LE(0x0c7e)
#define VD1943_REG_DIGITAL_GAIN_R			CCI_REG16_LE(0x0c80)
#define VD1943_REG_DIGITAL_GAIN_G			CCI_REG16_LE(0x0c82)
#define VD1943_REG_DIGITAL_GAIN_B			CCI_REG16_LE(0x0c84)
#define VD1943_REG_DIGITAL_GAIN_IR			CCI_REG16_LE(0x0c86)

/*
 * The VD1943/VD5943 pixel array are organized as follows:
 *
 * +--------------------------------------+
 * |                                      | \
 * |   +------------------------------+   |  |
 * |   |                              |   |  |
 * |   |                              |   |  |
 * |   |                              |   |  |
 * |   |      Default resolution      |   |  | Native height (1984)
 * |   |         1920 x 1080          |   |  |
 * |   |                              |   |  |
 * |   |                              |   |  |
 * |   +------------------------------+   |  |
 * |                                      | /
 * +--------------------------------------+
 *   <------------------------------------>
 *                     \-------------------  Native width (2560)
 *
 * The native resolution is 2560x1984.
 * The recommended/default resolution is 1920x1080.
 */
#define VD1943_NATIVE_WIDTH				2560
#define VD1943_NATIVE_HEIGHT				1984
#define VD1943_DEFAULT_WIDTH				1920
#define VD1943_DEFAULT_HEIGHT				1080
#define VD1943_DEFAULT_MODE				3

/* vd1943/vd5943 is a combined Rolling and Global Shutter sensor */
#define VD1943_GS_MODE					0
#define VD1943_RS_MODE					1

/* Line length and Frame length (settings are for standard 10bits ADC mode) */
#define VD1943_LINE_LENGTH_MIN				3375
#define VD1943_VBLANK_MIN				238U
#define VD1943_FRAME_LENGTH_MAX				0xffff
#define VD1943_DEFAULT_FPS30				30

/* Exposure settings */
#define VD1943_EXPOSURE_MARGIN				27
#define VD1943_EXPOSURE_DEFAULT				840U

/* Output Interface settings */
#define VD1943_MAX_CSI_DATA_LANES			4
#define VD1943_LINK_FREQ_DEF_2LANES			750000000UL
#define VD1943_LINK_FREQ_DEF_4LANES			650000000UL

/* GPIOs */
#define VD1943_NB_GPIOS					4

/* parse-SNIP: Custom-CIDs */
#define V4L2_CID_TEMPERATURE			(V4L2_CID_USER_BASE | 0x1020)
#define V4L2_CID_DARKCAL_PEDESTAL		(V4L2_CID_USER_BASE | 0x1024)
#define V4L2_CID_SLAVE_MODE			(V4L2_CID_USER_BASE | 0x1025)
#define V4L2_CID_SHUTTER_MODE			(V4L2_CID_USER_BASE | 0x1026)
/* parse-SNAP: */

#include "vd1943_fmwpatch_vd1943.c"
#include "vd1943_fmwpatch_vd5943.c"
#include "vd1943_vtpatch.c"

/* regulator supplies */
static const char *const vd1943_supply_names[] = {
	"vcore",
	"vddio",
	"vana",
};

/* -----------------------------------------------------------------------------
 * Models, Modes and formats
 */

enum vd1943_models {
	VD1943_MODEL_VD1943, // RGBNir variant
	VD1943_MODEL_VD5943, // Mono variant
};

struct vd1943_mode {
	u32 width;
	u32 height;
};

/**
 * DOC: Supported Modes
 *
 * The vd1943/vd5943 driver supports 6 modes described below :
 *
 * ======= ======== ====================
 *  Width   Height   Comment
 * ======= ======== ====================
 *   2560     1984   Native resolution
 *   2560     1600
 *   2560     1440
 *   1920     1080   Default resolution
 *   1280      720
 *    640      480
 * ======= ======== ====================
 *
 * Depending of the configured shutter mode (Rolling vs Global), the max frame
 * rate can be different. In full resolution the hardware supports up to :
 * - 50 fps in Rolling Shutter
 * - 100 fps in Global Shutter (depending on MIPI bandwidth)
 *
 * Note that in lower resolution, framerate can be higher.
 */

static const struct vd1943_mode vd1943_supported_modes[] = {
	{
		.width = VD1943_NATIVE_WIDTH,
		.height = VD1943_NATIVE_HEIGHT,
	},
	{
		.width = VD1943_NATIVE_WIDTH,
		.height = 1600,
	},
	{
		.width = VD1943_NATIVE_WIDTH,
		.height = 1440,
	},
	{
		.width = VD1943_DEFAULT_WIDTH,
		.height = VD1943_DEFAULT_HEIGHT,
	},
	{
		.width = 1280,
		.height = 720,
	},
	{
		.width = 640,
		.height = 480,
	},
};

struct vd1943_config {
	u8 sensor_conf;
	unsigned int mbus_code;
};

static const struct vd1943_config vdx941_configs[2][2][4] = {
	{
		/* VD1943_MODEL_VD1943 : RGBNir variant */
		{
			/* Global Shutter modes */
			{
				.sensor_conf = VD1943_CONFIG_GS_IR_RAW8,
				.mbus_code = MEDIA_BUS_FMT_Y8_1X8,
			},
			{
				.sensor_conf = VD1943_CONFIG_GS_IR_RAW10,
				.mbus_code = MEDIA_BUS_FMT_Y10_1X10,
			},
			{
				.sensor_conf = VD1943_CONFIG_GS_RGB_RAW8,
				.mbus_code = MEDIA_BUS_FMT_SGBRG8_1X8,
			},
			{
				.sensor_conf = VD1943_CONFIG_GS_RGB_RAW10,
				.mbus_code = MEDIA_BUS_FMT_SGBRG10_1X10,
			},
			/* TODO : Custom mbus codes break libcamera apps */
			/*{
			 *	.sensor_conf = VD1943_CONFIG_GS_NATIVE_RAW8,
			 *	.mbus_code = MEDIA_BUS_FMT_RGBNIR4X4_1X8,
			 *},
			 *{
			 *	.sensor_conf = VD1943_CONFIG_GS_NATIVE_RAW10,
			 *	.mbus_code = MEDIA_BUS_FMT_RGBNIR4X4_1X10,
			 *},
			 */
		},
		{
			/* Rolling Shutter modes */
			{
				.sensor_conf = VD1943_CONFIG_RS_RGB_RAW8,
				.mbus_code = MEDIA_BUS_FMT_SGBRG8_1X8,
			},
			{
				.sensor_conf = VD1943_CONFIG_RS_RGB_RAW10,
				.mbus_code = MEDIA_BUS_FMT_SGBRG10_1X10,
			},
			{
				.sensor_conf = VD1943_CONFIG_RS_RGB_RAW12,
				.mbus_code = MEDIA_BUS_FMT_SGBRG12_1X12,
			},
			/* TODO : Custom mbus codes break libcamera apps */
			/*{
			 *	.sensor_conf = VD1943_CONFIG_RS_NATIVE_RAW8,
			 *	.mbus_code = MEDIA_BUS_FMT_RGBNIR4X4_1X8,
			 *},
			 *{
			 *	.sensor_conf = VD1943_CONFIG_RS_NATIVE_RAW10,
			 *	.mbus_code = MEDIA_BUS_FMT_RGBNIR4X4_1X10,
			 *},
			 *{
			 *	.sensor_conf = VD1943_CONFIG_RS_NATIVE_RAW12,
			 *	.mbus_code = MEDIA_BUS_FMT_RGBNIR4X4_1X12,
			 *},
			 */
		},
	},
	{
		/* VD1943_MODEL_VD5943 : Mono variant */
		{
			/* Global Shutter modes */
			{
				.sensor_conf = VD1943_CONFIG_GS_NATIVE_RAW8,
				.mbus_code = MEDIA_BUS_FMT_Y8_1X8,
			},
			{
				.sensor_conf = VD1943_CONFIG_GS_NATIVE_RAW10,
				.mbus_code = MEDIA_BUS_FMT_Y10_1X10,
			},
		},
		{
			/* Rolling Shutter modes */
			{
				.sensor_conf = VD1943_CONFIG_RS_NATIVE_RAW8,
				.mbus_code = MEDIA_BUS_FMT_Y8_1X8,
			},
			{
				.sensor_conf = VD1943_CONFIG_RS_NATIVE_RAW10,
				.mbus_code = MEDIA_BUS_FMT_Y10_1X10,
			},
			{
				.sensor_conf = VD1943_CONFIG_RS_NATIVE_RAW12,
				.mbus_code = MEDIA_BUS_FMT_Y12_1X12,
			},
		},
	},
};

#if KERNEL_VERSION(6, 8, 0) > LINUX_VERSION_CODE
/* Big endian register addresses and 8b, 16b or 32b little endian values. */
static const struct regmap_config vd1943_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.reg_format_endian = REGMAP_ENDIAN_BIG,
};
#endif

struct vd1943 {
	struct i2c_client *i2c_client;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct regulator_bulk_data supplies[ARRAY_SIZE(vd1943_supply_names)];
	struct gpio_desc *reset_gpio;
	struct clk *xclk;
	struct regmap *regmap;
	u32 xclk_freq;
	u32 pixel_rate;
	u8 nb_of_lane;
	u64 mipi_bandwidth;
	u8 oif_lane_phy_map;
	u8 oif_lane_phy_swap;
	u32 gpios[VD1943_NB_GPIOS];
	u8 ext_vt_sync;
	unsigned long ext_leds_mask;
	enum vd1943_models model;
	bool is_fastboot;
#if KERNEL_VERSION(5, 19, 0) > LINUX_VERSION_CODE
	/* lock to protect all members below */
	struct mutex lock;
#endif
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *vt_pixrate_ctrl;
	struct v4l2_ctrl *hblank_ctrl;
	struct v4l2_ctrl *vblank_ctrl;
	struct {
		struct v4l2_ctrl *hflip_ctrl;
		struct v4l2_ctrl *vflip_ctrl;
	};
	struct v4l2_ctrl *patgen_ctrl;
	struct v4l2_ctrl *expo_ctrl;
	struct v4l2_ctrl *again_ctrl;
	struct v4l2_ctrl *dgain_ctrl;
	struct v4l2_ctrl *slave_ctrl;
	struct v4l2_ctrl *led_ctrl;
	struct v4l2_ctrl *shutter_ctrl;
	struct v4l2_ctrl *pedestal_ctrl;
	bool streaming;
#if KERNEL_VERSION(5, 19, 0) > LINUX_VERSION_CODE
	struct v4l2_mbus_framefmt active_fmt;
	struct v4l2_rect active_crop;
#endif
};

static inline struct vd1943 *to_vd1943(struct v4l2_subdev *sd)
{
#if KERNEL_VERSION(6, 2, 0) > LINUX_VERSION_CODE
	return container_of(sd, struct vd1943, sd);
#else
	return container_of_const(sd, struct vd1943, sd);
#endif
}

static inline struct v4l2_subdev *ctrl_to_sd(struct v4l2_ctrl *ctrl)
{
#if KERNEL_VERSION(6, 2, 0) > LINUX_VERSION_CODE
	return &container_of(ctrl->handler, struct vd1943, ctrl_handler)->sd;
#else
	return &container_of_const(ctrl->handler, struct vd1943, ctrl_handler)
			->sd;
#endif
}

/* -----------------------------------------------------------------------------
 * HW access : Big endian reg addresses and 8b, 16b or 32b little endian values
 */

#if KERNEL_VERSION(6, 8, 0) > LINUX_VERSION_CODE
static int vd1943_read(struct vd1943 *sensor, u32 reg, u32 *val, int *err)
{
	struct i2c_client *client = sensor->i2c_client;
	unsigned int len = (reg >> CCI_REG_WIDTH_SHIFT) & 7;
	u8 buf[4];
	int ret;

	if (err && *err)
		return *err;

	reg = reg & CCI_REG_ADDR_MASK;

	ret = regmap_bulk_read(sensor->regmap, reg, buf, len);
	if (ret) {
		dev_err(&client->dev, "%s: Error reading reg 0x%4x: %d\n",
			__func__, reg, ret);
		goto out;
	}

	switch (len) {
	case 1:
		*val = buf[0];
		break;
	case 2:
		*val = get_unaligned_le16(buf);
		break;
	case 4:
		*val = get_unaligned_le32(buf);
		break;
	default:
		dev_err(&client->dev,
			"%s: Error invalid reg-width %u for reg 0x%04x\n",
			__func__, len, reg);
		ret = -EINVAL;
		break;
	}

	dev_dbg(&client->dev, "%s: reg 0x%04x -> %d\n", __func__, reg, *val);

out:
	if (ret && err)
		*err = ret;

	return ret;
}

static int vd1943_write(struct vd1943 *sensor, u32 reg, u32 val, int *err)
{
	struct i2c_client *client = sensor->i2c_client;
	unsigned int len = (reg >> CCI_REG_WIDTH_SHIFT) & 7;
	u8 buf[4];
	int ret;

	if (err && *err)
		return *err;

	reg = reg & CCI_REG_ADDR_MASK;
	switch (len) {
	case 1:
		buf[0] = val;
		break;
	case 2:
		put_unaligned_le16(val, buf);
		break;
	case 4:
		put_unaligned_le32(val, buf);
		break;
	default:
		dev_err(&client->dev,
			"%s: Error invalid reg-width %u for reg 0x%04x\n",
			__func__, len, reg);
		ret = -EINVAL;
		goto out;
	}

	ret = regmap_bulk_write(sensor->regmap, reg, buf, len);
	if (ret)
		dev_err(&client->dev, "%s: Error writing reg 0x%04x: %d\n",
			__func__, reg, ret);

	dev_dbg(&client->dev, "%s: reg 0x%04x <- %d\n", __func__, reg, val);

out:
	if (ret && err)
		*err = ret;

	return ret;
}
#else
#define vd1943_read(sensor, reg, val, err) \
	cci_read((sensor)->regmap, reg, (u64 *)val, err)

#define vd1943_write(sensor, reg, val, err) \
	cci_write((sensor)->regmap, reg, (u64)val, err)
#endif

static int vd1943_write_array(struct vd1943 *sensor, u32 reg, unsigned int len,
			      const u8 *array, int *err)
{
	unsigned int chunk_sz = 1024;
	unsigned int sz;
	int ret;

	if (err && *err)
		return *err;

	/*
	 * This loop isn't necessary but in certains conditions (platforms, cpu
	 * load, etc.) it has been observed that the bulk write could timeout.
	 */
	while (len) {
		sz = min(len, chunk_sz);
		ret = regmap_bulk_write(sensor->regmap, reg, array, sz);
		if (ret < 0)
			goto out;
		len -= sz;
		reg += sz;
		array += sz;
	}

out:
	if (ret && err)
		*err = ret;

	return ret;
}

static int vd1943_poll_reg(struct vd1943 *sensor, u32 reg, u8 poll_val,
			   int *err)
{
	unsigned int val = 0;
	int ret;

	if (err && *err)
		return *err;

	ret = regmap_read_poll_timeout(sensor->regmap, CCI_REG_ADDR(reg), val,
				       (val == poll_val), 2000,
				       500 * USEC_PER_MSEC);

	if (ret && err)
		*err = ret;

	return ret;
}

static int vd1943_wait_state(struct vd1943 *sensor, int state, int *err)
{
	return vd1943_poll_reg(sensor, VD1943_REG_SYSTEM_FSM, state, err);
}

/* -----------------------------------------------------------------------------
 * Controls: definitions, helpers and handlers
 */

static const char *const vd1943_tp_menu[] = { "Disabled", "Dgrey" };

static const char *const vd1943_shutter_menu[] = { "Global Shutter",
						   "Rolling Shutter" };

static const s64 vd1943_link_freq_2lanes[] = { VD1943_LINK_FREQ_DEF_2LANES };

static const s64 vd1943_link_freq_4lanes[] = { VD1943_LINK_FREQ_DEF_4LANES };

static u8 vd1943_get_datatype(__u32 code)
{
	/* TODO : Correctly handle MEDIA_BUS_FMT_RGBNIR4X4 mbus codes */
	switch (code) {
	case MEDIA_BUS_FMT_Y8_1X8:
	case MEDIA_BUS_FMT_SGRBG8_1X8:
	case MEDIA_BUS_FMT_SRGGB8_1X8:
	case MEDIA_BUS_FMT_SBGGR8_1X8:
	case MEDIA_BUS_FMT_SGBRG8_1X8:
	default:
		return MIPI_CSI2_DT_RAW8;
	case MEDIA_BUS_FMT_Y10_1X10:
	case MEDIA_BUS_FMT_SGRBG10_1X10:
	case MEDIA_BUS_FMT_SRGGB10_1X10:
	case MEDIA_BUS_FMT_SBGGR10_1X10:
	case MEDIA_BUS_FMT_SGBRG10_1X10:
		return MIPI_CSI2_DT_RAW10;
	case MEDIA_BUS_FMT_Y12_1X12:
	case MEDIA_BUS_FMT_SGRBG12_1X12:
	case MEDIA_BUS_FMT_SRGGB12_1X12:
	case MEDIA_BUS_FMT_SBGGR12_1X12:
	case MEDIA_BUS_FMT_SGBRG12_1X12:
		return MIPI_CSI2_DT_RAW12;
	}
}

static u8 vd1943_get_bpp(__u32 code)
{
	/* TODO : Correctly handle MEDIA_BUS_FMT_RGBNIR4X4 mbus codes */
	switch (code) {
	case MEDIA_BUS_FMT_Y8_1X8:
	case MEDIA_BUS_FMT_SGRBG8_1X8:
	case MEDIA_BUS_FMT_SRGGB8_1X8:
	case MEDIA_BUS_FMT_SBGGR8_1X8:
	case MEDIA_BUS_FMT_SGBRG8_1X8:
	default:
		return 8;
	case MEDIA_BUS_FMT_Y10_1X10:
	case MEDIA_BUS_FMT_SGRBG10_1X10:
	case MEDIA_BUS_FMT_SRGGB10_1X10:
	case MEDIA_BUS_FMT_SBGGR10_1X10:
	case MEDIA_BUS_FMT_SGBRG10_1X10:
		return 10;
	case MEDIA_BUS_FMT_Y12_1X12:
	case MEDIA_BUS_FMT_SGRBG12_1X12:
	case MEDIA_BUS_FMT_SRGGB12_1X12:
	case MEDIA_BUS_FMT_SBGGR12_1X12:
	case MEDIA_BUS_FMT_SGBRG12_1X12:
		return 12;
	}
}

static int vd1943_get_temp_stream_enable(struct vd1943 *sensor, int *temp)
{
	return vd1943_read(sensor, VD1943_REG_TEMPERATURE, temp, NULL);
}

static int vd1943_get_temp_stream_disable(struct vd1943 *sensor, int *temp)
{
	int ret = 0;

	/* request temperature read */
	vd1943_write(sensor, VD1943_REG_SW_STBY, VD1943_CMD_THSENS_READ, &ret);
	vd1943_poll_reg(sensor, VD1943_REG_SW_STBY, VD1943_CMD_ACK, &ret);
	if (ret)
		return ret;

	return vd1943_get_temp_stream_enable(sensor, temp);
}

static int vd1943_get_temp(struct vd1943 *sensor, int *temp)
{
	*temp = 0;
	if (sensor->streaming)
		return vd1943_get_temp_stream_enable(sensor, temp);
	else
		return vd1943_get_temp_stream_disable(sensor, temp);
}

static int vd1943_write_sensor_conf(struct vd1943 *sensor, u8 shutter_mode)
{
#if KERNEL_VERSION(5, 19, 0) > LINUX_VERSION_CODE
	unsigned int code = sensor->active_fmt.code;
#elif KERNEL_VERSION(6, 8, 0) > LINUX_VERSION_CODE
	struct v4l2_subdev_state *state =
		v4l2_subdev_get_locked_active_state(&sensor->sd);
	unsigned int code =
		v4l2_subdev_get_pad_format(&sensor->sd, state, 0)->code;
#else
	struct v4l2_subdev_state *state =
		v4l2_subdev_get_locked_active_state(&sensor->sd);
	unsigned int code = v4l2_subdev_state_get_format(state, 0)->code;
#endif
	unsigned int model = sensor->model;
	unsigned int i;
	int ret = 0;

	for (i = 0; i < ARRAY_SIZE(vdx941_configs[model][shutter_mode]); i++)
		if (vdx941_configs[model][shutter_mode][i].mbus_code == code)
			break;

	if (i >= ARRAY_SIZE(vdx941_configs[model][shutter_mode]))
		i = 0;

	vd1943_write(sensor, VD1943_REG_SENSOR_CONFIGURATION,
		     vdx941_configs[model][shutter_mode][i].sensor_conf, &ret);

	return ret;
}

static int vd1943_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = ctrl_to_sd(ctrl);
	struct vd1943 *sensor = to_vd1943(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int temperature;
	int ret = 0;

	/* Interact with HW only when it is powered ON */
	if (!pm_runtime_get_if_in_use(&client->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_TEMPERATURE:
		ret = vd1943_get_temp(sensor, &temperature);
		if (ret)
			break;
		ctrl->val = temperature;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_mark_last_busy(&client->dev);
	pm_runtime_put_autosuspend(&client->dev);

	return ret;
}

static int vd1943_update_controls(struct vd1943 *sensor)
{
#if KERNEL_VERSION(5, 19, 0) > LINUX_VERSION_CODE
#else
	struct v4l2_subdev_state *state;
#endif
	const struct v4l2_rect *crop;
	const struct v4l2_mbus_framefmt *format;
	u8 bpp;
	u32 vt_pixelrate;
	u32 mipi_pixelrate;
	u32 mipi_transition_cst_in_px;
	u32 mipi_interpacket_delay_in_px;
	u64 mipi_virtual_linelength = 0;
	u32 linelength;
	unsigned int hblank_min;
	unsigned int vblank_min, vblank, vblank_max;
	unsigned int expo_min, expo_max;
	int ret;

#if KERNEL_VERSION(5, 19, 0) > LINUX_VERSION_CODE
	crop = &sensor->active_crop;
	format = &sensor->active_fmt;
#elif KERNEL_VERSION(6, 8, 0) > LINUX_VERSION_CODE
	state = v4l2_subdev_get_locked_active_state(&sensor->sd);
	crop = v4l2_subdev_get_pad_crop(&sensor->sd, state, 0);
	format = v4l2_subdev_get_pad_format(&sensor->sd, state, 0);
#else
	state = v4l2_subdev_get_locked_active_state(&sensor->sd);
	crop = v4l2_subdev_state_get_crop(state, 0);
	format = v4l2_subdev_state_get_format(state, 0);
#endif

	/* With 2 rows of ADC used in GS mode, the VT pixel rate is doubled */
	vt_pixelrate = sensor->pixel_rate;
	if (sensor->shutter_ctrl->val == VD1943_GS_MODE)
		vt_pixelrate = vt_pixelrate * 2;

	/* Compute the max pixelrate on the mipi link based on pixel depth */
	bpp = vd1943_get_bpp(format->code);
	mipi_pixelrate = sensor->mipi_bandwidth / bpp;

	/* Compute the mins linelength/hblank given the mipi bandwidth */
	if (mipi_pixelrate < vt_pixelrate) {
		/*
		 * Compute a virtual (mipi-equivalent) linelength that could fit
		 * with MIPI limitation (MIPI link being the bottleneck here).
		 */
		mipi_transition_cst_in_px = mipi_pixelrate / MEGA / 4;
		mipi_interpacket_delay_in_px =
			30 * 8 * sensor->nb_of_lane / bpp;
		mipi_virtual_linelength = mipi_transition_cst_in_px +
					  crop->width +
					  mipi_interpacket_delay_in_px;
		mipi_virtual_linelength =
			mipi_virtual_linelength * vt_pixelrate / mipi_pixelrate;
	}
	linelength = max(VD1943_LINE_LENGTH_MIN, (int)mipi_virtual_linelength);
	hblank_min = linelength - crop->width;

	/* Adjust vblank with a target of 30FPS */
	vblank_min = VD1943_VBLANK_MIN;
	vblank = vt_pixelrate / (VD1943_DEFAULT_FPS30 * linelength) -
		 crop->height;
	vblank = max(VD1943_VBLANK_MIN, vblank);
	vblank_max = VD1943_FRAME_LENGTH_MAX - crop->height;

	/*
	 * Exposure limits (expressed in lines) :
	 * - GS mode : [4   .. FRAME_LENGTH - 27]
	 * - RS mode : [1.5 .. FRAME_LENGTH - 10]
	 */
	expo_min = 4;
	expo_max = crop->height + vblank - VD1943_EXPOSURE_MARGIN;

	/* Update pixel_rate, blankings and exposure controls */
	ret = __v4l2_ctrl_modify_range(sensor->vt_pixrate_ctrl, vt_pixelrate,
				       vt_pixelrate, 1, vt_pixelrate);
	if (ret)
		return ret;

	ret = __v4l2_ctrl_modify_range(sensor->hblank_ctrl, hblank_min,
				       hblank_min, 1, hblank_min);
	if (ret)
		return ret;

	ret = __v4l2_ctrl_modify_range(sensor->vblank_ctrl, vblank_min,
				       vblank_max, 1, vblank);
	if (ret)
		return ret;

	ret = __v4l2_ctrl_s_ctrl(sensor->vblank_ctrl, vblank);
	if (ret)
		return ret;

	ret = __v4l2_ctrl_modify_range(sensor->expo_ctrl, expo_min, expo_max, 1,
				       VD1943_EXPOSURE_DEFAULT);
	if (ret)
		return ret;

	return __v4l2_ctrl_s_ctrl(sensor->expo_ctrl, VD1943_EXPOSURE_DEFAULT);
}

static int vd1943_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = ctrl_to_sd(ctrl);
	struct vd1943 *sensor = to_vd1943(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
#if KERNEL_VERSION(5, 19, 0) > LINUX_VERSION_CODE
#else
	struct v4l2_subdev_state *state;
#endif
	const struct v4l2_rect *crop;
	const struct v4l2_mbus_framefmt *format;
	unsigned int line_length = 0;
	unsigned int frame_length = 0;
	unsigned int expo_max;
	unsigned int gpio_ctrl;
	unsigned long gpio_ctrl_new;
	unsigned long io;
	int ret = 0;

#if KERNEL_VERSION(5, 19, 0) > LINUX_VERSION_CODE
	crop = &sensor->active_crop;
	format = &sensor->active_fmt;
#elif KERNEL_VERSION(6, 8, 0) > LINUX_VERSION_CODE
	state = v4l2_subdev_get_locked_active_state(&sensor->sd);
	crop = v4l2_subdev_get_pad_crop(&sensor->sd, state, 0);
	format = v4l2_subdev_get_pad_format(&sensor->sd, state, 0);
#else
	state = v4l2_subdev_get_locked_active_state(&sensor->sd);
	crop = v4l2_subdev_state_get_crop(state, 0);
	format = v4l2_subdev_state_get_format(state, 0);
#endif

	if (ctrl->flags & V4L2_CTRL_FLAG_READ_ONLY)
		return 0;

	/* Update controls state, range, etc. whatever the state of the HW */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		frame_length = crop->height + ctrl->val;
		if (sensor->shutter_ctrl->val == VD1943_GS_MODE)
			frame_length = frame_length / 2;
		expo_max = frame_length - VD1943_EXPOSURE_MARGIN;
		ret = __v4l2_ctrl_modify_range(sensor->expo_ctrl, 0, expo_max,
					       1,
					       min(VD1943_EXPOSURE_DEFAULT,
						   expo_max));
		break;
	case V4L2_CID_SHUTTER_MODE:
		/*
		 * Because __v4l2_ctrl_handler_setup() could trigger unnecessary
		 * vd1943_update_controls() call, ensure that the shutter mode
		 * really change before going further.
		 */
		if (ctrl->val != ctrl->cur.val)
			ret = vd1943_update_controls(sensor);
		break;
	default:
		break;
	}

	if (ret)
		return ret;

	/* Interact with HW only when it is powered ON */
	if (!pm_runtime_get_if_in_use(&client->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_HFLIP:
		ret = vd1943_write(sensor, VD1943_REG_ORIENTATION,
				   sensor->hflip_ctrl->val |
					   (sensor->vflip_ctrl->val << 1),
				   NULL);
		break;
	case V4L2_CID_TEST_PATTERN:
		vd1943_write(sensor, VD1943_REG_PATGEN_CTRL, ctrl->val, &ret);
		vd1943_write(sensor, VD1943_REG_DARKCAL_ENABLE, !ctrl->val,
			     &ret);
		break;
	case V4L2_CID_HBLANK:
		line_length = crop->width + ctrl->val;
		vd1943_write(sensor, VD1943_REG_LINE_LENGTH, line_length, &ret);
		break;
	case V4L2_CID_VBLANK:
		ret = vd1943_write(sensor, VD1943_REG_FRAME_LENGTH,
				   frame_length, NULL);
		break;
	case V4L2_CID_EXPOSURE:
		ret = vd1943_write(sensor, VD1943_REG_INTEGRATION_TIME_PRIMARY,
				   ctrl->val, NULL);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		ret = vd1943_write(sensor, VD1943_REG_ANALOG_GAIN, ctrl->val,
				   NULL);
		break;
	case V4L2_CID_DIGITAL_GAIN:
		vd1943_write(sensor, VD1943_REG_DIGITAL_GAIN_R, ctrl->val,
			     &ret);
		vd1943_write(sensor, VD1943_REG_DIGITAL_GAIN_G, ctrl->val,
			     &ret);
		vd1943_write(sensor, VD1943_REG_DIGITAL_GAIN_B, ctrl->val,
			     &ret);
		vd1943_write(sensor, VD1943_REG_DIGITAL_GAIN_IR, ctrl->val,
			     &ret);
		break;
	case V4L2_CID_SHUTTER_MODE:
		ret = vd1943_write_sensor_conf(sensor, ctrl->val);
		break;
	case V4L2_CID_SLAVE_MODE:
		vd1943_read(sensor, VD1943_REG_GPIO_CTRL, &gpio_ctrl, &ret);
		gpio_ctrl_new = gpio_ctrl;
		assign_bit(sensor->ext_vt_sync, &gpio_ctrl_new, ctrl->val);
		vd1943_write(sensor, VD1943_REG_GPIO_CTRL,
			     (unsigned int)gpio_ctrl_new, &ret);
		vd1943_write(sensor, VD1943_REG_VT_CTRL, ctrl->val, &ret);
		break;
	case V4L2_CID_FLASH_LED_MODE:
		vd1943_read(sensor, VD1943_REG_GPIO_CTRL, &gpio_ctrl, &ret);
		gpio_ctrl_new = gpio_ctrl;
		for_each_set_bit(io, &sensor->ext_leds_mask, VD1943_NB_GPIOS)
			assign_bit(io, &gpio_ctrl_new, ctrl->val);
		vd1943_write(sensor, VD1943_REG_GPIO_CTRL,
			     (unsigned int)gpio_ctrl_new, &ret);
		break;
	case V4L2_CID_DARKCAL_PEDESTAL:
		ret = vd1943_write(sensor, VD1943_REG_DARKCAL_PEDESTAL,
				   ctrl->val, NULL);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_mark_last_busy(&client->dev);
	pm_runtime_put_autosuspend(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops vd1943_ctrl_ops = {
	.g_volatile_ctrl = vd1943_g_volatile_ctrl,
	.s_ctrl = vd1943_s_ctrl,
};

/**
 * DOC: Temperature Control
 *
 * Return sensor temperature (in Celsius)
 *
 * :id:     ``V4L2_CID_TEMPERATURE``
 * :type:   ``V4L2_CTRL_TYPE_INTEGER``
 */
static const struct v4l2_ctrl_config vd1943_temp_ctrl = {
	.ops = &vd1943_ctrl_ops,
	.id = V4L2_CID_TEMPERATURE,
	.name = "Temperature in celsius",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = -1024,
	.max = 1023,
	.step = 1,
};

/**
 * DOC: Dark Calibration Pedestal
 *
 * The device embeds an automatic dark calibration mechanism.
 * This controls allows to set the dark calibration target.
 *
 * :id:     ``V4L2_CID_DARKCAL_PEDESTAL``
 * :type:   ``V4L2_CTRL_TYPE_INTEGER``
 * :min:    0
 * :max:    63
 * :def:    32
 *
 */
static const struct v4l2_ctrl_config vd1943_darkcal_pedestal_ctrl = {
	.ops = &vd1943_ctrl_ops,
	.id = V4L2_CID_DARKCAL_PEDESTAL,
	.name = "Dark Calibration Pedestal",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = 0,
	.max = 63,
	.step = 1,
	.def = 0x20,
};

/**
 * DOC: VT Slave Mode Control
 *
 * When the 'st,in-sync' property of the device tree is enabled on gpio0,
 * this control allows to enable/disable the Slave Mode of the sensor
 *
 * :id:     ``V4L2_CID_SLAVE``
 * :type:   ``V4L2_CTRL_TYPE_BOOLEAN``
 * :min:    False
 * :max:    True
 * :def:    True
 *
 */
static const struct v4l2_ctrl_config vd1943_slave_ctrl = {
	.ops = &vd1943_ctrl_ops,
	.id = V4L2_CID_SLAVE_MODE,
	.name = "VT Slave Mode",
	.type = V4L2_CTRL_TYPE_BOOLEAN,
	.min = 0,
	.max = 1,
	.step = 1,
	.def = 1,
};

/**
 * DOC: Shutter mode
 *
 * This control allows to select the desired shutter mode.
 *
 * :id:     ``V4L2_CID_SHUTTER_MODE``
 * :type:   ``V4L2_CTRL_TYPE_MENU``
 */
static const struct v4l2_ctrl_config vd1943_shutter_ctrl = {
	.ops = &vd1943_ctrl_ops,
	.id = V4L2_CID_SHUTTER_MODE,
	.name = "Shutter mode",
	.type = V4L2_CTRL_TYPE_MENU,
	.min = 0,
	.max = ARRAY_SIZE(vd1943_shutter_menu) - 1,
	.def = VD1943_GS_MODE,
	.qmenu = vd1943_shutter_menu,
};

static int vd1943_init_controls(struct vd1943 *sensor)
{
	const struct v4l2_ctrl_ops *ops = &vd1943_ctrl_ops;
	struct v4l2_ctrl_handler *hdl = &sensor->ctrl_handler;
#if KERNEL_VERSION(5, 8, 0) > LINUX_VERSION_CODE
#else
	struct v4l2_fwnode_device_properties fwnode_props;
#endif
	struct v4l2_ctrl *ctrl;
	int ret;

	v4l2_ctrl_handler_init(hdl, 25);

#if KERNEL_VERSION(5, 19, 0) > LINUX_VERSION_CODE
	/* we can use our own mutex for the ctrl lock */
	hdl->lock = &sensor->lock;
#endif

	/* Horizontal & vertical flips modify bayer code on RGB variant */
	sensor->hflip_ctrl =
		v4l2_ctrl_new_std(hdl, ops, V4L2_CID_HFLIP, 0, 1, 1, 0);
	if (sensor->hflip_ctrl)
		sensor->hflip_ctrl->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	sensor->vflip_ctrl =
		v4l2_ctrl_new_std(hdl, ops, V4L2_CID_VFLIP, 0, 1, 1, 0);
	if (sensor->vflip_ctrl)
		sensor->vflip_ctrl->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	sensor->patgen_ctrl =
		v4l2_ctrl_new_std_menu_items(hdl, ops, V4L2_CID_TEST_PATTERN,
					     ARRAY_SIZE(vd1943_tp_menu) - 1, 0,
					     0, vd1943_tp_menu);

	ctrl = v4l2_ctrl_new_int_menu(hdl, ops, V4L2_CID_LINK_FREQ,
				      ARRAY_SIZE(vd1943_link_freq_2lanes) - 1,
				      0, (sensor->nb_of_lane == 2) ?
					      vd1943_link_freq_2lanes :
					      vd1943_link_freq_4lanes);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/*
	 * Pixelrate may change depending of selected Shutter Mode, in GS mode
	 * PixelRate can be twice faster
	 */
	sensor->vt_pixrate_ctrl =
		v4l2_ctrl_new_std(hdl, ops, V4L2_CID_PIXEL_RATE,
				  sensor->pixel_rate, sensor->pixel_rate, 1,
				  sensor->pixel_rate);
	if (sensor->vt_pixrate_ctrl)
		sensor->vt_pixrate_ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/*
	 * Analog gain [1, 4] is computed with the following logic :
	 * 16/(16 - again_reg), with again_reg in the range [0:12]
	 * Digital gain [1.00, 32.00] is coded as a Fixed Point 5.8
	 */
	sensor->again_ctrl = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_ANALOGUE_GAIN,
					       0, 12, 1, 0);
	sensor->dgain_ctrl = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_DIGITAL_GAIN,
					       0x100, 0x2000, 1, 0x100);

	/*
	 * Set the exposure, horizontal and vertical blanking ctrls
	 * to hardcoded values, they will be updated in vd1943_update_controls.
	 */
	sensor->expo_ctrl = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_EXPOSURE,
					      VD1943_EXPOSURE_DEFAULT,
					      VD1943_EXPOSURE_DEFAULT, 1,
					      VD1943_EXPOSURE_DEFAULT);
	sensor->hblank_ctrl =
		v4l2_ctrl_new_std(hdl, ops, V4L2_CID_HBLANK, 1, 1, 1, 1);
	sensor->vblank_ctrl =
		v4l2_ctrl_new_std(hdl, ops, V4L2_CID_VBLANK, 1, 1, 1, 1);

	/*
	 * Custom controls : temperature, shutter_mode, pedestal and controls
	 * based on device tree properties
	 */
	ctrl = v4l2_ctrl_new_custom(hdl, &vd1943_temp_ctrl, NULL);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_VOLATILE |
			       V4L2_CTRL_FLAG_READ_ONLY;
	sensor->shutter_ctrl =
		v4l2_ctrl_new_custom(hdl, &vd1943_shutter_ctrl, NULL);
	if (sensor->shutter_ctrl)
		sensor->shutter_ctrl->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;
	sensor->pedestal_ctrl =
		v4l2_ctrl_new_custom(hdl, &vd1943_darkcal_pedestal_ctrl, NULL);
	if (sensor->ext_vt_sync < VD1943_NB_GPIOS)
		sensor->slave_ctrl =
			v4l2_ctrl_new_custom(hdl, &vd1943_slave_ctrl, NULL);
	if (sensor->ext_leds_mask)
		sensor->led_ctrl =
			v4l2_ctrl_new_std_menu(hdl, ops,
					       V4L2_CID_FLASH_LED_MODE,
					       V4L2_FLASH_LED_MODE_FLASH, 0,
					       V4L2_FLASH_LED_MODE_NONE);

	if (hdl->error) {
		ret = hdl->error;
		goto free_ctrls;
	}

	v4l2_ctrl_cluster(2, &sensor->hflip_ctrl);

#if KERNEL_VERSION(5, 8, 0) > LINUX_VERSION_CODE
#else
	/* Optional controls coming from fwnode (e.g. rotation, orientation). */
	ret = v4l2_fwnode_device_parse(&sensor->i2c_client->dev, &fwnode_props);
	if (ret)
		goto free_ctrls;

	ret = v4l2_ctrl_new_fwnode_properties(hdl, ops, &fwnode_props);
	if (ret)
		goto free_ctrls;
#endif

	sensor->sd.ctrl_handler = hdl;

	return 0;

free_ctrls:
	v4l2_ctrl_handler_free(hdl);

	return ret;
}

/* -----------------------------------------------------------------------------
 * Videos ops
 */

#if KERNEL_VERSION(5, 19, 0) > LINUX_VERSION_CODE
static int vd1943_stream_on(struct vd1943 *sensor)
#else
static int vd1943_stream_on(struct vd1943 *sensor,
			    struct v4l2_subdev_state *state)
#endif
{
	const struct v4l2_mbus_framefmt *format;
	const struct v4l2_rect *crop;
	unsigned int csi_mbps = ((sensor->nb_of_lane == 2) ?
					 VD1943_LINK_FREQ_DEF_2LANES :
					 VD1943_LINK_FREQ_DEF_4LANES) * 2;
	unsigned int lane_nb = ((sensor->nb_of_lane == 2) ? VD1943_LANES_NB_2 :
							    VD1943_LANES_NB_4);
	unsigned int io;
	int ret = 0;

#if KERNEL_VERSION(5, 19, 0) > LINUX_VERSION_CODE
	format = &sensor->active_fmt;
	crop = &sensor->active_crop;
#elif KERNEL_VERSION(6, 8, 0) > LINUX_VERSION_CODE
	format = v4l2_subdev_get_pad_format(&sensor->sd, state, 0);
	crop = v4l2_subdev_get_pad_crop(&sensor->sd, state, 0);
#else
	format = v4l2_subdev_state_get_format(state, 0);
	crop = v4l2_subdev_state_get_crop(state, 0);
#endif

	/* configure output */
	vd1943_write(sensor, VD1943_REG_LANE_NB_SEL, lane_nb, &ret);
	vd1943_write(sensor, VD1943_REG_OIF_LANE_PHY_MAP,
		     sensor->oif_lane_phy_map, &ret);
	vd1943_write(sensor, VD1943_REG_OIF_LANE_PHY_SWAP,
		     sensor->oif_lane_phy_swap, &ret);
	vd1943_write(sensor, VD1943_REG_MIPI_DATA_RATE, csi_mbps, &ret);
	vd1943_write(sensor, VD1943_REG_OIF_ISL_ENABLE, 0, &ret);

	/* configure ROIs */
	vd1943_write(sensor, VD1943_REG_ROI_WIDTH_OFFSET, crop->left, &ret);
	vd1943_write(sensor, VD1943_REG_ROI_HEIGHT_OFFSET, crop->top, &ret);
	vd1943_write(sensor, VD1943_REG_ROI_WIDTH, crop->width, &ret);
	vd1943_write(sensor, VD1943_REG_ROI_HEIGHT, crop->height, &ret);
	vd1943_write(sensor, VD1943_REG_ROI_DT,
		     vd1943_get_datatype(format->code), &ret);

	/* configure GPIOS */
	for (io = 0; io < VD1943_NB_GPIOS; io++)
		vd1943_write(sensor, VD1943_REG_GPIO_0_CTRL + io,
			     sensor->gpios[io], &ret);
	if (ret)
		return ret;

	/* Apply settings from V4L2 ctrls */
	ret = __v4l2_ctrl_handler_setup(&sensor->ctrl_handler);
	if (ret)
		return ret;

	/* start streaming */
	vd1943_write(sensor, VD1943_REG_SW_STBY, VD1943_CMD_START_STREAMING,
		     &ret);
	vd1943_poll_reg(sensor, VD1943_REG_SW_STBY, VD1943_CMD_ACK, &ret);
	vd1943_wait_state(sensor, VD1943_SYSTEM_FSM_STREAMING, &ret);

	return ret;
}

static int vd1943_stream_off(struct vd1943 *sensor)
{
	int ret;

	vd1943_write(sensor, VD1943_REG_STREAMING, VD1943_CMD_STOP_STREAM,
		     &ret);
	vd1943_poll_reg(sensor, VD1943_REG_STREAMING, VD1943_CMD_ACK, &ret);
	vd1943_wait_state(sensor, VD1943_SYSTEM_FSM_SW_STBY, &ret);

	return ret;
}

static int vd1943_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct vd1943 *sensor = to_vd1943(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
#if KERNEL_VERSION(5, 19, 0) > LINUX_VERSION_CODE
#else
	struct v4l2_subdev_state *state;
#endif
	int ret = 0;

#if KERNEL_VERSION(5, 19, 0) > LINUX_VERSION_CODE
	mutex_lock(&sensor->lock);
#else
	state = v4l2_subdev_lock_and_get_active_state(sd);
#endif

	if (enable) {
#if KERNEL_VERSION(5, 10, 0) > LINUX_VERSION_CODE
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock;
		}
#else
		ret = pm_runtime_resume_and_get(&client->dev);
		if (ret < 0)
			goto unlock;
#endif
#if KERNEL_VERSION(5, 19, 0) > LINUX_VERSION_CODE
		ret = vd1943_stream_on(sensor);
#else
		ret = vd1943_stream_on(sensor, state);
#endif
		if (ret) {
			dev_err(&client->dev, "Failed to start streaming\n");
			pm_runtime_put_sync(&client->dev);
		}
	} else {
		vd1943_stream_off(sensor);
		pm_runtime_mark_last_busy(&client->dev);
		pm_runtime_put_autosuspend(&client->dev);
	}

#if KERNEL_VERSION(4, 20, 0) > LINUX_VERSION_CODE
	if (!ret)
		sensor->streaming = enable;

unlock:
	mutex_unlock(&sensor->lock);

	if (!ret) {
		/* Controls related to static reg are locked during streaming */
		v4l2_ctrl_grab(sensor->hflip_ctrl, enable);
		v4l2_ctrl_grab(sensor->vflip_ctrl, enable);
		v4l2_ctrl_grab(sensor->patgen_ctrl, enable);
		v4l2_ctrl_grab(sensor->hblank_ctrl, enable);
		v4l2_ctrl_grab(sensor->vblank_ctrl, enable);
		v4l2_ctrl_grab(sensor->shutter_ctrl, enable);
		v4l2_ctrl_grab(sensor->pedestal_ctrl, enable);
		if (sensor->ext_vt_sync < VD1943_NB_GPIOS)
			v4l2_ctrl_grab(sensor->slave_ctrl, enable);
		if (sensor->ext_leds_mask)
			v4l2_ctrl_grab(sensor->led_ctrl, enable);
	}
#else
	if (!ret) {
		sensor->streaming = enable;

		/* Controls related to static reg are locked during streaming */
		__v4l2_ctrl_grab(sensor->hflip_ctrl, enable);
		__v4l2_ctrl_grab(sensor->vflip_ctrl, enable);
		__v4l2_ctrl_grab(sensor->patgen_ctrl, enable);
		__v4l2_ctrl_grab(sensor->hblank_ctrl, enable);
		__v4l2_ctrl_grab(sensor->vblank_ctrl, enable);
		__v4l2_ctrl_grab(sensor->shutter_ctrl, enable);
		__v4l2_ctrl_grab(sensor->pedestal_ctrl, enable);
		if (sensor->ext_vt_sync < VD1943_NB_GPIOS)
			__v4l2_ctrl_grab(sensor->slave_ctrl, enable);
		if (sensor->ext_leds_mask)
			__v4l2_ctrl_grab(sensor->led_ctrl, enable);
	}

unlock:
#if KERNEL_VERSION(5, 19, 0) > LINUX_VERSION_CODE
	mutex_unlock(&sensor->lock);
#else
	v4l2_subdev_unlock_state(state);
#endif
#endif

	return ret;
}

static const struct v4l2_subdev_video_ops vd1943_video_ops = {
	.s_stream = vd1943_s_stream,
};

/* -----------------------------------------------------------------------------
 * Pad ops
 */

#if KERNEL_VERSION(5, 19, 0) > LINUX_VERSION_CODE
/*
 * Media bus code is dependent of :
 *      - 8bits, 10bits or 12bits output
 *      - shutter mode : GS or RS
 *      - H/V flips parameters in case of RGB (RS mode)
 */
static u32 vd1943_get_mbus_code(struct vd1943 *sensor, u32 code)
{
	unsigned int model = sensor->model;
	unsigned int shutter = sensor->shutter_ctrl->val;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(vdx941_configs[model][shutter]); i++)
		if (vdx941_configs[model][shutter][i].mbus_code == code)
			break;

	if (i >= ARRAY_SIZE(vdx941_configs[model][shutter]))
		i = 0;

	return vdx941_configs[model][shutter][i].mbus_code;
}
#endif

#if KERNEL_VERSION(5, 14, 0) > LINUX_VERSION_CODE
static int vd1943_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
#else
static int vd1943_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
#endif
{
	struct vd1943 *sensor = to_vd1943(sd);
	unsigned int model = sensor->model;
	unsigned int shutter = sensor->shutter_ctrl->val;

	if (code->index >= ARRAY_SIZE(vdx941_configs[model][shutter]))
		return -EINVAL;

	if (!vdx941_configs[model][shutter][code->index].mbus_code)
		return -EINVAL;

	code->code = vdx941_configs[model][shutter][code->index].mbus_code;

	return 0;
}

#if KERNEL_VERSION(5, 14, 0) > LINUX_VERSION_CODE
static int vd1943_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_frame_size_enum *fse)
#else
static int vd1943_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fse)
#endif
{
	if (fse->index >= ARRAY_SIZE(vd1943_supported_modes))
		return -EINVAL;

	fse->min_width = vd1943_supported_modes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = vd1943_supported_modes[fse->index].height;
	fse->max_height = fse->min_height;

	return 0;
}

static void vd1943_update_img_pad_format(struct vd1943 *sensor,
					 const struct vd1943_mode *mode,
					 u32 mbus_code,
					 struct v4l2_mbus_framefmt *mbus_fmt)
{
	mbus_fmt->width = mode->width;
	mbus_fmt->height = mode->height;
	mbus_fmt->code = mbus_code;
	mbus_fmt->colorspace = V4L2_COLORSPACE_RAW;
	mbus_fmt->field = V4L2_FIELD_NONE;
	mbus_fmt->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	mbus_fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	mbus_fmt->xfer_func = V4L2_XFER_FUNC_NONE;
}

#if KERNEL_VERSION(5, 19, 0) > LINUX_VERSION_CODE
#if KERNEL_VERSION(5, 14, 0) > LINUX_VERSION_CODE
static int vd1943_get_pad_fmt(struct v4l2_subdev *sd,
			      struct v4l2_subdev_pad_config *cfg,
			      struct v4l2_subdev_format *sd_fmt)
#else
static int vd1943_get_pad_fmt(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *sd_state,
			      struct v4l2_subdev_format *sd_fmt)
#endif
{
	struct vd1943 *sensor = to_vd1943(sd);
	struct v4l2_mbus_framefmt *pad_fmt;

	mutex_lock(&sensor->lock);

	if (sd_fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#if KERNEL_VERSION(5, 14, 0) > LINUX_VERSION_CODE
		pad_fmt = v4l2_subdev_get_try_format(&sensor->sd, cfg,
						     sd_fmt->pad);
#else
		pad_fmt = v4l2_subdev_get_try_format(&sensor->sd, sd_state,
						     sd_fmt->pad);
#endif
		/* Image mbus code could change with H/V flips */
		pad_fmt->code = vd1943_get_mbus_code(sensor, pad_fmt->code);
		sd_fmt->format = *pad_fmt;
	} else {
		sd_fmt->format = sensor->active_fmt;
	}

	mutex_unlock(&sensor->lock);

	return 0;
}
#endif

#if KERNEL_VERSION(5, 14, 0) > LINUX_VERSION_CODE
static int vd1943_set_pad_fmt(struct v4l2_subdev *sd,
			      struct v4l2_subdev_pad_config *cfg,
			      struct v4l2_subdev_format *sd_fmt)
#else
static int vd1943_set_pad_fmt(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *sd_state,
			      struct v4l2_subdev_format *sd_fmt)
#endif
{
	struct vd1943 *sensor = to_vd1943(sd);
	const struct vd1943_mode *new_mode;
	struct v4l2_mbus_framefmt *pad_fmt;
	struct v4l2_rect pad_crop;
	int ret = 0;

	if (sensor->streaming)
		return -EBUSY;

#if KERNEL_VERSION(5, 19, 0) > LINUX_VERSION_CODE
	mutex_lock(&sensor->lock);
#endif

	/* Identify the mode that best suits the requested resolution */
	new_mode = v4l2_find_nearest_size(vd1943_supported_modes,
					  ARRAY_SIZE(vd1943_supported_modes),
					  width, height, sd_fmt->format.width,
					  sd_fmt->format.height);

	/* Update fmt struct with identified resolution and mbus code */
	vd1943_update_img_pad_format(sensor, new_mode, sd_fmt->format.code,
				     &sd_fmt->format);

	/* Compute crop rectangle */
	pad_crop.width = sd_fmt->format.width;
	pad_crop.height = sd_fmt->format.height;
	pad_crop.left = (VD1943_NATIVE_WIDTH - pad_crop.width) / 2;
	pad_crop.top = (VD1943_NATIVE_HEIGHT - pad_crop.height) / 2;

#if KERNEL_VERSION(5, 19, 0) > LINUX_VERSION_CODE
	if (sd_fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#if KERNEL_VERSION(5, 14, 0) > LINUX_VERSION_CODE
		pad_fmt = v4l2_subdev_get_try_format(sd, cfg, sd_fmt->pad);
#else
		pad_fmt = v4l2_subdev_get_try_format(sd, sd_state, sd_fmt->pad);
#endif
		*pad_fmt = sd_fmt->format;
	} else {
		sensor->active_fmt = sd_fmt->format;
		sensor->active_crop = pad_crop;
		ret = vd1943_update_controls(sensor);
	}
#else
#if KERNEL_VERSION(6, 8, 0) > LINUX_VERSION_CODE
	pad_fmt = v4l2_subdev_get_pad_format(sd, sd_state, sd_fmt->pad);
#else
	pad_fmt = v4l2_subdev_state_get_format(sd_state, sd_fmt->pad);
#endif
	*pad_fmt = sd_fmt->format;

#if KERNEL_VERSION(6, 8, 0) > LINUX_VERSION_CODE
	*v4l2_subdev_get_pad_crop(sd, sd_state, sd_fmt->pad) = pad_crop;
#else
	*v4l2_subdev_state_get_crop(sd_state, sd_fmt->pad) = pad_crop;
#endif

	if (sd_fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE)
		ret = vd1943_update_controls(sensor);
#endif

#if KERNEL_VERSION(5, 19, 0) > LINUX_VERSION_CODE
	mutex_unlock(&sensor->lock);
#endif

	return ret;
}

#if KERNEL_VERSION(5, 14, 0) > LINUX_VERSION_CODE
static int vd1943_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_pad_config *cfg,
				struct v4l2_subdev_selection *sel)
#else
static int vd1943_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_selection *sel)
#endif
{
#if KERNEL_VERSION(5, 19, 0) > LINUX_VERSION_CODE
	struct vd1943 *sensor = to_vd1943(sd);
#endif

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
#if KERNEL_VERSION(5, 19, 0) > LINUX_VERSION_CODE
		sel->r = sensor->active_crop;
#elif KERNEL_VERSION(6, 8, 0) > LINUX_VERSION_CODE
		sel->r = *v4l2_subdev_get_pad_crop(sd, sd_state, 0);
#else
		sel->r = *v4l2_subdev_state_get_crop(sd_state, 0);
#endif
		break;
	case V4L2_SEL_TGT_NATIVE_SIZE:
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.top = 0;
		sel->r.left = 0;
		sel->r.width = VD1943_NATIVE_WIDTH;
		sel->r.height = VD1943_NATIVE_HEIGHT;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

#if KERNEL_VERSION(5, 14, 0) > LINUX_VERSION_CODE
static int vd1943_init_cfg(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg)
#elif KERNEL_VERSION(6, 8, 0) > LINUX_VERSION_CODE
static int vd1943_init_cfg(struct v4l2_subdev *sd,
			   struct v4l2_subdev_state *sd_state)
#else
static int vd1943_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *sd_state)
#endif
{
	struct vd1943 *sensor = to_vd1943(sd);
	unsigned int def_mode = VD1943_DEFAULT_MODE;
	unsigned int def_mbus_code =
		vdx941_configs[sensor->model][VD1943_GS_MODE][0].mbus_code;

#if KERNEL_VERSION(5, 19, 0) > LINUX_VERSION_CODE
	vd1943_update_img_pad_format(sensor, &vd1943_supported_modes[def_mode],
				     def_mbus_code, &sensor->active_fmt);

	return 0;
#else
	struct v4l2_subdev_format fmt = {
		.which = V4L2_SUBDEV_FORMAT_TRY,
		.pad = 0,
		.format = {
			.code = def_mbus_code,
			.width = vd1943_supported_modes[def_mode].width,
			.height = vd1943_supported_modes[def_mode].height,
		},
	};

	return vd1943_set_pad_fmt(sd, sd_state, &fmt);
#endif
}

static const struct v4l2_subdev_core_ops vd1943_core_ops = {
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_pad_ops vd1943_pad_ops = {
#if KERNEL_VERSION(6, 8, 0) > LINUX_VERSION_CODE
	.init_cfg = vd1943_init_cfg,
#endif
	.enum_mbus_code = vd1943_enum_mbus_code,
	.enum_frame_size = vd1943_enum_frame_size,
#if KERNEL_VERSION(5, 19, 0) > LINUX_VERSION_CODE
	.get_fmt = vd1943_get_pad_fmt,
#else
	.get_fmt = v4l2_subdev_get_fmt,
#endif
	.set_fmt = vd1943_set_pad_fmt,
	.get_selection = vd1943_get_selection,
};

static const struct v4l2_subdev_ops vd1943_subdev_ops = {
	.core = &vd1943_core_ops,
	.video = &vd1943_video_ops,
	.pad = &vd1943_pad_ops,
};

static const struct media_entity_operations vd1943_subdev_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

#if KERNEL_VERSION(6, 8, 0) > LINUX_VERSION_CODE
#else
static const struct v4l2_subdev_internal_ops vd1943_internal_ops = {
	.init_state = vd1943_init_state,
};
#endif
/* -----------------------------------------------------------------------------
 * Boot section (includes Certificate configuration, FMW and VT Patches)
 */
static int vd1943_patch(struct vd1943 *sensor)
{
	struct i2c_client *client = sensor->i2c_client;
	const u8 *certificate = (sensor->model == VD1943_MODEL_VD1943) ?
					vd1943_certificate_vd1943 :
					vd1943_certificate_vd5943;
	int cert_size = (sensor->model == VD1943_MODEL_VD1943) ?
				sizeof(vd1943_certificate_vd1943) :
				sizeof(vd1943_certificate_vd5943);
	const u8 *patch = (sensor->model == VD1943_MODEL_VD1943) ?
				  vd1943_fmwpatch_vd1943 :
				  vd1943_fmwpatch_vd5943;
	int patch_size = (sensor->model == VD1943_MODEL_VD1943) ?
				 sizeof(vd1943_fmwpatch_vd1943) :
				 sizeof(vd1943_fmwpatch_vd5943);
	int ret = 0;

	vd1943_write_array(sensor, 0x1aa8, cert_size, certificate, &ret);
	vd1943_write(sensor, VD1943_REG_BOOT, VD1943_CMD_LOAD_CERTIFICATE,
		     &ret);
	vd1943_poll_reg(sensor, VD1943_REG_BOOT, VD1943_CMD_ACK, &ret);
	if (ret) {
		dev_err(&client->dev, "sensor cert setup failed %d", ret);
		return ret;
	}
	dev_info(&client->dev, "certificate applied");

	vd1943_write_array(sensor, 0x2000, patch_size, patch, &ret);
	vd1943_write(sensor, VD1943_REG_BOOT, VD1943_CMD_LOAD_FWP, &ret);
	vd1943_poll_reg(sensor, VD1943_REG_BOOT, VD1943_CMD_ACK, &ret);
	if (ret) {
		dev_err(&client->dev, "firmware patch failed %d", ret);
		return ret;
	}
	dev_info(&client->dev, "firmware patch applied");

	return ret;
}

static int vd1943_boot(struct vd1943 *sensor)
{
	int ret = 0;

	vd1943_write(sensor, VD1943_REG_BOOT, VD1943_CMD_END_BOOT, &ret);
	vd1943_poll_reg(sensor, VD1943_REG_BOOT, VD1943_CMD_ACK, &ret);
	vd1943_wait_state(sensor, VD1943_SYSTEM_FSM_SW_STBY, &ret);

	return ret;
}

static int vd1943_vt_patch(struct vd1943 *sensor)
{
	struct i2c_client *client = sensor->i2c_client;
	int i;
	int vtpatch_offset = 0;
	int ret = 0;

	vd1943_wait_state(sensor, VD1943_SYSTEM_FSM_SW_STBY, &ret);
	vd1943_write(sensor, VD1943_REG_SW_STBY, VD1943_CMD_UPDATE_VT_RAM_START,
		     &ret);

	for (i = 0; i < vtpatch_area_nb; i++) {
		vd1943_write_array(sensor, vtpatch_desc[i].offset,
				   vtpatch_desc[i].size,
				   vd1943_vtpatch + vtpatch_offset, &ret);

		vtpatch_offset += vtpatch_desc[i].size;
	}

	vd1943_write(sensor, VD1943_REG_SW_STBY, VD1943_CMD_UPDATE_VT_RAM_END,
		     &ret);
	vd1943_wait_state(sensor, VD1943_SYSTEM_FSM_SW_STBY, &ret);
	if (ret)
		return ret;

	dev_info(&client->dev, "VT patch applied");

	return ret;
}

/* -----------------------------------------------------------------------------
 * Power management
 */

static int vd1943_power_on(struct vd1943 *sensor)
{
	struct i2c_client *client = sensor->i2c_client;
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(sensor->supplies),
				    sensor->supplies);
	if (ret) {
		dev_err(&client->dev, "Failed to enable regulators %d", ret);
		return ret;
	}

	ret = clk_prepare_enable(sensor->xclk);
	if (ret) {
		dev_err(&client->dev, "Failed to enable clock %d", ret);
		goto disable_reg;
	}

	gpiod_set_value_cansleep(sensor->reset_gpio, 0);
	usleep_range(6000, 6500);
	ret = vd1943_wait_state(sensor, VD1943_SYSTEM_FSM_SYSTEM_UP, NULL);
	if (ret) {
		dev_err(&client->dev, "Sensor reset failed %d\n", ret);
		goto disable_clock;
	}

	if (sensor->xclk_freq != 25 * HZ_PER_MHZ)
		vd1943_write(sensor, VD1943_REG_EXT_CLOCK, sensor->xclk_freq,
			     &ret);

	vd1943_write(sensor, VD1943_REG_SYSTEM_UP, VD1943_CMD_START_SENSOR,
		     &ret);
	vd1943_poll_reg(sensor, VD1943_REG_SYSTEM_UP, VD1943_CMD_ACK, &ret);
	vd1943_wait_state(sensor, VD1943_SYSTEM_FSM_BOOT, &ret);
	if (ret) {
		dev_err(&client->dev, "Sensor system up failed %d\n", ret);
		goto disable_clock;
	}

	return 0;

disable_clock:
	clk_disable_unprepare(sensor->xclk);
	gpiod_set_value_cansleep(sensor->reset_gpio, 1);
disable_reg:
	regulator_bulk_disable(ARRAY_SIZE(sensor->supplies), sensor->supplies);

	return ret;
}

static int vd1943_power_patch(struct vd1943 *sensor)
{
	struct i2c_client *client = sensor->i2c_client;
	int ret;

	ret = vd1943_power_on(sensor);
	if (ret) {
		dev_err(&client->dev, "Failed to power on %d", ret);
		return ret;
	}

	if (!sensor->is_fastboot) {
		ret = vd1943_patch(sensor);
		if (ret) {
			dev_err(&client->dev, "sensor patch failed %d", ret);
			return ret;
		}
	}

	ret = vd1943_boot(sensor);
	if (ret) {
		dev_err(&client->dev, "sensor boot failed %d", ret);
		return ret;
	}

	if (!sensor->is_fastboot) {
		ret = vd1943_vt_patch(sensor);
		if (ret) {
			dev_err(&client->dev, "sensor VT patch failed %d", ret);
			return ret;
		}
	}

	return 0;
}

static int vd1943_power_off(struct vd1943 *sensor)
{
	clk_disable_unprepare(sensor->xclk);
	gpiod_set_value_cansleep(sensor->reset_gpio, 1);
	regulator_bulk_disable(ARRAY_SIZE(sensor->supplies), sensor->supplies);

	return 0;
}

static int vd1943_runtime_resume(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct vd1943 *vd1943 = to_vd1943(sd);

	return vd1943_power_patch(vd1943);
}

static int vd1943_runtime_suspend(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct vd1943 *vd1943 = to_vd1943(sd);

	return vd1943_power_off(vd1943);
}

static const struct dev_pm_ops vd1943_pm_ops = {
	SET_RUNTIME_PM_OPS(vd1943_runtime_suspend, vd1943_runtime_resume, NULL)
};

/* -----------------------------------------------------------------------------
 * Probe and initialization
 */

static int vd1943_check_csi_conf(struct vd1943 *sensor,
				 struct fwnode_handle *endpoint)
{
	struct i2c_client *client = sensor->i2c_client;
#if KERNEL_VERSION(4, 20, 0) > LINUX_VERSION_CODE
	struct v4l2_fwnode_endpoint ep = { .bus_type = V4L2_MBUS_CSI2 };
#else
	struct v4l2_fwnode_endpoint ep = { .bus_type = V4L2_MBUS_CSI2_DPHY };
#endif
	u32 phy_data_lanes[VD1943_MAX_CSI_DATA_LANES] = { ~0, ~0, ~0, ~0 };
	u8 n_lanes;
	u64 frequency;
	int p, l;
	int ret = 0;

#if KERNEL_VERSION(4, 20, 0) > LINUX_VERSION_CODE
	struct v4l2_fwnode_endpoint *ep_ptr =
		v4l2_fwnode_endpoint_alloc_parse(endpoint);
	if (IS_ERR(ep_ptr))
		return -EINVAL;
	ep = (*ep_ptr);
#else
	ret = v4l2_fwnode_endpoint_alloc_parse(endpoint, &ep);
	if (ret)
		return -EINVAL;
#endif

	/* Check lanes number */
	n_lanes = ep.bus.mipi_csi2.num_data_lanes;
	if (n_lanes != 2 && n_lanes != 4) {
		dev_err(&client->dev, "Invalid data lane number %d\n", n_lanes);
		ret = -EINVAL;
		goto done;
	}
	sensor->nb_of_lane = n_lanes;

	/* Clock lane must be first */
	if (ep.bus.mipi_csi2.clock_lane != 0) {
		dev_err(&client->dev, "Clk lane must be mapped to lane 0\n");
		ret = -EINVAL;
		goto done;
	}

	/*
	 * Prepare Output Interface conf based on lane settings
	 * logical to physical lane conversion (+ pad remaining slots)
	 */
	for (l = 0; l < n_lanes; l++)
		phy_data_lanes[ep.bus.mipi_csi2.data_lanes[l] - 1] = l;
	for (p = 0; p < VD1943_MAX_CSI_DATA_LANES; p++) {
		if (phy_data_lanes[p] != ~0)
			continue;
		phy_data_lanes[p] = l;
		l++;
	}
	sensor->oif_lane_phy_map = phy_data_lanes[0] |
				   (phy_data_lanes[1] << 2) |
				   (phy_data_lanes[2] << 4) |
				   (phy_data_lanes[3] << 6);
	sensor->oif_lane_phy_swap = ep.bus.mipi_csi2.lane_polarities[1] |
				    (ep.bus.mipi_csi2.lane_polarities[2] << 1) |
				    (ep.bus.mipi_csi2.lane_polarities[3] << 2) |
				    (ep.bus.mipi_csi2.lane_polarities[4] << 3) |
				    (ep.bus.mipi_csi2.lane_polarities[0] << 4);

	/* Check link frequency */
	if (!ep.nr_of_link_frequencies) {
		dev_err(&client->dev, "link-frequency not found in DT\n");
		ret = -EINVAL;
		goto done;
	}
	frequency = (n_lanes == 2) ? VD1943_LINK_FREQ_DEF_2LANES :
				     VD1943_LINK_FREQ_DEF_4LANES;
	if (ep.nr_of_link_frequencies != 1 ||
	    ep.link_frequencies[0] != frequency) {
		dev_err(&client->dev, "Link frequency not supported: %lld\n",
			ep.link_frequencies[0]);
		ret = -EINVAL;
		goto done;
	}

	/* Compute MIPI bandwidth */
	sensor->mipi_bandwidth = ((n_lanes == 2) ?
					  VD1943_LINK_FREQ_DEF_2LANES :
					  VD1943_LINK_FREQ_DEF_4LANES) *
				 2 * n_lanes;

done:
#if KERNEL_VERSION(4, 20, 0) > LINUX_VERSION_CODE
	v4l2_fwnode_endpoint_free(ep_ptr);
#else
	v4l2_fwnode_endpoint_free(&ep);
#endif

	return ret;
}

static int vd1943_parse_dt_gpios_array(struct vd1943 *sensor, char *prop_name,
				       u32 *array, int *nb)
{
	struct i2c_client *client = sensor->i2c_client;
	struct device_node *np = client->dev.of_node;
	unsigned int i;

	*nb = of_property_read_variable_u32_array(np, prop_name, array, 0,
						  VD1943_NB_GPIOS);

	if (*nb == -EINVAL) {
		*nb = 0;
		return *nb;
	} else if (*nb < 0) {
		dev_err(&client->dev, "Failed to read %s prop\n", prop_name);
		return *nb;
	}

	for (i = 0; i < *nb; i++) {
		if (array[i] >= VD1943_NB_GPIOS) {
			dev_err(&client->dev, "Invalid GPIO : %d\n", array[i]);
			return -EINVAL;
		}
	}

	return 0;
}

static int vd1943_parse_dt_gpios(struct vd1943 *sensor)
{
	struct i2c_client *client = sensor->i2c_client;
	struct device_node *np = client->dev.of_node;
	u32 led_gpios[VD1943_NB_GPIOS];
	int nb_gpios_leds;
	u32 in_sync_gpio;
	unsigned int i;
	int ret;

	/* Initialize GPIOs to default */
	for (i = 0; i < VD1943_NB_GPIOS; i++)
		sensor->gpios[i] = VD1943_GPIOX_GPIO_IN;
	sensor->ext_leds_mask = 0;
	sensor->ext_vt_sync = VD1943_NB_GPIOS;

	/* Take into account optional 'st,leds' output for GPIOs */
	ret = vd1943_parse_dt_gpios_array(sensor, "st,leds", led_gpios,
					  &nb_gpios_leds);
	if (ret)
		return ret;

	for (i = 0; i < nb_gpios_leds; i++) {
		sensor->gpios[led_gpios[i]] = VD1943_GPIOX_STROBE_MODE;
		set_bit(led_gpios[i], &sensor->ext_leds_mask);
	}

	/* Take into account optional 'st,in-sync' input for GPIOs */
	ret = of_property_read_u32(np, "st,in-sync", &in_sync_gpio);
	if (ret < 0 && ret != -EINVAL) {
		dev_err(&client->dev, "Failed to read st,in-sync prop\n");
		return ret;
	}

	if (ret != -EINVAL) {
		if (in_sync_gpio >= VD1943_NB_GPIOS) {
			dev_err(&client->dev, "Invalid GPIO : %d\n",
				in_sync_gpio);
			return -EINVAL;
		}
		if (sensor->gpios[in_sync_gpio] != VD1943_GPIOX_GPIO_IN) {
			dev_err(&client->dev, "Multiple use of GPIO %d\n",
				in_sync_gpio);
			return -EINVAL;
		}
		sensor->gpios[in_sync_gpio] = VD1943_GPIOX_FSYNC_IN;
		sensor->ext_vt_sync = in_sync_gpio;
	}

	return 0;
}

static int vd1943_parse_dt(struct vd1943 *sensor)
{
	struct i2c_client *client = sensor->i2c_client;
	struct device *dev = &client->dev;
	struct fwnode_handle *endpoint;
	int ret;

#if KERNEL_VERSION(5, 2, 0) > LINUX_VERSION_CODE
	endpoint =
		fwnode_graph_get_next_endpoint(of_fwnode_handle(dev->of_node),
					       NULL);
#else
	endpoint = fwnode_graph_get_endpoint_by_id(dev_fwnode(dev), 0, 0, 0);
#endif
	if (!endpoint) {
		dev_err(dev, "endpoint node not found\n");
		return -EINVAL;
	}

	ret = vd1943_check_csi_conf(sensor, endpoint);
	fwnode_handle_put(endpoint);
	if (ret)
		return ret;

	return vd1943_parse_dt_gpios(sensor);
}

static int vd1943_get_regulators(struct vd1943 *sensor)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(sensor->supplies); i++)
		sensor->supplies[i].supply = vd1943_supply_names[i];

	return devm_regulator_bulk_get(&sensor->i2c_client->dev,
				       ARRAY_SIZE(sensor->supplies),
				       sensor->supplies);
}

static int vd1943_prepare_clock_tree(struct vd1943 *sensor)
{
	struct i2c_client *client = sensor->i2c_client;
	u32 ndiv;
	u32 pll_clk;

	/* External clock must be in [12Mhz-50Mhz] */
	if (sensor->xclk_freq < 12 * HZ_PER_MHZ ||
	    sensor->xclk_freq > 50 * HZ_PER_MHZ) {
		dev_err(&client->dev,
			"Ext clock must be in [12Mhz-50Mhz]. Provided %lu MHz\n",
			sensor->xclk_freq / HZ_PER_MHZ);
		return -EINVAL;
	}

	if (sensor->xclk_freq < 27 * HZ_PER_MHZ) {
		ndiv = 1500 * HZ_PER_MHZ / sensor->xclk_freq;
		pll_clk = ndiv * sensor->xclk_freq;
	} else {
		ndiv = 3000 * HZ_PER_MHZ / sensor->xclk_freq;
		pll_clk = ndiv * 2 * sensor->xclk_freq;
	}

	/* vd1943 is designed to run with a pixel rate at 375 Mhz. */
	sensor->pixel_rate = pll_clk / 4;

	return 0;
}

static int vd1943_detect(struct vd1943 *sensor)
{
	struct i2c_client *client = sensor->i2c_client;
	struct device *dev = &client->dev;
	int model_id = 0;
	int rom_version = 0;
	int optical_version = 0;
	int is_rgbnir = 0;
	int ret = 0;

	sensor->model = (uintptr_t)device_get_match_data(dev);

	ret = vd1943_read(sensor, VD1943_REG_MODEL_ID, &model_id, NULL);
	if (ret)
		return ret;

	if (model_id != VD1943_MODEL_ID_1_3 &&
	    model_id != VD1943_MODEL_ID_1_4) {
		dev_err(&client->dev, "Unsupported sensor id : %x", model_id);
		return -ENODEV;
	}

	ret = vd1943_read(sensor, VD1943_REG_ROM_REVISION, &rom_version, NULL);
	if (ret)
		return ret;

	if (rom_version == VD1943_ROM_1_3) {
		sensor->is_fastboot = false;
	} else if (rom_version == VD1943_ROM_1_4) {
		sensor->is_fastboot = true;
	} else {
		dev_err(&client->dev, "Unsupported rom version : %x",
			rom_version);
		return -ENODEV;
	}

	if (sensor->is_fastboot) {
		ret = vd1943_read(sensor, VD1943_REG_CFA_SELECTION,
				  &optical_version, NULL);
		if (ret)
			return ret;

		is_rgbnir = ((optical_version & 0x0f) == VD1943_OPTICAL_RGBIR);
		if ((is_rgbnir && sensor->model == VD1943_MODEL_VD5943) ||
		    (!is_rgbnir && sensor->model == VD1943_MODEL_VD1943)) {
			dev_warn(&client->dev,
				 "Found %s sensor, while %s model is defined in DT",
				 (is_rgbnir) ? "RGBNir" : "Mono",
				 (sensor->model == VD1943_MODEL_VD1943) ?
					 "vd1943" :
					 "vd5943");
			return -ENODEV;
		}
	}

	return 0;
}

static int vd1943_subdev_init(struct vd1943 *sensor)
{
	struct i2c_client *client = sensor->i2c_client;
#if KERNEL_VERSION(5, 19, 0) > LINUX_VERSION_CODE
	unsigned int def_mode = VD1943_DEFAULT_MODE;
	unsigned int def_mbus_code =
		vdx941_configs[sensor->model][VD1943_GS_MODE][0].mbus_code;
#endif
	int ret;

#if KERNEL_VERSION(5, 19, 0) > LINUX_VERSION_CODE
	mutex_init(&sensor->lock);
#endif

	/* Init sub device */
	v4l2_i2c_subdev_init(&sensor->sd, client, &vd1943_subdev_ops);
#if KERNEL_VERSION(6, 8, 0) > LINUX_VERSION_CODE
#else
	sensor->sd.internal_ops = &vd1943_internal_ops;
#endif
	sensor->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
			    V4L2_SUBDEV_FL_HAS_EVENTS;
	sensor->sd.entity.ops = &vd1943_subdev_entity_ops;

	/* Init source pad */
	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;
	sensor->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sensor->sd.entity, 1, &sensor->pad);
	if (ret) {
		dev_err(&client->dev, "Failed to init media entity : %d", ret);
		return ret;
	}

	/* Init controls */
	ret = vd1943_init_controls(sensor);
	if (ret) {
		dev_err(&client->dev, "Controls initialization failed %d", ret);
		goto err_media;
	}

	/* Init vd1943 struct : default resolution + raw8 */
	sensor->streaming = false;
#if KERNEL_VERSION(5, 19, 0) > LINUX_VERSION_CODE
	vd1943_update_img_pad_format(sensor, &vd1943_supported_modes[def_mode],
				     def_mbus_code, &sensor->active_fmt);
	sensor->active_crop.width = vd1943_supported_modes[def_mode].width;
	sensor->active_crop.height = vd1943_supported_modes[def_mode].height;
	sensor->active_crop.left = 320;
	sensor->active_crop.top = 352;
#else
	sensor->sd.state_lock = sensor->ctrl_handler.lock;
	ret = v4l2_subdev_init_finalize(&sensor->sd);
	if (ret) {
		dev_err(&client->dev, "subdev init error: %d", ret);
		goto err_ctrls;
	}
#endif

	return vd1943_update_controls(sensor);

#if KERNEL_VERSION(5, 19, 0) > LINUX_VERSION_CODE
#else
err_ctrls:
	v4l2_ctrl_handler_free(sensor->sd.ctrl_handler);
#endif

err_media:
	media_entity_cleanup(&sensor->sd.entity);

	return ret;
}

static void vd1943_subdev_cleanup(struct vd1943 *sensor)
{
	v4l2_async_unregister_subdev(&sensor->sd);
#if KERNEL_VERSION(5, 19, 0) > LINUX_VERSION_CODE
	mutex_destroy(&sensor->lock);
#else
	v4l2_subdev_cleanup(&sensor->sd);
#endif
	media_entity_cleanup(&sensor->sd.entity);
	v4l2_ctrl_handler_free(sensor->sd.ctrl_handler);
}

static int vd1943_err_probe(struct device *dev, int ret, char *msg)
{
#if KERNEL_VERSION(5, 9, 0) > LINUX_VERSION_CODE
	dev_err(dev, "%s", msg);
	return ret;
#else
	return dev_err_probe(dev, ret, msg);
#endif
}

static int vd1943_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct vd1943 *sensor;
	int ret;

	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	sensor->i2c_client = client;

	ret = vd1943_parse_dt(sensor);
	if (ret)
		return vd1943_err_probe(dev, ret, "Failed to parse Device Tree.");

	/* Get (and check) resources : power regs, ext clock, reset gpio */
	ret = vd1943_get_regulators(sensor);
	if (ret)
		return vd1943_err_probe(dev, ret, "Failed to get regulators.");

	sensor->xclk = devm_clk_get(dev, NULL);
	if (IS_ERR(sensor->xclk))
		return vd1943_err_probe(dev, PTR_ERR(sensor->xclk),
				     "Failed to get xclk.");
	sensor->xclk_freq = clk_get_rate(sensor->xclk);
	ret = vd1943_prepare_clock_tree(sensor);
	if (ret)
		return ret;

	sensor->reset_gpio =
		devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(sensor->reset_gpio))
		return vd1943_err_probe(dev, PTR_ERR(sensor->reset_gpio),
				     "Failed to get reset gpio.");

#if KERNEL_VERSION(6, 8, 0) > LINUX_VERSION_CODE
	sensor->regmap = devm_regmap_init_i2c(client, &vd1943_regmap_config);
#else
	sensor->regmap = devm_cci_regmap_init_i2c(client, 16);
#endif
	if (IS_ERR(sensor->regmap))
		return vd1943_err_probe(dev, PTR_ERR(sensor->regmap),
				     "Failed to init regmap.");

	/* Power ON */
	ret = vd1943_power_on(sensor);
	if (ret)
		return vd1943_err_probe(dev, ret, "Sensor power on failed.");

	/* Enable PM runtime with autosuspend (sensor being ON, set active) */
	pm_runtime_set_active(dev);
	pm_runtime_get_noresume(dev);
	pm_runtime_enable(dev);
	pm_runtime_set_autosuspend_delay(dev, 4000);
	pm_runtime_use_autosuspend(dev);

	/* Check HW model/version */
	ret = vd1943_detect(sensor);
	if (ret) {
		dev_err(&client->dev, "Sensor detect failed : %d", ret);
		goto err_power_off;
	}

	/* Initialize, then register V4L2 subdev */
	ret = vd1943_subdev_init(sensor);
	if (ret) {
		dev_err(&client->dev, "V4l2 init failed : %d", ret);
		goto err_power_off;
	}

	ret = v4l2_async_register_subdev(&sensor->sd);
	if (ret) {
		dev_err(&client->dev, "async subdev register failed %d", ret);
		goto err_subdev;
	}

	/* Sensor could now be powered off (after the autosuspend delay) */
	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_autosuspend(dev);

	dev_info(&client->dev, "Successfully probe %s sensor",
		 (sensor->model == VD1943_MODEL_VD1943) ? "vd1943" : "vd5943");

	return 0;

err_subdev:
	vd1943_subdev_cleanup(sensor);
err_power_off:
	pm_runtime_disable(dev);
	pm_runtime_put_noidle(dev);
	vd1943_power_off(sensor);

	return ret;
}

#if KERNEL_VERSION(6, 1, 0) > LINUX_VERSION_CODE
static int vd1943_remove(struct i2c_client *client)
#else
static void vd1943_remove(struct i2c_client *client)
#endif
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct vd1943 *sensor = to_vd1943(sd);

	vd1943_subdev_cleanup(sensor);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		vd1943_power_off(sensor);
	pm_runtime_set_suspended(&client->dev);
#if KERNEL_VERSION(6, 1, 0) > LINUX_VERSION_CODE

	return 0;
#endif
}

static const struct of_device_id vd1943_dt_ids[] = {
	{ .compatible = "st,vd1943", .data = (void *)VD1943_MODEL_VD1943 },
	{ .compatible = "st,vd5943", .data = (void *)VD1943_MODEL_VD5943 },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, vd1943_dt_ids);

static struct i2c_driver vd1943_i2c_driver = {
	.driver = {
		.name  = "vd1943",
		.of_match_table = vd1943_dt_ids,
		.pm = &vd1943_pm_ops,
	},
#if KERNEL_VERSION(6, 3, 0) > LINUX_VERSION_CODE
	.probe_new = vd1943_probe,
#else
	.probe = vd1943_probe,
#endif
	.remove = vd1943_remove,
};

module_i2c_driver(vd1943_i2c_driver);

MODULE_AUTHOR("Sylvain Petinot <sylvain.petinot@foss.st.com>");
MODULE_DESCRIPTION("ST VD1943 sensor driver");
MODULE_LICENSE("GPL");
