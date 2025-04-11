/*
 * Copyright (C) 2012-2015 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2018 NXP
 * Copyright (c) 2020 VeriSilicon Holdings Co., Ltd.
 */
/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/of_graph.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/v4l2-mediabus.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <linux/uaccess.h>

#include "vvsensor.h"

#define IMX351_VOLTAGE_ANALOG 2800000
#define IMX351_VOLTAGE_DIGITAL_CORE 1500000
#define IMX351_VOLTAGE_DIGITAL_IO 1800000

#define IMX351_XCLK_MIN 6000000
#define IMX351_XCLK_MAX 24000000

#define IMX351_SENS_PAD_SOURCE 0
#define IMX351_SENS_PADS_NUM 1

#define IMX351_RESERVE_ID 0x0351
#define DCG_CONVERSION_GAIN 11

#define IMX351_DEFAULT_CLK_FREQ 24000000
#define IMX351_DEFAULT_LINK_FREQ 676800000
#define IMX351_DEFAULT_LINK_MFREQ (IMX351_DEFAULT_LINK_FREQ / 1000000)
#define IMX351_DEFAULT_PIXEL_RATE ((IMX351_DEFAULT_LINK_FREQ * 8LL) / 10)
#define IMX351_DEFAULT_MBUS_CODE MEDIA_BUS_FMT_SRGGB10_1X10
#define IMX351_DEFAULT_FIVAL {1, 30}
#define IMX351_LINE_LENGTH 6032 /* 676.8 MHz * 4656 pixels per line */
#define IMX351_MIN_WIDTH 4
#define IMX351_MIN_HEIGHT 4
#define IMX351_MAX_DEFAULT_WIDTH 2020
#define IMX351_MAX_DEFAULT_HEIGHT 1136
#define IMX351_MAX_BOUNDS_WIDTH 2020
#define IMX351_MAX_BOUNDS_HEIGHT 1136
#define IMX351_CID_CUSTOM_BASE (V4L2_CID_USER_BASE | 0xf000)
#define IMX351_CID_GREEN_BALANCE (IMX351_CID_CUSTOM_BASE + 0)

#define client_to_imx351(client) \
    container_of(i2c_get_clientdata(client), struct imx351, subdev)

struct imx351_capture_properties
{
    __u64 max_lane_frequency;
    __u64 max_pixel_frequency;
    __u64 max_data_rate;
};

struct imx351
{
    struct i2c_client *i2c_client;
    struct regulator *io_regulator;
    struct regulator *core_regulator;
    struct regulator *analog_regulator;
    unsigned int pwn_gpio;
    unsigned int rst_gpio;
    unsigned int mclk;
    unsigned int mclk_source;
    struct clk *sensor_clk;
    unsigned int csi_id;
    struct imx351_capture_properties ocp;

    struct v4l2_subdev subdev;
    struct media_pad pads[IMX351_SENS_PADS_NUM];

    struct v4l2_mbus_framefmt format;
    vvcam_mode_info_t cur_mode;
    sensor_blc_t blc;
    sensor_white_balance_t wb;
    struct mutex lock;
    u32 stream_status;
    u32 resume_status;
};
static struct vvcam_sccb_data_s imx351_2328x1744_setting[] = {
    /*       4Lane
       reg_2
       1/2Binning
       H: 2328
       V: 1744
       MIPI output setting
           Address value*/
    {0x0112, 0x0A}, //
    {0x0113, 0x0A}, //
    {0x0114, 0x01}, // Line Length PCK Setting
    {0x0342, 0x2F}, //
    {0x0343, 0x20}, // Frame Length Lines Setting
    {0x0340, 0x07}, //
    {0x0341, 0x14}, // ROI Setting
    {0x0344, 0x00}, //
    {0x0345, 0x00}, //
    {0x0346, 0x00}, //
    {0x0347, 0x04}, //
    {0x0348, 0x12}, //
    {0x0349, 0x2F}, //
    {0x034A, 0x0D}, //
    {0x034B, 0xA3}, // Mode Setting
    {0x0220, 0x00}, //
    {0x0221, 0x11}, //
    {0x0222, 0x01}, //
    {0x0381, 0x01}, //
    {0x0383, 0x01}, //
    {0x0385, 0x01}, //
    {0x0387, 0x01}, //
    {0x0900, 0x01}, //
    {0x0901, 0x22}, //
    {0x0902, 0x08}, // 0A}, //
    {0x3243, 0x00}, //
    {0x3F4C, 0x01}, //
    {0x3F4D, 0x03}, //
    {0x4254, 0x7F}, // Digital Crop & Scaling
    {0x0401, 0x00}, //
    {0x0404, 0x00}, //
    {0x0405, 0x10}, //
    {0x0408, 0x00}, //
    {0x0409, 0x00}, //
    {0x040A, 0x00}, //
    {0x040B, 0x00}, //
    {0x040C, 0x09}, //
    {0x040D, 0x18}, //
    {0x040E, 0x06}, //
    {0x040F, 0xD0}, // Output Size Setting
    {0x034C, 0x09}, //
    {0x034D, 0x18}, //
    {0x034E, 0x06}, //
    {0x034F, 0xD0}, // Clock Setting
    {0x0301, 0x05}, //
    {0x0303, 0x02}, //
    {0x0305, 0x04}, //
    {0x0306, 0x01}, //
    {0x0307, 0x12}, //
    {0x030B, 0x04}, //
    {0x030D, 0x06}, //
    {0x030E, 0x01}, //
    {0x030F, 0x76}, //
    {0x0310, 0x01}, //
    {0x0820, 0x05}, //
    {0x0821, 0xD8}, //
    {0x0822, 0x00}, //
    {0x0823, 0x00}, //
    {0xBC41, 0x01}, // PDAF Setting
    {0x3E20, 0x01}, //
    {0x3E37, 0x01}, // 0x01},// enable
    {0x3E3B, 0x00}, // Other Setting
    {0x0106, 0x00}, //
    {0x0B00, 0x00}, //
    {0x3230, 0x00}, //
    {0x3C00, 0x6D}, //
    {0x3C01, 0x5B}, //
    {0x3C02, 0x77}, //
    {0x3C03, 0x66}, //
    {0x3C04, 0x00}, //
    {0x3C05, 0x84}, //
    {0x3C06, 0x14}, //
    {0x3C07, 0x00}, //
    {0x3C08, 0x01}, //
    {0x3F14, 0x01}, //
    {0x3F17, 0x00}, //
    {0x3F3C, 0x01}, //
    {0x3F78, 0x03}, //
    {0x3F79, 0xA4}, //
    {0x3F7C, 0x00}, //
    {0x3F7D, 0x00}, //
    {0x97C1, 0x04}, //
    {0x97C5, 0x0C}, // Integration Setting
    {0x0202, 0x07}, //
    {0x0203, 0x00}, //
    {0x0224, 0x01}, //
    {0x0225, 0xF4}, // Gain Setting
    {0x0204, 0x00}, //
    {0x0205, 0x00}, //
    {0x0216, 0x00}, //
    {0x0217, 0x00}, //
    {0x020E, 0x01}, //
    {0x020F, 0x00}, //
    {0x0218, 0x01}, //
    {0x0219, 0x00}, //

};

static struct vvcam_sccb_data_s imx351_3840x2160_setting[] = {

    {0x0136, 0x18}, //
    {0x0137, 0x00}, //
    {0x3C7D, 0x28}, //
    {0x3C7E, 0x01}, //
    {0x3C7F, 0x03}, //
    {0x3140, 0x02}, //
    {0x4430, 0x05}, //
    {0x4431, 0xDC}, //
    {0x5222, 0x02}, //
    {0x562B, 0x0A}, //
    {0x562D, 0x0C}, //
    {0x56B7, 0x74}, //
    {0x6200, 0x95}, //
    {0x6201, 0xB9}, //
    {0x6202, 0x58}, //
    {0x6220, 0x05}, //
    {0x6229, 0x70}, //
    {0x622A, 0xC3}, //
    {0x622C, 0x54}, //
    {0x622F, 0xA8}, //
    {0x6231, 0xD3}, //
    {0x6234, 0x6A}, //
    {0x6235, 0x8C}, //
    {0x6236, 0x37}, //
    {0x6237, 0x45}, //
    {0x623A, 0x96}, //
    {0x6240, 0x59}, //
    {0x6241, 0x83}, //
    {0x6243, 0x54}, //
    {0x6246, 0xA8}, //
    {0x6248, 0xD3}, //
    {0x624B, 0x65}, //
    {0x624C, 0x98}, //
    {0x624D, 0x37}, //
    {0x6265, 0x45}, //
    {0x626E, 0x72}, //
    {0x626F, 0xC3}, //
    {0x6271, 0x58}, //
    {0x6279, 0xA7}, //
    {0x627B, 0x37}, //
    {0x6282, 0x82}, //
    {0x6283, 0x80}, //
    {0x6286, 0x07}, //
    {0x6287, 0xC0}, //
    {0x6288, 0x08}, //
    {0x628A, 0x18}, //
    {0x628B, 0x80}, //
    {0x628C, 0x20}, //
    {0x628E, 0x32}, //
    {0x6290, 0x40}, //
    {0x6292, 0x0A}, //
    {0x6296, 0x50}, //
    {0x629A, 0xF8}, //
    {0x629B, 0x01}, //
    {0x629D, 0x03}, //
    {0x629F, 0x04}, //
    {0x62B1, 0x06}, //
    {0x62B5, 0x3C}, //
    {0x62B9, 0xC8}, //
    {0x62BC, 0x02}, //
    {0x62BD, 0x70}, //
    {0x62D0, 0x06}, //
    {0x62D4, 0x38}, //
    {0x62D8, 0xB8}, //
    {0x62DB, 0x02}, //
    {0x62DC, 0x40}, //
    {0x62DD, 0x03}, //
    {0x637A, 0x31}, //
    {0x637B, 0xD4}, //
    {0x6388, 0x22}, //
    {0x6389, 0x82}, //
    {0x638A, 0xC8}, //
    {0x9006, 0x01}, //
    {0x935D, 0x01}, //
    {0x9389, 0x05}, //
    {0x938B, 0x05}, //
    {0x9391, 0x05}, //
    {0x9393, 0x05}, //
    {0x9395, 0x82}, //
    {0x9397, 0x78}, //
    {0x9399, 0x05}, //
    {0x939B, 0x05}, //
    {0xBC40, 0x03}, //
    {0xE000, 0x01}, //
    {0xE0A5, 0x0B}, //
    {0xE0A6, 0x0B}, //
    {0x0112, 0x0A}, //
    {0x0113, 0x0A}, //
    {0x0114, 0x03}, //
    {0x0342, 0x17}, //
    {0x0343, 0x90}, //
    {0x0340, 0x0D}, //
    {0x0341, 0xFC}, //
    {0x0344, 0x00}, //
    {0x0345, 0x00}, //
    {0x0346, 0x00}, //
    {0x0347, 0x00}, //
    {0x0348, 0x12}, //
    {0x0349, 0x2F}, //
    {0x034A, 0x0D}, //
    {0x034B, 0xA7}, //
    {0x0220, 0x00}, //
    {0x0221, 0x11}, //
    {0x0222, 0x01}, //
    {0x0381, 0x01}, //
    {0x0383, 0x01}, //
    {0x0385, 0x01}, //
    {0x0387, 0x01}, //
    {0x0900, 0x00}, //
    {0x0901, 0x11}, //
    {0x0902, 0x0A}, //
    {0x3243, 0x00}, //
    {0x3F4C, 0x01}, //
    {0x3F4D, 0x01}, //
    {0x4254, 0x7F}, //
    {0x9385, 0x5B}, //
    {0x9387, 0x54}, //
    {0x938D, 0x77}, //
    {0x938F, 0x66}, //
    {0x0401, 0x00}, //
    {0x0404, 0x00}, //
    {0x0405, 0x10}, //
    {0x0408, 0x01}, //
    {0x0409, 0x98}, //
    {0x040A, 0x02}, //
    {0x040B, 0x9C}, //
    {0x040C, 0x0F}, //
    {0x040D, 0x00}, //
    {0x040E, 0x08}, //
    {0x040F, 0x70}, //
    {0xBC41, 0x01}, //
    {0x034C, 0x0F}, //
    {0x034D, 0x00}, //
    {0x034E, 0x08}, //
    {0x034F, 0x70}, //
    {0x0301, 0x05}, //
    {0x0303, 0x02}, //
    {0x0305, 0x04}, //
    {0x0306, 0x01}, //
    {0x0307, 0x0E}, //
    {0x030B, 0x01}, //
    {0x030D, 0x04}, //
    {0x030E, 0x00}, //
    {0x030F, 0xDC}, //
    {0x0310, 0x01}, //
    {0x0820, 0x14}, //
    {0x0821, 0xA0}, //
    {0x0822, 0x00}, //
    {0x0823, 0x00}, //
    {0x3E20, 0x01}, //
    {0x3E37, 0x01}, //
    {0x3E3B, 0x00}, //
    {0x3614, 0x00}, //
    {0x3616, 0x0E}, //
    {0x3617, 0x66}, //
    {0x0106, 0x00}, //
    {0x0B00, 0x00}, //
    {0x3230, 0x00}, //
    {0x3F14, 0x01}, //
    {0x3F17, 0x00}, //
    {0x3F3C, 0x01}, //
    {0x0202, 0x0D}, // Integration time
    {0x0203, 0xE8}, //
    {0x0224, 0x01}, //
    {0x0225, 0xF4}, //
    {0x0204, 0x00}, // Analog gain
    {0x0205, 0x00}, //
    {0x0216, 0x00}, // Digital gain
    {0x0217, 0x00}, //
    {0x020E, 0x01}, //
    {0x020F, 0x00}, //
    {0x0218, 0x01}, //
    {0x0219, 0x00}, //

};

static struct vvcam_sccb_data_s imx351_1280x720_setting[] = {
    /*    4Lane
        reg_3
        720p@90fps
        H: 1280
        V: 720
        MIPI output setting
            Address value*/
    {0x0112, 0x0A}, //
    {0x0113, 0x0A}, //
    {0x0114, 0x01}, // Line Length PCK Setting
    {0x0342, 0x11}, //
    {0x0343, 0x9C}, // Frame Length Lines Setting
    {0x0340, 0x07}, //
    {0x0341, 0x18}, // ROI Setting
    {0x0344, 0x00}, //
    {0x0345, 0x00}, //
    {0x0346, 0x00}, //
    {0x0347, 0x00}, //
    {0x0348, 0x12}, //
    {0x0349, 0x2F}, //
    {0x034A, 0x0D}, //
    {0x034B, 0x9F}, // Mode Setting
    {0x0220, 0x00}, //
    {0x0221, 0x11}, //
    {0x0222, 0x01}, //
    {0x0381, 0x01}, //
    {0x0383, 0x01}, //
    {0x0385, 0x01}, //
    {0x0387, 0x01}, //
    {0x0900, 0x01}, //
    {0x0901, 0x22}, //
    {0x0902, 0x08}, // A}, //
    {0x3243, 0x00}, //
    {0x3F4C, 0x01}, //
    {0x3F4D, 0x01}, //
    {0x4254, 0x7F}, // Digital Crop & Scaling
    {0x0401, 0x02}, //
    {0x0404, 0x00}, //
    {0x0405, 0x1D}, //
    {0x0408, 0x00}, //
    {0x0409, 0x04}, //
    {0x040A, 0x00}, //
    {0x040B, 0xDA}, //
    {0x040C, 0x09}, //
    {0x040D, 0x10}, //
    {0x040E, 0x05}, //
    {0x040F, 0x1A}, // Output Size Setting
    {0x034C, 0x05}, //
    {0x034D, 0x00}, //
    {0x034E, 0x02}, //
    {0x034F, 0xD0}, // Clock Setting
    {0x0301, 0x05}, //
    {0x0303, 0x02}, //
    {0x0305, 0x04}, //
    {0x0306, 0x01}, //
    {0x0307, 0x34}, //
    {0x030B, 0x02}, //
    {0x030D, 0x06}, //
    {0x030E, 0x01}, //
    {0x030F, 0x7B}, //
    {0x0310, 0x01}, //
    {0x0820, 0x0B}, //
    {0x0821, 0xD8}, //
    {0x0822, 0x00}, //
    {0x0823, 0x00}, //
    {0xBC41, 0x01}, // PDAF Setting
    {0x3E20, 0x01}, //
    {0x3E37, 0x00}, //
    {0x3E3B, 0x00}, // Other Setting
    {0x0106, 0x00}, //
    {0x0B00, 0x00}, //
    {0x3230, 0x00}, //
    {0x3C00, 0x5B}, //
    {0x3C01, 0x54}, //
    {0x3C02, 0x77}, //
    {0x3C03, 0x66}, //
    {0x3C04, 0x00}, //
    {0x3C05, 0x9A}, //
    {0x3C06, 0x14}, //
    {0x3C07, 0x00}, //
    {0x3C08, 0x01}, //
    {0x3F14, 0x01}, //
    {0x3F17, 0x00}, //
    {0x3F3C, 0x01}, //
    {0x3F78, 0x02}, //
    {0x3F79, 0xAC}, //
    {0x3F7C, 0x00}, //
    {0x3F7D, 0x00}, //
    {0x97C1, 0x04}, //
    {0x97C5, 0x0C}, // Integration Setting
    {0x0202, 0x07}, //
    {0x0203, 0x04}, //
    {0x0224, 0x01}, //
    {0x0225, 0xF4}, // Gain Setting
    {0x0204, 0x00}, //
    {0x0205, 0x00}, //
    {0x0216, 0x00}, //
    {0x0217, 0x00}, //
    {0x020E, 0x01}, //
    {0x020F, 0x00}, //
    {0x0218, 0x01}, //
    {0x0219, 0x00}, //

};

static struct vvcam_sccb_data_s imx351_2020x1136_setting[] = {
    /*    4Lane
        reg_3
        1080p@240fs
        H: 2020
        V: 1136
        MIPI output setting
            Address value*/
    {0x0112, 0x0A},
    {0x0113, 0x0A},
    {0x0114, 0x03},

    // Line Length PCK Setting
    {0x0342, 0x0B},
    {0x0343, 0x50},

    // Frame Length Lines Setting
    {0x0340, 0x04},
    {0x0341, 0xB8},

    // ROI Setting
    {0x0344, 0x00},
    {0x0345, 0x00},
    {0x0346, 0x02},
    {0x0347, 0x60},
    {0x0348, 0x12},
    {0x0349, 0x2F},
    {0x034A, 0x0B},
    {0x034B, 0x3F},

    // Mode Setting
    {0x0220, 0x00},
    {0x0221, 0x11},
    {0x0222, 0x01},
    {0x0381, 0x01},
    {0x0383, 0x01},
    {0x0385, 0x01},
    {0x0387, 0x01},
    {0x0900, 0x01},
    {0x0901, 0x22},
    {0x0902, 0x0A},
    {0x3243, 0x00},
    {0x3F4C, 0x01},
    {0x3F4D, 0x01},
    {0x4254, 0x7F},

    // Digital Crop & Scaling
    {0x0401, 0x00},
    {0x0404, 0x00},
    {0x0405, 0x10},
    {0x0408, 0x00},
    {0x0409, 0x9A},
    {0x040A, 0x00},
    {0x040B, 0x00},
    {0x040C, 0x07},
    {0x040D, 0xE4},
    {0x040E, 0x04},
    {0x040F, 0x70},

    // Output Size Setting
    {0x034C, 0x07},
    {0x034D, 0xE4},
    {0x034E, 0x04},
    {0x034F, 0x70},

    // Clock Setting
    {0x0301, 0x05},
    {0x0303, 0x02},
    {0x0305, 0x02},
    {0x0306, 0x00},
    {0x0307, 0xAF},
    {0x030B, 0x01},
    {0x030D, 0x04},
    {0x030E, 0x01},
    {0x030F, 0x1A},
    {0x0310, 0x01},
    {0x0820, 0x1A},
    {0x0821, 0x70},
    {0x0822, 0x00},
    {0x0823, 0x00},
    {0xBC41, 0x01},

    // PDAF Setting
    {0x3E20, 0x01},
    {0x3E37, 0x00},
    {0x3E3B, 0x00},

    // Other Setting
    {0x0106, 0x00},
    {0x0B00, 0x00},
    {0x3230, 0x00},
    {0x3C00, 0x5B},
    {0x3C01, 0x54},
    {0x3C02, 0x66},
    {0x3C03, 0x56},
    {0x3C04, 0x00},
    {0x3C05, 0x8C},
    {0x3C06, 0x14},
    {0x3C07, 0x00},
    {0x3C08, 0x01},
    {0x3F14, 0x01},
    {0x3F17, 0x00},
    {0x3F3C, 0x02},
    {0x3F78, 0x00},
    {0x3F79, 0x00},
    {0x3F7C, 0x00},
    {0x3F7D, 0x00},
    {0x97C1, 0x04},
    {0x97C5, 0x0C},
    {0x620D, 0xD0},
    {0x620E, 0x27},
    {0x6399, 0xF4},
    {0x6F6E, 0x02},

    // Integration Setting
    {0x0202, 0x04},
    {0x0203, 0xA4},
    {0x0224, 0x01},
    {0x0225, 0xF4},

    // Gain Setting
    {0x0204, 0x00},
    {0x0205, 0x00},
    {0x0216, 0x00},
    {0x0217, 0x00},
    {0x020E, 0x01},
    {0x020F, 0x00},
    {0x0218, 0x01},
    {0x0219, 0x00},

};
static struct vvcam_sccb_data_s imx351_init_setting[] = {

    {0x0136, 0x18}, {0x0137, 0x00}, {0x3C7D, 0x28}, {0x3C7E, 0x01}, {0x3C7F, 0x0F}, {0x3140, 0x02}, {0x3F7F, 0x01}, {0x4430, 0x05}, {0x4431, 0xDC}, {0x4ED0, 0x01}, {0x4ED1, 0x3E}, {0x4EDE, 0x01}, {0x4EDF, 0x45}, {0x5222, 0x02}, {0x5617, 0x0A}, {0x562B, 0x0A}, {0x562D, 0x0C}, {0x56B7, 0x74}, {0x6282, 0x82}, {0x6283, 0x80}, {0x6286, 0x07}, {0x6287, 0xC0}, {0x6288, 0x08}, {0x628A, 0x18}, {0x628B, 0x80}, {0x628C, 0x20}, {0x628E, 0x32}, {0x6290, 0x40}, {0x6292, 0x0A}, {0x6296, 0x50}, {0x629A, 0xF8}, {0x629B, 0x01}, {0x629D, 0x03}, {0x629F, 0x04}, {0x62B1, 0x06}, {0x62B5, 0x3C}, {0x62B9, 0xC8}, {0x62BC, 0x02}, {0x62BD, 0x70}, {0x62D0, 0x06}, {0x62D4, 0x38}, {0x62D8, 0xB8}, {0x62DB, 0x02}, {0x62DC, 0x40}, {0x62DD, 0x03}, {0x637A, 0x11}, {0x7BA0, 0x01}, {0x7BA9, 0x00}, {0x7BAA, 0x01}, {0x7BAD, 0x00}, {0x886B, 0x00}, {0x9002, 0x00}, {0x9003, 0x00}, {0x9004, 0x09}, {0x9006, 0x01}, {0x9200, 0x93}, {0x9201, 0x85}, {0x9202, 0x93}, {0x9203, 0x87}, {0x9204, 0x93}, {0x9205, 0x8D}, {0x9206, 0x93}, {0x9207, 0x8F}, {0x9208, 0x6A}, {0x9209, 0x22}, {0x920A, 0x6A}, {0x920B, 0x23}, {0x920C, 0x6A}, {0x920D, 0x0F}, {0x920E, 0x71}, {0x920F, 0x03}, {0x9210, 0x71}, {0x9211, 0x0B}, {0x935D, 0x01}, {0x9389, 0x05}, {0x938B, 0x05}, {0x9391, 0x05}, {0x9393, 0x05}, {0x9395, 0x82}, {0x9397, 0x78}, {0x9399, 0x05}, {0x939B, 0x05}, {0xA91F, 0x04}, {0xA921, 0x03}, {0xA923, 0x02}, {0xA93D, 0x05}, {0xA93F, 0x03}, {0xA941, 0x02}, {0xA9AF, 0x04}, {0xA9B1, 0x03}, {0xA9B3, 0x02}, {0xA9CD, 0x05}, {0xA9CF, 0x03}, {0xA9D1, 0x02}, {0xAA3F, 0x04}, {0xAA41, 0x03}, {0xAA43, 0x02}, {0xAA5D, 0x05}, {0xAA5F, 0x03}, {0xAA61, 0x02}, {0xAACF, 0x04}, {0xAAD1, 0x03}, {0xAAD3, 0x02}, {0xAAED, 0x05}, {0xAAEF, 0x03}, {0xAAF1, 0x02}, {0xAB87, 0x04}, {0xAB89, 0x03}, {0xAB8B, 0x02}, {0xABA5, 0x05}, {0xABA7, 0x03}, {0xABA9, 0x02}, {0xABB7, 0x04}, {0xABB9, 0x03}, {0xABBB, 0x02}, {0xABD5, 0x05}, {0xABD7, 0x03}, {0xABD9, 0x02}, {0xB388, 0x28}, {0xBC40, 0x03}, //  Image Quality adjustment setting
    {0x7B80, 0x00},
    {0x7B81, 0x00},
    {0xA900, 0x20},
    {0xA901, 0x20},
    {0xA902, 0x20},
    {0xA903, 0x15},
    {0xA904, 0x15},
    {0xA905, 0x15},
    {0xA906, 0x20},
    {0xA907, 0x20},
    {0xA908, 0x20},
    {0xA909, 0x15},
    {0xA90A, 0x15},
    {0xA90B, 0x15},
    {0xA915, 0x3F},
    {0xA916, 0x3F},
    {0xA917, 0x3F},
    {0xA949, 0x03},
    {0xA94B, 0x03},
    {0xA94D, 0x03},
    {0xA94F, 0x06},
    {0xA951, 0x06},
    {0xA953, 0x06},
    {0xA955, 0x03},
    {0xA957, 0x03},
    {0xA959, 0x03},
    {0xA95B, 0x06},
    {0xA95D, 0x06},
    {0xA95F, 0x06},
    {0xA98B, 0x1F},
    {0xA98D, 0x1F},
    {0xA98F, 0x1F},
    {0xAA21, 0x20},
    {0xAA22, 0x20},
    {0xAA24, 0x15},
    {0xAA25, 0x15},
    {0xAA26, 0x20},
    {0xAA27, 0x20},
    {0xAA28, 0x20},
    {0xAA29, 0x15},
    {0xAA2A, 0x15},
    {0xAA2B, 0x15},
    {0xAA35, 0x3F},
    {0xAA36, 0x3F},
    {0xAA37, 0x3F},
    {0xAA6B, 0x03},
    {0xAA6D, 0x03},
    {0xAA71, 0x06},
    {0xAA73, 0x06},
    {0xAA75, 0x03},
    {0xAA77, 0x03},
    {0xAA79, 0x03},
    {0xAA7B, 0x06},
    {0xAA7D, 0x06},
    {0xAA7F, 0x06},
    {0xAAAB, 0x1F},
    {0xAAAD, 0x1F},
    {0xAAAF, 0x1F},
    {0xAAB0, 0x20},
    {0xAAB1, 0x20},
    {0xAAB2, 0x20},
    {0xAB53, 0x20},
    {0xAB54, 0x20},
    {0xAB55, 0x20},
    {0xAB57, 0x40},
    {0xAB59, 0x40},
    {0xAB5B, 0x40},
    {0xAB63, 0x03},
    {0xAB65, 0x03},
    {0xAB67, 0x03},
    {0x9A00, 0x0C},
    {0x9A01, 0x0C},
    {0x9A06, 0x0C},
    {0x9A18, 0x0C},
    {0x9A19, 0x0C},
    {0xAA20, 0x3F},
    {0xAA23, 0x3F},
    {0xAA32, 0x3F},
    {0xAA69, 0x3F},
    {0xAA6F, 0x3F},
    {0xAAC2, 0x3F},
    {0x8D1F, 0x00},
    {0x8D27, 0x00},
    {0x9963, 0x64},
    {0x9964, 0x50},
    {0xAC01, 0x0A},
    {0xAC03, 0x0A},
    {0xAC05, 0x0A},
    {0xAC06, 0x01},
    {0xAC07, 0xC0},
    {0xAC09, 0xC0},
    {0xAC17, 0x0A},
    {0xAC19, 0x0A},
    {0xAC1B, 0x0A},
    {0xAC1C, 0x01},
    {0xAC1D, 0xC0},
    {0xAC1F, 0xC0},
    {0xBCF1, 0x02},

#if defined(CONFIG_DUAL_MODULE)
    {0x3F0B, 0x01},
    {0x3041, 0x01},
    {0x3040, 0x01},
    {0x4B81, 0x01},
    {0x3F39, 0x00},
    {0x3F3A, 0x0C},
    {0x3F3B, 0x74},
    {0x0350, 0x00},
    {0x423D, 0xFF},
    {0x4BD7, 0x16},
    {0x3040, 0x01},
#endif
    {0x0350, 0x01},
    {0x0101, 0x00}, // mirror

};

static struct vvcam_mode_info_s pimx351_mode_info[] = {
    {
        .index = 0,
        .size = {
            .bounds_width = 1920,
            .bounds_height = 1080,
            .top = 0,
            .left = 0,
            .width = 1920,
            .height = 1080,
        },
        .hdr_mode = SENSOR_MODE_LINEAR,
        .bit_width = 10,
        .data_compress = {
            .enable = 0,
        },
        .bayer_pattern = BAYER_GBRG,
        .ae_info = {
            .def_frm_len_lines = 0x8D,          // 141 lines
            .curr_frm_len_lines = 0x8D,         // 141 lines
            .one_line_exp_time_ns = 29625,      // unchanged
        
            .max_integration_line = 0x8D - 4,   // 137
            .min_integration_line = 1,
        
            .max_again = 8 * 1024,
            .min_again = 2 * 1024,
            .max_dgain = 4 * 1024,
            .min_dgain = 1.5 * 1024,
            .gain_step = 4,
            .start_exposure = 3 * 141 * 1024,   // optional; can adjust
            .cur_fps = 240 * 1024,
            .max_fps = 240 * 1024,
            .min_fps = 5 * 1024,
            .min_afps = 5 * 1024,
            .int_update_delay_frm = 1,
            .gain_update_delay_frm = 1,
        },
        .mipi_info = {
            .mipi_lane = 4, // Keeping it at 4 lane
        },
        .preg_data = imx351_2020x1136_setting,
        .reg_data_count = ARRAY_SIZE(imx351_2020x1136_setting),
    },

    {
        .index = 1,
        .size = {
            .bounds_width = 1280,
            .bounds_height = 720,
            .top = 0,
            .left = 0,
            .width = 1280,
            .height = 720,
        },
        .hdr_mode = SENSOR_MODE_LINEAR,
        .bit_width = 10,
        .data_compress = {
            .enable = 0,
        },
        .bayer_pattern = BAYER_GBRG,
        .ae_info = {
            .def_frm_len_lines = 0x35E0, // Increased for 30 FPS
            .curr_frm_len_lines = 0x35E0,
            .one_line_exp_time_ns = 0x1790, // Keeping same line time

            .max_integration_line = 0x35DC, // Frame length - 4
            .min_integration_line = 1,

            .max_again = 0x03D2,
            .min_again = 0,
            .max_dgain = 0x0FFF,
            .min_dgain = 0x0100,
            .gain_step = 1,
            .start_exposure = 3 * 400 * 1024,
            .cur_fps = 30 * 1024, // Now at 30 FPS
            .max_fps = 30 * 1024,
            .min_fps = 5 * 1024, // Allow flexibility
            .min_afps = 5 * 1024,
            .int_update_delay_frm = 1,
            .gain_update_delay_frm = 1,
        },

        .mipi_info = {
            .mipi_lane = 4, // Keeping it at 2 lanes
        },
        .preg_data = imx351_1280x720_setting,
        .reg_data_count = ARRAY_SIZE(imx351_1280x720_setting),
    },

    {
        .index = 2,
        .size = {
            .bounds_width = 2328,
            .bounds_height = 1744,
            .top = 0,
            .left = 0,
            .width = 2328,
            .height = 1744,
        },
        .hdr_mode = SENSOR_MODE_LINEAR,
        .bit_width = 10,
        .data_compress = {
            .enable = 0,
        },
        .bayer_pattern = BAYER_GBRG,
        .ae_info = {
            .def_frm_len_lines = 0x35E0, // Increased for 30 FPS
            .curr_frm_len_lines = 0x35E0,
            .one_line_exp_time_ns = 0x1790, // Keeping same line time

            .max_integration_line = 0x35DC, // Frame length - 4
            .min_integration_line = 1,

            .max_again = 0x03D2,
            .min_again = 0,
            .max_dgain = 0x0FFF,
            .min_dgain = 0x0100,
            .gain_step = 1,
            .start_exposure = 3 * 400 * 1024,
            .cur_fps = 30 * 1024, // Now at 30 FPS
            .max_fps = 30 * 1024,
            .min_fps = 5 * 1024, // Allow flexibility
            .min_afps = 5 * 1024,
            .int_update_delay_frm = 1,
            .gain_update_delay_frm = 1,
        },

        .mipi_info = {
            .mipi_lane = 4, // Keeping it at 2 lanes
        },
        .preg_data = imx351_2328x1744_setting,
        .reg_data_count = ARRAY_SIZE(imx351_2328x1744_setting),
    },

    {
        .index = 3,
        .size = {
            .bounds_width = 3840,
            .bounds_height = 2160,
            .top = 0,
            .left = 0,
            .width = 3840,
            .height = 2160,
        },
        .hdr_mode = SENSOR_MODE_LINEAR,
        .bit_width = 10,
        .data_compress = {
            .enable = 0,
        },
        .bayer_pattern = BAYER_GBRG,
        .ae_info = {
            .def_frm_len_lines = 0x4650, // Adjusted for 30 FPS
            .curr_frm_len_lines = 0x4650,
            .one_line_exp_time_ns = 0x1A2C, // Line time based on pixel clock

            .max_integration_line = 0x464C, // Frame length - 4
            .min_integration_line = 1,

            .max_again = 0x03D2,
            .min_again = 0,
            .max_dgain = 0x0FFF,
            .min_dgain = 0x0100,
            .gain_step = 1,
            .start_exposure = 3 * 400 * 1024,
            .cur_fps = 30 * 1024, // 30 FPS
            .max_fps = 30 * 1024,
            .min_fps = 5 * 1024, // Allow flexibility
            .min_afps = 5 * 1024,
            .int_update_delay_frm = 1,
            .gain_update_delay_frm = 1,
        },

        .mipi_info = {
            .mipi_lane = 4, // Using 4 lanes for higher bandwidth
        },
        .preg_data = imx351_3840x2160_setting,
        .reg_data_count = ARRAY_SIZE(imx351_3840x2160_setting),
    },

};

static int imx351_get_clk(struct imx351 *sensor, void *clk)
{
    struct vvcam_clk_s vvcam_clk;
    int ret = 0;
    pr_info("enter %s\n", __func__);
    vvcam_clk.sensor_mclk = clk_get_rate(sensor->sensor_clk);
    vvcam_clk.csi_max_pixel_clk = sensor->ocp.max_pixel_frequency;
    ret = copy_to_user(clk, &vvcam_clk, sizeof(struct vvcam_clk_s));
    if (ret != 0)
        ret = -EINVAL;
    return ret;
}

static int imx351_power_on(struct imx351 *sensor)
{
    int ret;
    pr_info("enter %s\n", __func__);

    if (gpio_is_valid(sensor->pwn_gpio))
        gpio_set_value_cansleep(sensor->pwn_gpio, 1);

    ret = clk_prepare_enable(sensor->sensor_clk);
    if (ret < 0)
        pr_err("%s: enable sensor clk fail\n", __func__);

    return ret;
}

static int imx351_power_off(struct imx351 *sensor)
{
    pr_info("enter %s\n", __func__);
    if (gpio_is_valid(sensor->pwn_gpio))
        gpio_set_value_cansleep(sensor->pwn_gpio, 0);
    clk_disable_unprepare(sensor->sensor_clk);

    return 0;
}

static int imx351_s_power(struct v4l2_subdev *sd, int on)
{
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    struct imx351 *sensor = client_to_imx351(client);

    pr_info("enter %s\n", __func__);
    if (on)
        imx351_power_on(sensor);
    else
        imx351_power_off(sensor);

    return 0;
}

static int imx351_write_reg(struct imx351 *sensor, u16 reg, u8 val)
{
    struct device *dev = &sensor->i2c_client->dev;
    u8 au8Buf[3] = {0};
    pr_info("enter %s\n", __func__);

    au8Buf[0] = reg >> 8;
    au8Buf[1] = reg & 0xff;
    au8Buf[2] = val;

    if (i2c_master_send(sensor->i2c_client, au8Buf, 3) < 0)
    {
        dev_err(dev, "Write reg error: reg=%x, val=%x\n", reg, val);
        return -1;
    }
    

    return 0;
}

static int imx351_read_reg(struct imx351 *sensor, u16 reg, u8 *val)
{
    struct device *dev = &sensor->i2c_client->dev;
    u8 au8RegBuf[2] = {0};
    u8 u8RdVal = 0;
    pr_info("enter %s\n", __func__);

    au8RegBuf[0] = reg >> 8;
    au8RegBuf[1] = reg & 0xff;

    if (i2c_master_send(sensor->i2c_client, au8RegBuf, 2) != 2)
    {
        dev_err(dev, "Read reg error: reg=%x\n", reg);
        return -1;
    }

    if (i2c_master_recv(sensor->i2c_client, &u8RdVal, 1) != 1)
    {
        dev_err(dev, "Read reg error: reg=%x, val=%x\n", reg, u8RdVal);
        return -1;
    }

    *val = u8RdVal;

    return 0;
}

static int imx351_write_reg_arry(struct imx351 *sensor,
                                 struct vvcam_sccb_data_s *reg_arry,
                                 u32 size)
{
    int i = 0;
    int ret = 0;
    struct i2c_msg msg;
    u8 *send_buf;
    u32 send_buf_len = 0;
    struct i2c_client *i2c_client = sensor->i2c_client;
    pr_info("enter %s\n", __func__);

    send_buf = (u8 *)kmalloc(size + 2, GFP_KERNEL);
    if (!send_buf)
        return -ENOMEM;

    send_buf[send_buf_len++] = (reg_arry[0].addr >> 8) & 0xff;
    send_buf[send_buf_len++] = reg_arry[0].addr & 0xff;
    send_buf[send_buf_len++] = reg_arry[0].data & 0xff;
    for (i = 1; i < size; i++)
    {
        if (reg_arry[i].addr == (reg_arry[i - 1].addr + 1))
        {
            send_buf[send_buf_len++] = reg_arry[i].data & 0xff;
        }
        else
        {
            msg.addr = i2c_client->addr;
            msg.flags = i2c_client->flags;
            msg.buf = send_buf;
            msg.len = send_buf_len;
            ret = i2c_transfer(i2c_client->adapter, &msg, 1);
            if (ret < 0)
            {
                pr_err("%s:i2c transfer error\n", __func__);
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

    if (send_buf_len > 0)
    {
        msg.addr = i2c_client->addr;
        msg.flags = i2c_client->flags;
        msg.buf = send_buf;
        msg.len = send_buf_len;
        ret = i2c_transfer(i2c_client->adapter, &msg, 1);
        if (ret < 0)
            pr_err("%s:i2c transfer end meg error\n", __func__);
        else
            ret = 0;
    }
    kfree(send_buf);
    return ret;
}

static int imx351_query_capability(struct imx351 *sensor, void *arg)
{
    struct v4l2_capability *pcap = (struct v4l2_capability *)arg;

    pr_info("enter %s\n", __func__);
    strcpy((char *)pcap->driver, "imx351");
    sprintf((char *)pcap->bus_info, "csi%d", sensor->csi_id);
    if (sensor->i2c_client->adapter)
    {
        pcap->bus_info[VVCAM_CAP_BUS_INFO_I2C_ADAPTER_NR_POS] =
            (__u8)sensor->i2c_client->adapter->nr;
    }
    else
    {
        pcap->bus_info[VVCAM_CAP_BUS_INFO_I2C_ADAPTER_NR_POS] = 0xFF;
    }
    return 0;
}

static int imx351_query_supports(struct imx351 *sensor, void *parry)
{
    int ret = 0;
    struct vvcam_mode_info_array_s *psensor_mode_arry = parry;
    pr_info("enter %s\n", __func__);
    psensor_mode_arry->count = ARRAY_SIZE(pimx351_mode_info);
    ret = copy_to_user(&psensor_mode_arry->modes, pimx351_mode_info,
                       sizeof(pimx351_mode_info));
    if (ret != 0)
        ret = -ENOMEM;
    return ret;
}

static int imx351_get_sensor_id(struct imx351 *sensor, void *pchip_id)
{
    int ret = 0;
    u16 chip_id;
    u8 chip_id_high = 0;
    u8 chip_id_low = 0;
    pr_info("enter %s\n", __func__);
    ret = imx351_read_reg(sensor, 0x0016, &chip_id_high);
    ret |= imx351_read_reg(sensor, 0x0017, &chip_id_low);

    chip_id = ((chip_id_high & 0xff) << 8) | (chip_id_low & 0xff);

    ret = copy_to_user(pchip_id, &chip_id, sizeof(u16));
    if (ret != 0)
        ret = -ENOMEM;
    return ret;
}

static int imx351_get_reserve_id(struct imx351 *sensor, void *preserve_id)
{
    int ret = 0;
    u16 reserve_id = IMX351_RESERVE_ID;
    pr_info("enter %s\n", __func__);
    ret = copy_to_user(preserve_id, &reserve_id, sizeof(u16));
    if (ret != 0)
        ret = -ENOMEM;
    return ret;
}

static int imx351_get_sensor_mode(struct imx351 *sensor, void *pmode)
{
    int ret = 0;
    pr_info("enter %s\n", __func__);
    ret = copy_to_user(pmode, &sensor->cur_mode,
                       sizeof(struct vvcam_mode_info_s));
    if (ret != 0)
        ret = -ENOMEM;
    return ret;
}

static int imx351_set_sensor_mode(struct imx351 *sensor, void *pmode)
{
    int ret = 0;
    int i = 0;
    struct vvcam_mode_info_s sensor_mode;
    pr_info("enter %s\n", __func__);
    ret = copy_from_user(&sensor_mode, pmode,
                         sizeof(struct vvcam_mode_info_s));
    if (ret != 0)
        return -ENOMEM;
    for (i = 0; i < ARRAY_SIZE(pimx351_mode_info); i++)
    {
        if (pimx351_mode_info[i].index == sensor_mode.index)
        {
            memcpy(&sensor->cur_mode, &pimx351_mode_info[i],
                   sizeof(struct vvcam_mode_info_s));
            return 0;
        }
    }

    return -ENXIO;
}

static int imx351_set_lexp(struct imx351 *sensor, u32 exp)
{
    pr_info("enter %s\n", __func__);
    return 0;
}

static int imx351_set_exp(struct imx351 *sensor, u32 exp)
{
    int ret = 0;
    pr_info("enter %s\n", __func__);
    ret |= imx351_write_reg(sensor, 0x104, 0x01);

    ret |= imx351_write_reg(sensor, 0x202, (exp >> 8) & 0xff);
    ret |= imx351_write_reg(sensor, 0x203, exp & 0xff);

    ret |= imx351_write_reg(sensor, 0x104, 0x0);
    return ret;
}

static int imx351_set_vsexp(struct imx351 *sensor, u32 exp)
{
    pr_info("enter %s\n", __func__);
    return 0;
}

static int imx351_set_lgain(struct imx351 *sensor, u32 gain)
{
    pr_info("enter %s\n", __func__);
    return 0;
}

static int imx351_set_gain(struct imx351 *sensor, u32 gain)
{
    int ret = 0;
    //<Adjusted Gain formaula
    u32 again = 1024 - (1024 * 64) / gain;

    pr_info("enter %s\n", __func__);
    if (again > sensor->cur_mode.ae_info.max_again)
    {
        again = sensor->cur_mode.ae_info.max_again;
    }
    else if (again < sensor->cur_mode.ae_info.min_again)
    {
        again = sensor->cur_mode.ae_info.min_again;
    }

    /* Only set the analog gain for now.
     * Expect to use individual color digital gain for tuning */
    ret |= imx351_write_reg(sensor, 0x104, 0x01);

    ret |= imx351_write_reg(sensor, 0x204, (again >> 8) & 0xff);
    ret |= imx351_write_reg(sensor, 0x205, again & 0xff);

    ret |= imx351_write_reg(sensor, 0x104, 0x0);

    return ret;
}

static int imx351_set_vsgain(struct imx351 *sensor, u32 gain)
{
    pr_info("enter %s\n", __func__);
    return 0;
}


static int imx351_set_fps(struct imx351 *sensor, u32 fps)
{
    u32 vts;
    int ret = 0;
    pr_info("enter %s\n", __func__);

    if (fps > sensor->cur_mode.ae_info.max_fps)
        fps = sensor->cur_mode.ae_info.max_fps;
    else if (fps < sensor->cur_mode.ae_info.min_fps)
        fps = sensor->cur_mode.ae_info.min_fps;

    vts = sensor->cur_mode.ae_info.max_fps *
          sensor->cur_mode.ae_info.def_frm_len_lines / fps;

    ret |= imx351_write_reg(sensor, 0x0104, 0x01);

    ret |= imx351_write_reg(sensor, 0x0340, 0x04);

    ret |= imx351_write_reg(sensor, 0x0341, 0xB8);
    ret |= imx351_write_reg(sensor, 0x0342, 0x0B);
    ret |= imx351_write_reg(sensor, 0x0343, 0x50);

    ret |= imx351_write_reg(sensor, 0x0104, 0x00);
   

    sensor->cur_mode.ae_info.cur_fps = fps;

    if (sensor->cur_mode.hdr_mode == SENSOR_MODE_LINEAR) {
        sensor->cur_mode.ae_info.max_integration_line = vts - 4;
    } else {
        if (sensor->cur_mode.stitching_mode == SENSOR_STITCHING_DUAL_DCG) {
            sensor->cur_mode.ae_info.max_vsintegration_line = 44;
            sensor->cur_mode.ae_info.max_integration_line = vts - 4 - 44;
        } else {
            sensor->cur_mode.ae_info.max_integration_line = vts - 4;
        }
    }
    sensor->cur_mode.ae_info.curr_frm_len_lines = vts;
    return ret;
}


static int imx351_get_fps(struct imx351 *sensor, u32 *pfps)
{
    pr_info("enter %s\n", __func__);
    *pfps = sensor->cur_mode.ae_info.cur_fps;
    return 0;
}

static int imx351_set_test_pattern(struct imx351 *sensor, void *arg)
{
    int ret;
    struct sensor_test_pattern_s test_pattern;
    pr_info("enter %s\n", __func__);

    ret = copy_from_user(&test_pattern, arg, sizeof(test_pattern));
    if (ret != 0)
        return -ENOMEM;

    if (test_pattern.enable)
    {
        switch (test_pattern.pattern)
        {
        case 0:
            pr_info("Test Set Pattern Mode Case 1 used \n");
            ret |= imx351_write_reg(sensor, 0x104, 0x01);
            ret = imx351_write_reg(sensor, 0x0600, 0x0);
            ret |= imx351_write_reg(sensor, 0x0601, 0x1);
            ret |= imx351_write_reg(sensor, 0x0624, 0x06);
            ret |= imx351_write_reg(sensor, 0x0625, 0x68);
            ret |= imx351_write_reg(sensor, 0x0626, 0x04);
            ret |= imx351_write_reg(sensor, 0x0627, 0xD0);

            ret |= imx351_write_reg(sensor, 0x613C, 0x06);
            ret |= imx351_write_reg(sensor, 0x613D, 0x68);
            ret |= imx351_write_reg(sensor, 0x613E, 0x04);
            ret |= imx351_write_reg(sensor, 0x613F, 0xD0);

            ret |= imx351_write_reg(sensor, 0x104, 0x00);
            break;
        case 1:
            ret |= imx351_write_reg(sensor, 0x104, 0x01);
            ret = imx351_write_reg(sensor, 0x0600, 0x0);
            ret |= imx351_write_reg(sensor, 0x0601, 0x2);
            ret |= imx351_write_reg(sensor, 0x0624, 0x0C);
            ret |= imx351_write_reg(sensor, 0x0625, 0xD0);
            ret |= imx351_write_reg(sensor, 0x0626, 0x09);
            ret |= imx351_write_reg(sensor, 0x0627, 0xA0);

            ret |= imx351_write_reg(sensor, 0x613C, 0x0C);
            ret |= imx351_write_reg(sensor, 0x613D, 0xD0);
            ret |= imx351_write_reg(sensor, 0x613E, 0x09);
            ret |= imx351_write_reg(sensor, 0x613F, 0xA0);
            ret |= imx351_write_reg(sensor, 0x104, 0x00);
            break;
        case 2:
            ret |= imx351_write_reg(sensor, 0x104, 0x01);
            ret = imx351_write_reg(sensor, 0x0600, 0x0);
            ret |= imx351_write_reg(sensor, 0x0601, 0x3);
            ret |= imx351_write_reg(sensor, 0x0624, 0x06);
            ret |= imx351_write_reg(sensor, 0x0625, 0x68);
            ret |= imx351_write_reg(sensor, 0x0626, 0x04);
            ret |= imx351_write_reg(sensor, 0x0627, 0xD0);

            ret |= imx351_write_reg(sensor, 0x613C, 0x06);
            ret |= imx351_write_reg(sensor, 0x613D, 0x68);
            ret |= imx351_write_reg(sensor, 0x613E, 0x04);
            ret |= imx351_write_reg(sensor, 0x613F, 0xD0);
            ret |= imx351_write_reg(sensor, 0x104, 0x00);
            break;
        case 3:
            ret |= imx351_write_reg(sensor, 0x104, 0x01);
            ret = imx351_write_reg(sensor, 0x0600, 0x0);
            ret |= imx351_write_reg(sensor, 0x0601, 0x4);
            ret |= imx351_write_reg(sensor, 0x0624, 0x0C);
            ret |= imx351_write_reg(sensor, 0x0625, 0xD0);
            ret |= imx351_write_reg(sensor, 0x0626, 0x09);
            ret |= imx351_write_reg(sensor, 0x0627, 0xA0);

            ret |= imx351_write_reg(sensor, 0x613C, 0x0C);
            ret |= imx351_write_reg(sensor, 0x613D, 0xD0);
            ret |= imx351_write_reg(sensor, 0x613E, 0x09);
            ret |= imx351_write_reg(sensor, 0x613F, 0xA0);
            ret |= imx351_write_reg(sensor, 0x104, 0x00);
            break;
        default:
            ret = -1;
            break;
        }
    }
    else
    {
        ret |= imx351_write_reg(sensor, 0x104, 0x01);
        ret = imx351_write_reg(sensor, 0x0600, 0x0);
        ret |= imx351_write_reg(sensor, 0x0601, 0x0);
        ret |= imx351_write_reg(sensor, 0x0624, 0x06);
        ret |= imx351_write_reg(sensor, 0x0625, 0x68);
        ret |= imx351_write_reg(sensor, 0x0626, 0x04);
        ret |= imx351_write_reg(sensor, 0x0627, 0xD0);

        ret |= imx351_write_reg(sensor, 0x613C, 0x06);
        ret |= imx351_write_reg(sensor, 0x613D, 0x68);
        ret |= imx351_write_reg(sensor, 0x613E, 0x04);
        ret |= imx351_write_reg(sensor, 0x613F, 0xD0);
        ret |= imx351_write_reg(sensor, 0x104, 0x00);
    }
    return 0;
}

static int imx351_set_ratio(struct imx351 *sensor, void *pratio)
{
    int ret = 0;
    struct sensor_hdr_artio_s hdr_ratio;
    struct vvcam_ae_info_s *pae_info = &sensor->cur_mode.ae_info;
    pr_info("enter %s\n", __func__);

    ret = copy_from_user(&hdr_ratio, pratio, sizeof(hdr_ratio));

    if ((hdr_ratio.ratio_l_s != pae_info->hdr_ratio.ratio_l_s) ||
        (hdr_ratio.ratio_s_vs != pae_info->hdr_ratio.ratio_s_vs) ||
        (hdr_ratio.accuracy != pae_info->hdr_ratio.accuracy))
    {
        pae_info->hdr_ratio.ratio_l_s = hdr_ratio.ratio_l_s;
        pae_info->hdr_ratio.ratio_s_vs = hdr_ratio.ratio_s_vs;
        pae_info->hdr_ratio.accuracy = hdr_ratio.accuracy;
        /*imx351 vs exp is limited for isp,so no need update max exp*/
    }

    return 0;
}

static int imx351_set_blc(struct imx351 *sensor, sensor_blc_t *pblc)
{
    pr_info("enter %s\n", __func__);
    /*
        int ret = 0;
        u32 r_offset, gr_offset, gb_offset, b_offset;
        u32 r_gain, gr_gain, gb_gain, b_gain;

        r_gain  = sensor->wb.r_gain;
        gr_gain = sensor->wb.gr_gain;
        gb_gain = sensor->wb.gb_gain;
        b_gain  = sensor->wb.b_gain;

        if (r_gain < 0x100)
            r_gain = 0x100;
        if (gr_gain < 0x100)
            gr_gain = 0x100;
        if (gb_gain < 0x100)
            gb_gain = 0x100;
        if (b_gain < 0x100)
            b_gain = 0x100;

        r_offset  = (r_gain  - 0x100) * pblc->red;
        gr_offset = (gr_gain - 0x100) * pblc->gr;
        gb_offset = (gb_gain - 0X100) * pblc->gb;
        b_offset  = (b_gain  - 0X100) * pblc->blue;
        //R,Gr,Gb,B HCG Offset
        ret |= imx351_write_reg(sensor, 0x3378, (r_offset >> 16) & 0xff);
        ret |= imx351_write_reg(sensor, 0x3379, (r_offset >> 8)  & 0xff);
        ret |= imx351_write_reg(sensor, 0x337a,  r_offset        & 0xff);

        ret |= imx351_write_reg(sensor, 0x337b, (gr_offset >> 16) & 0xff);
        ret |= imx351_write_reg(sensor, 0x337c, (gr_offset >> 8)  & 0xff);
        ret |= imx351_write_reg(sensor, 0x337d,  gr_offset        & 0xff);

        ret |= imx351_write_reg(sensor, 0x337e, (gb_offset >> 16) & 0xff);
        ret |= imx351_write_reg(sensor, 0x337f, (gb_offset >> 8)  & 0xff);
        ret |= imx351_write_reg(sensor, 0x3380,  gb_offset        & 0xff);

        ret |= imx351_write_reg(sensor, 0x3381, (b_offset >> 16) & 0xff);
        ret |= imx351_write_reg(sensor, 0x3382, (b_offset >> 8)  & 0xff);
        ret |= imx351_write_reg(sensor, 0x3383,  b_offset        & 0xff);

        //R,Gr,Gb,B LCG Offset
        ret |= imx351_write_reg(sensor, 0x3384, (r_offset >> 16) & 0xff);
        ret |= imx351_write_reg(sensor, 0x3385, (r_offset >> 8)  & 0xff);
        ret |= imx351_write_reg(sensor, 0x3386,  r_offset        & 0xff);

        ret |= imx351_write_reg(sensor, 0x3387, (gr_offset >> 16) & 0xff);
        ret |= imx351_write_reg(sensor, 0x3388, (gr_offset >> 8)  & 0xff);
        ret |= imx351_write_reg(sensor, 0x3389,  gr_offset        & 0xff);

        ret |= imx351_write_reg(sensor, 0x338a, (gb_offset >> 16) & 0xff);
        ret |= imx351_write_reg(sensor, 0x338b, (gb_offset >> 8)  & 0xff);
        ret |= imx351_write_reg(sensor, 0x338c,  gb_offset        & 0xff);

        ret |= imx351_write_reg(sensor, 0x338d, (b_offset >> 16) & 0xff);
        ret |= imx351_write_reg(sensor, 0x338e, (b_offset >> 8)  & 0xff);
        ret |= imx351_write_reg(sensor, 0x338f,  b_offset        & 0xff);

        //R,Gr,Gb,B VS Offset
        ret |= imx351_write_reg(sensor, 0x3390, (r_offset >> 16) & 0xff);
        ret |= imx351_write_reg(sensor, 0x3391, (r_offset >> 8)  & 0xff);
        ret |= imx351_write_reg(sensor, 0x3392,  r_offset        & 0xff);

        ret |= imx351_write_reg(sensor, 0x3393, (gr_offset >> 16) & 0xff);
        ret |= imx351_write_reg(sensor, 0x3394, (gr_offset >> 8)  & 0xff);
        ret |= imx351_write_reg(sensor, 0x3395,  gr_offset        & 0xff);

        ret |= imx351_write_reg(sensor, 0x3396, (gb_offset >> 16) & 0xff);
        ret |= imx351_write_reg(sensor, 0x3397, (gb_offset >> 8)  & 0xff);
        ret |= imx351_write_reg(sensor, 0x3398,  gb_offset        & 0xff);

        ret |= imx351_write_reg(sensor, 0x3399, (b_offset >> 16) & 0xff);
        ret |= imx351_write_reg(sensor, 0x339a, (b_offset >> 8)  & 0xff);
        ret |= imx351_write_reg(sensor, 0x339b,  b_offset        & 0xff);

        memcpy(&sensor->blc, pblc, sizeof(sensor_blc_t));
    */
    return 0;
}

static int imx351_set_wb(struct imx351 *sensor, void *pwb_cfg)
{
    pr_info("enter %s\n", __func__);
    /*
        int ret = 0;
        bool update_flag = false;
        struct sensor_white_balance_s wb;
        ret = copy_from_user(&wb, pwb_cfg, sizeof(wb));
        if (ret != 0)
            return -ENOMEM;
        if (wb.r_gain != sensor->wb.r_gain) {
            ret |= imx351_write_reg(sensor, 0x3360, (wb.r_gain >> 8) & 0xff);
            ret |= imx351_write_reg(sensor, 0x3361,  wb.r_gain & 0xff);
            ret |= imx351_write_reg(sensor, 0x3368, (wb.r_gain >> 8) & 0xff);
            ret |= imx351_write_reg(sensor, 0x3369,  wb.r_gain & 0xff);
            ret |= imx351_write_reg(sensor, 0x3370, (wb.r_gain >> 8) & 0xff);
            ret |= imx351_write_reg(sensor, 0x3371,  wb.r_gain & 0xff);
            update_flag = true;
        }

        if (wb.gr_gain != sensor->wb.gr_gain) {
            ret |= imx351_write_reg(sensor, 0x3362, (wb.gr_gain >> 8) & 0xff);
            ret |= imx351_write_reg(sensor, 0x3363,  wb.gr_gain & 0xff);
            ret |= imx351_write_reg(sensor, 0x336a, (wb.gr_gain >> 8) & 0xff);
            ret |= imx351_write_reg(sensor, 0x336b,  wb.gr_gain & 0xff);
            ret |= imx351_write_reg(sensor, 0x3372, (wb.gr_gain >> 8) & 0xff);
            ret |= imx351_write_reg(sensor, 0x3373,  wb.gr_gain & 0xff);
            update_flag = true;
        }

        if (wb.gb_gain != sensor->wb.gb_gain) {
            ret |= imx351_write_reg(sensor, 0x3364, (wb.gb_gain >> 8) & 0xff);
            ret |= imx351_write_reg(sensor, 0x3365,  wb.gb_gain & 0xff);
            ret |= imx351_write_reg(sensor, 0x336c, (wb.gb_gain >> 8) & 0xff);
            ret |= imx351_write_reg(sensor, 0x336d,  wb.gb_gain & 0xff);
            ret |= imx351_write_reg(sensor, 0x3374, (wb.gb_gain >> 8) & 0xff);
            ret |= imx351_write_reg(sensor, 0x3375,  wb.gb_gain & 0xff);
            update_flag = true;
        }

        if (wb.b_gain != sensor->wb.b_gain) {
            ret |= imx351_write_reg(sensor, 0x3366, (wb.b_gain >> 8) & 0xff);
            ret |= imx351_write_reg(sensor, 0x3367,  wb.b_gain & 0xff);
            ret |= imx351_write_reg(sensor, 0x336e, (wb.b_gain >> 8) & 0xff);
            ret |= imx351_write_reg(sensor, 0x336f,  wb.b_gain & 0xff);
            ret |= imx351_write_reg(sensor, 0x3376, (wb.b_gain >> 8) & 0xff);
            ret |= imx351_write_reg(sensor, 0x3377,  wb.b_gain & 0xff);
            update_flag = true;
        }
        if (ret != 0)
            return ret;

        memcpy (&sensor->wb, &wb, sizeof(struct sensor_white_balance_s));

        if (update_flag) {
            ret = imx351_set_blc(sensor, &sensor->blc);
        }
    */
    return 0;
}

static int imx351_get_expand_curve(struct imx351 *sensor,
                                   sensor_expand_curve_t *pexpand_curve)
{
    pr_info("enter %s\n", __func__);
    /* Not using data compression */
    return -1;
}

static int imx351_get_format_code(struct imx351 *sensor, u32 *code)
{
    pr_info("enter %s\n", __func__);
    switch (sensor->cur_mode.bayer_pattern)
    {
    case BAYER_RGGB:
        if (sensor->cur_mode.bit_width == 8)
        {
            *code = MEDIA_BUS_FMT_SRGGB8_1X8;
        }
        else if (sensor->cur_mode.bit_width == 10)
        {
            *code = MEDIA_BUS_FMT_SRGGB10_1X10;
        }
        else
        {
            *code = MEDIA_BUS_FMT_SRGGB12_1X12;
        }
        break;
    case BAYER_GRBG:
        if (sensor->cur_mode.bit_width == 8)
        {
            *code = MEDIA_BUS_FMT_SGRBG8_1X8;
        }
        else if (sensor->cur_mode.bit_width == 10)
        {
            *code = MEDIA_BUS_FMT_SGRBG10_1X10;
        }
        else
        {
            *code = MEDIA_BUS_FMT_SGRBG12_1X12;
        }
        break;
    case BAYER_GBRG:
        if (sensor->cur_mode.bit_width == 8)
        {
            *code = MEDIA_BUS_FMT_SGBRG8_1X8;
        }
        else if (sensor->cur_mode.bit_width == 10)
        {
            *code = MEDIA_BUS_FMT_SGBRG10_1X10;
        }
        else
        {
            *code = MEDIA_BUS_FMT_SGBRG12_1X12;
        }
        break;
    case BAYER_BGGR:
        if (sensor->cur_mode.bit_width == 8)
        {
            *code = MEDIA_BUS_FMT_SBGGR8_1X8;
        }
        else if (sensor->cur_mode.bit_width == 10)
        {
            *code = MEDIA_BUS_FMT_SBGGR10_1X10;
        }
        else
        {
            *code = MEDIA_BUS_FMT_SBGGR12_1X12;
        }
        break;
    default:
        /*nothing need to do*/
        break;
    }
    return 0;
}

static int imx351_s_stream(struct v4l2_subdev *sd, int enable)
{
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    struct imx351 *sensor = client_to_imx351(client);

    pr_info("enter %s\n", __func__);
    sensor->stream_status = enable;
    if (enable)
    {
        imx351_write_reg(sensor, 0x0100, 0x01);
    }
    else
    {
        imx351_write_reg(sensor, 0x0100, 0x00);
    }

    return 0;
}

static int imx351_enum_mbus_code(struct v4l2_subdev *sd,
                                 struct v4l2_subdev_state *state,
                                 struct v4l2_subdev_mbus_code_enum *code)
{
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    struct imx351 *sensor = client_to_imx351(client);

    u32 cur_code = MEDIA_BUS_FMT_SGBRG10_1X10 ;
    pr_info("enter %s\n", __func__);

    if (code->index > 0)
        return -EINVAL;
    imx351_get_format_code(sensor, &cur_code);
    pr_info("Current media bus code set: 0x%X\n", cur_code);
    code->code = cur_code;

    return 0;
}
static int imx351_set_fmt(struct v4l2_subdev *sd,
                          struct v4l2_subdev_state *state,
                          struct v4l2_subdev_format *fmt)

{
    int ret = 0;
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    struct imx351 *sensor = client_to_imx351(client);
    pr_info("enter %s\n", __func__);
    mutex_lock(&sensor->lock);

    pr_info("%s: Requested format: width=%d, height=%d\n", __func__,
        fmt->format.width, fmt->format.height);

    if ((fmt->format.width != sensor->cur_mode.size.width) ||
        (fmt->format.height != sensor->cur_mode.size.height))
    {
        pr_err("%s:set sensor format %dx%d error\n",
               __func__, fmt->format.width, fmt->format.height);
        mutex_unlock(&sensor->lock);
        return -EINVAL;
    }
   pr_info("%s: Using preg_data at address %p with count %d\n", __func__,
            sensor->cur_mode.preg_data, sensor->cur_mode.reg_data_count);

    ret = imx351_write_reg_arry(sensor,
                                (struct vvcam_sccb_data_s *)sensor->cur_mode.preg_data,
                                sensor->cur_mode.reg_data_count);
    if (ret < 0)
    {
        pr_err("%s:imx351_write_reg_arry error\n", __func__);
        mutex_unlock(&sensor->lock);
        return -EINVAL;
    }
   pr_info("%s: Format successfully set. Code: 0x%X\n", __func__, fmt->format.code);
    imx351_get_format_code(sensor, &fmt->format.code);
    fmt->format.field = V4L2_FIELD_NONE;
    sensor->format = fmt->format;
    mutex_unlock(&sensor->lock);
    return 0;
}

static int imx351_get_fmt(struct v4l2_subdev *sd,
                          struct v4l2_subdev_state *state,
                          struct v4l2_subdev_format *fmt)

{
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    struct imx351 *sensor = client_to_imx351(client);

    pr_info("enter %s\n", __func__);
    mutex_lock(&sensor->lock);
    fmt->format = sensor->format;
    mutex_unlock(&sensor->lock);
    return 0;
}

static long imx351_priv_ioctl(struct v4l2_subdev *sd,
                              unsigned int cmd,
                              void *arg)
{
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    struct imx351 *sensor = client_to_imx351(client);
    long ret = 0;
    struct vvcam_sccb_data_s sensor_reg;
    pr_info("enter %s (cmd = %d)\n", __func__, cmd);

    mutex_lock(&sensor->lock);
    switch (cmd)
    {
    case VVSENSORIOC_S_POWER:
        ret = 0;
        break;
    case VVSENSORIOC_S_CLK:
        ret = 0;
        break;
    case VVSENSORIOC_G_CLK:
        ret = imx351_get_clk(sensor, arg);
        break;
    case VVSENSORIOC_RESET:
        ret = 0;
        break;
    case VIDIOC_QUERYCAP:
        ret = imx351_query_capability(sensor, arg);
        break;
    case VVSENSORIOC_QUERY:
        ret = imx351_query_supports(sensor, arg);
        break;
    case VVSENSORIOC_G_CHIP_ID:
        ret = imx351_get_sensor_id(sensor, arg);
        break;
    case VVSENSORIOC_G_RESERVE_ID:
        ret = imx351_get_reserve_id(sensor, arg);
        break;
    case VVSENSORIOC_G_SENSOR_MODE:
        ret = imx351_get_sensor_mode(sensor, arg);
        break;
    case VVSENSORIOC_S_SENSOR_MODE:
        ret = imx351_set_sensor_mode(sensor, arg);
        break;
    case VVSENSORIOC_S_STREAM:
        ret = imx351_s_stream(&sensor->subdev, *(int *)arg);
        break;
    case VVSENSORIOC_WRITE_REG:
        ret = copy_from_user(&sensor_reg, arg,
                             sizeof(struct vvcam_sccb_data_s));
        ret |= imx351_write_reg(sensor, sensor_reg.addr,
                                sensor_reg.data);
        break;
    case VVSENSORIOC_READ_REG:
        ret = copy_from_user(&sensor_reg, arg,
                             sizeof(struct vvcam_sccb_data_s));
        ret |= imx351_read_reg(sensor, sensor_reg.addr,
                               (u8 *)&sensor_reg.data);
        ret |= copy_to_user(arg, &sensor_reg,
                            sizeof(struct vvcam_sccb_data_s));
        break;
    case VVSENSORIOC_S_LONG_EXP:
        ret = imx351_set_lexp(sensor, *(u32 *)arg);
        break;
    case VVSENSORIOC_S_EXP:
        ret = imx351_set_exp(sensor, *(u32 *)arg);
        break;
    case VVSENSORIOC_S_VSEXP:
        ret = imx351_set_vsexp(sensor, *(u32 *)arg);
        break;
    case VVSENSORIOC_S_LONG_GAIN:
        ret = imx351_set_lgain(sensor, *(u32 *)arg);
        break;
    case VVSENSORIOC_S_GAIN:
        ret = imx351_set_gain(sensor, *(u32 *)arg);
        break;
    case VVSENSORIOC_S_VSGAIN:
        ret = imx351_set_vsgain(sensor, *(u32 *)arg);
        break;
    case VVSENSORIOC_S_FPS:
        ret = imx351_set_fps(sensor, *(u32 *)arg);
        break;
    case VVSENSORIOC_G_FPS:
        ret = imx351_get_fps(sensor, (u32 *)arg);
        break;
    case VVSENSORIOC_S_HDR_RADIO:
        ret = imx351_set_ratio(sensor, arg);
        break;
    case VVSENSORIOC_S_BLC:
        ret = imx351_set_blc(sensor, arg);
        break;
    case VVSENSORIOC_S_WB:
        ret = imx351_set_wb(sensor, arg);
        break;
    case VVSENSORIOC_G_EXPAND_CURVE:
        ret = imx351_get_expand_curve(sensor, arg);
        break;
    case VVSENSORIOC_S_TEST_PATTERN:
        ret = imx351_set_test_pattern(sensor, arg);
        break;
    default:
        break;
    }

    mutex_unlock(&sensor->lock);
    return ret;
}

static struct v4l2_subdev_video_ops imx351_subdev_video_ops = {
    .s_stream = imx351_s_stream,
};

static const struct v4l2_subdev_pad_ops imx351_subdev_pad_ops = {
    .enum_mbus_code = imx351_enum_mbus_code,
    .set_fmt = imx351_set_fmt,
    .get_fmt = imx351_get_fmt,
};

static struct v4l2_subdev_core_ops imx351_subdev_core_ops = {
    .s_power = imx351_s_power,
    .ioctl = imx351_priv_ioctl,
};

static struct v4l2_subdev_ops imx351_subdev_ops = {
    .core = &imx351_subdev_core_ops,
    .video = &imx351_subdev_video_ops,
    .pad = &imx351_subdev_pad_ops,
};

static int imx351_link_setup(struct media_entity *entity,
                             const struct media_pad *local,
                             const struct media_pad *remote, u32 flags)
{
    pr_info("enter %s\n", __func__);
    return 0;
}

static const struct media_entity_operations imx351_sd_media_ops = {
    .link_setup = imx351_link_setup,
};

static int imx351_regulator_enable(struct imx351 *sensor)
{
    int ret = 0;
    struct device *dev = &(sensor->i2c_client->dev);

    pr_info("enter %s\n", __func__);

    if (sensor->io_regulator)
    {
        regulator_set_voltage(sensor->io_regulator,
                              IMX351_VOLTAGE_DIGITAL_IO,
                              IMX351_VOLTAGE_DIGITAL_IO);
        ret = regulator_enable(sensor->io_regulator);
        if (ret < 0)
        {
            dev_err(dev, "set io voltage failed\n");
            return ret;
        }
    }

    if (sensor->analog_regulator)
    {
        regulator_set_voltage(sensor->analog_regulator,
                              IMX351_VOLTAGE_ANALOG,
                              IMX351_VOLTAGE_ANALOG);
        ret = regulator_enable(sensor->analog_regulator);
        if (ret)
        {
            dev_err(dev, "set analog voltage failed\n");
            goto err_disable_io;
        }
    }

    if (sensor->core_regulator)
    {
        regulator_set_voltage(sensor->core_regulator,
                              IMX351_VOLTAGE_DIGITAL_CORE,
                              IMX351_VOLTAGE_DIGITAL_CORE);
        ret = regulator_enable(sensor->core_regulator);
        if (ret)
        {
            dev_err(dev, "set core voltage failed\n");
            goto err_disable_analog;
        }
    }

    return 0;

err_disable_analog:
    regulator_disable(sensor->analog_regulator);
err_disable_io:
    regulator_disable(sensor->io_regulator);
    return ret;
}

static void imx351_regulator_disable(struct imx351 *sensor)
{
    int ret = 0;
    struct device *dev = &(sensor->i2c_client->dev);
    pr_info("enter %s\n", __func__);

    if (sensor->core_regulator)
    {
        ret = regulator_disable(sensor->core_regulator);
        if (ret < 0)
            dev_err(dev, "core regulator disable failed\n");
    }

    if (sensor->analog_regulator)
    {
        ret = regulator_disable(sensor->analog_regulator);
        if (ret < 0)
            dev_err(dev, "analog regulator disable failed\n");
    }

    if (sensor->io_regulator)
    {
        ret = regulator_disable(sensor->io_regulator);
        if (ret < 0)
            dev_err(dev, "io regulator disable failed\n");
    }
    return;
}

static int imx351_set_clk_rate(struct imx351 *sensor)
{
    int ret;
    unsigned int clk;

    clk = sensor->mclk;
    clk = min_t(u32, clk, (u32)IMX351_XCLK_MAX);
    clk = max_t(u32, clk, (u32)IMX351_XCLK_MIN);
    sensor->mclk = clk;
    pr_info("enter %s\n", __func__);

    pr_debug("   Setting mclk to %d MHz\n", sensor->mclk / 1000000);
    ret = clk_set_rate(sensor->sensor_clk, sensor->mclk);
    if (ret < 0)
        pr_debug("set rate filed, rate=%d\n", sensor->mclk);
    return ret;
}

static void imx351_reset(struct imx351 *sensor)
{
    pr_info("enter %s\n", __func__);
    if (!gpio_is_valid(sensor->rst_gpio))
        return;

    gpio_set_value_cansleep(sensor->rst_gpio, 0);
    msleep(20);

    gpio_set_value_cansleep(sensor->rst_gpio, 1);
    msleep(20);

    return;
}

static int imx351_retrieve_capture_properties(
    struct imx351 *sensor,
    struct imx351_capture_properties *ocp)
{
    struct device *dev = &sensor->i2c_client->dev;
    __u64 mlf = 0;
    __u64 mpf = 0;
    __u64 mdr = 0;

    struct device_node *ep;
    int ret;
    /*Collecting the information about limits of capture path
     * has been centralized to the sensor
     * * also into the sensor endpoint itself.
     */

    pr_info("enter %s\n", __func__);

    ep = of_graph_get_next_endpoint(dev->of_node, NULL);
    if (!ep)
    {
        dev_err(dev, "missing endpoint node\n");
        return -ENODEV;
    }

    /*ret = fwnode_property_read_u64(of_fwnode_handle(ep),
        "max-lane-frequency", &mlf);
    if (ret || mlf == 0) {
        dev_dbg(dev, "no limit for max-lane-frequency\n");
    }*/
    ret = fwnode_property_read_u64(of_fwnode_handle(ep),
                                   "max-pixel-frequency", &mpf);
    if (ret || mpf == 0)
    {
        dev_dbg(dev, "no limit for max-pixel-frequency\n");
    }

    /*ret = fwnode_property_read_u64(of_fwnode_handle(ep),
            "max-data-rate", &mdr);
    if (ret || mdr == 0) {
            dev_dbg(dev, "no limit for max-data_rate\n");
    }*/

    ocp->max_lane_frequency = mlf;
    ocp->max_pixel_frequency = mpf;
    ocp->max_data_rate = mdr;

    return ret;
}

static int imx351_probe(struct i2c_client *client)
{
    int retval;
    struct device *dev = &client->dev;
    struct v4l2_subdev *sd;
    struct imx351 *sensor;
    u32 chip_id = 0;
    u8 reg_val = 0;

    pr_info("enter %s\n", __func__);

    sensor = devm_kmalloc(dev, sizeof(*sensor), GFP_KERNEL);
    if (!sensor)
        return -ENOMEM;
    memset(sensor, 0, sizeof(*sensor));

    sensor->i2c_client = client;

    sensor->pwn_gpio = of_get_named_gpio(dev->of_node, "pwn-gpios", 0);
    if (!gpio_is_valid(sensor->pwn_gpio))
        dev_warn(dev, "No sensor pwdn pin available");
    else
    {
        retval = devm_gpio_request_one(dev, sensor->pwn_gpio,
                                       GPIOF_OUT_INIT_HIGH,
                                       "imx351_mipi_pwdn");
        if (retval < 0)
        {
            dev_warn(dev, "Failed to set power pin\n");
            dev_warn(dev, "retval=%d\n", retval);
            return retval;
        }
    }

    sensor->rst_gpio = of_get_named_gpio(dev->of_node, "rst-gpios", 0);
    if (!gpio_is_valid(sensor->rst_gpio))
        dev_warn(dev, "No sensor reset pin available");
    else
    {
        retval = devm_gpio_request_one(dev, sensor->rst_gpio,
                                       GPIOF_OUT_INIT_HIGH,
                                       "imx351_mipi_reset");
        if (retval < 0)
        {
            dev_warn(dev, "Failed to set reset pin\n");
            return retval;
        }
    }

    sensor->sensor_clk = devm_clk_get(dev, "csi_mclk");
    if (IS_ERR(sensor->sensor_clk))
    {
        sensor->sensor_clk = NULL;
        dev_err(dev, "clock-frequency missing or invalid\n");
        return PTR_ERR(sensor->sensor_clk);
    }

    retval = of_property_read_u32(dev->of_node, "mclk", &(sensor->mclk));
    if (retval)
    {
        dev_err(dev, "mclk missing or invalid\n");
        return retval;
    }

    retval = of_property_read_u32(dev->of_node, "mclk_source",
                                  (u32 *)&(sensor->mclk_source));
    if (retval)
    {
        dev_err(dev, "mclk_source missing or invalid\n");
        return retval;
    }

    retval = of_property_read_u32(dev->of_node, "csi_id", &(sensor->csi_id));
    if (retval)
    {
        dev_err(dev, "csi id missing or invalid\n");
        return retval;
    }

    retval = imx351_retrieve_capture_properties(sensor, &sensor->ocp);
    if (retval)
    {
        dev_warn(dev, "retrive capture properties error\n");
    }

    sensor->io_regulator = devm_regulator_get(dev, "DOVDD");
    if (IS_ERR(sensor->io_regulator))
    {
        dev_err(dev, "cannot get io regulator\n");
        return PTR_ERR(sensor->io_regulator);
    }

    sensor->core_regulator = devm_regulator_get(dev, "DVDD");
    if (IS_ERR(sensor->core_regulator))
    {
        dev_err(dev, "cannot get core regulator\n");
        return PTR_ERR(sensor->core_regulator);
    }

    sensor->analog_regulator = devm_regulator_get(dev, "AVDD");
    if (IS_ERR(sensor->analog_regulator))
    {
        dev_err(dev, "cannot get analog  regulator\n");
        return PTR_ERR(sensor->analog_regulator);
    }

    retval = imx351_regulator_enable(sensor);
    if (retval)
    {
        dev_err(dev, "regulator enable failed\n");
        dev_err(dev, "regulator enable failed however for custom not an issue\n");
        //  return retval;
    }

    imx351_set_clk_rate(sensor);
    retval = clk_prepare_enable(sensor->sensor_clk);
    if (retval < 0)
    {
        dev_err(dev, "%s: enable sensor clk fail\n", __func__);
        goto probe_err_regulator_disable;
    }

    retval = imx351_power_on(sensor);
    if (retval < 0)
    {
        dev_err(dev, "%s: sensor power on fail\n", __func__);
        goto probe_err_regulator_disable;
    }

    imx351_reset(sensor);

    imx351_read_reg(sensor, 0x0016, &reg_val);
    chip_id |= reg_val << 8;
    imx351_read_reg(sensor, 0x0017, &reg_val);
    chip_id |= reg_val;
    if (chip_id != IMX351_RESERVE_ID)
    {
        dev_err(dev, "camera imx351 is not found (expected: %d  actual: %d)\n", IMX351_RESERVE_ID, chip_id);
        retval = -ENODEV;
        goto probe_err_power_off;
    }
    msleep(20);

    int ret = imx351_write_reg_arry(sensor, imx351_init_setting, ARRAY_SIZE(imx351_init_setting));
    if (ret < 0)
    {
        pr_err("Failed to initialize imx351 sensor\n");
    }
    else
    {
        pr_info("IMX351 sensor initialized successfully\n");
    }

    msleep(20);

    sd = &sensor->subdev;
    v4l2_i2c_subdev_init(sd, client, &imx351_subdev_ops);
    sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
    sd->dev = &client->dev;
    sd->entity.ops = &imx351_sd_media_ops;
    sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
    sensor->pads[IMX351_SENS_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
    retval = media_entity_pads_init(&sd->entity,
                                    IMX351_SENS_PADS_NUM,
                                    sensor->pads);
    if (retval < 0)
        goto probe_err_power_off;
    retval = v4l2_async_register_subdev_sensor(sd);
    if (retval < 0)
    {
        dev_err(&client->dev, "%s--Async register failed, ret=%d\n",
                __func__, retval);
        goto probe_err_free_entiny;
    }

    memcpy(&sensor->cur_mode, &pimx351_mode_info[0],
           sizeof(struct vvcam_mode_info_s));
    mutex_init(&sensor->lock);
    pr_info("%s camera mipi imx351, is found\n", __func__);

    return 0;

probe_err_free_entiny:
    media_entity_cleanup(&sd->entity);

probe_err_power_off:
    imx351_power_off(sensor);

probe_err_regulator_disable:
    imx351_regulator_disable(sensor);

    return retval;
}

static void imx351_remove(struct i2c_client *client)
{
    struct v4l2_subdev *sd = i2c_get_clientdata(client);
    struct imx351 *sensor = client_to_imx351(client);

    pr_info("enter %s\n", __func__);

    v4l2_async_unregister_subdev(sd);
    media_entity_cleanup(&sd->entity);
    imx351_power_off(sensor);
    imx351_regulator_disable(sensor);

    mutex_destroy(&sensor->lock);
}

static int __maybe_unused imx351_suspend(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct imx351 *sensor = client_to_imx351(client);

    sensor->resume_status = sensor->stream_status;
    if (sensor->resume_status)
    {
        imx351_s_stream(&sensor->subdev, 0);
    }

    return 0;
}

static int __maybe_unused imx351_resume(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct imx351 *sensor = client_to_imx351(client);

    if (sensor->resume_status)
    {
        imx351_s_stream(&sensor->subdev, 1);
    }

    return 0;
}

static const struct dev_pm_ops imx351_pm_ops = {
    SET_SYSTEM_SLEEP_PM_OPS(imx351_suspend, imx351_resume)};

static const struct i2c_device_id imx351_id[] = {
    {"imx351", 0},
    {},
};
MODULE_DEVICE_TABLE(i2c, imx351_id);

static const struct of_device_id imx351_of_match[] = {
    {.compatible = "sony,imx351"},
    {/* sentinel */}};
MODULE_DEVICE_TABLE(of, imx351_of_match);

static struct i2c_driver imx351_i2c_driver = {
    .driver = {
        .owner = THIS_MODULE,
        .name = "imx351",
        .pm = &imx351_pm_ops,
        .of_match_table = imx351_of_match,
    },
    .probe = imx351_probe,
    .remove = imx351_remove,
    .id_table = imx351_id,
};

module_i2c_driver(imx351_i2c_driver);
MODULE_DESCRIPTION("imx351 MIPI Camera Subdev Driver");
MODULE_LICENSE("GPL");
