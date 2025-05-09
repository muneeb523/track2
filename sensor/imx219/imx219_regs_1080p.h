/****************************************************************************
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 VeriSilicon Holdings Co., Ltd.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 *****************************************************************************
 *
 * The GPL License (GPL)
 *
 * Copyright (c) 2020 VeriSilicon Holdings Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program;
 *
 *****************************************************************************
 *
 * Note: This software is released under dual MIT and GPL licenses. A
 * recipient may use this file under the terms of either the MIT license or
 * GPL License. If you wish to use only one license not the other, you can
 * indicate your decision by deleting one of the above license notices in your
 * version of this file.
 *
 *****************************************************************************/

 #ifndef _VVCAM_IMX219_REGS_1080P_H_
 #define _VVCAM_IMX219_REGS_1080P_H_
 
 #include "vvsensor.h"
 
 /* 1080P RAW10 */
 static struct vvcam_sccb_data_s imx219_init_setting_1080p[] = {
     {0x0100, 0x00},
     {0x30eb, 0x05},
     {0x30eb, 0x0c},
     {0x300a, 0xff},
     {0x300b, 0xff},
     {0x30eb, 0x05},
     {0x30eb, 0x09},
     {0x0114, 0x01},
     {0x0128, 0x00},
     {0x012a, 0x18},
     {0x012b, 0x00},
     // vts
     {0x0160, 0x06},
     {0x0161, 0xe4},
 
     {0x0162, 0x0d},
     {0x0163, 0x78},
     {0x0164, 0x02},
     {0x0165, 0xa8},
     {0x0166, 0x0a},
     {0x0167, 0x27},
     {0x0168, 0x02},
     {0x0169, 0xb4},
     {0x016a, 0x06},
     {0x016b, 0xeb},
     {0x016c, 0x07},
     {0x016d, 0x80},
     {0x016e, 0x04},
     {0x016f, 0x38},
     {0x0170, 0x01},
     {0x0171, 0x01},
     // mirror
     //{0x0172, 0x01},
 
     {0x0174, 0x00},
     {0x0175, 0x00},
     {0x0301, 0x05},
     {0x0303, 0x01},
     {0x0304, 0x03},
     {0x0305, 0x03},
     {0x0306, 0x00},
     {0x0307, 0x39},
     {0x030b, 0x01},
     {0x030c, 0x00},
     {0x030d, 0x72},
     {0x0624, 0x07},
     {0x0625, 0x80},
     {0x0626, 0x04},
     {0x0627, 0x38},
     {0x455e, 0x00},
     {0x471e, 0x4b},
     {0x4767, 0x0f},
     {0x4750, 0x14},
     {0x4540, 0x00},
     {0x47b4, 0x14},
     {0x4713, 0x30},
     {0x478b, 0x10},
     {0x478f, 0x10},
     {0x4793, 0x10},
     {0x4797, 0x0e},
     {0x479b, 0x0e},
 
     // vts
     {0x0160, 0x06},
     {0x0161, 0xe4},
 
     {0x0162, 0x0d},
     {0x0163, 0x78},
 
 
      {0xFFFF, 0x00}
 
 
 };
 
 #endif
 