/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2008
*
*  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
*  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
*  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
*  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
*  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
*  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
*  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
*  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
*  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
*  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
*
*  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
*  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
*  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
*  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
*  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
*  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
*  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
*  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
*  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
*  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
*
*****************************************************************************/


#ifdef BUILD_LK
#include <platform/mt_gpio.h>
#include <platform/mt_pmic.h>

#define Lcd_Log printf

#else
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
    #include <linux/string.h>
   
#define Lcd_Log printk

#endif
#include "lcm_drv.h"

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  										(540)
#define FRAME_HEIGHT 										(960)

#define REGFLAG_DELAY             							0XFFE
#define REGFLAG_END_OF_TABLE      							0xFFF   // END OF REGISTERS MARKER

#define LCM_DSI_CMD_MODE									0
#define GPIO_LCD_ID_PIN GPIO_LCM_ID_PIN

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))

#define UDELAY(n) 											(lcm_util.udelay(n))
#define MDELAY(n) 											(lcm_util.mdelay(n))

//static kal_bool IsFirstBoot = KAL_TRUE;

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)    

struct LCM_setting_table {
    unsigned char cmd;
    unsigned char count;
    unsigned char para_list[64];
};

static struct LCM_setting_table lcm_initialization_setting0[] = {

            /*
            Note :

            Data ID will depends on the following rule.

            	count of parameters > 1	=> Data ID = 0x39
            	count of parameters = 1	=> Data ID = 0x15
            	count of parameters = 0	=> Data ID = 0x05

            Structure Format :

            {DCS command, count of parameters, {parameter list}}
            {REGFLAG_DELAY, milliseconds of time, {}},

            ...

            Setting ending by predefined flag

            {REGFLAG_END_OF_TABLE, 0x00, {}}
            */

            {0x00,1,{0x00}},
            {0xFF,3,{0x96,0x05,0x01}},

            {0x00,1,{0x80}},
            {0xFF,2,{0x96,0x05}},

            {0x00,1,{0x92}}, // mipi 2 lane
            {0xFF,2,{0x10,0x02}},

            {0x00,1,{0x80}},
            {0xC1,2,{0x36,0x77}}, //65Hz
            
            {0x00,1,{0xa0}},
            {0xC1,1,{0x02}},

            {0x00,1,{0xB1}},
            {0xC5,1,{0x28}},//VDD18=1.9V

            {0x00,1,{0x89}},
            {0xC0,1,{0x01}},// TCON OSC turbo mode
            ////auo request
            {0x00,1,{0xB4}},
            {0xC0,1,{0x10}},// 1+2 dot

            {0x00,1,{0x80}},
            {0xC4,1,{0x9c}},

            //Address Shift//Power Control Setting 2 for Normal Mode
            {0x00,1,{0x90}},
            {0xC5,7,{0x96,0x79,0x01,0x79,0x33,0x33,0x34}},//VGH VHL

            {0x00,1,{0x00}},
            {0xD8,2,{0x67,0x67}},//gvdd=4.4V

            //Address Shift//VCOM setting
            {0x00,1,{0x00}},
            {0xD9,1,{0x68}},

            {0x00,1,{0xA6}},//zigzag off
            {0xB3,1,{0x27}},

            {0x00,1,{0xA0}},//panel mode
            {0xB3,1,{0x10}},
            //GOA
            {0x00,1,{0x80}},
            {0xCB,10,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
            {0x00,1,{0x90}},
            {0xCB,15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
            {0x00,1,{0xA0}},
            {0xCB,15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
            {0x00,1,{0xB0}},
            {0xCB,10,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
            {0x00,1,{0xC0}},
            {0xCB,15,{0x00,0x00,0x04,0x04,0x04,0x04,0x04,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
            {0x00,1,{0xD0}},
            {0xCB,15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x04,0x04,0x04,0x04,0x04,0x00,0x00}},
            {0x00,1,{0xE0}},
            {0xCB,10,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
            {0x00,1,{0xF0}},
            {0xCB,10,{0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}},

            {0x00,1,{0x80}},
            {0xCC,10,{0x00,0x00,0x01,0x0F,0x0D,0x0B,0x09,0x05,0x00,0x00}},
            {0x00,1,{0x90}},
            {0xCC,15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x02,0x10,0x0E}},
            {0x00,1,{0xA0}},
            {0xCC,15,{0x0C,0x0A,0x06,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

            {0x00,1,{0x80}},
            {0xCE,12,{0x81,0x01,0x0E,0x82,0x01,0x0E,0x00,0x0F,0x00,0x00,0x0F,0x00}},
            {0x00,1,{0x90}},
            {0xCE,14,{0x13,0xBE,0x0E,0x13,0xBF,0x0E,0xF0,0x00,0x00,0xF0,0x00,0x00,0x00,0x00}},
            {0x00,1,{0xA0}},
            {0xCE,14,{0x18,0x01,0x03,0xBC,0x00,0x0E,0x00,0x18,0x00,0x03,0xBD,0x00,0x0E,0x00}},
            {0x00,1,{0xB0}},
            {0xCE,14,{0x10,0x00,0x03,0xBE,0x00,0x0E,0x00,0x10,0x01,0x03,0xBF,0x00,0x0E,0x00}},
            {0x00,1,{0xC0}},
            {0xCE,14,{0x38,0x03,0x03,0xC0,0x00,0x09,0x05,0x38,0x02,0x03,0xC1,0x00,0x09,0x05}},
            {0x00,1,{0xD0}},
            {0xCE,14,{0x30,0x00,0x03,0xBC,0x00,0x09,0x05,0x30,0x01,0x03,0xBD,0x00,0x09,0x05}},
            {0x00,1,{0xC0}},
            {0xCF,10,{0x01,0x01,0x00,0x00,0x00,0x00,0x01,0x02,0x00,0x00}},
            ////auo request
            //Address Shift//Gamma +
            {0x00,1,{0x00}},
            {0xE1,16,{0x05,0x0B,0x10,0x0C,0x05,0x0D,0x0B,0x0A,0x04,0x07,0x0D,0x07,0x0E,0x13,0x0E,0x00}},

            //Address Shift//Gamma -
            {0x00,1,{0x00}},
            {0xE2,16,{0x05,0x0B,0x10,0x0C,0x05,0x0D,0x0B,0x0A,0x04,0x07,0x0D,0x07,0x0E,0x13,0x0E,0x00}},

            {0x11,1,{0x00}},//SLEEP OUT
            {REGFLAG_DELAY,120,{}},

            {0x29,1,{0x00}},//Display ON
            {REGFLAG_DELAY,20,{}},
            // Note
            // Strongly recommend not to set Sleep out / Display On here. That will cause messed frame to be shown as later the backlight is on.


            // Setting ending by predefined flag
            {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_initialization_setting1[] = {
	
{0x00,1,{0x00}},
{0xff,3,{0x96,0x08,0x01}},
{0x00,1,{0x80}},
{0xff,2,{0x96,0x08}},
{0x00,1,{0x00}},
{0xA0,1,{0x00}},
{0x00,1,{0x80}},
{0xb3,5,{0x00,0x00,0x20,0x00,0x00}},
{0x00,1,{0xc0}},
{0xb3,1,{0x09}},

{0x00,1,{0x80}},
{0xc0,9,{0x00,0x48,0x00,0x10,0x10,0x00,0x48,0x10,0x10}},
{0x00,1,{0x92}},
{0xc0,4,{0x00,0x10,0x00,0x13}},
{0x00,1,{0xa2}},	
{0xC0,3,{0x0c,0x05,0x02}},
{0x00,1,{0xB3}},
{0xC0,2,{0x00,0x50}},
{0x00,1,{0x87}},
{0xc4,1,{0x08}},

{0x00,1,{0x81}},
{0xc1,1,{0x55}},
{0x00,1,{0x80}},
{0xC4,2,{0x30,0x84}},
{0x00,1,{0x88}},
{0xC4,1,{0x40}},
{0x00,1,{0xa0}},
{0xC4,8,{0x33,0x09,0x90,0x2b,0x33,0x09,0x90,0x54}},
{0x00,1,{0x80}},
{0xC5,4,{0x08,0x00,0xa0,0x11}},

{0x00,1,{0x90}},
{0xC5,7,{0x96,0x16,0x01,0x79,0x33,0x33,0x34}},
{0x00,1,{0xa0}},
{0xC5,7,{0x96,0x16,0x00,0x79,0x33,0x33,0x34}},
{0x00,1,{0x00}},
{0xd8,2,{0x6f,0x6f}},
{0x00,1,{0x00}},
{0xd9,1,{0x33}},
{0x00,1,{0xB0}},//
{0xc5,2,{0x04,0xa8}},

{0x00,1,{0x80}},	
{0xc6,1,{0x64}},
{0x00,1,{0xb0}},	
{0xc6,5,{0x03,0x10,0x00,0x1f,0x12}},
{0x00,1,{0x00}},
{0xd0,1,{0x40}},
{0x00,1,{0x00}},	
{0xd1,2,{0x00,0x00}},
{0x00,1,{0xB7}},
{0xb0,1,{0x10}},

{0x00,1,{0xc0}},
{0xb0,1,{0x55}},
{0x00,1,{0xb1}},
{0xb0,1,{0x03}},
{0x00,1,{0x80}},
{0xCB,10,{0x05,0x00,0x00,0x00,0x00,0x05,0x00,0x00,0x00,0x00}},
{0x00,1,{0x90}},
{0xCB,15,{0x55,0x55,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
{0x00,1,{0xa0}},
{0xCB,15,{0x00,0x00,0x00,0x00,0x00,0x55,0x55,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

{0x00,1,{0xb0}},
{0xCB,10,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
{0x00,1,{0xc0}},
{0xCB,15,{0x55,0x55,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x00,0x04,0x04,0x00,0x00}},
{0x00,1,{0xd0}},
{0xCB,15,{0x00,0x00,0x00,0x00,0x00,0x55,0x55,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04}},
{0x00,1,{0xe0}},
{0xCB,10,{0x00,0x04,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
{0x00,1,{0xf0}},
{0xCB,10,{0x0f,0x00,0xcc,0x00,0x00,0x0f,0x00,0xcc,0x03,0x00}},

{0x00,1,{0x80}},
{0xCC,10,{0x25,0x26,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0a}},
{0x00,1,{0x90}},
{0xCC,15,{0x00,0x0c,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x25,0x26,0x00,0x00,0x00}},
{0x00,1,{0xa0}},
{0xCC,15,{0x00,0x00,0x00,0x00,0x09,0x00,0x0b,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
{0x00,1,{0xb0}},		
{0xcc,12,{0x26,0x25,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0b}},
{0x00,1,{0xc0}},
{0xcc,15,{0x00,0x09,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x26,0x25,0x00,0x00,0x00}},

{0x00,1,{0xd0}},
{0xcc,15,{0x00,0x00,0x00,0x00,0x0c,0x00,0x0a,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
{0x00,1,{0x80}},
{0xCE,12,{0x84,0x01,0x24,0x83,0x01,0x24,0x00,0x00,0x00,0x00,0x00,0x00}},
{0x00,1,{0x90}},
{0xCE,14,{0xf0,0x00,0x00,0xf0,0x00,0x00,0xf0,0x00,0x00,0xf0,0x00,0x00,0x00,0x00}},
{0x00,1,{0xa0}},
{0xCE,14,{0x18,0x02,0x03,0xC1,0x00,0x24,0x00,0x18,0x01,0x03,0xC2,0x00,0x24,0x00}},
{0x00,1,{0xb0}},
{0xce,14,{0x18,0x00,0x03,0xc3,0x00,0x24,0x00,0x10,0x00,0x03,0xc4,0x00,0x24,0x00}},

{0x00,1,{0xc0}},
{0xce,14,{0xF0,0x00,0x00,0x10,0x00,0x00,0x00,0xF0,0x00,0x00,0x10,0x00,0x00,0x00}},
{0x00,1,{0xd0}},
{0xce,14,{0xF0,0x00,0x00,0x10,0x00,0x00,0x00,0xF0,0x00,0x00,0x10,0x00,0x00,0x00}},
{0x00,1,{0x80}},
{0xCF,14,{0xF0,0x00,0x00,0x10,0x00,0x00,0x00,0xF0,0x00,0x00,0x10,0x00,0x00,0x00}},
{0x00,1,{0x90}},
{0xCF,14,{0xF0,0x00,0x00,0x10,0x00,0x00,0x00,0xF0,0x00,0x00,0x10,0x00,0x00,0x00}},
{0x00,1,{0xa0}},
{0xCF,14,{0xF0,0x00,0x00,0x10,0x00,0x00,0x00,0xF0,0x00,0x00,0x10,0x00,0x00,0x00}},

{0x00,1,{0xb0}},
{0xCF,14,{0xF0,0x00,0x00,0x10,0x00,0x00,0x00,0xF0,0x00,0x00,0x10,0x00,0x00,0x00}},
{0x00,1,{0xc0}},
{0xCF,10,{0x01,0x01,0x20,0x20,0x00,0x00,0x02,0x81,0x00,0x10}},
{0x00,1,{0x00}},
{0xE1,16,{0x08,0x11,0x19,0x0E,0x07,0x10,0x0B,0x0A,0x03,0x07,0x0C,0x09,0x0F,0x0C,0x06,0x03}},
{0x00,1,{0x00}},
{0xE2,16,{0x08,0x11,0x19,0x0E,0x07,0x10,0x0B,0x0A,0x03,0x07,0x0C,0x09,0x0F,0x0C,0x06,0x03}},
{0x00,1,{0x00}},
{0xff,3,{0xff,0xff,0xff}},





{0x11,1,{0x00}},//SLEEP OUT
{REGFLAG_DELAY,200,{}},
                                 				                                                                                
{0x29,1,{0x00}},//Display ON 
{REGFLAG_DELAY,20,{}},	
{REGFLAG_END_OF_TABLE, 0x00, {}}
};



static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
	{0x11, 0, {0x00}},
    {REGFLAG_DELAY, 120, {}},

    // Display ON
	{0x29, 0, {0x00}},
    {REGFLAG_DELAY, 100, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
	// Display off sequence
	{0x28, 0, {0x00}},
        {REGFLAG_DELAY, 120, {}},
    // Sleep Mode On
	{0x10, 0, {0x00}},
        {REGFLAG_DELAY, 20, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

    for(i = 0; i < count; i++) {
		
        unsigned cmd;
        cmd = table[i].cmd;
		
        switch (cmd) {
			
            case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;
				
            case REGFLAG_END_OF_TABLE :
                break;
				
            default:
				dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
       	}
    }
	
}


// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
		memset(params, 0, sizeof(LCM_PARAMS));
		params->type   = LCM_TYPE_DSI;
		params->width  = FRAME_WIDTH;
		params->height = FRAME_HEIGHT;

		// enable tearing-free
            params->dbi.te_mode				= LCM_DBI_TE_MODE_DISABLED;
             params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;
            params->dsi.mode   = SYNC_EVENT_VDO_MODE;
        // params->dsi.mode   = CMD_MODE;
          // params->dbi.te_mode                 = LCM_DBI_TE_MODE_VSYNC_ONLY;
    //params->dbi.te_edge_polarity        = LCM_POLARITY_RISING;
		// DSI
		/* Command mode setting */
		params->dsi.LANE_NUM				= LCM_TWO_LANE;
		//The following defined the fomat for data coming from LCD engine.
		params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
		params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
		params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

		// Highly depends on LCD driver capability.
		// Not support in MT6573
		params->dsi.packet_size=256;

		// Video mode setting		
		params->dsi.intermediat_buffer_num = 2;

		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

		params->dsi.vertical_sync_active				= 4;//2
		params->dsi.vertical_backporch					= 16;//50
		params->dsi.vertical_frontporch					= 15;//20
		params->dsi.vertical_active_line				= FRAME_HEIGHT; 

		params->dsi.horizontal_sync_active				= 10;//2
		params->dsi.horizontal_backporch				= 64;//100
		params->dsi.horizontal_frontporch				= 64;//100
		params->dsi.horizontal_blanking_pixel			= 60;   //add
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;


		// Bit rate calculation
		//params->dsi.pll_div1=30;//32		// fref=26MHz, fvco=fref*(div1+1)	(div1=0~63, fvco=500MHZ~1GHz)
		//params->dsi.pll_div2=1; 		// div2=0~15: fout=fvo/(2*div2)
                params->dsi.pll_div1=1;//48;		// fref=26MHz, fvco=fref*(div1+1)	(div1=0~63, fvco=500MHZ~1GHz)
		params->dsi.pll_div2=1; 		// div2=0~15: fout=fvo/(2*div2)
		 params->dsi.fbk_sel=1;	//add
                #ifdef BIRD_13C43_BT45B
                params->dsi.fbk_div =30;//22
                #else
                params->dsi.fbk_div =30;//30
                #endif
}

static int get_lcd_id(void)
{
    mt_set_gpio_mode(GPIO_LCD_ID_PIN,0);
    mt_set_gpio_dir(GPIO_LCD_ID_PIN,0);
    mt_set_gpio_pull_enable(GPIO_LCD_ID_PIN,1);
    mt_set_gpio_pull_select(GPIO_LCD_ID_PIN,1);
    MDELAY(1);

    return mt_get_gpio_in(GPIO_LCD_ID_PIN);
}

static void lcm_init(void)
{
	unsigned int data_array[64];
	
	#if defined(BUILD_LK)
	upmu_set_rg_vgp2_vosel(5);
	upmu_set_rg_vgp2_en(1);

	upmu_set_rg_vgp3_vosel(3);
	upmu_set_rg_vgp3_en(1);	
#else
	hwPowerOn(MT6323_POWER_LDO_VGP2, VOL_2800, "Lance_LCM");
       hwPowerOn(MT6323_POWER_LDO_VGP3, VOL_1800, "Lance_LCM");
#endif

#ifdef BUILD_LK
	printf("MYCAT nt35512 lk lcm_init\n");
#else
      printk("MYCAT nt35512  kernel lcm_init\n");
#endif

    SET_RESET_PIN(1);
    SET_RESET_PIN(0);
    MDELAY(30);//Must > 10ms
    SET_RESET_PIN(1);
    MDELAY(120);//Must > 120ms
		
		if(get_lcd_id()==0)
    	push_table(lcm_initialization_setting0, sizeof(lcm_initialization_setting0) / sizeof(struct LCM_setting_table), 1);
    else
    	push_table(lcm_initialization_setting1, sizeof(lcm_initialization_setting1) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_suspend(void)
{
    SET_RESET_PIN(1);
    SET_RESET_PIN(0);
    MDELAY(10);//Must > 10ms
    SET_RESET_PIN(1);
    MDELAY(120);//Must > 120m
	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_resume(void)
{
	lcm_init();
}

static unsigned int lcm_compare_id(void)
{
	//int id=0;
	//id = get_lcd_id();
		
   // Lcd_Log("mycat OTM9608A id=%d \n",id);
		
//	return (LCM_ID == id)?1:0;
   // return (get_lcd_id()==1)?1:0;
   return 1;
}
static unsigned int flag_first=1;
static unsigned int lcm_esd_check()
{
	
	
	if((flag_first==1)){
		flag_first=0;
	return 1;
  }

return 0;

}
static unsigned int lcm_esd_recover()
{
	
	
	lcm_suspend();
	lcm_resume();
	
	
	
}
// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------
LCM_DRIVER otm9608a_daxian_D450N3178VO_TIANMA_dsi_2_qhd_lcm_drv = 
{
    .name			= "otm9608a_daxian_D450N3178VO_TIANMA_dsi_2_qhd",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
    .compare_id    = lcm_compare_id,
     .esd_check = lcm_esd_check,
	 .esd_recover = lcm_esd_recover,
};

