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
#define LCM_ID       (0x9605)
#define GPIO_LCD_ID_PIN                  97
#define REGFLAG_DELAY             							0XFE
#define REGFLAG_END_OF_TABLE      							0xFFF   // END OF REGISTERS MARKER
#define   __LCM_TE_ON__

#define LCM_DSI_CMD_MODE									0
#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif

static unsigned int lcm_esd_test = FALSE;      ///only for ESD test

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))

#define UDELAY(n) 											(lcm_util.udelay(n))
#define MDELAY(n) 											(lcm_util.mdelay(n))

static unsigned int lcm_compare_id();

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	        lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

static struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};
static struct LCM_setting_table lcm_initialization_setting[] = {
	{0x00,	1,	{0x00}},
	{0xff,	3,	{0x96,0x08,0x01}},
	{0x00,	1,	{0x80}},
	{0xff,	2,	{0x96,0x08}},
	{0x00,	1,	{0x00}},
	{0xa0,	1,	{0x00}},
	{0x00,	1,	{0x80}},
	{0xb3,	5,	{0x00,0x00,0x20,0x00,0x00}},
	{0x00,	1,	{0xc0}},
	{0xb3,	1,	{0x09}},
	{0x00,	1,	{0x80}},
	{0xc0,	9,	{0x00,0x48,0x00,0x10,0x10,0x00,0x47,0x10,0x10}},
	{0x00,	1,	{0x92}},
	{0xc0,	4,	{0x00,0x10,0x00,0x13}},
	{0x00,	1,	{0xa2}},
	{0xc0,	3,	{0x0c,0x05,0x02}},
	{0x00,	1,	{0xb3}},
	{0xc0,	2,	{0x00,0x50}},
	{0x00,	1,	{0x81}},
	{0xc1,	1,	{0x66}},//44  55  66  65HZ
	{0x00,	1,	{0x80}},
	{0xc4,	3,	{0x00,0x84,0xfc}},
	{0x00,	1,	{0xa0}},
	{0xb3,	2,	{0x10,0x00}},
	{0x00,	1,	{0xa0}},
	{0xc0,	1,	{0x00}},
	{0x00,	1,	{0xa0}},
	{0xc4,	8,	{0x33,0x09,0x90,0x2b,0x33,0x09,0x90,0x54}},
	{0x00,	1,	{0x80}},
	{0xc5,	4,	{0x08,0x00,0xa0,0x11}},
	{0x00,	1,	{0x90}},
	{0xc5,	7,	{0x96,0x57,0x00,0x57,0x33,0x33,0x34}},
	{0x00,	1,	{0xa0}},
	{0xc5,	7,	{0x00,0x00,0x00,0x00,0x33,0x33,0x34}},
	{0x00,	1,	{0xb0}},
	{0xc5,	7,	{0x04,0xac,0x01,0x00,0x71,0xb1,0x83}},
	{0x00,	1,	{0x00}},
	{0xd9,	1,	{0x62}},//  6e //Vcom
	{0x00,	1,	{0x80}},
	{0xc6,	1,	{0x64}},
	{0x00,	1,	{0xb0}},
	{0xc6,	5,	{0x03,0x10,0x00,0x1f,0x12}},
	{0x00,	1,	{0xe1}},
	{0xc0,	1,	{0x9f}},
	{0x00,	1,	{0xb7}},
	{0xb0,	1,	{0x10}},
	{0x00,	1,	{0xc0}},
	{0xb0,	1,	{0x55}},
	{0x00,	1,	{0xb1}},
	{0xb0,	1,	{0x03}},
	{0x00,	1,	{0x81}},
	{0xd6,	1,	{0x00}},
	{0x00,	1,	{0xb4}},
	{0xc0,	1,	{0x50}},//50  10 Inversion
	{0x00,	1,	{0x00}},
	{0xe1,	16,	{0x01,0x06,0x0A,0x0C,0x05,0x0C,0x0A,0x09,0x05,0x08,0x0F,0x09,0x0F,0x15,0x0F,0x09}},
	{0x00,	1,	{0x00}},
	{0xe2,	16,	{0x01,0x06,0x0A,0x0C,0x05,0x0C,0x0A,0x09,0x05,0x08,0x0F,0x09,0x0F,0x15,0x0F,0x09}},
	{0x00,	1,	{0x80}},
	{0xcb,	10,	{0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
	{0x00,	1,	{0x90}},
	{0xcb,	15,	{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
	{0x00,	1,	{0xa0}},
	{0xcb,	15,	{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
	{0x00,	1,	{0xb0}},
	{0xcb,	10,	{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
	{0x00,	1,	{0xc0}},
	{0xcb,	15,	{0x00,0x00,0x00,0x04,0x00,0x00,0x04,0x04,0x00,0x00,0x04,0x04,0x04,0x00,0x00}},
	{0x00,	1,	{0xd0}},
	{0xcb,	15,	{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x04,0x04,0x00,0x00,0x04,0x04}},
	{0x00,	1,	{0xe0}},
	{0xcb,	10,	{0x00,0x00,0x00,0x00,0x04,0x00,0x00,0x00,0x00,0x00}},
	{0x00,	1,	{0xf0}},
	{0xcb,	10,	{0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff}},
	{0x00,	1,	{0x80}},
	{0xcc,	10,	{0x00,0x00,0x00,0x02,0x00,0x00,0x0a,0x0e,0x00,0x00}},
	{0x00,	1,	{0x90}},
	{0xcc,	15,	{0x0c,0x10,0x06,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x09}},
	{0x00,	1,	{0xa0}},
	{0xcc,	15,	{0x0d,0x00,0x00,0x0b,0x0f,0x00,0x00,0x00,0x00,0x05,0x00,0x00,0x00,0x00,0x00}},
	{0x00,	1,	{0xb0}},
	{0xcc,	10,	{0x00,0x00,0x00,0x02,0x00,0x00,0x0a,0x0e,0x00,0x00}},
	{0x00,	1,	{0xc0}},
	{0xcc,	15,	{0x0c,0x10,0x06,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
	{0x00,	1,	{0xd0}},
	{0xcc,	15,	{0x05,0x00,0x00,0x00,0x00,0x0f,0x0b,0x00,0x00,0x0d,0x09,0x01,0x00,0x00,0x00}},
	{0x00,	1,	{0x80}},
	{0xce,	12,	{0x84,0x03,0x18,0x83,0x03,0x18,0x00,0x00,0x00,0x00,0x00,0x00}},
	{0x00,	1,	{0x90}},
	{0xce,	14,	{0x33,0xbf,0x18,0x33,0xc0,0x18,0x10,0x0f,0x18,0x10,0x10,0x18,0x00,0x00}},
	{0x00,	1,	{0xa0}},
	{0xce,	14,	{0x38,0x02,0x03,0xc1,0x00,0x18,0x00,0x38,0x01,0x03,0xc2,0x00,0x18,0x00}},
	{0x00,	1,	{0xb0}},
	{0xce,	14,	{0x38,0x00,0x03,0xc3,0x00,0x18,0x00,0x30,0x00,0x03,0xc4,0x00,0x18,0x00}},
	{0x00,	1,	{0xc0}},
	{0xce,	14,	{0x30,0x01,0x03,0xc5,0x00,0x18,0x00,0x30,0x02,0x03,0xc6,0x00,0x18,0x00}},
	{0x00,	1,	{0xd0}},
	{0xce,	14,	{0x30,0x03,0x03,0xc7,0x00,0x18,0x00,0x30,0x04,0x03,0xc8,0x00,0x18,0x00}},
	{0x00,	1,	{0x80}},
	{0xcf,	14,	{0xf0,0x00,0x00,0x00,0x00,0x00,0x00,0xf0,0x00,0x00,0x00,0x00,0x00,0x00}},
	{0x00,	1,	{0x90}},
	{0xcf,	14,	{0xf0,0x00,0x00,0x00,0x00,0x00,0x00,0xf0,0x00,0x00,0x00,0x00,0x00,0x00}},
	{0x00,	1,	{0xa0}},
	{0xcf,	14,	{0xf0,0x00,0x00,0x00,0x00,0x00,0x00,0xf0,0x00,0x00,0x00,0x00,0x00,0x00}},
	{0x00,	1,	{0xb0}},
	{0xcf,	14,	{0xf0,0x00,0x00,0x00,0x00,0x00,0x00,0xf0,0x00,0x00,0x00,0x00,0x00,0x00}},
	{0x00,	1,	{0xc0}},
	{0xcf,	10,	{0x01,0x01,0x20,0x20,0x00,0x00,0x02,0x04,0x00,0x00}},
	{0x00,	1,	{0x00}},
	{0xd8,	2,	{0x6F,0x6F}},//{0x6F,0x6F}}  //GVDD NGVDD
	
	
	{0x51,	1,	{0xff}},
	{0x53,	1,	{0x2c}},
	{0x55,	1,	{0x00}},
	
	{0x00,	1,	{0xc7}},
	{0xcf,	1,	{0x00}},
	
	{0x00,	1,	{0x00}},
	{0xff,	3,	{0xff,0xff,0xff}},
	
	{0x00,	1,	{0x00}},
	{0x3A,	1,	{0x77}},
	
	{0x35,	1,	{0x00}},//TE On 
	{0x11,	1,	{0x00}},
	{REGFLAG_DELAY, 150, {}},
	{0x29,	1,	{0x00}},
	{0x2C,	1,	{0x00}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_set_window[] = {
	{0x2A,	4,	{0x00, 0x00, (FRAME_WIDTH>>8), (FRAME_WIDTH&0xFF)}},
	{0x2B,	4,	{0x00, 0x00, (FRAME_HEIGHT>>8), (FRAME_HEIGHT&0xFF)}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_sleep_out_setting[] = {
#if 1
    // Sleep Out
	{0x11, 1, {0x00}},
        {REGFLAG_DELAY, 120, {}},

    // Display ON
	{0x29, 1, {0x00}},
        {REGFLAG_DELAY, 50, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
#else

        {0x00,  1,  {0x00}},
        {0xd9,  1,  {0x64}},//vcom  0x61
        
	{REGFLAG_END_OF_TABLE, 0x00, {}}

#endif
};


static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
#if 1
    
	// Display off sequence
	{0x28, 1, {0x00}},

    // Sleep Mode On
	{0x10, 1, {0x00}},
#else
    {0x00,  1,  {0x00}},
    {0xd9,  1,  {0x5E}},//vcom  0x61
    
#endif
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_compare_id_setting[] = {
	// Display off sequence
	{0xd2,	4,	{}},
	{REGFLAG_DELAY, 10, {}},

    // Sleep Mode On
//	{0xC3, 1, {0xFF}},

	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_backlight_level_setting[] = {
	{0x51, 1, {0xFF}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;
	#if defined(BUILD_UBOOT)
		printf("herman-push-test\n");
	#endif
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
    	#ifdef  __LCM_TE_ON__
        // enable tearing-free
        params->dbi.te_mode                 = LCM_DBI_TE_MODE_VSYNC_ONLY;
        params->dbi.te_edge_polarity        = LCM_POLARITY_RISING;
        #else 
        params->dbi.te_mode                 = LCM_DBI_TE_MODE_DISABLED;
        #endif


#if (LCM_DSI_CMD_MODE)
		params->dsi.mode   = CMD_MODE;
#else
		params->dsi.mode   = SYNC_PULSE_VDO_MODE;
#endif
	
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

                params->dsi.word_count=540*3;
		params->dsi.vertical_sync_active				= 4;
		params->dsi.vertical_backporch					= 8;
		params->dsi.vertical_frontporch					= 8;
		params->dsi.vertical_active_line				= 960; 

		params->dsi.horizontal_sync_active				= 6;
		params->dsi.horizontal_backporch				= 37;
		params->dsi.horizontal_frontporch				= 37;
		params->dsi.horizontal_blanking_pixel			= 60;   //add
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

		// Bit rate calculation
                params->dsi.pll_div1=1;//48;		// fref=26MHz, fvco=fref*(div1+1)	(div1=0~63, fvco=500MHZ~1GHz)
		params->dsi.pll_div2=1; 		// div2=0~15: fout=fvo/(2*div2)
		 params->dsi.fbk_sel=1;	//add
         params->dsi.fbk_div =30;

		/* ESD or noise interference recovery For video mode LCM only. */ // Send TE packet to LCM in a period of n frames and check the response. 
		params->dsi.lcm_int_te_monitor = FALSE; 
		params->dsi.lcm_int_te_period = 1; // Unit : frames 
 
		// Need longer FP for more opportunity to do int. TE monitor applicably. 
		if(params->dsi.lcm_int_te_monitor) 
			params->dsi.vertical_frontporch *= 2; 
 
		// Monitor external TE (or named VSYNC) from LCM once per 2 sec. (LCM VSYNC must be wired to baseband TE pin.) 
		params->dsi.lcm_ext_te_monitor = FALSE; 
		// Non-continuous clock 
		params->dsi.noncont_clock = TRUE; 
		params->dsi.noncont_clock_period = 2; // Unit : frames



}


static void lcm_init(void)
{
	unsigned char testreg;

#if defined(BUILD_LK)
	upmu_set_rg_vgp2_vosel(5);
	upmu_set_rg_vgp2_en(1);

	upmu_set_rg_vgp3_vosel(3);
	upmu_set_rg_vgp3_en(1);	
#else
	hwPowerOn(MT6323_POWER_LDO_VGP2, VOL_2800, "Lance_LCM");
       hwPowerOn(MT6323_POWER_LDO_VGP3, VOL_1800, "Lance_LCM");
#endif

    SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(200);
    
    push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);

}

static void lcm_suspend(void)
{
	//printk("vito1 ---- %s,Line:%d,lr=%x\r\n",__func__,__LINE__,__builtin_return_address(0));
	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);

    SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(100);

	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_resume(void)
{           
	//printk("vito1 ---- %s,Line:%d,lr=%x\r\n",__func__,__LINE__,__builtin_return_address(0));
	//push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
	//lcm_initialization_setting[37].para_list[0]-=1;
//Lcd_Log("mycat dxc vcom id = 0x%02x", lcm_initialization_setting[37].para_list[0]);
	lcm_init();
}


static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0>>8)&0xFF);
	unsigned char x0_LSB = (x0&0xFF);
	unsigned char x1_MSB = ((x1>>8)&0xFF);
	unsigned char x1_LSB = (x1&0xFF);
	unsigned char y0_MSB = ((y0>>8)&0xFF);
	unsigned char y0_LSB = (y0&0xFF);
	unsigned char y1_MSB = ((y1>>8)&0xFF);
	unsigned char y1_LSB = (y1&0xFF);

	unsigned int data_array[16];

	data_array[0]= 0x00053902;
	data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
	data_array[2]= (x1_LSB);
	data_array[3]= 0x00053902;
	data_array[4]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
	data_array[5]= (y1_LSB);
	data_array[6]= 0x002c3909;

	dsi_set_cmdq(&data_array, 7, 0);

}


static void lcm_setbacklight(unsigned int level)
{
	// Refresh value of backlight level.
	lcm_backlight_level_setting[0].para_list[0] = level;

	push_table(lcm_backlight_level_setting, sizeof(lcm_backlight_level_setting) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_setpwm(unsigned int divider)
{
	// TBD
}


static unsigned int lcm_getpwm(unsigned int divider)
{
	// ref freq = 15MHz, B0h setting 0x80, so 80.6% * freq is pwm_clk;
	// pwm_clk / 255 / 2(lcm_setpwm() 6th params) = pwm_duration = 23706
	unsigned int pwm_clk = 23706 / (1<<divider);
	return pwm_clk;
}

static unsigned int lcm_esd_check()
{

	int tem;
#ifndef BUILD_LK
	Lcd_Log("lcm_esd_check is begning...\n");
        if(lcm_esd_test)
        {
            lcm_esd_test = FALSE;
            return TRUE;
        }

        /// please notice: the max return packet size is 1
        /// if you want to change it, you can refer to the following marked code
        /// but read_reg currently only support read no more than 4 bytes....
        /// if you need to read more, please let BinHan knows.

                unsigned int data_array[16];
                unsigned int max_return_size = 1;

       //         data_array[0]= 0x00003700 | (max_return_size << 16);

        //        dsi_set_cmdq(&data_array, 1, 1);

		//nasri@2012-3-12. Based on ini
 		tem=read_reg(0x0A);

		Lcd_Log("lcm_esd_check_tem = 0x%x\n",tem);
	//printk("herman otm9605a tem  = 0x%x\n",tem);

		if(read_reg(0x0A) != 0x9c)
        {
		Lcd_Log("lcm_esd_check to false...\n");
            return FALSE;
        }
        else
        {
		Lcd_Log("lcm_esd_check to true...\n");
            return TRUE;
        }
#endif
}

static unsigned int lcm_dsi_read_test(unsigned char cmd)
{
#ifndef BUILD_UBOOT
        /// please notice: the max return packet size is 1
        /// if you want to change it, you can refer to the following marked code
        /// but read_reg currently only support read no more than 4 bytes....
        /// if you need to read more, please let BinHan knows.
/*
        unsigned int data_array[16];
        unsigned int max_return_size = 1;

        data_array[0]= 0x00003700 | (max_return_size << 16);

        dsi_set_cmdq(&data_array, 1, 1);
*/
        return read_reg(cmd);
#endif
}

static unsigned int lcm_esd_recover()
{
    unsigned char para = 0;

    SET_RESET_PIN(1);
    MDELAY(1);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(120);
//nasri@2012-4-18
//Rotate LCM if MTK_LCM_PHYSICAL_ROTATION equals 180
#if 0
	if(0 == strncmp(MTK_LCM_PHYSICAL_ROTATION,"180",3))
	{
	  push_table(lcm_initialization_setting_rotate_180, sizeof(lcm_initialization_setting_rotate_180) / sizeof(struct LCM_setting_table), 1);
	}
	else
	{
	  push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
	}
#endif    
    MDELAY(10);
	  push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
    MDELAY(10);
   // dsi_set_cmdq_V2(0x35, 1, &para, 1);     ///enable TE
    MDELAY(10);

    return TRUE;
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

static unsigned int lcm_compare_id()
{
	unsigned int id = 0;
	unsigned char buffer[5];
	unsigned int array[16];
        SET_RESET_PIN(1);  //NOTE:should reset LCM firstly
        MDELAY(10);
    	SET_RESET_PIN(0);
    	MDELAY(10);
    	SET_RESET_PIN(1);
    	MDELAY(200);

//	push_table(lcm_compare_id_setting, sizeof(lcm_compare_id_setting) / sizeof(struct LCM_setting_table), 1);
	//array[0]=0x00043902;
	//array[1]=0x010980ff;
	//array[2]=0x80001500;
	//array[3]=0x00033902;
	//array[4]=0x010980ff;
	//dsi_set_cmdq(array, 5, 1);
	//MDELAY(10);

	/*read_reg_v2(0xa1, buffer, 5);
#if defined(BUILD_UBOOT)
	printf("herman a1 buffer[0] = 0x%x\n",buffer[0]);
	printf("herman a1 buffer[1] = 0x%x\n",buffer[1]);
	printf("herman a1 buffer[2] = 0x%x\n",buffer[2]);
	printf("herman a1 buffer[3] = 0x%x\n",buffer[3]);
	printf("herman a1 buffer[4] = 0x%x\n",buffer[4]);
#endif*/


//nasri@2012-5-9

	array[0]=0x00053700;
	dsi_set_cmdq(array, 1, 1);
	MDELAY(10);
	read_reg_v2(0xA1, buffer, 5);	//only id =0x9605

	Lcd_Log("herman A1 buffer[2] = 0x%x\n",buffer[2]);
	Lcd_Log("herman A1 buffer[3] = 0x%x\n",buffer[3]);
	
	read_reg_v2(0xA1, buffer, 5);	//only id =0x9605

	id = (buffer[2]<<8)|(buffer[3]);

	Lcd_Log("%s otm9608a qicai id  = 0x%x\n",__func__,id);

	//id=0x9605;

	//return (LCM_ID == id)?1:0;
//	return (LCM_ID == id&&get_lcd_id()==0)?1:0;
  return 1;
}
// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------
LCM_DRIVER otm9608a_qicai_K45CO23_1322A_1N_CMO_dsi_2_qhd_lcm_drv =
    {
        .name			= "otm9608a_qicai_K45CO23_1322A_1N_CMO_dsi_2_qhd",
        .set_util_funcs = lcm_set_util_funcs,
        .get_params     = lcm_get_params,
        .init           = lcm_init,
        .suspend        = lcm_suspend,
        .resume         = lcm_resume,
#if (LCM_DSI_CMD_MODE)       
	.update         = lcm_update,
#endif	
//	.set_backlight	= lcm_setbacklight,
//	.set_pwm        = lcm_setpwm,
//	.get_pwm        = lcm_getpwm,
//	.esd_check   = lcm_esd_check,
//        .esd_recover   = lcm_esd_recover,
        .compare_id    = lcm_compare_id,
    };

