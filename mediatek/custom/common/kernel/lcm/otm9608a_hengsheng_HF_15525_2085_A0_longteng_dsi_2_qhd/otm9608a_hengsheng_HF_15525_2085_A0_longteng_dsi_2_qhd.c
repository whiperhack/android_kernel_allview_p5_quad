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
#include <linux/string.h>
#include <linux/kernel.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>

#define Lcd_Log printk
#endif
#include "lcm_drv.h"

static unsigned int lcm_compare_id(void);

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  (540)
#define FRAME_HEIGHT (960)
#define REGFLAG_DELAY             							0xAB
#define REGFLAG_END_OF_TABLE      							0xAA   // END OF REGISTERS MARKER
#define GPIO_LCD_ID_PIN 50


#define LCM_DSI_CMD_MODE									1


// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))

#define LCM_ID       (0x9608)


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	        lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)									lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)				lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

static unsigned int lcm_compare_id();



static struct LCM_setting_table {
    unsigned char cmd;
    unsigned char count;
    unsigned char para_list[64];
};


static struct LCM_setting_table lcm_sleep_out_setting[] = {

            // Sleep Out
            {0x11, 1, {0x00}},
            {REGFLAG_DELAY, 120, {}},

            // Display ON
            {0x29, 1, {0x00}},
            {REGFLAG_DELAY, 20, {}},

            {REGFLAG_END_OF_TABLE, 0x00, {}}
        };


static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
            // Display off sequence
            {0x28, 1, {0x00}},
            {REGFLAG_DELAY, 200, {}},

            // Sleep Mode On
            {0x10, 1, {0x00}},
            {REGFLAG_DELAY, 100, {}},

            {REGFLAG_END_OF_TABLE, 0x00, {}}
        };


static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
    unsigned int i;

    for (i = 0; i < count; i++) {

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
            //MDELAY(1);//MDELAY(10);
        }
    }

}



// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------
static struct LCM_setting_table lcm_initialization_setting[] = {
            {0x00, 1,{0x00}},      // Enable cmd

   {0xff, 3,{0x96,0x08,0x01}},  // Enable cmd
   {0x00, 1,{0x80}},      // Enable cmd
   {0xff, 2,{0x96,0x08}}, 
             
   {0x00, 1,{0x80}},      // Enable cmd
   {0xff, 2,{0x96,0x08}}, 
             
   {0x00, 1,{0x00}},      // Enable cmd
   {0xA0, 1,{0x00}}, 
             
   {0x00, 1,{0x80}},      // Enable cmd
   {0xB3, 5,{0x00,0x00,0x20,0x00,0x00}},
             
   {0x00, 1,{0xC0}},      // Enable cmd
   {0xB3, 1,{0x09}},
             
   {0x00, 1,{0x80}},      // Enable cmd
   {0xC0, 9,{0x00,0x48,0x00,0x10,0x10,0x00,0x48,0x10,0x10}},
             
   {0x00, 1,{0x92}},      // Enable cmd
   {0xC0, 4,{0x00,0x10,0x00,0x13}},
             
   {0x00, 1,{0xa2}},      // Enable cmd
   {0xc0, 3,{0x0c,0x05,0x02}},
             
   {0x00, 1,{0xb3}},      // Enable cmd
   {0xc0, 2,{0x00,0x50}},

   {0x00, 1,{0xb4}},      // Enable cmd
   {0xc0, 1,{0x50}},
             
   {0x00, 1,{0x81}},      // Enable cmd
   //{0xc1, 1,{0x33}},
   {0xc1, 1,{0x55}},
             
   {0x00, 1,{0x80}},      // Enable cmd
   {0xc4, 2,{0x00,0x84}},
             
   {0x00, 1,{0xa0}},      // Enable cmd
   {0xc4, 8,{0x33,0x09,0x90,0x2b,0x33,0x09,0x90,0x54}},
             
   {0x00, 1,{0x80}},      // Enable cmd
   {0xc5, 4,{0x08,0x00,0xa0,0x11}},
             
   {0x00, 1,{0x90}},      // Enable cmd
   {0xc5, 7,{0x96,0xb7,0x01,0x79,0x33,0x33,0x34}},
             
   {0x00, 1,{0xa0}},      // Enable cmd
   {0xc5, 7,{0x96,0xb7,0x01,0x79,0x33,0x33,0x34}},

   {0x00, 1,{0xb1}},      // Enable cmd
   {0xc5, 2,{0x04,0xa8}},

   {0x00, 1,{0x80}},      // Enable cmd
   {0xc6, 1,{0x64}}, 
       
   {0x00, 1,{0x00}},      // Enable cmd
   {0xd8, 2,{0x87,0x87}},//{0xd8, 2,{0xb7,0xb7}},
   

   {0x00, 1,{0xb0}},      // Enable cmd
   {0xc5, 2,{0x04,0xa9}},
         
   {0x00, 1,{0x82}},      // Enable cmd
   {0xb2, 1,{0x20}},              

   {0x00, 1,{0x00}},      // Enable cmd
   //{0xd9, 1,{0x3a}},
   {0xd9, 1,{0x30}},
             

   {0x00, 1,{0x80}},      // Enable cmd
   {0xc6, 1,{0x64}},
           
   {0x00, 1,{0xb0}},      // Enable cmd
   {0xc6, 5,{0x03,0x10,0x00,0x1f,0x12}},
             
   {0x00, 1,{0x00}},      // Enable cmd
   {0xd0, 1,{0x40}},

   {0x00, 1,{0x00}},      // Enable cmd
   {0xd1, 2,{0x00,0x00}},
      
   {0x00, 1,{0xb7}},      // Enable cmd
   {0xb0, 1,{0x10}},       
             
   {0x00, 1,{0xc0}},      // Enable cmd
   {0xb0, 1,{0x55}},

   {0x00, 1,{0xb1}},      // Enable cmd
   {0xb0, 1,{0x03}},


		// Memory Data Access Control   
		//{0x00,1,{0x00}},{0x36,1,{0xd0}},  //0Xd0£¬180¡ã

		{0x00,1,{0x00}},{0x36,1,{0x0}},  //0X00£¬0¡ã

     
   {0x00, 1,{0x80}},      // Enable cmd
   {0xCB,10,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},  

   {0x00, 1,{0x90}},      // Enable cmd
   {0xCB,15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
    
   {0x00, 1,{0xa0}},      // Enable cmd
   {0xCB,15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

   {0x00, 1,{0xb0}},      // Enable cmd
   //{0xCB,15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
   {0xCB,10,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
           
   {0x00, 1,{0xC0}},      // Enable cmd
   {0xCB,15,{0x00,0x00,0x00,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x00,0x00,0x00,0x00}},
             
   {0x00, 1,{0xd0}},      // Enable cmd
   {0xCB,15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x04,0x04,0x04,0x04,0x04,0x04}},
             
   {0x00, 1,{0xE0}},      // Enable cmd
   {0xCB,10,{0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
             
   {0x00, 1,{0xF0}},      // Enable cmd
   {0xCB,10,{0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}},
             
   {0x00, 1,{0x80}},      // Enable cmd
   {0xCC,10,{0x00,0x00,0x00,0x03,0x01,0x09,0x0B,0x0D,0x0F,0x05}},
             
   {0x00, 1,{0x90}},      // Enable cmd
   {0xCC,15,{0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x02}},
             
   {0x00, 1,{0xa0}},      // Enable cmd
   {0xCC,15,{0x0A,0x0C,0x0E,0x10,0x06,0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
             
   {0x00, 1,{0xB0}},      // Enable cmd
   {0xCC,10,{0x00,0x00,0x00,0x06,0x08,0x0C,0x0A,0x10,0x0E,0x04}},
             
   {0x00, 1,{0xC0}},      // Enable cmd
   {0xCC,15,{0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x05,0x07}},
             
   {0x00, 1,{0xD0}},      // Enable cmd
   {0xCC,15,{0x0B,0x09,0x0F,0x0D,0x03,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
             
   {0x00, 1,{0x80}},      // Enable cmd
   {0xCE,12,{0x86,0x03,0x06,0x85,0x03,0x06,0x84,0x03,0x06,0x83,0x03,0x06}},
             
   {0x00, 1,{0x90}},      // Enable cmd
   {0xCE,14,{0x33,0xBD,0x06,0x33,0xBE,0x00,0x33,0xBF,0x06,0x33,0xC0,0x06,0x00,0x00}},
             
   {0x00, 1,{0xA0}},      // Enable cmd
   //{0xCE,14,{0x38,0x04,0x03,0xC7,0x00,0x00,0x20,0x38,0x03,0x03,0xC8,0x00,0x00,0x20}},
   {0xCE,14,{0x38,0x04,0x03,0xC7,0x00,0x00,0x00,0x38,0x03,0x03,0xC8,0x00,0x00,0x00}},//REV.0.04    
      
   {0x00, 1,{0xB0}},      // Enable cmd
   //{0xCE,14,{0x38,0x02,0x03,0xC9,0x00,0x00,0x20,0x38,0x01,0x03,0xCA,0x00,0x00,0x20}},
   {0xCE,14,{0x38,0x02,0x03,0xC9,0x00,0x00,0x00,0x38,0x01,0x03,0xCA,0x00,0x00,0x00}},//REV.0.04
          
   {0x00, 1,{0xC0}},      // Enable cmd
   //{0xCE,14,{0x38,0x00,0x03,0xCB,0x00,0x00,0x20,0x30,0x00,0x03,0xCC,0x00,0x00,0x20}},
   {0xCE,14,{0x38,0x00,0x03,0xCB,0x00,0x00,0x00,0x30,0x00,0x03,0xCC,0x00,0x00,0x00}},//REV.0.04
             
   {0x00, 1,{0xD0}},      // Enable cmd
   //{0xCE,14,{0x30,0x01,0x03,0xCD,0x00,0x00,0x20,0x30,0x02,0x03,0xCE,0x00,0x00,0x20}},
   {0xCE,14,{0x30,0x01,0x03,0xCD,0x00,0x00,0x00,0x30,0x02,0x03,0xCE,0x00,0x00,0x00}},//REV.0.04
             
   {0x00, 1,{0x80}},      // Enable cmd
   {0xCF,14,{0xF0,0x00,0x00,0x10,0x00,0x00,0x00,0xF0,0x00,0x00,0x10,0x00,0x00,0x00}},
             
   {0x00, 1,{0x90}},      // Enable cmd
   {0xCF,14,{0xF0,0x00,0x00,0x10,0x00,0x00,0x00,0xF0,0x00,0x00,0x10,0x00,0x00,0x00}},
             
   {0x00, 1,{0xA0}},      // Enable cmd
   {0xCF,14,{0xF0,0x00,0x00,0x10,0x00,0x00,0x00,0xF0,0x00,0x00,0x10,0x00,0x00,0x00}},
             
   {0x00, 1,{0xB0}},      // Enable cmd
   {0xCF,14,{0xF0,0x00,0x00,0x10,0x00,0x00,0x00,0xF0,0x00,0x00,0x10,0x00,0x00,0x00}},
             
   {0x00, 1,{0xC0}},      // Enable cmd
   //{0xCF,10,{0x01,0x01,0x20,0x20,0x00,0x00,0x02,0x00,0x00,0x00}},	
   {0xCF,10,{0x01,0x01,0x00,0x00,0x00,0x00,0x02,0x00,0x00,0x00}},//REV.0.04	

   {0x00, 1,{0xa0}},      // Enable cmd
   {0xcf, 1,{0x80}},

   {0x00, 1,{0xa0}},      // Enable cmd
   {0x0c, 1,{0x08}},

{0x00, 1,{0x00}},      // Enable cmd
   //{0xe1,16,{0x01,0x06,0x0a,0x0c,0x05,0x0d,0x0a,0x08,0x06,0x0a,0x0f,0x09,0x0f,0x09,0x0f,0x15,0x0e,0x00}},
   {0xe1,16,{0x02,0x06,0x09,0x0b,0x04,0x0a,0x09,0x07,0x07,0x0a,0x11,0x09,0x10,0x15,0x0f,0x08}},

{0x00, 1,{0x00}},      // Enable cmd
   //{0xe1,16,{0x01,0x06,0x0a,0x0c,0x05,0x0d,0x0a,0x08,0x06,0x0a,0x0f,0x09,0x0f,0x09,0x0f,0x15,0x0e,0x00}},
   {0xe2,16,{0x02,0x06,0x09,0x0b,0x04,0x0a,0x09,0x07,0x07,0x0a,0x11,0x09,0x10,0x15,0x0f,0x08}},

{0x00,1,{0x00}},{0x35,1,{0x00}}, //te on


        //{0x36,1,{0xD0}},
        //{REGFLAG_DELAY,20,{}},

            {0x11,1,{0x00}},//SLEEP OUT
            {REGFLAG_DELAY,120,{}},

            {0x29,1,{0x00}},//Display ON
            {REGFLAG_DELAY,20,{}},
            {REGFLAG_END_OF_TABLE, 0x00, {}}
        };

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

#if (LCM_DSI_CMD_MODE)
    params->dsi.mode   = CMD_MODE;
#else
    params->dsi.mode   = SYNC_PULSE_VDO_MODE;
#endif

    // DSI
    /* Command mode setting */
    params->dsi.LANE_NUM				= LCM_TWO_LANE;
		//The following defined the fomat for data coming from LCD engine.
    params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;


    params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
    // Bit rate calculation
    params->dsi.pll_div1=1;
    params->dsi.pll_div2=1;
    params->dsi.fbk_div=30;
}


static void lcm_init(void)
{
#if defined(BUILD_LK)
    upmu_set_rg_vgp2_vosel(5);
    upmu_set_rg_vgp2_en(1);

    upmu_set_rg_vgp3_vosel(3);
    upmu_set_rg_vgp3_en(1);
#else
    hwPowerOn(MT6323_POWER_LDO_VGP2,VOL_2800,"LCM");
    hwPowerOn(MT6323_POWER_LDO_VGP3,VOL_1800,"LCM");
#endif
    MDELAY(100);

    SET_RESET_PIN(1);
    MDELAY(5);
    SET_RESET_PIN(0);
    MDELAY(30);
    SET_RESET_PIN(1);
    MDELAY(120);

    push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_suspend(void)
{
    push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);

}


static void lcm_resume(void)
{
#if 0
	lcm_init();

#else
    push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
#endif

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
    dsi_set_cmdq(data_array, 3, 1);

    data_array[0]= 0x00053902;
    data_array[1]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
    data_array[2]= (y1_LSB);
    dsi_set_cmdq(data_array, 3, 1);

    data_array[0]= 0x002c3909;
    dsi_set_cmdq(data_array, 1, 0);

}

static int get_lcd_id(void)
{
    mt_set_gpio_mode(GPIO_LCD_ID_PIN,0);
    mt_set_gpio_dir(GPIO_LCD_ID_PIN,0);
    mt_set_gpio_pull_enable(GPIO_LCD_ID_PIN,1);
    mt_set_gpio_pull_select(GPIO_LCD_ID_PIN,0);
    MDELAY(1);

    return mt_get_gpio_in(GPIO_LCD_ID_PIN);
}

static unsigned int lcm_compare_id(void)
{
    int   array[4];
    char  buffer[5];
    unsigned int id=0;

#if defined(BUILD_LK)
    upmu_set_rg_vgp2_vosel(5);
    upmu_set_rg_vgp2_en(1);

    upmu_set_rg_vgp3_vosel(3);
    upmu_set_rg_vgp3_en(1);
#else
    hwPowerOn(MT6323_POWER_LDO_VGP2, VOL_2800, "Lance_LCM");
    hwPowerOn(MT6323_POWER_LDO_VGP3, VOL_1800, "Lance_LCM");
#endif
    MDELAY(100);

    SET_RESET_PIN(1);
    MDELAY(5);
    SET_RESET_PIN(0);
    MDELAY(30);
    SET_RESET_PIN(1);
    MDELAY(120);

    array[0] = 0x00083700;// read id return two byte,version and id
    dsi_set_cmdq(array, 1, 1);

    read_reg_v2(0xA1,buffer,4);
    id=(buffer[2]<<8)+buffer[3];
	
    Lcd_Log("mycat lcd id: 0x%08x\n", id);
	
    return (LCM_ID == id)?1:0;
}
static unsigned int lcm_esd_recover()
{

    lcm_init();

    push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
}


LCM_DRIVER otm9608a_hengsheng_HF_15525_2085_A0_longteng_dsi_2_qhd_lcm_drv =
    {
        .name			= "otm9608a_hengsheng_HF_15525_2085_A0_longteng_dsi_2_qhd",
        .set_util_funcs = lcm_set_util_funcs,
        .compare_id    = lcm_compare_id,
        .get_params     = lcm_get_params,
        .init           = lcm_init,
        .suspend        = lcm_suspend,
        .resume         = lcm_resume,
#if (LCM_DSI_CMD_MODE)
        .update         = lcm_update,
#endif
    };
