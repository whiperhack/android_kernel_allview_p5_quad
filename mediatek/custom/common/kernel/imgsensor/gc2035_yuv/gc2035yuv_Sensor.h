/*****************************************************************************
 *
 * Filename:
 * ---------
 *   sensor.h
 *
 * Project:
 * --------
 *   DUMA
 *
 * Description:
 * ------------
 *   Header file of Sensor driver
 *
 *
 * Author:
 * -------
 *   Lanking.zhou
 *
 *============================================================================
 *             HISTORY
 * Below this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *------------------------------------------------------------------------------
 * $Revision:$
 * $Modtime:$
 * $Log:$
 *
 * [GC2035YUV V1.0.0]
 * 9.19.2012 Lanking.zhou
 * .First Release
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by GalaxyCoreinc. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
/* SENSOR FULL SIZE */
#ifndef __SENSOR_H
#define __SENSOR_H



typedef enum _GC2035_OP_TYPE_ {
        GC2035_MODE_NONE,
        GC2035_MODE_PREVIEW,
        GC2035_MODE_CAPTURE,
        GC2035_MODE_QCIF_VIDEO,
        GC2035_MODE_CIF_VIDEO,
        GC2035_MODE_QVGA_VIDEO
    } GC2035_OP_TYPE;

extern GC2035_OP_TYPE GC2035_g_iGC2035_Mode;

/* START GRAB PIXEL OFFSET */
#define IMAGE_SENSOR_START_GRAB_X		        2	// 0 or 1 recommended
#define IMAGE_SENSOR_START_GRAB_Y		        2	// 0 or 1 recommended

/* MAX/MIN FRAME RATE (FRAMES PER SEC.) */
#define MAX_FRAME_RATE							15		// Limitation for MPEG4 Encode Only
#define MIN_FRAME_RATE							12

/* SENSOR PIXEL/LINE NUMBERS IN ONE PERIOD */
    #define GC2035_FULL_PERIOD_PIXEL_NUMS  (1940)  // default pixel#(w/o dummy pixels) in UXGA mode
    #define GC2035_FULL_PERIOD_LINE_NUMS   (1238)  // default line#(w/o dummy lines) in UXGA mode
    #define GC2035_PV_PERIOD_PIXEL_NUMS    (970)  // default pixel#(w/o dummy pixels) in SVGA mode
    #define GC2035_PV_PERIOD_LINE_NUMS     (670)   // default line#(w/o dummy lines) in SVGA mode

    /* SENSOR EXPOSURE LINE LIMITATION */
    #define GC2035_FULL_EXPOSURE_LIMITATION    (1236)
    #define GC2035_PV_EXPOSURE_LIMITATION      (671)

/* SENSOR FULL SIZE */
   #define GC2035_IMAGE_SENSOR_FULL_WIDTH	   (1600)
   #define GC2035_IMAGE_SENSOR_FULL_HEIGHT	  (1200)


/* SENSOR PV SIZE */
#define GC2035_IMAGE_SENSOR_PV_WIDTH   (800)
#define GC2035_IMAGE_SENSOR_PV_HEIGHT  (600)

#define GC2035_VIDEO_QCIF_WIDTH   (176)
#define GC2035_VIDEO_QCIF_HEIGHT  (144)

#define GC2035_VIDEO_30FPS_FRAME_LENGTH   (0x29E)
#define GC2035_VIDEO_20FPS_FRAME_LENGTH   (0x3ED)
#define GC2035_VIDEO_15FPS_FRAME_LENGTH   (0x53C)
#define GC2035_VIDEO_10FPS_FRAME_LENGTH   (0x7DA)
 
// SETUP TIME NEED TO BE INSERTED
#define GC2035_IMAGE_SENSOR_PV_INSERTED_PIXELS (390)
#define GC2035_IMAGE_SENSOR_PV_INSERTED_LINES  (9 - 6)

#define GC2035_IMAGE_SENSOR_FULL_INSERTED_PIXELS   (248)
#define GC2035_IMAGE_SENSOR_FULL_INSERTED_LINES    (11 - 2)

#define GC2035_PV_DUMMY_PIXELS			(600)
#define GC2035_VIDEO__CIF_DUMMY_PIXELS  (100)
#define GC2035_VIDEO__QCIF_DUMMY_PIXELS (0)

/* SENSOR SCALER FACTOR */
#define PV_SCALER_FACTOR					    3
#define FULL_SCALER_FACTOR					    1


/* DUMMY NEEDS TO BE INSERTED */
/* SETUP TIME NEED TO BE INSERTED */


/* SENSOR READ/WRITE ID */
	#define GC2035_WRITE_ID							    0x78
	#define GC2035_READ_ID								    0x79

/* SENSOR CHIP VERSION */
//	#define GC2035_SENSOR_ID							0x2035



//s_add for porting
//s_add for porting
//s_add for porting

//export functions
UINT32 GC2035Open(void);
UINT32 GC2035GetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution);
UINT32 GC2035GetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_INFO_STRUCT *pSensorInfo, MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData);
UINT32 GC2035Control(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow, MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData);
UINT32 GC2035FeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId, UINT8 *pFeaturePara,UINT32 *pFeatureParaLen);
UINT32 GC2035Close(void);


//e_add for porting
//e_add for porting
//e_add for porting


#endif /* __SENSOR_H */
