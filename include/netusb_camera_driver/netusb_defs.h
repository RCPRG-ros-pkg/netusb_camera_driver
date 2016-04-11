#ifndef NETUSB_DEFS
#define NETUSB_DEFS

//---------------------------------------------------------------------------
// Global Defines

#define     IC_SUCCESS          0
#define     IC_ERROR            1

// Additional error codes

#define     IC_IF_NOT_OPEN      -1
#define     IC_WRONG_PARAM      -2
#define     IC_OUT_OF_MEMORY    -3
#define     IC_ALREADY_DONE     -4
#define     IC_WRONG_CLOCK_VAL  -5
#define     IC_COM_LIB_INIT     -6
#define     IC_NOT_IF_STARTED   -7
#define     IC_WRONG_ROI_ID     -8
#define     IC_IF_NOT_ENABLED   -9
#define     IC_COLOR_CAM_ONLY   -10
#define     IC_DRIVER_VERSION   -11
#define     IC_D3D_INIT         -12
#define     IC_BAD_POINTER      -13
#define     IC_ERROR_FILE_SIZE  -14

// Video Mode definitions
// (used with: ICubeSDK_SetMode,ICubeSDK_GetMode)

#define     MODE_320x240        0
#define     MODE_640x480        1
#define     MODE_752x480        2
#define     MODE_800x600        3
#define     MODE_1024x768       4
#define     MODE_1280x1024      5
#define     MODE_1600x1200      6
#define     MODE_2048x1536      7
#define     MODE_2592x1944      8
#define     MODE_3840x2748      9

// BinSkip definitions
// (used with: ICubeSDK_SetBinSkip,ICubeSDK_GetBinSkip,ICubeSDK_GetBinSkipList)

#define     MODE_SKIP            0
#define     MODE_BIN             1
#define     BIN_SKIP_OFF         0
#define     BIN_SKIP_2ND_PIXEL   1
#define     BIN_SKIP_4TH_PIXEL   2


// Parameter definitions 
// (used with: ICubeSDK_SetCamParameter,ICubeSDK_GetCamParameter,
// ICubeSDK_GetCamParameterRange,ICubeSDK_GetParamAuto,ICubeSDK_SetParamAuto,
// ICubeSDK_SetParamAutoDef,ICubeSDK_SetParamOnePush)

// Measure field definitions
// (used with REG_MEASURE_FIELD_AE)

#define MEASUREFIELD_100         2  // 100% (full frame,default)
#define MEASUREFIELD_60          1  // 60%
#define MEASUREFIELD_30          0  // 30%

// Shutter definitions
// (used with REG_SHUTTER)

#define SHUTTER_ROLLING         0   // all models
#define SHUTTER_GLOBAL_RESET    1   // (4133CU/BU,4133CU only)
#define SHUTTER_GLOBAL          2   // (4133CU/BU (default),4133CU (default) only)

// Parameter ON||OFF definitions

#define OFF                     0
#define ON                      1

// Trigger invert definitions
// hardware trigger will be detected on the defined edge
// (used with REG_TRIGGER_INVERT)

#define FALLING_EDGE            0   // (default)
#define RISING_EDGE             1   

// Software trigger mode definitions
// (used with REG_SW_TRIG_MODE)

#define DELAYED_TRIGGER_RETURN   0  // camera checks for frame arrival (default),
                                    // ICubeSDK_SetTrigger(..,TRIG_SW_DO) returns this status
#define IMMEDIATE_TRIGGER_RETURN 1  // no camera check

// Data transmission definitions
// (used with REG_DATA_TRANSMISSION)

#define IS_8BIT                 0 // (default)  
#define IS_16BIT                1           

// Bayer Conversion definitions
// (used with REG_BAYER_CONVERSION)

#define DE_BAYER_3x3            0          
#define DE_BAYER_5x5            1 // (default)           

// Gamma definitions
// (used with REG_GAMMA_MODE)

#define GAMMA_MODE_DEFAULT      2   // (default)   
#define GAMMA_MODE_sRGB         1
#define GAMMA_MODE_ITU709       0   // (HDTV)

// Graphic definitions 
// (used with REG_GRAPHIC_MODE)

#define DISPLAY_GDI             0
#define DISPLAY_D3D             1


// Camera parameters definitions

#define REG_BRIGHTNESS          1   // write: only when REG_COLOR_PROCCESSING enabled, read: always
#define REG_CONTRAST            2   // write: only when REG_COLOR_PROCCESSING enabled, read: always
#define REG_GAMMA               3   // write: only when REG_GAMMA_ENABLE enabled, read: always
#define REG_FLIPPED_V           4   // (ON||OFF); OFF (default)
#define REG_FLIPPED_H           5   // (ON||OFF); OFF (default)
#define REG_WHITE_BALANCE       6   // (one push)
#define REG_EXPOSURE_TIME       7   // all models
#define REG_EXPOSURE_TARGET     8   // sets the target-value for the auto exposure algorithm 
#define REG_RED                 9   // only for color models; RGB Gain value
#define REG_GREEN               10  // only for color models
#define REG_BLUE                11  // only for color models
#define REG_BLACKLEVEL          12  // sensor blacklevel
#define REG_GAIN                13  // sensor gain
#define REG_COLOR               14  // (ON||OFF) read: color || bw model
#define REG_PLL                 15  // all models
#define REG_STROBE_LENGHT       16  // length of strobe pulse output (msec)
#define REG_STROBE_DELAY        17  // delay before strobe pulse is executed (msec)
#define REG_TRIGGER_DELAY       18  // delay before hardware trigger is executed (msec)
#define REG_SATURATION          19  // only for color models, when REG_COLOR_PROCCESSING enabled 
#define REG_COLOR_ENHANCE       20  // obsolet
#define REG_TRIGGER_INVERT      21  // (FALLING_EDGE (default) || RISING_EDGE)
#define REG_RECONNECTIONS       22  // get the value of usb reconnections
#define REG_MEASURE_FIELD_AE    23  // measure field for auto exposure
#define REG_AVI_STATE           24  // get the avi stream save status (ON||OFF)
#define REG_FOCUS               25  // not implemented yet
#define REG_SHUTTER             26  // (4133CU/BU,4133CU)
#define REG_ROI_ID              27  // (4133CU/BU,4133CU,fpga board cameras)
#define REG_ROI_CYCLE           28  // (4133CU/BU,4133CU) number of repetitions of the particular roi

#define REG_TRIGGER             29

#define REG_DEFECT_COR          43  // (4133CU/BU,4133CU) DefectPixelCorrection (ON||OFF); OFF(default)
#define REG_BAD_FRAME           81  // read bad frame count (also possible with ICubeSDK_GetBrokenFrames)
#define REG_GOOD_FRAME          82  // read good frame count (also possible with ICubeSDK_GetGoodFrames)
#define REG_SW_TRIG_MODE        94  // (DELAYED_TRIGGER_RETURN (default) || IMMEDIATE_TRIGGER_RETURN)
#define REG_ROI_FPGA_ONE_FRAME  96  // (fpga board only) pseudo frame format (ON||OFF)  
#define REG_CALLBACK_BR_FRAMES  97  // Broken frames also triggering the callback function (ON||OFF); OFF(default)
#define REG_FGPA_VBLANKING      98  // (fpga board only) output Frame blanking
#define REG_FGPA_HBLANKING      99  // (fpga board only) output Frame blanking
#define REG_FGPA_CLK_DIVIDER    100 // (fpga board only) output clock divider (ON||OFF)
#define REG_FGPA_ON_BOARD       101 // check if camera has fpga board (ON||OFF)

#define REG_SET_GPIO            102 // bit masked: XXXX X J3-7 J3-6 J3-5
#define REG_GET_GPIO            103 // bit masked: XXXX X J3-7 J3-6 J3-5
#define REG_SET_GPIO_MODE       104 // (0==Input || 1==Output),bit masked: XXXX X J3-7 J3-6 J3-5

#define REG_MASK_ROI_ID         105 // roi_id masked in buffersize of callback 

#define REG_RED_OFFSET          106 // only for color models; RGB offset value
#define REG_GREEN_OFFSET        107 // only for color models
#define REG_BLUE_OFFSET         108 // only for color models

#define REG_HUE                 109 // only for color models; write: only when REG_COLOR_PROCCESSING enabled, read: always
#define REG_COLOR_CORRECTION    110 // (ON||OFF) ColorCorrection matrix; OFF (default)
#define REG_GAMMA_ENABLE        111 // (ON||OFF) en/disable gamma module to save CPU load; ON (default)
#define REG_GAMMA_MODE          112 // only for color models ; write: only when REG_GAMMA_ENABLE enabled, read: always
#define REG_INVERT_PIXEL        113 // (ON||OFF)  bw cams only
#define REG_TNR                 115 // (ON||OFF)  only for color models, temporal noise reduction
#define REG_BAYER_CONVERSION    116 // only for color models; choose different debayering
#define REG_COLOR_PROCESSING    117 // (ON||OFF) en/disable color proc module module to save CPU load;  ON (default)    
#define REG_USB_SPEED           118 // 0: usb port is high speed, 1: usb port is super speed capable (useful for super speed camera)
#define REG_DEVICE_SPEED        119 // 0: camera is high speed, 1: camera is super speed capable
#define REG_DATA_TRANSMISSION   120 // 0: 8 bit per pixel, 1: 16 bit per pixel in raw mode
#define REG_SENSOR_TIMING       123 // (4133CU/BU,4133CU)
#define REG_SIGNIFICANT_BITS    125 // number of significant bits in pixel 
#define REG_GRAPHIC_MODE        126 // GDI (default)|| D3D 
#define REG_EDGE_ENHANCEMENT    128 // (ON||OFF) edge enhancement; OFF (default)
#define REG_EDGE_ENHANCEMENT_GAIN   129 // 0-128 (default 64)

#define PARAM_MAX               129

#endif // NETUSB_DEFS
