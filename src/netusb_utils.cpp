#include <netusb_camera_driver/netusb_utils.hpp>
#include <netusb_camera_driver/netusb_defs.h>

#include <string>

std::string err2str(int errcode) {
    switch(errcode) {
    case IC_IF_NOT_OPEN:     return "camera interface is not open";
    case IC_WRONG_PARAM:     return "parameter is out of range";
    case IC_OUT_OF_MEMORY:   return "memory could not be allocated";
    case IC_ALREADY_DONE:    return "already done";
    case IC_WRONG_CLOCK_VAL: return "wrong PLL value";
    case IC_COM_LIB_INIT:    return "wrong library called";
    case IC_NOT_IF_STARTED:  return "parameter not usable when video stream is started";
    case IC_WRONG_ROI_ID:    return "wrong roi id number";
    case IC_IF_NOT_ENABLED:  return "parameter not enabled";
    case IC_COLOR_CAM_ONLY:  return "parameter is only for color cameras";
    case IC_DRIVER_VERSION:  return "driver version mismatch";
    case IC_D3D_INIT:        return "D3D init error";
    case IC_BAD_POINTER:     return "bad pointer passed";
    case IC_ERROR_FILE_SIZE: return "file size error";
    default: return "unspecified error.";
    }
}

std::string param2str(int param) {
    switch(param) {
    case REG_BRIGHTNESS:        return "BRIGHTNESS";
    case REG_CONTRAST:          return "CONTRAST";
    case REG_GAMMA:             return "GAMMA";
    case REG_FLIPPED_V:         return "FLIPPED_V";
    case REG_FLIPPED_H:         return "FLIPPED_H";
    case REG_WHITE_BALANCE:     return "WHITE_BALANCE";
    case REG_EXPOSURE_TIME:     return "EXPOSURE_TIME";
    case REG_EXPOSURE_TARGET:   return "EXPOSURE_TARGET";
    case REG_RED:               return "RED";
    case REG_GREEN:             return "GREEN";
    case REG_BLUE:              return "BLUE";
    case REG_BLACKLEVEL:        return "BLACKLEVEL";
    case REG_GAIN:              return "GAIN";
    case REG_COLOR:             return "COLOR";
    case REG_PLL:               return "PLL";
    case REG_STROBE_LENGHT:     return "STROBE_LENGHT";
    case REG_STROBE_DELAY:      return "STROBE_DELAY";
    case REG_TRIGGER_DELAY:     return "TRIGGER_DELAY";
    case REG_SATURATION:        return "SATURATION";
    case REG_COLOR_ENHANCE:     return "COLOR_ENHANCE";
    case REG_TRIGGER_INVERT:    return "TRIGGER_INVERT";
    case REG_RECONNECTIONS:     return "RECONNECTIONS";
    case REG_MEASURE_FIELD_AE:  return "MEASURE_FIELD_AE";
    case REG_AVI_STATE:         return "AVI_STATE";
    case REG_FOCUS:             return "FOCUS";
    case REG_SHUTTER:           return "SHUTTER";
    case REG_ROI_ID:            return "ROI_ID";
    case REG_ROI_CYCLE:         return "ROI_CYCLE";

    case REG_TRIGGER:           return "TRIGGER";

    case REG_DEFECT_COR:        return "DEFECT_COR";
    case REG_BAD_FRAME:         return "BAD_FRAME";
    case REG_GOOD_FRAME:        return "GOOD_FRAME";
    case REG_SW_TRIG_MODE:      return "SW_TRIG_MODE";
    case REG_ROI_FPGA_ONE_FRAME: return "ROI_FPGA_ONE_FRAME";
    case REG_CALLBACK_BR_FRAMES: return "CALLBACK_BR_FRAMES";
    case REG_FGPA_VBLANKING:    return "FGPA_VBLANKING";
    case REG_FGPA_HBLANKING:    return "FGPA_HBLANKING";
    case REG_FGPA_CLK_DIVIDER:  return "FGPA_CLK_DIVIDER";
    case REG_FGPA_ON_BOARD:     return "FGPA_ON_BOARD";

    case REG_SET_GPIO:          return "SET_GPIO";
    case REG_GET_GPIO:          return "GET_GPIO";
    case REG_SET_GPIO_MODE:     return "SET_GPIO_MODE";

    case REG_MASK_ROI_ID:       return "MASK_ROI_ID";

    case REG_RED_OFFSET:        return "RED_OFFSET";
    case REG_GREEN_OFFSET:      return "GREEN_OFFSET";
    case REG_BLUE_OFFSET:       return "BLUE_OFFSET";

    case REG_HUE:               return "HUE";
    case REG_COLOR_CORRECTION:  return "COLOR_CORRECTION";
    case REG_GAMMA_ENABLE:      return "GAMMA_ENABLE";
    case REG_GAMMA_MODE:        return "GAMMA_MODE";
    case REG_INVERT_PIXEL:      return "INVERT_PIXEL";
    case REG_TNR:               return "TNR";
    case REG_BAYER_CONVERSION:  return "BAYER_CONVERSION";
    case REG_COLOR_PROCESSING:  return "COLOR_PROCESSING";
    case REG_USB_SPEED:         return "USB_SPEED";
    case REG_DEVICE_SPEED:      return "DEVICE_SPEED";
    case REG_DATA_TRANSMISSION: return "DATA_TRANSMISSION";
    case REG_SENSOR_TIMING:     return "SENSOR_TIMING";
    case REG_SIGNIFICANT_BITS:  return "SIGNIFICANT_BITS";
    case REG_GRAPHIC_MODE:      return "GRAPHIC_MODE";
    case REG_EDGE_ENHANCEMENT:  return "EDGE_ENHANCEMENT";
    case REG_EDGE_ENHANCEMENT_GAIN: return "EDGE_ENHANCEMENT_GAIN";
    
    default: return "UNKNOWN";
    }
}


char chariftrue(bool param, char ch) {
    return param ? ch : ' ';
}

