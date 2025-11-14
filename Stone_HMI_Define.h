/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    Stone_GUI_Define.h
  * @brief   This file contains all the function prototypes for
  *          the Stone GUI files
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STONE.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STONE_GUI_DEFINE_H__
#define __STONE_GUI_DEFINE_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
// #include "main.h"

/* USER CODE BEGIN Includes */
#include "yyjson.h"

/* USER CODE END Includes */



/* USER CODE BEGIN Private defines */
#define HMI_GUI_LIB_VERSION "V2.1"

/*

V2.0    Added the interface for receiving and sending STONE HMI command sets, and completed the command set interface for window, label, edit and other controls;
V2.1    Added system instruction set, general instructions and control related instruction set, and completed the adaptation of instruction set stable V2.7 version;

*/

/* USER CODE END Private defines */


/* exact-width signed integer types */
//typedef   signed          char int8_t;
//typedef   signed short     int int16_t;
//typedef   signed           int int32_t;
//typedef   signed       __INT64 int64_t;

/* exact-width unsigned integer types */
//typedef unsigned          char uint8_t;
//typedef unsigned short     int uint16_t;
//typedef unsigned           int uint32_t;
//typedef unsigned       __INT64 uint64_t;


#define HMI_MAX_DATA_LEN 256
#define HMI_MAX_SEND_LEN 1024   /* The maximum sending space is 1024 bytes, and the user can adjust the space size according to their own situation */

/* USER CODE BEGIN Prototypes */
#define HMI_DATA_HEARD1    'S'
#define HMI_DATA_HEARD2    'T'
#define HMI_DATA_HEARD3    '<'

#define HMI_DATA_END1      '>'
#define HMI_DATA_END2      'E'
#define HMI_DATA_END3      'T'

/* HMI->MCU SYSTEM CMD */
#define HMI_CMD_ACK_SYS_BOOT_CMD                    0x0000          //Boot cmd, After HMI is successfully started, it is sent,Send three times, 100ms interval
#define HMI_CMD_ACK_SYS_SYS_HELLO                   0x0001          //System return code for communication test command (sys_hello)
#define HMI_CMD_ACK_SYS_SYS_VERSION                 0x0002          //System firmware information return for get_version command
#define HMI_CMD_ACK_SYS_WRITE_HEX                   0x0021          //The response to the write_hex instruction
#define HMI_CMD_ACK_SYS_WRITE_LINE                  0x0023          //The response to the write_line instruction
#define HMI_CMD_ACK_SYS_WRITE_TEXT                  0x0025          //The response to the write_text instruction
#define HMI_CMD_ACK_SYS_READ_HEX                    0x0020          //The response to the read_hex instruction
#define HMI_CMD_ACK_SYS_READ_LINE                   0x0022          //The response to the read_line instruction
#define HMI_CMD_ACK_SYS_GET_FILE_SIZE               0x0024          //The response to the get_file_size instruction
#define HMI_CMD_ACK_SYS_DELETE_FILE                 0x0026          //The response to the delete_file instruction
#define HMI_CMD_ACK_SYS_SET_REANME                  0x0027          //The response to the set_rename instruction
#define HMI_CMD_ACK_SYS_DOWNLOAD_FILE               0x0028          //The response to the download_file instruction
#define HMI_CMD_ACK_SYS_UNMOUNT_DISK                0x002A          //The response to the unmount_disk instruction
#define HMI_CMD_ACK_SYS_WRITE_HEX_EX                0x0031			    //The response to the write_hex_ex instruction
#define HMI_CMD_ACK_SYS_WRITE_LINE_EX               0x0033			    //The response to the write_line_ex instruction
#define HMI_CMD_ACK_SYS_WRITE_TEXT_EX               0x0035			    //The response to the write_text_ex instruction
#define HMI_CMD_ACK_SYS_READ_HEX_EX                 0x0030			    //The response to the read_hex_ex instruction
#define HMI_CMD_ACK_SYS_READ_LINE_EX                0x0032			    //The response to the read_line_ex instruction
#define HMI_CMD_ACK_SYS_GET_FILE_SIZE_EX            0x0034			    //The response to the get_file_ex instruction
#define HMI_CMD_ACK_SYS_DELETE_FILE_EX              0x0036			    //The response to the delete_file_ex instruction
#define HMI_CMD_ACK_SYS_SET_RENAME_EX               0x0037			    //The response to the set_rename_ex instruction
#define HMI_CMD_ACK_SYS_SET_UPLOAD_FILE             0x0038			    //The response to the upload_file instruction

/* HMI->MCU WIDGET COMMON CMD */
#define HMI_CMD_ACK_WIDGET_GET_XY                   0x0400          //The response to the get_xy instruction
#define HMI_CMD_ACK_WIDGET_GET_WH                   0x0401          //The response to the get_wh instruction

/* HMI->MCU WIDGET CMD */
#define HMI_CMD_ACK_BUTTON_SYS                      0x1001          //button status send
#define HMI_CMD_ACK_BUTTON_USER                     0x1002          //button key value send (will be released soon)
#define HMI_CMD_ACK_SWTICH_CLICK                    0x1010          //switch value changed return
#define HMI_CMD_ACK_CHECK_BUTTON_CLICK              0x1020          //check button value changed return
#define HMI_CMD_ACK_RADIO_BUTTON_CLICK              0x1030          //radio button value changed return initiative
#define HMI_CMD_ACK_RADIO_BUTTON_STATE              0x1031          //radio button value changed return passively(by get_checked command from MCU)
#define HMI_CMD_ACK_SLIDER_VALUE_CHANGING           0x1040          //slider value changing return
#define HMI_CMD_ACK_SLIDER_VALUE_CHANGED            0x1041          //slider value changed return
#define HMI_CMD_ACK_PG_BAR_VALUE_CHANGED            0x1050          //progress bar value changed return
#define HMI_CMD_ACK_PG_BAR_PERCENT                  0x1051          //progress bar percentage return passively, by get_percent command from MCU
#define HMI_CMD_ACK_LABLE_GET_TEXT                  0x1060          //label text content return passively (The display will only return the command after received the get_text command from MCU)
#define HMI_CMD_ACK_LABLE_GET_VALUE                 0x1062          //label value return (float type)(while the label was targeted by the set_value function on the button, the code will return once the value is changed)
#define HMI_CMD_ACK_EDIT_GET_TEXT                   0x1070          //edit text return (Initiatively or passively. It can be returned after the edit text changed, or returned by get_text command)
#define HMI_CMD_ACK_EDIT_GET_VALUE_INT              0x1071          //edit data return(int type)
#define HMI_CMD_ACK_EDIT_GET_VALUE_FLOAT            0x1072          //edit data return(float type)
#define HMI_CMD_ACK_TEXT_SELECTOR_GET_TEXT          0x1080          //text selector text content return passively (by get_text command)
#define HMI_CMD_ACK_TEXT_SELECTOR_GET_VALUE         0x1081          //text selector value return initiatively (int type value, can be read by get_value command from MCU)
#define HMI_CMD_ACK_TEXT_SELECTOR_GET_SELECTED      0x1082          //text selector index number return passively (int type value by get_value command from MCU)
#define HMI_CMD_ACK_IMAGE_SYS                       0x1090          //Image key Delivers the system key
#define HMI_CMD_ACK_IMAGE_USER                      0x1091          //Image key Delivers a user-defined key value
#define HMI_CMD_ACK_IMAGE_VALUE_CHANGED             0x1092          //image_value value return (float type, can be returned initiatively or passively)
#define HMI_CMD_ACK_SPIN_BOX_GET_TEXT               0x10A0          //Spin_box text delivery
#define HMI_CMD_ACK_SPIN_BOX_GET_VALUE_INT          0x10A1          //Spin_box value delivered (int type)
#define HMI_CMD_ACK_SPIN_BOX_GET_VALUE_FLOAT        0x10A2          //Spin_box value delivered (float type)
#define HMI_CMD_ACK_COMBO_BOX_GET_TEXT              0x10B0          //combo_box text delivery
#define HMI_CMD_ACK_COMBO_BOX_GET_VALUE_INT         0x10B1          //combo_box value delivered (int type)
#define HMI_CMD_ACK_COMBO_BOX_GET_VALUE_FLOAT       0x10B2          //combo_box value delivered (float type)
#define HMI_CMD_ACK_COMBO_BOX_GET_SELECTED          0x10B8          //combo_box serial number is delivered
#define HMI_CMD_ACK_MLEDIT_GET_TEXT                 0x10C0          //mledit text delivery
#define HMI_CMD_ACK_CHART_VIEW_GET_VALUE            0x10D1          //chart_view value delivered
#define HMI_CMD_ACK_CHART_VIEW_CAPACITY             0x10D2          //chart_view capacity delivery (response to the instruction get_capacity)
#define HMI_CMD_ACK_PG_CIRCLE_GET_VALUE             0x10E0          //progress_circle value delivered (float type)
#define HMI_CMD_ACK_PG_CIRCLE_GET_PERCENT           0x10E1          //progress_circle percentage return passively, by get_percent command from MCU
#define HMI_CMD_ACK_DIGET_CLOCK_GET_DATE            0x10F0          //digit_clock Date + time return (get_date in MCU)
#define HMI_CMD_ACK_HSCROLL_LABEL_GET_TEXT          0x1100          //hscroll_label Text return (the GEt_TEXT instruction is used in the MCU)
#define HMI_CMD_ACK_SLIDE_INDICATOR_GET_VALUE       0x1110          //gets the current value of the indicator (option)
#define HMI_CMD_ACK_SLIDE_VIEW_GET_VIEW             0x1120          //gets the sequence number of the slide_view
#define HMI_CMD_ACK_SLIDE_MENU_GET_VALUE            0x1130          //gets the sequence number of the slide_menu
#define HMI_CMD_ACK_TAB_BUTTON_GET_VALUE            0x1140          //gets the tab_button value
#define HMI_CMD_ACK_TAB_VIEW_GET_VIEW               0x1150          //gets the serial number of the tab_view
#define HMI_CMD_ACK_GET_MIN                         0x1160          //The response to the get_min instruction
#define HMI_CMD_ACK_GET_MAX                         0x1161          //The response to the get_max instruction
#define HMI_CMD_ACK_WINDOW_GET_WINDOW               0x2001          //get the current displayed window name
#define HMI_CMD_ACK_WINDOW_OPENED                   0x2007          //window is opened
#define HMI_CMD_ACK_WINDOW_CLOSED                   0x2008          //window is closed


/* MCU->HMI WIDGET CMD */
#define HMI_HOME_PAGE                   "home_page"
/* CMD define */
#define HMI_CMD_CODE                    "cmd_code"
#define HMI_CMD_TYPE                    "type"
#define HMI_CMD_WIDGET                  "widget"

/* CMD type define */
#define HMI_CMD_TYPE_SYSTEM             "system"
#define HMI_CMD_TYPE_WIDGET             "widget"

/* widget type define */
#define HMI_WIDGET_TYPE_WINDOW   		          "window"
#define HMI_WIDGET_TYPE_LABEL   		          "label"
#define HMI_WIDGET_TYPE_EDIT   			          "edit"
#define HMI_WIDGET_TYPE_SPIN_BOX		          "spin_box"
#define HMI_WIDGET_TYPE_COMBO_BOX   	        "combo_box_ex"
#define HMI_WIDGET_TYPE_MLEDIT			          "mledit"
#define HMI_WIDGET_TYPE_PROGRESS_BAR   	      "progress_bar"
#define HMI_WIDGET_TYPE_PROGRESS_CIRCLE       "progress_circle"
#define HMI_WIDGET_TYPE_HSCROLL_LABEL	        "hscroll_label"
#define HMI_WIDGET_TYPE_TEXT_SELECTOR	        "text_selector"
#define HMI_WIDGET_TYPE_SLIDER   		          "slider"
#define HMI_WIDGET_TYPE_IMAGE   		          "image"
#define HMI_WIDGET_TYPE_IMAGE_VALUE		        "image_value"
#define HMI_WIDGET_TYPE_IMAGE_ANIMATION	      "image_animation"
#define HMI_WIDGET_TYPE_GIF				            "gif"
#define HMI_WIDGET_TYPE_SVG				            "svg"
#define HMI_WIDGET_TYPE_BUTTON			          "button"
#define HMI_WIDGET_TYPE_CHECK_BUTTON	        "check_button"
#define HMI_WIDGET_TYPE_RADIO_BUTTON	        "radio_button"
#define HMI_WIDGET_TYPE_SWITCH			          "switch"
#define HMI_WIDGET_TYPE_DIGIT_CLOCK   	      "digit_clock"
#define HMI_WIDGET_TYPE_TIME_CLOCK   	        "time_clock"
#define HMI_WIDGET_TYPE_GAUGE			            "gauge"
#define HMI_WIDGET_TYPE_GAUGE_POINTER	        "gauge_pointer"
#define HMI_WIDGET_X_AXIS				              "x_axis"
#define HMI_WIDGET_Y_AXIS				              "y_axis"
#define HMI_WIDGET_LINE_SERIES			          "line_series"
#define HMI_WIDGET_BAR_SERIES			            "bar_series"
#define HMI_WIDGET_QR					                "qr"
#define HMI_WIDGET_PIE_SLICE			            "pie_slice"
#define HMI_WIDGET_SLIDE_INDICATOR		        "slide_indicator"
#define HMI_WIDGET_SLIDE_INDICATOR_ARC	      "slide_indicator_arc"
#define HMI_WIDGET_SLIDE_VIEW			            "slide_view"
#define	HMI_WIDGET_SLIDE_MENU			            "slide_menu"
#define HMI_WIDGET_TAB_BUTTON			            "tab_button"
#define HMI_WIDGET_TAB_VIEW				            "tab_view"
#define	HMI_WIDGET_SCROLL_VIEW			          "scroll_view"
#define	HMI_WIDGET_LIST_VIEW			            "list_view"
#define HMI_WIDGET_LIST_VIEW_H			          "list_view_h"


/* widget properties */
#define HMI_WIDGET_PROP_TIME            "time"
#define HMI_WIDGET_PROP_IMAGW           "image"
#define HMI_WIDGET_PROP_TEXT            "text"
#define HMI_WIDGET_PROP_VALUE           "value"
#define HMI_WIDGET_PROP_FORMAT          "format"
#define HMI_WIDGET_PROP_ENABLE          "enable"
#define HMI_WIDGET_PROP_VISIBLE         "visible"
#define HMI_WIDGET_PROP_STATE           "state"
#define HMI_WIDGET_PROP_BRIGHTNESS      "brightness"
#define HMI_WIDGET_PROP_DATE            "date"
#define HMI_WIDGET_PROP_SLEEP           "sleep"
#define HMI_WIDGET_PROP_VOL             "vol"
#define HMI_WIDGET_PROP_STEP 			      "step"
#define HMI_WIDGET_PROP_MUTE 			      "mute"
#define HMI_WIDGET_PROP_AUDIO 			    "audio"
#define HMI_WIDGET_PROP_PAUSE 			    "pause"
#define HMI_WIDGET_PROP_X   			      "x"
#define HMI_WIDGET_PROP_Y   			      "y"
#define HMI_WIDGET_PROP_VIDEO 			    "video"
#define HMI_WIDGET_PROP_FILE_NAME 		  "file_name"
#define HMI_WIDGET_PROP_DATA 			      "data"
#define HMI_WIDGET_PROP_OFFSET 			    "offset"
#define HMI_WIDGET_PROP_LEN  			      "len"
#define HMI_WIDGET_PROP_LINE 			      "line"
#define HMI_WIDGET_PROP_NEW_NAME 		    "new_name"
#define HMI_WIDGET_PROP_MODE 			      "mode"
#define HMI_WIDGET_PROP_PATH 			      "path"
#define HMI_WIDGET_PROP_SORT 			      "sort"
#define HMI_WIDGET_PROP_WIDTH  			    "width"
#define HMI_WIDGET_PROP_FG_IMAGE 		    "fg_image"
#define HMI_WIDGET_PROP_BG_IMAGE 		    "bg_image"
#define HMI_WIDGET_PROP_FONT  			    "font"
#define HMI_WIDGET_PROP_SIZE  			    "size"
#define HMI_WIDGET_PROP_ALIGN_H 		    "align_h"
#define HMI_WIDGET_PROP_ALIGN_V 		    "align_v"
#define HMI_WIDGET_PROP_COLOR_OBJECT	  "color_object"
#define HMI_WIDGET_PROP_COLOR  		      "color"
#define HMI_WIDGET_PROP_FILE_URL        "file_url"
#define HMI_WIDGET_PROP_PORT            "port"
#define HMI_WIDGET_PROP_FILE_SIZE       "file_size"
#define HMI_WIDGET_PROP_FILE_MD5        "file_md5"
#define HMI_WIDGET_PROP_SOFTWARE_VERSION "software_version"
#define HMI_WIDGET_PROP_FILE_TYPE       "file_type"
#define HMI_WIDGET_PROP_SELECTED_INDEX 	"selected_index"
#define HMI_WIDGET_PROP_OPTIONS			    "options"
#define HMI_WIDGET_PROP_MAX				      "max"
#define HMI_WIDGET_PROP_MIN				      "min"
#define HMI_WIDGET_PROP_SHOW_TEXT	      "show_text"
#define HMI_WIDGET_PROP_LOOP			      "loop"
#define HMI_WIDGET_PROP_YOYO			      "yoyo"
#define HMI_WIDGET_PROP_DIRECTION	      "direction"
#define HMI_WIDGET_PROP_DURATION 		    "duration"
#define HMI_WIDGET_PROP_STEP			      "step"
#define HMI_WIDGET_PROP_DRAW_TYPE		    "draw_type"
#define HMI_WIDGET_PROP_SCALE			      "scale"
#define HMI_WIDGET_PROP_SCALE_X			    "scale_x"
#define HMI_WIDGET_PROP_SCALE_Y			    "scale_y"
#define HMI_WIDGET_PROP_ROTATION		    "rotation"
#define HMI_WIDGET_PROP_FORMAT			    "format"
#define HMI_WIDGET_PROP_INTERVAL		    "interval"
#define HMI_WIDGET_PROP_START_INDEX		  "start_index"
#define HMI_WIDGET_PROP_END_INDEX		    "end_index"
#define HMI_WIDGET_PROP_FRAME			      "frame"
#define HMI_WIDGET_PROP_ANGLE			      "angle"
#define HMI_WIDGET_PROP_DATA			      "data"
#define HMI_WIDGET_PROP_SHOW			      "show"
#define	HMI_WIDGET_PROP_SMOOTH			    "smooth"
#define HMI_WIDGET_PROP_CAPACITY		    "capacity"
#define HMI_WIDGET_PROP_RADIUS			    "radius"
#define HMI_WIDGET_PROP_SIZE			      "size"
#define HMI_WIDGET_PROP_SPACING			    "spacing"
#define HMI_WIDGET_PROP_INDEX			      "index"
#define HMI_WIDGET_PROP_AUTO_PLAY		    "auto_play"
#define HMI_WIDGET_PROP_SCALE			      "scale"
#define HMI_WIDGET_PROP_ALIGN_V			    "align_v"
#define HMI_WIDGET_PROP_XOFFSET		      "xoffset"
#define HMI_WIDGET_PROP_YOFFSET		      "yoffset"
#define HMI_WIDGET_PROP_HEIGHT			    "height"
#define HMI_WIDGET_PROP_WIDTH			      "width"
#define HMI_WIDGET_PROP_MODE			      "mode"
#define HMI_WIDGET_PROP_PUSH			      "push"

/* system cmd type define */
#define HMI_CMD_SYS_REBOOT                      "sys_reboot"             /* Set HMI reboot */
#define HMI_CMD_SYS_HELLO                       "sys_hello"              /* Detection HMI communication */
#define HMI_CMD_SYS_VERSION                     "sys_version"            /* Get the GUI version */
#define HMI_CMD_SET_SLEEP                       "set_sleep"              /* Set HMI sleep */
#define HMI_CMD_SET_BUZZER                      "set_buzzer"             /* Set the buzzer to ring. */
#define HMI_CMD_SET_BRIGHTNESS                  "set_brightness"         /* Set HMI LCD brightness, 0--100 */
#define HMI_CMD_SET_TOUCH_CAL                   "set_touch_cal"          /* Set LCD calibration */
#define HMI_CMD_CLEAR_TOUCH_CAL                 "clear_touch_cal"        /* Clear LCD calibration */
#define HMI_CMD_SET_TOUCH_TEST                  "set_touch_test"         /* Set LCD Touch screen test */
#define HMI_CMD_SET_VOL                         "set_vol"                /* Set LCD volume */
#define HMI_CMD_SET_VOL_INC                     "set_vol_inc"            /* Set LCD volume Add */
#define HMI_CMD_SET_VOL_DEC                     "set_vol_dec"            /* Set LCD volume subtract */
#define HMI_CMD_SET_MUTE                        "set_mute"               /* Set LCD mute */
#define HMI_CMD_SET_AUDIO_PLAY                  "set_audio_play"         /* HMI play audio */
#define HMI_CMD_SET_AUDIO_PAUSE                 "set_audio_pause"        /* HMI pause audio */
#define HMI_CMD_SET_AUDIO_STOP                  "set_audio_stop"         /* HMI stop audio */
#define HMI_CMD_SET_VIDEO_PLAY                  "set_video_play"         /* HMI play video */
#define HMI_CMD_SET_VIDEO_PAUSE                 "set_video_pause"        /* HMI pause video */
#define HMI_CMD_SET_VIDEO_STOP                  "set_video_stop"         /* HMI stop video */
#define HMI_CMD_WRITE_HEX                       "write_hex"              /* write hexadecimal data at the end */
#define HMI_CMD_WRITE_LINE                      "write_line"             /* write text data at the end,word wrap */
#define HMI_CMD_WRITE_TEXT                      "write_text"             /* write text data at the end,not word wrap */
#define HMI_CMD_READ_HEX                        "read_hex"               /* Read data in hexadecimal from */
#define HMI_CMD_READ_LINE                       "read_line"              /* Read text data by line */
#define HMI_CMD_GET_FILE_SIZE                   "get_file_size"          /* get file size */
#define HMI_CMD_DELETE_FILE                     "delete_file"            /* delete file */
#define HMI_CMD_SET_RENAME                      "set_rename"             /* rename file */
#define HMI_CMD_DOWNLOAD_FILE                   "download_file"          /* download file to USB flash drive */
#define HMI_CMD_UNMOUNT_DISK                    "unmount_disk"           /* Unmount the mounted disk */
#define HMI_CMD_WRITE_HEX_EX                    "write_hex_ex"           /* Write hexadecimal numbers to an external disk */
#define HMI_CMD_WRITE_LINE_EX                   "write_line_ex"          /* Write text numbers to an external disk;word wrap; */
#define HMI_CMD_WRITE_TEXT_EX                   "write_text_ex"          /* Write text numbers to an external disk;not word wrap; */
#define HMI_CMD_READ_HEX_EX                     "read_hex_ex"            /* Read hexadecimal data from an external disk */
#define HMI_CMD_READ_LINE_EX                    "read_line_ex"           /* Read text data by line from an external disk */
#define HMI_CMD_GET_FILE_SIZE_EX                "get_file_size_ex"       /* get the size of the external disk */
#define HMI_CMD_DELETE_FILE_EX                  "delete_file_ex"         /* delete the external disk */
#define HMI_CMD_SET_RENAME_EX                   "set_rename_ex"          /* rename the external disk */
#define HMI_CMD_UPLOAD_FILE                     "upload_file"               /* upload file from the external disk */
#define HMI_CMD_REQUEST_UPGRADE_FIRMWARE        "request_upgrade_firmware"  /* remote firmware upgrade */
#define HMI_CMD_REQUEST_UPGRADE_FILE            "request_upgrade_file"      /* remote upgrade of a single file */
#define HMI_CMD_REQUEST_UPGRADE_PROJECT         "request_upgrade_project"   /* remote upgrade of project files */

/* widget general cmd type define */
#define HMI_CMD_SET_ENABLE              "set_enable"             /* Set the enable state of the widgets */
#define HMI_CMD_SET_VISIBLE             "set_visible"            /* Set the visible state of the widgets */
#define HMI_CMD_SET_EVENT               "set_event"              /* Set the event trigger of the widgets */
#define HMI_CMD_SET_XY                  "set_xy"                 /* Set control coordinates */
#define HMI_CMD_GET_XY                  "get_xy"				        /* Get control coordinates */
#define HMI_CMD_GET_WH                  "get_wh"                 /* Get control sizes */
#define HMI_CMD_SET_STATE               "set_state"              /* Set the state of the widgets */
#define HMI_CMD_SET_BORDER_TYPE         "set_border_type"        /* Set control border type */
#define HMI_CMD_SET_BORDER_WIDTH        "set_border_width"       /* Set control border wide */
#define HMI_CMD_SET_FG_IMAGE            "set_fg_image"           /* Set the foreground photo of the control */
#define HMI_CMD_SET_BG_IMAGE            "set_bg_image"           /* Set the background photo of the control */
#define HMI_CMD_SET_FONT                "set_font"               /* Set the control to change the font */
#define HMI_CMD_SET_FONT_SIZE           "set_font_size"          /* Set the control to change the font size */
#define HMI_CMD_SET_TEXT_ALIGN_H        "set_text_align_h"       /* Set font alignment horizontally */
#define HMI_CMD_SET_TEXT_ALIGN_V        "set_text_align_v"       /* Set font alignment vertical */
#define HMI_CMD_SET_COLOR               "set_color"              /* Set control color */
#define HMI_CMD_TAKE_SNAPSHOT           "take_snapshot"          /* Set screenshot */
#define HMI_CMD_SET_TEXT_ARRAY          "set_text_array"         /* Set the widgets(label/edit/...) text to be displayed */
#define HMI_CMD_SET_VALUE_ARRAY         "set_value_array"        /* Set the widgets(label/edit/...) value to be displayed */

/* widget cmd type define */
#define HMI_CMD_OPEN_WIN                "open_win"              /* Open any window. Windows running in the background can also be opened by using this command. */
#define HMI_CMD_CLOSE_WIN               "close_win"             /* Close any window and do not cache the data of the current window, except home_page; */
#define HMI_CMD_BACK_WIN                "back_win"              /* Return to the upper window, close the current window, and do not cache the data of the current window */
#define HMI_CMD_BACK_WIN_TO             "back_win_to"           /* Return to any upper window, and other opened windows will run in the background */
#define HMI_CMD_BACK_HOME               "back_home"             /* Return to the main window(home_page), do not close the previously opened window, and other windows run in the background */
#define HMI_CMD_GET_DISPLAYED_WINDOW    "get_displayed_window"  /* Get the currently displayed window and the main window currently displayed in the foreground (excluding pop-up windows/keyboards) */
#define HMI_CMD_SET_TEXT                "set_text"              /* Set the widget(label/edit/...) text to be displayed */
#define HMI_CMD_SET_VALUE               "set_value"             /* Set the widget(label/edit/...) value to be displayed */
#define HMI_CMD_GET_TEXT			          "get_text"			        /* Get the widght(lable/edit/...) text */
#define HMI_CMD_GET_VALUE			          "get_value"			        /* Get the widght(lable/edit/...) value */
#define HMI_CMD_SET_SELECTED 	          "set_selected"	        /* Set the combo_box current option */
#define HMI_CMD_SET_OPTION 		          "set_option"		        /* Set the combo_box option content */
#define HMI_CMD_GET_SELECTED 	          "get_selected"	        /* Get the combo_box current option */
#define HMI_CMD_SET_MAX				          "set_max"				        /* Set the progress_bar maximun */
#define HMI_CMD_SET_MIN				          "set_min"				        /* Set the progress_bar minimun */
#define HMI_CMD_SHOW_TEXT			          "set_show_text"         /* Set whether the progress bar displays text */
#define HMI_CMD_GET_PERCENT		          "get_percent"		        /* Get the progress bar percentage */
#define HMI_CMD_SET_LOOP			          "set_loop"			        /* Set whether to loop the hscroll label */
#define HMI_CMD_SET_YOYO			          "set_yoyo"			        /* Set whether to rool-back the hscroll label */
#define HMI_CMD_SET_DIRECTION           "set_direction"	        /* Set the scrolling direction of the hscroll label */
#define HMI_CMD_SET_LULL			          "set_lull"			        /* Set the interval between hscroll label scrolls */
#define HMI_CMD_SET_DURATION	          "set_duration"	        /* Set the time for scrolling the hscroll label once */
#define HMI_CMD_SET_STEP			          "set_step"			        /* Set the slider step value */
#define HMI_CMD_SET_IMAGE			          "set_image"			        /* Set the name of the image to be displayed */
#define HMI_CMD_SET_DRAW_TYPE           "set_draw_type"	        /* Set how the picture is drawn */
#define HMI_CMD_SET_SCALE			          "set_scale"			        /* Set the zoom ratio of the picture */
#define HMI_CMD_SET_RORATION	          "set_rotation"	        /* Set the rotation Angle of the picture */
#define HMI_CMD_SET_FORMAT		          "set_format"		        /* The format of the control property */
#define HMI_CMD_SET_PLAY			          "set_play"			        /* Set image playback */
#define HMI_CMD_SET_PAUSE			          "set_pause"			        /* Set the picture to pause */
#define HMI_CMD_SET_STOP			          "set_stop"			        /* Set the picture to stop */
#define HMI_CMD_SET_INTERVAL	          "set_interval"	        /* Set the picture animation interval */
#define HMI_CMD_SET_RANGE			          "set_range"			        /* Set Range */
#define HMI_CMD_SET_FRAME			          "set_frame"			        /* Displays a specific frame picture */
#define HMI_CMD_GET_CHECKED		          "get_checked"		        /* Gets the currently selected radio button */
#define HMI_CMD_SET_DATE                "set_date"              /* Set HMI RTC date and time */
#define HMI_CMD_GET_DATE                "get_date"              /* Get HMI RTC date and time */
#define HMI_CMD_SET_ANGLE			          "set_angle"			        /* Set the rotation Angle of the pointer */
#define HMI_CMD_SET_DATA			          "set_data"			        /* Set the curve sequence point data */
#define HMI_CMD_SET_LINE			          "set_line"			        /* Sets whether the curve sequence boundaries are displayed and smooth */
#define HMI_CMD_SET_AREA			          "set_area"			        /* Sets whether the curve sequence area is displayed */
#define HMI_CMD_SET_SYMBOL		          "set_symbol"		        /* Sets whether the curve sequence point markers are displayed */
#define HMI_CMD_SET_CAPACITY	          "set_capacity"	        /* Set FIFO capacity for curve sequence points */
#define HMI_CMD_GET_CAPACITY	          "get_capacity"	        /* Get FIFO capacity for curve sequence points */
#define HMI_CMD_SET_START_ANGLE	        "set_start_angle"	      /* Set the pie chart starting Angle */
#define HMI_CMD_SET_RADIUS		          "set_radius"		        /* Set the pie circle thickness radius */
#define HMI_CMD_SET_SIZE			          "set_size"			        /* Set indicator size */
#define HMI_CMD_SET_SPACING		          "set_spacing"		        /* Set indicator interval */
#define HMI_CMD_SET_VIEW			          "set_view"			        /* Set the interface number of the current view */
#define HMI_CMD_SET_AUTO_PLAY	          "set_auto_play"	        /* Set the current view to play automatically */
#define HMI_CMD_GET_VIEW			          "get_view"			        /* Get the interface number of the current view */
#define HMI_CMD_SET_ALIGN_V		          "set_align_v"		        /* Set the current menu alignment */
#define HMI_CMD_SET_XSLIDABLE	          "set_xslidable"	        /* Set whether to allow x direction sliding */
#define HMI_CMD_SET_YSLIDABLE	          "set_yslidable"	        /* Set whether to allow y direction sliding */
#define HMI_CMD_SET_SNAP_TO_PAGE	      "set_snap_to_page"	    /* Sets whether to align by page when scrolling */
#define HMI_CMD_SET_MOVE_TO_PAGE	      "set_move_to_page"	    /* Set whether to scroll one page at a time */
#define HMI_CMD_SET_SCROLL_TO			      "set_scroll_to"			    /* Sets scroll to the specified offset */
#define HMI_CMD_SET_SCROLL_DELTA_TO		  "set_scroll_delta_to"	  /* Sets the offset specified by scrolling */
#define HMI_CMD_SET_HEIGHT				      "set_height"			      /* Sets the height of the list item */
#define HMI_CMD_SET_WIDTH				        "set_width"				      /* Sets the width of the list item */


typedef struct
{
    uint16_t    cmd;   							 	    /* Stone HMI CMD code 			 	*/
    uint16_t    len;  							      /* Stone HMI widget data len; only the data len  		*/
//    uint8_t     widget[64];             /*Stone HMI widget name */
    uint8_t     data[HMI_MAX_DATA_LEN]; 	/* Stone HMI cmd data,data type is int float or text 	*/
    uint16_t	  crc;                      /* Stone HMI cmd CRC value */

} HMI_ProtocolMsg;

typedef struct
{
    uint16_t    cmd;   							 	    /* Stone HMI Cmd code 			 	*/
    uint16_t    len;  							      /* Stone HMI widget data len; only the data len  		*/
    uint8_t     widget_name[64];          /*Stone HMI widget name */
    uint8_t     text[HMI_MAX_DATA_LEN]; 	/* Stone HMI cmd data,data type is int float or text 	*/
    uint32_t    value_uint;               /* Stone HMI widget uint value */
    int         value_int;                /* Stone HMI widget int value */
    float       value_float;              /* stone HMI widget float value */


} HMI_WidgetInfo;


static union
{
    float value_float;
    char  data[4];
} HexFloat;

/*********************************************************************************************************
**  @FunctionName               write_hex_to_float
**  @Description                Hex to float value
**  @InputParam                 buf:	hex data
**  @OutputParam				        NONE
**  @ReturnValue                float value
**
*********************************************************************************************************/
float write_hex_to_float(unsigned char* buf);

/*********************************************************************************************************
**  @FunctionName               write_hex_to_int
**  @Description                Hex to int value
**  @InputParam                 buf:	hex data
**  @OutputParam				        NONE
**  @ReturnValue                int value
**
*********************************************************************************************************/
int write_hex_to_int(unsigned char* buf);

/*********************************************************************************************************
**  @FunctionName               CRC16_CCITT
**  @Description                Stone HMI Calculate CRC16 checksum
**  @InputParam                 buf:	data
**                              len:    data len
**  @OutputParam				        NONE
**  @ReturnValue                CRC16 value
**
*********************************************************************************************************/
uint16_t CRC16_CCITT(const uint8_t *buf, size_t len);

/*********************************************************************************************************
**  @FunctionName               calculateCRC16Modbus
**  @Description                Stone HMI Calculate CRC16 checksum
**  @InputParam                 buf:	    data
**                              length:   data len
**  @OutputParam				        NONE
**  @ReturnValue                CRC16 value
**
*********************************************************************************************************/
uint16_t calculateCRC16Modbus(const uint8_t *data, size_t length);

/*********************************************************************************************************
**  @FunctionName               Uart_HMI_Data_Receive
**  @Description                Data reception and processing
**  @InputParam                 NONE
**  @OutputParam				        NONE
**  @ReturnValue                CRC16 value
**
*********************************************************************************************************/
void Uart_HMI_Data_Receive();

/*********************************************************************************************************
**  @FunctionName               calculateCRC16Modbus
**  @Description                Stone HMI Calculate CRC16 checksum
**  @InputParam                 buf:	    data
**                              length:   data len
**  @OutputParam				        NONE
**  @ReturnValue                CRC16 value
**
*********************************************************************************************************/
uint16_t calculateCRC16Modbus(const uint8_t *data, size_t length);

/*********************************************************************************************************
**  @FunctionName               Uart_HMI_Data_Receive
**  @Description                Data reception and processing
**  @InputParam                 NONE
**  @OutputParam				        NONE
**  @ReturnValue                CRC16 value
**
*********************************************************************************************************/
void Uart_HMI_Data_Receive();

/*********************************************************************************************************
**  @FunctionName               Uart_GUI_Data_Analysis
**  @Description                Parsing Uart port related HMI commands
**  @InputParam                 rev:	uart rev data
**                              size:    data len
**  @OutputParam				        NONE
**  @ReturnValue                0:Fail; 1:OK
**
*********************************************************************************************************/
uint8_t Uart_HMI_Data_Analysis(uint8_t *rev, uint16_t size);

/*********************************************************************************************************
**  @FunctionName               Stone_HMI_Cmd_Send
**  @Description                Stone HMI Send CMD
**  @InputParam                 json_buf:		json data
**  @OutputParam				        NONE
**  @ReturnValue                0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Cmd_Send(const char *json_buf);

/*********************************************************************************************************
**  @FunctionName               Stone_HMI_Transmit_Data
**  @Description                Stone HMI Transmit Uart Data
**  @InputParam                 cmd_buf:        Transmit Data buf
**                              json_buf:		json data
**                              json_len:       json data len
**  @OutputParam				        NONE
**  @ReturnValue                0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Transmit_Data(char *cmd_buf, const char *json_buf, unsigned short json_len);




/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Sys_Reboot
**  @Description        Stone HMI Set HMI reboot, System CMD
**  @InputParam         NONE
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Sys_Reboot(void);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Sys_Hello
**  @Description        Stone HMI Detection communication
**  @InputParam         NONE
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Sys_Hello(void);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Sys_Version
**  @Description        Stone HMI Get the GUI version
**  @InputParam         NONE
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Sys_Version(void);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Sleep
**  @Description        Stone HMI Set LCD sleep, System CMD
**  @InputParam         sleep:	  true or false  
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Sleep(bool sleep);

/*********************************************************************************************************
**  @FunctionName               Stone_HMI_Set_Buzzer
**  @Description                Stone HMI Set Set how long the buzzer sounds, System CMD
**  @InputParam                 time:	The duration of the buzzer beeping
**  @OutputParam				        NONE
**  @ReturnValue                0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Buzzer(unsigned short time);

/*********************************************************************************************************
**  @FunctionName               Stone_HMI_Set_Brightness
**  @Description                Stone HMI Set LCD Brigghtness, System CMD
**  @InputParam                 brightness:	    brightness,0-100
**  @OutputParam				        NONE
**  @ReturnValue                0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Brightness(unsigned char brightness);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Touch_Cal
**  @Description        Stone HMI Set LCD calibration
**  @InputParam         NONE 
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Touch_Cal(void);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Clear_Touch_Cal
**  @Description        Stone HMI Clear LCD calibration, System CMD
**  @InputParam         NONE 
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Clear_Touch_Cal(void);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Touch_Test
**  @Description        Stone HMI Set LCD Touch screen test, System CMD
**  @InputParam         NONE  
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Touch_Test(void);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Vol
**  @Description        Stone HMI Set LCD volume, System CMD
**  @InputParam         volume:	    volume,0-100
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Vol(unsigned char volume);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Vol_Inc
**  @Description        Stone HMI Set LCD volume Add, System CMD
**  @InputParam         step:	 step,0-100
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Vol_Inc(unsigned char step);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Vol_Dec
**  @Description        Stone HMI Set LCD volume subtract, System CMD
**  @InputParam         step:	 step,0-100
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Vol_Dec(unsigned char step);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Vol_Mute
**  @Description        Stone HMI Set LCD mute, System CMD
**  @InputParam         mute:	ture or false
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Mute(bool mute);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Audio_Play
**  @Description        Stone HMI play audio, System CMD
**  @InputParam         name: audio name; .mp3 or .wav
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Audio_Play(const char *name);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Audio_Pause
**  @Description        Stone HMI pause audio, System CMD
**  @InputParam         pause:	    ture or false
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Audio_Pause(bool pause);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Audio_Stop
**  @Description        Stone HMI stop audio, System CMD
**  @InputParam         NONE
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Audio_Stop(void);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Video_Play
**  @Description        Stone HMI play video, System CMD
**  @InputParam         x: x-coordinate
**						          y: y-coordinate
**						          name:video name , .mp4,H264
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Video_Play(int x, int y, const char *name);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Video_Pause
**  @Description        Stone HMI pause video, System CMD
**  @InputParam         pause : true or false
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Video_Pause(bool pause);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Video_Stop
**  @Description        Stone HMI stop video, System CMD
**  @InputParam         NONE
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Video_Stop(void);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Write_Hex
**  @Description        Stone HMI write hexadecimal data at the end, System CMD
**  @InputParam         name: file name 
**						          array :data array name
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Write_Hex(const char *name, const char *data, unsigned short size);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Write_Line
**  @Description        Stone HMI write text data at the end , word wrap, System CMD
**  @InputParam         name : file name 
**						          data : need write text data
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Write_Line(const char *name, const char *data);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Write_Text
**  @Description        Stone HMI write text data at the end , not word wrap, System CMD
**  @InputParam         name : file name 
**						text : need write text data
**  @OutputParam		NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Write_Text(const char *name, const char *text);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Read_Hex
**  @Description        Stone HMI Read data in hexadecimal from; System CMD
**  @InputParam         name : file name 
**						          offest :excursion
**						          len: Intercept length
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Read_Hex(const char *name, int offest, int len);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Read_Line
**  @Description        Stone HMI Read text data by line; System CMD
**  @InputParam         name : file name 
**						          line: the number of rows read
**  @OutputParam	      NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Read_Line(const char *name, int line);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Get_File_Size
**  @Description        Stone HMI get file size; System CMD
**  @InputParam         name : file name 
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Get_File_Size(const char *name);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Delete_File
**  @Description        Stone HMI delete file; System CMD
**  @InputParam         name : file name 
**  @OutputParam		NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Delete_File(const char *name);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Rename
**  @Description        Stone HMI rename file; System CMD
**  @InputParam         name : file name 
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Rename(const char *oldname, const char *newname);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Download_file
**  @Description        Stone HMI download file to USB flash drive; System CMD
**  @InputParam         name : file name 
**						          mode: download mode, usually u_disk,support FAT16 and FAT32,but multiple partitions are not supported
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Download_file(const char *name, const char *mode);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Unmount_Disk
**  @Description        Stone HMI Unmount the mounted disk; System CMD
**  @InputParam         NONE
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Unmount_Disk(void);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Write_Hex_Ex
**  @Description        Stone HMI Write hexadecimal numbers to an external disk; System CMD
**  @InputParam         path: path name 
**						          name: file name 
**						          data: data array name 
**						          size:	data nembers
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Write_Hex_Ex(const char *path, const char *name, const char *data, unsigned short size);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Write_Line_Ex
**  @Description        Stone HMI Write text numbers to an external disk;word wrap; System CMD
**  @InputParam         path: path name 
**						          name: file name 
**						          data: write data 
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Write_Line_Ex(const char *path, const char *name, const  char *data);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Read_Hex_Ex
**  @Description        Stone HMI Read hexadecimal data from an external disk; System CMD
**  @InputParam         path: path name 
**						          name: file name 
**						          offset: excursion
**						          len: Intercept length
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Read_Hex_Ex(const  char *path, const char *name, int offest, int len);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Read_Line_Ex
**  @Description        Stone HMI Read text data by line from an external disk; System CMD
**  @InputParam         path: path name 
**     					        name: file name 
**						          line : the number of rows read
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Read_Line_Ex(const char *path, const char *name, int line);

/*********************************************************************************************************
**  @FunctionName      	Stone_HMI_Get_File_Size_Ex
**  @Description        Stone HMI get the size of the external disk; System CMD
**  @InputParam         path: path name 
**						          name: file name 
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Get_File_Size_Ex(const char *path, const char *name);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Delete_File_Ex
**  @Description        Stone HMI delete the external disk; System CMD
**  @InputParam         path: path name 
**						          name: file name 
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Delete_File_Ex(const char *path, const char *name);

/*********************************************************************************************************
**  @FunctionName      	Stone_HMI_Set_Rename_Ex
**  @Description        Stone HMI rename the external disk; System CMD
**  @InputParam         path: path name 
**						          oldname: need rename file name 
**						          newname: rename name 
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Rename_Ex(const char *path, const char *oldname, const char *newname);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Upload_File
**  @Description        Stone HMI upload file from the external disk; System CMD
**  @InputParam         path: path name 
**						          name:  file name 
**						          sort: file type fonts/images/ui/audios/videos/data/user 
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Upload_File(const char *path, const char *name, const char *sort);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Request_Upgrade_Firmware
**  @Description        Stone HMI remote firmware upgrade; System CMD
**  @InputParam         url: file download address
**						          md5: file md5 value
**						          version: file version number
**                      port: file download port
**                      size: file size 
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Request_Upgrade_Firmware(const char *url, const char *md5 , const char *version, int port, int size);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Request_Upgrade_File
**  @Description        Stone HMI remote upgrade of a single file; System CMD
**  @InputParam         url: file download address
**						          md5: file md5 value 
**                      type: file type
**						          port: file download port
**                      size: file size 
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Request_Upgrade_File(const char *url,  const char *md5, const char *type, int port, int size);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Request_Upgrade_Project
**  @Description        Stone HMI remote upgrade of project files; System CMD
**  @InputParam         url: file download address
**						          md5: file md5 value
**                      type: file type
**						          port: file download port
**                      size: file size 
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Request_Upgrade_Project(const char *url, const char *md5, const char *type, int port,  int size);





/*********************************************************************************************************
**  @FunctionName               Stone_HMI_Set_Enable
**  @Description                Stone HMI Set widget enable or disable
**  @InputParam                 widget_name: widget name
**                              bool_value:	 bool value
**  @OutputParam				        NONE
**  @ReturnValue                0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Enable(const char *widget_name, bool bool_value);

/*********************************************************************************************************
**  @FunctionName               Stone_HMI_Set_Visible
**  @Description                Stone HMI Set widget visible
**  @InputParam                 widget_name: widget name
**                              bool_value:	 bool value
**  @OutputParam				        NONE
**  @ReturnValue                0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Visible(const char *widget_name, bool bool_value);

/*********************************************************************************************************
**  @FunctionName               Stone_HMI_Set_Event
**  @Description                Stone HMI Set widget event triggers
**  @InputParam                 widget_name: widget name
**                              bool_value:	 bool value
**  @OutputParam				NONE
**  @ReturnValue                0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Event(const char *widget_name, bool bool_value);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_XY
**  @Description        Stone HMI Set control coordinates
**  @InputParam         widget_name: widget name
**                 		  x: x coordinate
**						          y: y coordinate
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_XY(const char *widget_name, int x, int y);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Get_XY
**  @Description        Stone HMI Get control coordinates
**  @InputParam         widget_name: widget name
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Get_XY(const char *widget_name);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Get_WH
**  @Description        Stone HMI Get control sizes
**  @InputParam         widget_name: widget name		
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Get_WH(const char *widget_name);

/*********************************************************************************************************
**  @FunctionName               Stone_HMI_Set_State
**  @Description                Stone HMI Set widget state
**  @InputParam                 widget_name: widget name
**                              state:	 widget state
**  @OutputParam				        NONE
**  @ReturnValue                0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_State(const char *widget_name, const char * state);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Border_Type
**  @Description        Stone HMI Set control border type
**  @InputParam         widget_name: widget name
**                 	    state: widget state
**						          value: border type;  0: not have border 1: left border 2: right border 4: up border 8: down border 15: all border
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Border_Type(const char *widget_name, const char *state, int value);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Border_Width
**  @Description        Stone HMI Set control border wide
**  @InputParam         widget_name: widget name
**                 		  state: widget state
**						          value: border wide
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Border_Width(const char *widget_name, const char *state, int width);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Fg_Image
**  @Description        Stone HMI Set the foreground photo of the widget
**  @InputParam         widget_name: widget name
**                 		  state: widget state
**						          image_name: image name
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Fg_Image(const char *widget_name, const char *state, const char *image_name);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Bg_Image
**  @Description        Stone HMI Set the background photo of the widget
**  @InputParam         widget_name: widget name
**                 		  state: widget state
**						          image_name: image name
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Bg_Image(const char *widget_name, const char *state, const char *image_name);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Font
**  @Description        Stone HMI Set the widget to change the font
**  @InputParam         widget_name: widget name
**                 		  state: widget state
**						          font_name: font name
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Font(const char *widget_name, const char *state, const char *font_name);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Font_Size
**  @Description        Stone HMI Set the widget to change the font size
**  @InputParam         widget_name: widget name
**                 		  state: widget state
**						          size: font size
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Font_Size(const char *widget_name, const char *state,int size);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Text_Align_H
**  @Description        Stone HMI Set font alignment horizontally
**  @InputParam         widget_name: widget name
**                 		  state: widget state
**						          align: font align 0:not alignment 1:middle alignment 2:left alignment 3:right alignment
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Text_Align_H(const char *widget_name, const char *state, int align);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Text_Align_V
**  @Description        Stone HMI Set font alignment vertical
**  @InputParam         widget_name: widget name
**                 		  state: widget state
**						          align: font align ;0:not alignment 1:middle alignment 2:up alignment 3:down alignment
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Text_Align_V(const char *widget_name, const char *state, int align);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Color
**  @Description        Stone HMI Set widget color
**  @InputParam         widget_name: widget name
**                 		  object: color object
**						          color: color array
**						          size: array size
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Color(const char *widget_name, const char *object, uint64_t color);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Color_Ex
**  @Description        Stone HMI Set widgets color by array
**  @InputParam         widget_name: widget name
**                 		  object: color object
**						          color: color array
**						          size: array size
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Color_Ex(const char *widget_name, const char *object, uint64_t *color, unsigned short size);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Take_Snapshot
**  @Description        Stone HMI Set screenshot
**  @InputParam         widget_name: window name
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Take_Snapshot(const char *window_name);

/*********************************************************************************************************
**  @FunctionName               Stone_HMI_Set_Text_Array
**  @Description                Stone HMI Set widget text by array
**  @InputParam                 widget_name: widget name array
**                              text:	 widget text array
**                              size:	 text array size
**  @OutputParam				        NONE
**  @ReturnValue                0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Text_Array(const char *widget_name[], const char *text[], unsigned short size);

/*********************************************************************************************************
**  @FunctionName               Stone_HMI_Set_Value_Array
**  @Description                Stone HMI Set widget value by array
**  @InputParam                 widget_name: widget name array
**                              value:	 widget value array
**                              size:	 value array size
**  @OutputParam				        NONE
**  @ReturnValue                0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Value_Array(const char *widget_name[], float *value, unsigned short size);




/*********************************************************************************************************
**  @FunctionName               Stone_HMI_Open_Win
**  @Description                Stone HMI open win
**  @InputParam                 win_name:		window name
**  @OutputParam				        NONE
**  @ReturnValue                0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Open_Win(const char *win_name);

/*********************************************************************************************************
**  @FunctionName               Stone_HMI_Close_Win
**  @Description                Stone HMI Close Window
**  @InputParam                 win_name:		window name
**  @OutputParam				        NONE
**  @ReturnValue                0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Close_Win(const char *win_name);

/*********************************************************************************************************
**  @FunctionName               Stone_HMI_Back_Win
**  @Description                Stone HMI Back Window
**  @InputParam                 NONE
**  @OutputParam				        NONE
**  @ReturnValue                0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Back_Win(void);

/*********************************************************************************************************
**  @FunctionName               Stone_HMI_Back_Win_To
**  @Description                Stone HMI Back To Window
**  @InputParam                 win_name:		window name
**  @OutputParam				        NONE
**  @ReturnValue                0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Back_Win_To(const char *win_name);

/*********************************************************************************************************
**  @FunctionName               Stone_HMI_Back_Home
**  @Description                Stone HMI Back to home_page
**  @InputParam                 NONE
**  @OutputParam				        NONE
**  @ReturnValue                0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Back_Home(void);

/*********************************************************************************************************
**  @FunctionName               Stone_HMI_Get_Displayed_Win
**  @Description                Stone HMI get the current displayed main window
**  @InputParam                 NONE
**  @OutputParam				        NONE
**  @ReturnValue                0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Get_Displayed_Win(void);

/*********************************************************************************************************
**  @FunctionName               Stone_HMI_Set_Text
**  @Description                Stone HMI Set text
**  @InputParam                 widget_type: widget type
**                              widget_name: widget name
**                              image_name:	 image name
**  @OutputParam				        NONE
**  @ReturnValue                0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Text(const char *widget_type, const char *widget_name, const char *text);

/*********************************************************************************************************
**  @FunctionName               Stone_HMI_Set_Text_Ex
**  @Description                Stone HMI Set widget text by array
**  @InputParam                 widget_type: widget type
**                              widget_name: widget name
**                              text:	 widget text array
**                              size:	 text array size
**  @OutputParam				        NONE
**  @ReturnValue                0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Text_Ex(const char *widget_type, const char *widget_name, const char *text[], unsigned short size);

/*********************************************************************************************************
**  @FunctionName               Stone_HMI_Set_Value
**  @Description                Stone HMI Set widget value
**  @InputParam                 widget_type: widget type
**                              widget_name: widget name
**                              image_name:	 image name
**                              format:	 value format
**                              value:	 widget value
**  @OutputParam				        NONE
**  @ReturnValue                0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Value(const char *widget_type, const char *widget_name, const char *format, unsigned int value);

/*********************************************************************************************************
**  @FunctionName               Stone_HMI_Set_Value_Ex
**  @Description                Stone HMI Set widget value by array
**  @InputParam                 widget_type: widget type
**                              widget_name: widget name
**                              value:	 widget value
**                              size:	 value size
**  @OutputParam				        NONE
**  @ReturnValue                0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Value_Ex(const char *widget_type, const char *widget_name, float *value, unsigned short size);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Get_Text
**  @Description        Stone HMI Get widget text 
**  @InputParam         widget_type: widget type
**                      widget_name: widget name
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Get_Text(const char *widget_type, const char *widget_name);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Get_Value
**  @Description        Stone HMI Get widget Value
**  @InputParam         widget_type: widget type
**                      widget_name: widget name
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Get_Value(const char *widget_type, const char *widget_name);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Selected
**  @Description        Stone HMI Sets widget(combo_box/text_selector/...) current selection
**  @InputParam         widget_type: widget type
**						          widget_name: widget name
**						          index: widget selected_index
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Selected(const char *widget_type, const char *widget_name, unsigned short index);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Option
**  @Description        Stone HMI Set widget(combo_box/text_selector/...) option content
**  @InputParam         widget_type: widget type
**						          widget_name: widget name
**						          options	: combo_box option content
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Option(const char *widget_type,const char *widget_name,const char *options);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Gelected
**  @Description        Stone HMI Get widget(combo_box/text_selector/...) current option
**  @InputParam         idget_type: widget type
**						          widget_name: widget name
**						          index: combo_box selected_index
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Get_Selected(const char *widget_type, const char *widget_name);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Max
**  @Description        Stone HMI Set widget(progress_bar/progress_circle/slider/...) MAXIMUM
**  @InputParam         widget_type: widget type
**						          widget_name: widget name
**						          max	: widget maximum
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Max(const char *widget_type, const char *widget_name, unsigned int max);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Min
**  @Description        Stone HMI Set widget(progress_bar/progress_circle/slider/...) MINIMUM
**  @InputParam         widget_type: widget type
**						          widget_name: widget name
**						          max	: widget minimum
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Min(const char *widget_type, const char *widget_name, unsigned short min);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Show_Text
**  @Description        Stone HMI Sets whether the widget(progress_bar/progress_circle/...) displays text
**  @InputParam         widget_name: widget name
**						          show_text: whether the progress bar displays text (True or False)
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Show_Text(const char *widget_type, const char *widget_name, bool show_text);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Get_Percent
**  @Description        Stone HMI Gets the widget(progress_bar/progress_circle/...) percentage
**  @InputParam         widget_name: widget name
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Get_Percent(const char *widget_type, const char *widget_name);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Loop
**  @Description        Stone HMI Set widget whether to loop
**  @InputParam         widget_type: widget type
**						          widget_name: widget name
**						          loop: whether to loop the hscroll label
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
//char Stone_HMI_Set_Loop(const char *widget_type, const char *widget_name, bool loop);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Loop
**  @Description        Stone HMI Set the number of frames to be looped
**  @InputParam         widget_type: widget type
**						          widget_name: widget name
**						          loop: the number of frames to be looped
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Loop(const char *widget_type, const char *widget_name, unsigned short loop);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Yoyo
**  @Description        Stone HMI Set whether to rool-back the hscroll label
**  @InputParam         widget_name: widget name
**						          loop: whether to rool-back the hscroll label
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Yoyo(const char *widget_name, bool yoyo);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Direction
**  @Description        Stone HMI Set the scrolling direction of the hscroll label
**  @InputParam         widget_name: widget name
**						          direction: the scrolling direction of the hscroll label
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Direction(const char *widget_name, bool direction);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Lull
**  @Description        Stone HMI Set the interval between hscroll label scrolls
**  @InputParam         widget_name: widget name
**						          lull: the interval between hscroll label scrolls
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Lull(const char *widget_name, unsigned int lull);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Duration
**  @Description        Stone HMI Set the time for scrolling the hscroll label once
**  @InputParam         widget_name: widget name
**						          lull: the time for scrolling the hscroll label once
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Duration(const char *widget_name, unsigned int duration);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Step
**  @Description        Stone HMI Set the slider step value
**  @InputParam         widget_name: widget name
**						          step: step value
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Step(const char *widget_name, unsigned int step);

/*********************************************************************************************************
**  @FunctionName               Stone_HMI_Set_Image
**  @Description                Stone HMI Set the display image
**  @InputParam                 widget_type: widget type
**                              widget_name: widget name
**                              image_name:	 image name
**  @OutputParam				        NONE
**  @ReturnValue                0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Image(const char *widget_type, const char *widget_name, const char *image_name);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Draw_Type
**  @Description        Stone HMI Set how the picture is drawn
**  @InputParam         widget_type: widget type
**						          widget_name: widget name
**						          draw_type: drawing mode
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Draw_Type(const char *widget_type, const char *widget_name, unsigned short draw_type);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Scale
**  @Description        Stone HMI Set the zoom ratio of the picture
**  @InputParam         widget_type: widget type
**						          widget_name: widget name
**						          scale_x: X-axis scaling
**						          scale_y: Y-axis scaling
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Scale(const char *widget_type, const char *widget_name, float scale_x, float scale_y);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Rotation
**  @Description        Stone HMI Set the rotation Angle of the picture
**  @InputParam         widget_type: widget type
**						          widget_name: widget name
**						          scale_x: rotation
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Rotation(const char *widget_type, const char *widget_name, unsigned short rotation);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Format
**  @Description        Stone HMI Set the format of the control property
**  @InputParam         widget_type: widget type
**						          widget_name: widget name
**						          format: the format of the control property
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Format(const char *widget_type, const char *widget_name, const char *format);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Play
**  @Description        Stone HMI Set image animation/gif playback
**  @InputParam         widget_type: widget type
**						          widget_name: widget name
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Play(const char *widget_type, const char *widget_name);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Pause
**  @Description        Stone HMI Set the image animation/gif to pause
**  @InputParam         widget_type: widget type
**						          widget_name: widget name
**  @OutputParam	      NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Pause(const char *widget_type, const char *widget_name);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Stop
**  @Description        Stone HMI Set the image animation/gif to stop
**  @InputParam         widget_type: widget type
**						          widget_name: widget name
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Stop(const char *widget_type, const char *widget_name);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Interval
**  @Description        Stone HMI Set the image animation interval
**  @InputParam         widget_name: widget name
**						          interval: image animation interval
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Interval(const char *widget_name, unsigned short interval);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Range
**  @Description        Stone HMI Set Range
**  @InputParam         widget_type: widget type
**						          widget_name: widget name
**						          start_index: start sequence
**						          end_index: end sequence
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Range(const char *widget_type, const char *widget_name, unsigned short start_index, unsigned short end_index);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Frame
**  @Description        Stone HMI Displays a specific frame picture
**  @InputParam         widget_type: widget type
**						          widget_name: widget name
**						          frame: Picture specific frame number
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Frame(const char *widget_type, const char *widget_name, unsigned short frame);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Get_Checked
**  @Description        Stone HMI Gets the currently selected radio button
**  @InputParam         widget_name: widget name
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Get_Checked(const char *widget_name);

/*********************************************************************************************************
**  @FunctionName               Stone_HMI_Set_Date
**  @Description                Stone HMI Set widget Date
**  @InputParam                 widget_name: widget name
**                              date: rtc date
**  @OutputParam				        NONE
**  @ReturnValue                0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Date(const char *widget_name, const char * date);

/*********************************************************************************************************
**  @FunctionName               Stone_HMI_Get_Date
**  @Description                Stone HMI Get the RTC Date
**  @InputParam                 widget_name: widget name
**  @OutputParam				        NONE
**  @ReturnValue                0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Get_Date(const char *widget_name);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Angle
**  @Description        Stone HMI Set the rotation Angle of the pointer
**  @InputParam         widget_name: widget name
**						          angle: the rotation Angle of the pointer
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Angle(const char *widget_name, unsigned short angle);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Range
**  @Description        Stone HMI Set Range for chart view 
**  @InputParam         widget_type: widget type
**						          widget_name: widget name
**						          start_index: start sequence
**						          end_index: end sequence
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Range_Chart_View(const char *widget_type, const char *widget_name, unsigned int min, unsigned int max);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Data
**  @Description        Stone HMI Set the curve sequence point data
**  @InputParam         widget_type: widget type
**						          widget_name: widget name
**						          data: Scale display value
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Data(const char *widget_type, const char *widget_name, const char *data);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Line
**  @Description        Stone HMI Sets whether the curve sequence boundaries are displayed and smooth
**  @InputParam         widget_name: widget name
**						          show: Whether the curve sequence boundary line is displayed
**						          smooth: Whether the boundary line of the curve sequence is smooth
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Line(const char *widget_name, bool show, bool smooth);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Area
**  @Description        Stone HMI Sets whether the curve sequence area is displayed
**  @InputParam         widget_name: widget name
**						          show: whether the curve sequence area is displayed
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Area(const char *widget_name, bool show);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Symbol
**  @Description        Stone HMI Sets whether the curve sequence point markers are displayed
**  @InputParam         widget_name: widget name
**						          show: whether the curve sequence point markers are displayed
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Symbol(const char *widget_name,bool show);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Series_Set_Value
**  @Description        Stone HMI Set line_series/bar_series value
**  @InputParam         widget_type: widget type
**                      widget_name: widget name
**                      index: line_series value
**                      value: widget value
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Series_Set_Value(const char *widget_type, const char *widget_name, unsigned int index, unsigned int value);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Series_Push_Set_Value
**  @Description        Stone HMI Append data to the end 
**  @InputParam         widget_type: widget type
**                      widget_name: widget name
**                      value: push value
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Series_Push_Set_Value(const char *widget_type, const char *widget_name, unsigned int value);
char STONE_push_series(const char *widget_type, const char *widget_name, unsigned short value);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Series_Get_Value
**  @Description        Stone HMI Append data to the end 
**  @InputParam         widget_type: widget type
**                      widget_name: widget name
**                      value: push value
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Series_Get_Value(const char *widget_type, const char *widget_name, unsigned short index);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Capacity
**  @Description        Stone HMI Set FIFO capacity for curve sequence points
**  @InputParam         widget_type: widget type
**						          widget_name: widget name
**						          capacity: FIFO capacity
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Capacity(const char *widget_type, const char *widget_name, unsigned short capacity);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Get_Capacity
**  @Description        Stone HMI Get FIFO capacity for curve sequence points
**  @InputParam         widget_type: widget type
**						          widget_name: widget name
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Get_Capacity(const char *widget_type, const char *widget_name);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Start_Angle
**  @Description        Stone HMI Set the pie chart starting Angle
**  @InputParam         widget_name: widget name
**						          angle: the pie chart starting Angle
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Start_Angle(const char *widget_name,int angle);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Radius
**  @Description        Stone HMI Set the pie circle thickness radius
**  @InputParam         widget_name: widget name
**						          radius: the pie circle thickness radius
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Radius(const char *widget_name, unsigned short radius);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Size
**  @Description        Stone HMI Set indicator size
**  @InputParam         widget_name: widget name
**						          size: the indicator size
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Size(const char *widget_name, unsigned short size);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Spacing
**  @Description        Stone HMI Set indicator interval
**  @InputParam         widget_type: widget type
**						          widget_name: widget name
**						          spacing: the indicator interval
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Spacing(const char *widget_type, const char *widget_name, unsigned int spacing);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_View
**  @Description        Stone HMI Set the interface number of the current view
**  @InputParam         widget_name: widget name
**						          index: serial number
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_View(const char *widget_name, unsigned short index);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Auto_Play
**  @Description        Stone HMI Set the current view to play automatically
**  @InputParam         widget_name: widget name
**						          auto_play: Autoplay time
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Auto_Play(const char *widget_name, unsigned short auto_play);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Tab_Button_Value
**  @Description        Stone HMI Set widget value
**  @InputParam         widget_type: widget type
**                      widget_name: widget name
**                      format:	value format
**                      value:	widget value
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Tab_Button_Value(const char *widget_type, const char *widget_name, bool value);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Get_View
**  @Description        Stone HMI Get the interface number of the current view
**  @InputParam         widget_type: widget type
**						          widget_name: widget name
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Get_View(const char *widget_type, const char *widget_name);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Slider_Menu_Scale
**  @Description        Stone HMI Sets the current menu scaling ratio
**  @InputParam         widget_name: widget name
**						          scale: the current menu scaling ratio
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Slide_Menu_Scale(const char *widget_name, float scale);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Align_V
**  @Description        Stone HMI Set the current menu alignment
**  @InputParam         widget_name: widget name
**						          align_v: the current menu alignment
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Align_V(const char *widget_name, unsigned short align_v);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Xslidable
**  @Description        Stone HMI Set whether to allow x direction sliding
**  @InputParam         widget_name: widget name
**						          value: whether to allow x direction sliding
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Xslidable(const char *widget_name, bool value);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Yslidable
**  @Description        Stone HMI Set whether to allow y direction sliding
**  @InputParam         widget_name: widget name
**						          value: whether to allow y direction sliding
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Yslidable(const char *widget_name, bool value);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Snap_To_Page
**  @Description        Stone HMI Sets whether to align by page when scrolling
**  @InputParam         widget_name: widget name
**						          value: whether to align by page when scrolling
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Snap_To_Page(const char *widget_name,bool value);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Move_To_Page
**  @Description        Stone HMI Set whether to scroll one page at a time
**  @InputParam         widget_name: widget name
**						          value: whether to scroll one page at a time
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Move_To_Page(const char *widget_name, bool value);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Scroll_To_X
**  @Description        Stone HMI Sets scroll to the specified offset
**  @InputParam         widget_name: widget name
**						          xoffset: Offset in the X direction
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Scroll_To_X(const char *widget_name, int xoffset);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Scroll_To_Y
**  @Description        Stone HMI Sets scroll to the specified offset
**  @InputParam         widget_name: widget name
**						          yoffset: Offset in the Y direction
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Scroll_To_Y(const char *widget_name, int yoffset);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Scroll_To
**  @Description        Stone HMI Sets scroll to the specified offset
**  @InputParam         widget_name: widget name
**						          xoffset: Offset in the X direction
**						          yoffset: Offset in the Y direction
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Scroll_To(const char *widget_name,int xoffset,int yoffset);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Scroll_Delta_To_X
**  @Description        Stone HMI Sets the offset specified by scrolling
**  @InputParam         widget_name: widget name
**						          xoffset: Offset in the X direction
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Scroll_Delta_To_X(const char *widget_name, int xoffset);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Scroll_Delta_To_Y
**  @Description        Stone HMI Sets the offset specified by scrolling
**  @InputParam         widget_name: widget name
**						          yoffset	: Offset in the Y direction
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Scroll_Delta_To_Y(const char *widget_name,int yoffset);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Scroll_Delta_To
**  @Description        Stone HMI Sets scroll to the specified offset
**  @InputParam         widget_name: widget name
**						          xoffset: Offset in the X direction
**						          yoffset: Offset in the Y direction
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Scroll_Delta_To(const char *widget_name, int xoffset, int yoffset);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Height
**  @Description        Stone HMI Sets the height of the list item
**  @InputParam         widget_name: widget name
**						          height: the height of the list item
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Height(const char *widget_name,unsigned short height);

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Width
**  @Description        Stone HMI Sets the width of the list item
**  @InputParam         widget_name: widget name
**						          width: the width of the list item
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Width(const char *widget_name, unsigned short width);



/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __STONE_GUI_DEFINE_H__ */

