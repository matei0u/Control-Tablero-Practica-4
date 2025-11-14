/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : Stone_HMI_CmdSystem.c
  * Description        : Code for Stone HMI System CMD applications
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

/* Includes ------------------------------------------------------------------*/


/* USER CODE BEGIN Includes */
#include "Stone_HMI_Define.h"
#include <Arduino.h>
// #include "Thread_Define.h"
//#include "yyjson.h"
//#include "usart.h"

/* USER CODE END Includes */


/* USER CODE BEGIN Stone_HMI_CmdSystem */

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Sys_Reboot
**  @Description        Stone HMI Set HMI reboot, System CMD
**  @InputParam         NONE
**  @OutputParam		NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Sys_Reboot(void)
{
    char ret = 0;

    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SYS_REBOOT);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_CMD_TYPE_SYSTEM);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* {"cmd_code":"sys_reboot","type":"system"} */
//            printf("%s JSON: %s\n",__func__ , json);
            ret = Stone_HMI_Cmd_Send(json);
            free((void *)json);
        }
        else
        {
          Serial.print(__func__);
          Serial.println(" JSON create err");
        }
    }
    // Free the doc
    yyjson_mut_doc_free(doc);
		return ret;
}

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Sys_Hello
**  @Description        Stone HMI Detection communication
**  @InputParam         NONE
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Sys_Hello(void)
{
    char ret = 0;

    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SYS_HELLO);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_CMD_TYPE_SYSTEM);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* {"cmd_code":"sys_hello","type":"system"} */
//            printf("%s JSON: %s\n",__func__ , json);
            ret = Stone_HMI_Cmd_Send(json);
            free((void *)json);
        }
        else
        {
          Serial.print(__func__);
          Serial.println(" JSON create err");
        }
    }
    // Free the doc
    yyjson_mut_doc_free(doc);
    return ret;
}

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Sys_Version
**  @Description        Stone HMI Get the GUI version
**  @InputParam         NONE
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Sys_Version(void)
{
    char ret = 0;
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SYS_VERSION);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_CMD_TYPE_SYSTEM);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* {"cmd_code":"sys_version","type":"system"} */
//            printf("%s JSON: %s\n",__func__ , json);
            ret = Stone_HMI_Cmd_Send(json);
            free((void *)json);
        }
        else
        {
          Serial.print(__func__);
          Serial.println(" JSON create err");
        }
    }
    // Free the doc
    yyjson_mut_doc_free(doc);
    return ret;
}

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Sleep
**  @Description        Stone HMI Set LCD sleep, System CMD
**  @InputParam         sleep:	  true or false  
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Sleep(bool sleep)
{
  char ret = 0;

  // Create a mutable doc
  yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
  yyjson_mut_val *root = yyjson_mut_obj(doc);
  if(root)
  {
      yyjson_mut_doc_set_root(doc, root);
      yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_SLEEP);
      yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_CMD_TYPE_SYSTEM);
      yyjson_mut_obj_add_bool(doc, root, HMI_WIDGET_PROP_SLEEP, sleep);
      const char *json = yyjson_mut_write(doc, 0, NULL);
      if (json)
      {
          /* {"cmd_code":"set_sleep","type":"system","sleep":true} */
//            printf("%s JSON: %s\n",__func__ , json);
          ret = Stone_HMI_Cmd_Send(json);
          free((void *)json);
      }
      else
      {
        Serial.print(__func__);
        Serial.println(" JSON create err");
      }
  }
  // Free the doc
  yyjson_mut_doc_free(doc);
  return ret;
}

/*********************************************************************************************************
**  @FunctionName               Stone_HMI_Set_Buzzer
**  @Description                Stone HMI Set Set how long the buzzer sounds, System CMD
**  @InputParam                 time:	The duration of the buzzer beeping
**  @OutputParam				        NONE
**  @ReturnValue                0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Buzzer(unsigned short time)
{
    char ret = 0;

    if(time <= 0)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_BUZZER);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_CMD_TYPE_SYSTEM);
        yyjson_mut_obj_add_int(doc, root, HMI_WIDGET_PROP_TIME, time);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* {"cmd_code":"set_buzzer","type":"system","time":100} */
//            printf("%s JSON: %s\n",__func__ , json);
            ret = Stone_HMI_Cmd_Send(json);
            free((void *)json);
        }
        else
        {
          Serial.print(__func__);
          Serial.println(" JSON create err");
        }
    }
    // Free the doc
    yyjson_mut_doc_free(doc);
    return ret;
}

/*********************************************************************************************************
**  @FunctionName               Stone_HMI_Set_Brightness
**  @Description                Stone HMI Set LCD Brigghtness, System CMD
**  @InputParam                 brightness:	    brightness,0-100
**  @OutputParam				        NONE
**  @ReturnValue                0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Brightness(unsigned char brightness)
{
    char ret = 0;

    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_BRIGHTNESS);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_CMD_TYPE_SYSTEM);
        yyjson_mut_obj_add_int(doc, root, HMI_WIDGET_PROP_BRIGHTNESS, brightness);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* {"cmd_code":"set_brightness","type":"system","brightness":100} */
//            printf("%s JSON: %s\n",__func__ , json);
            ret = Stone_HMI_Cmd_Send(json);
            free((void *)json);
        }
        else
        {
          Serial.print(__func__);
          Serial.println(" JSON create err");
        }
    }
    // Free the doc
    yyjson_mut_doc_free(doc);
    return ret;
}

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Touch_Cal
**  @Description        Stone HMI Set LCD calibration
**  @InputParam         NONE 
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Touch_Cal(void)
{
    char ret = 0;

    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_TOUCH_CAL);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_CMD_TYPE_SYSTEM);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* {"cmd_code":"set_touch_cal","type":"system"} */
//            printf("%s JSON: %s\n",__func__ , json);
            ret = Stone_HMI_Cmd_Send(json);
            free((void *)json);
        }
        else
        {
          Serial.print(__func__);
          Serial.println(" JSON create err");
        }
    }
    // Free the doc
    yyjson_mut_doc_free(doc);
    return ret;
}

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Clear_Touch_Cal
**  @Description        Stone HMI Clear LCD calibration, System CMD
**  @InputParam         NONE 
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Clear_Touch_Cal(void)
{
    char ret = 0;

    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_CLEAR_TOUCH_CAL);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_CMD_TYPE_SYSTEM);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* {"cmd_code":"clear_touch_cal","type":"system"} */
//            printf("%s JSON: %s\n",__func__ , json);
            ret = Stone_HMI_Cmd_Send(json);
            free((void *)json);
        }
        else
        {
          Serial.print(__func__);
          Serial.println(" JSON create err");
        }
    }
    // Free the doc
    yyjson_mut_doc_free(doc);
    return ret;
}

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Touch_Test
**  @Description        Stone HMI Set LCD Touch screen test, System CMD
**  @InputParam         NONE  
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Touch_Test(void)
{
    char ret = 0;

    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_TOUCH_TEST);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_CMD_TYPE_SYSTEM);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* {"cmd_code":"set_touch_test","type":"system"} */
//            printf("%s JSON: %s\n",__func__ , json);
            ret = Stone_HMI_Cmd_Send(json);
            free((void *)json);
        }
        else
        {
          Serial.print(__func__);
          Serial.println(" JSON create err");
        }
    }
    // Free the doc
    yyjson_mut_doc_free(doc);
    return ret;
}

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Vol
**  @Description        Stone HMI Set LCD volume, System CMD
**  @InputParam         volume:	    volume,0-100
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Vol(unsigned char volume)
{
    char ret = 0;
    if(volume < 0 || volume > 100)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_VOL);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_CMD_TYPE_SYSTEM);
        yyjson_mut_obj_add_int(doc, root, HMI_WIDGET_PROP_VOL,volume);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* {"cmd_code":"set_vol","type":"system","vol":50} */
//            printf("%s JSON: %s\n",__func__ , json);
            ret = Stone_HMI_Cmd_Send(json);
            free((void *)json);
        }
        else
        {
          Serial.print(__func__);
          Serial.println(" JSON create err");
        }
    }
    // Free the doc
    yyjson_mut_doc_free(doc);
    return ret;
}

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Vol_Inc
**  @Description        Stone HMI Set LCD volume Add, System CMD
**  @InputParam         step:	 step,0-100
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Vol_Inc(unsigned char step)
{
    char ret = 0;
    if(step < 0 || step > 100)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_VOL_INC);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_CMD_TYPE_SYSTEM);
        yyjson_mut_obj_add_int(doc, root, HMI_WIDGET_PROP_STEP,step);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* {"cmd_code":"set_vol_inc","type":"system","step":5} */
//            printf("%s JSON: %s\n",__func__ , json);
            ret = Stone_HMI_Cmd_Send(json);
            free((void *)json);
        }
        else
        {
          Serial.print(__func__);
          Serial.println(" JSON create err");
        }
    }
    // Free the doc
    yyjson_mut_doc_free(doc);
    return ret;
}

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Vol_Dec
**  @Description        Stone HMI Set LCD volume subtract, System CMD
**  @InputParam         step:	 step,0-100
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Vol_Dec(unsigned char step)
{
    char ret = 0;
    if(step < 0 || step > 100)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_VOL_INC);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_CMD_TYPE_SYSTEM);
        yyjson_mut_obj_add_int(doc, root, HMI_WIDGET_PROP_STEP,step);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* {"cmd_code":"set_vol_inc","type":"system","step":5} */
//            printf("%s JSON: %s\n",__func__ , json);
            ret = Stone_HMI_Cmd_Send(json);
            free((void *)json);
        }
        else
        {
          Serial.print(__func__);
          Serial.println(" JSON create err");
        }
    }
    // Free the doc
    yyjson_mut_doc_free(doc);
    return ret;
}

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Vol_Mute
**  @Description        Stone HMI Set LCD mute, System CMD
**  @InputParam         mute:	ture or false
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Mute(bool mute)
{
    char ret = 0;

    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_MUTE);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_CMD_TYPE_SYSTEM);
        yyjson_mut_obj_add_bool(doc, root, HMI_WIDGET_PROP_MUTE,mute);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* {"cmd_code":"set_mute","type":"system","mute":true} */
//            printf("%s JSON: %s\n",__func__ , json);
            ret = Stone_HMI_Cmd_Send(json);
            free((void *)json);
        }
        else
        {
          Serial.print(__func__);
          Serial.println(" JSON create err");
        }
    }
    // Free the doc
    yyjson_mut_doc_free(doc);
    return ret;
}

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Audio_Play
**  @Description        Stone HMI play audio, System CMD
**  @InputParam         name: audio name; .mp3 or .wav
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Audio_Play(const char *name)
{
    char ret = 0;
    if(name == NULL)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_AUDIO_PLAY);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_CMD_TYPE_SYSTEM);
        yyjson_mut_obj_add_str(doc, root, HMI_WIDGET_PROP_AUDIO,name);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* {"cmd_code":"set_audio_play","type":"system","audio":"01.wav"} */
//            printf("%s JSON: %s\n",__func__ , json);
            ret = Stone_HMI_Cmd_Send(json);
            free((void *)json);
        }
        else
        {
          Serial.print(__func__);
          Serial.println(" JSON create err");
        }
    }
    // Free the doc
    yyjson_mut_doc_free(doc);
    return ret;
}

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Audio_Pause
**  @Description        Stone HMI pause audio, System CMD
**  @InputParam         pause:	    ture or false
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Audio_Pause(bool pause)
{
    char ret = 0;

    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_AUDIO_PAUSE);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_CMD_TYPE_SYSTEM);
        yyjson_mut_obj_add_bool(doc, root,HMI_WIDGET_PROP_PAUSE,pause);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* {"cmd_code":"set_audio_pause","type":"system","pause":true} */
//            printf("%s JSON: %s\n",__func__ , json);
            ret = Stone_HMI_Cmd_Send(json);
            free((void *)json);
        }
        else
        {
          Serial.print(__func__);
          Serial.println(" JSON create err");
        }
    }
    // Free the doc
    yyjson_mut_doc_free(doc);
    return ret;
}

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Audio_Stop
**  @Description        Stone HMI stop audio, System CMD
**  @InputParam         NONE
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Audio_Stop(void)
{
    char ret = 0;

    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_AUDIO_STOP);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_CMD_TYPE_SYSTEM);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* {"cmd_code":"set_audio_stop","type":"system"} */
//            printf("%s JSON: %s\n",__func__ , json);
            ret = Stone_HMI_Cmd_Send(json);
            free((void *)json);
        }
        else
        {
          Serial.print(__func__);
          Serial.println(" JSON create err");
        }
    }
    // Free the doc
    yyjson_mut_doc_free(doc);
    return ret;
}

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
char Stone_HMI_Set_Video_Play(int x, int y, const char *name)
{
    char ret = 0;
    if(x <0 || y <0 || name == NULL)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_VIDEO_PLAY);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_CMD_TYPE_SYSTEM);
        yyjson_mut_obj_add_int(doc, root, HMI_WIDGET_PROP_X, x);
        yyjson_mut_obj_add_int(doc, root, HMI_WIDGET_PROP_Y, y);
        yyjson_mut_obj_add_str(doc, root, HMI_WIDGET_PROP_VIDEO, name);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
			/* {"cmd_code":"set_video_play","type":"system","x":50,"y":50,"video":"video.mp4"} */
//            printf("%s JSON: %s\n",__func__ , json);
            ret = Stone_HMI_Cmd_Send(json);
            free((void *)json);
        }
        else
        {
          Serial.print(__func__);
          Serial.println(" JSON create err");
        }
    }
    // Free the doc
    yyjson_mut_doc_free(doc);
    return ret;
}

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Video_Pause
**  @Description        Stone HMI pause video, System CMD
**  @InputParam         pause : true or false
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Video_Pause(bool pause)
{
    char ret = 0;
	
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE,HMI_CMD_SET_VIDEO_PAUSE);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_CMD_TYPE_SYSTEM);
		yyjson_mut_obj_add_bool(doc, root, HMI_WIDGET_PROP_PAUSE, pause);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
			/* {"cmd_code":"set_audio_pause","type":"system","pause":true} */
//            printf("%s JSON: %s\n",__func__ , json);
            ret = Stone_HMI_Cmd_Send(json);
            free((void *)json);
        }
        else
        {
          Serial.print(__func__);
          Serial.println(" JSON create err");
        }
    }
    // Free the doc
    yyjson_mut_doc_free(doc);
    return ret;
}

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Video_Stop
**  @Description        Stone HMI stop video, System CMD
**  @InputParam         NONE
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Video_Stop(void)
{
    char ret = 0;

    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE,HMI_CMD_SET_VIDEO_STOP);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_CMD_TYPE_SYSTEM);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
			/* {"cmd_code":"set_audio_stop","type":"system"} */
//            printf("%s JSON: %s\n",__func__ , json);
            ret = Stone_HMI_Cmd_Send(json);
            free((void *)json);
        }
        else
        {
          Serial.print(__func__);
          Serial.println(" JSON create err");
        }
    }
    // Free the doc
    yyjson_mut_doc_free(doc);
    return ret;
}

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Write_Hex
**  @Description        Stone HMI write hexadecimal data at the end, System CMD
**  @InputParam         name: file name 
**						          array :data array name
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Write_Hex(const char *name, const char *data, unsigned short size)
{
    char ret = 0;
    if(data ==NULL || size <=0 || name == NULL)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
	yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE,HMI_CMD_WRITE_HEX);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_CMD_TYPE_SYSTEM);
        yyjson_mut_obj_add_str(doc, root, HMI_WIDGET_PROP_FILE_NAME, name);
        yyjson_mut_val *data_value = yyjson_mut_arr_with_uint8(doc, (const uint8_t *)data, size);
        if(data_value)
        {
            yyjson_mut_obj_add_val(doc, root, HMI_WIDGET_PROP_DATA,data_value);
        }
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
			/* {"cmd_code":"write_hex","type":"system","file_name":"config.bin","data":[20,40,31,32]} */
//            printf("%s JSON: %s\n",__func__ , json);
            ret = Stone_HMI_Cmd_Send(json);
            free((void *)json);
        }
        else
        {
          Serial.print(__func__);
          Serial.println(" JSON create err");
        }
    }
    // Free the doc
    yyjson_mut_doc_free(doc);
    return ret;
}

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Write_Line
**  @Description        Stone HMI write text data at the end , word wrap, System CMD
**  @InputParam         name : file name 
**						          data : need write text data
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Write_Line(const char *name, const  char *data)
{
    char ret = 0;
    if(name == NULL || data == NULL)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE,HMI_CMD_WRITE_LINE);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_CMD_TYPE_SYSTEM);
        yyjson_mut_obj_add_str(doc, root, HMI_WIDGET_PROP_FILE_NAME, name);
        yyjson_mut_obj_add_str(doc, root, HMI_WIDGET_PROP_TEXT, data);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
			/* {"cmd_code":"write_line","type":"system","file_name":"log.txt","text":"2022-10-20 12:56:34....l"} */
//            printf("%s JSON: %s\n",__func__ , json);
            ret = Stone_HMI_Cmd_Send(json);
            free((void *)json);
        }
        else
        {
          Serial.print(__func__);
          Serial.println(" JSON create err");
        }
    }
    // Free the doc
    yyjson_mut_doc_free(doc);
    return ret;
}

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Write_Text
**  @Description        Stone HMI write text data at the end , not word wrap, System CMD
**  @InputParam         name : file name 
**						          text : need write text data
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Write_Text(const char *name, const char *text)
{
    char ret = 0;
    if(name == NULL || text == NULL)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE,HMI_CMD_WRITE_TEXT);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_CMD_TYPE_SYSTEM);
        yyjson_mut_obj_add_str(doc, root, HMI_WIDGET_PROP_FILE_NAME, name);
        yyjson_mut_obj_add_str(doc, root, HMI_WIDGET_PROP_TEXT, text);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
			/* {"cmd_code":"write_line","type":"system","file_name":"log.txt","text":"2022-10-20 12:56:34....0\n"} */
//            printf("%s JSON: %s\n",__func__ , json);
            ret = Stone_HMI_Cmd_Send(json);
            free((void *)json);
        }
        else
        {
          Serial.print(__func__);
          Serial.println(" JSON create err");
        }
    }
    // Free the doc
    yyjson_mut_doc_free(doc);
    return ret;
}

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
char Stone_HMI_Read_Hex(const char *name, int offest, int len)
{
    char ret = 0;
    if(name == NULL || offest <0 || len <=0)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE,HMI_CMD_READ_HEX);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_CMD_TYPE_SYSTEM);
        yyjson_mut_obj_add_str(doc, root, HMI_WIDGET_PROP_FILE_NAME, name);
        yyjson_mut_obj_add_uint(doc, root, HMI_WIDGET_PROP_OFFSET, offest);
        yyjson_mut_obj_add_uint(doc, root, HMI_WIDGET_PROP_LEN, len);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
			/* {"cmd_code":"read_hex","type":"system","file_name":"config.bin","offset":0,"len":10} */
//            printf("%s JSON: %s\n",__func__ , json);
            ret = Stone_HMI_Cmd_Send(json);
            free((void *)json);
        }
        else
        {
          Serial.print(__func__);
          Serial.println(" JSON create err");
        }
    }
    // Free the doc
    yyjson_mut_doc_free(doc);
    return ret;
}

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Read_Line
**  @Description        Stone HMI Read text data by line; System CMD
**  @InputParam         name : file name 
**						          line: the number of rows read
**  @OutputParam	      NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Read_Line(const char *name, int line)
{
    char ret = 0;
    if(name == NULL || line <0)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE,HMI_CMD_READ_LINE);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_CMD_TYPE_SYSTEM);
        yyjson_mut_obj_add_str(doc, root, HMI_WIDGET_PROP_FILE_NAME, name);
        yyjson_mut_obj_add_uint(doc, root, HMI_WIDGET_PROP_LINE, line);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
			/* {"cmd_code":"read_line","type":"system","file_name":"log.txt","line":10} */
//            printf("%s JSON: %s\n",__func__ , json);
            ret = Stone_HMI_Cmd_Send(json);
            free((void *)json);
        }
        else
        {
          Serial.print(__func__);
          Serial.println(" JSON create err");
        }
    }
    // Free the doc
    yyjson_mut_doc_free(doc);
    return ret;
}

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Get_File_Size
**  @Description        Stone HMI get file size; System CMD
**  @InputParam         name : file name 
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Get_File_Size(const char *name)
{
    char ret = 0;
    if(name == NULL)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE,HMI_CMD_GET_FILE_SIZE);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_CMD_TYPE_SYSTEM);
		    yyjson_mut_obj_add_str(doc, root, HMI_WIDGET_PROP_FILE_NAME, name);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
			/* {"cmd_code":"get_file_size","type":"system","file_name":"log.txt"} */
//            printf("%s JSON: %s\n",__func__ , json);
            ret = Stone_HMI_Cmd_Send(json);
            free((void *)json);
        }
        else
        {
          Serial.print(__func__);
          Serial.println(" JSON create err");
        }
    }
    // Free the doc
    yyjson_mut_doc_free(doc);
    return ret;
}

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Delete_File
**  @Description        Stone HMI delete file; System CMD
**  @InputParam         name : file name 
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Delete_File(const char *name)
{
    char ret = 0;
    if(name == NULL)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_DELETE_FILE);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_CMD_TYPE_SYSTEM);
		    yyjson_mut_obj_add_str(doc, root, HMI_WIDGET_PROP_FILE_NAME, name);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
			/* {"cmd_code":"delete_file","type":"system","file_name":"log.txt"} */
//            printf("%s JSON: %s\n",__func__ , json);
            ret = Stone_HMI_Cmd_Send(json);
            free((void *)json);
        }
        else
        {
          Serial.print(__func__);
          Serial.println(" JSON create err");
        }
    }
    // Free the doc
    yyjson_mut_doc_free(doc);
    return ret;
}

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_Rename
**  @Description        Stone HMI rename file; System CMD
**  @InputParam         name : file name 
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Rename(const char *oldname, const char *newname)
{
    char ret = 0;
    if(oldname == NULL || newname == NULL)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE,HMI_CMD_SET_RENAME);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_CMD_TYPE_SYSTEM);
        yyjson_mut_obj_add_str(doc, root, HMI_WIDGET_PROP_FILE_NAME, oldname);
        yyjson_mut_obj_add_str(doc, root, HMI_WIDGET_PROP_NEW_NAME, newname);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
			/* {"cmd_code":"set_rename","type":"system","file_name":"app.bin","new_name":"con.bin"} */
//            printf("%s JSON: %s\n",__func__ , json);
            ret = Stone_HMI_Cmd_Send(json);
            free((void *)json);
        }
        else
        {
          Serial.print(__func__);
          Serial.println(" JSON create err");
        }
    }
    // Free the doc
    yyjson_mut_doc_free(doc);
    return ret;
}

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Download_file
**  @Description        Stone HMI download file to USB flash drive; System CMD
**  @InputParam         name : file name 
**						          mode: download mode, usually u_disk,support FAT16 and FAT32,but multiple partitions are not supported
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Download_file(const char *name, const char *mode)
{
    char ret = 0;
    if(name == NULL || mode == NULL)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE,HMI_CMD_DOWNLOAD_FILE);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_CMD_TYPE_SYSTEM);
        yyjson_mut_obj_add_str(doc, root, HMI_WIDGET_PROP_FILE_NAME, name);
        yyjson_mut_obj_add_str(doc, root, HMI_WIDGET_PROP_MODE, mode);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
			/* {"cmd_code":"download_file","type":"system","file_name":"app.bin","mode":"u_disk"} */
//            printf("%s JSON: %s\n",__func__ , json);
            ret = Stone_HMI_Cmd_Send(json);
            free((void *)json);
        }
        else
        {
          Serial.print(__func__);
          Serial.println(" JSON create err");
        }
    }
    // Free the doc
    yyjson_mut_doc_free(doc);
    return ret;
}

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Unmount_Disk
**  @Description        Stone HMI Unmount the mounted disk; System CMD
**  @InputParam         NONE
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Unmount_Disk(void)
{
    char ret = 0;

    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE,HMI_CMD_UNMOUNT_DISK);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_CMD_TYPE_SYSTEM);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
			/* {"cmd_code":"unmount_disk","type":"system"} */
//            printf("%s JSON: %s\n",__func__ , json);
            ret = Stone_HMI_Cmd_Send(json);
            free((void *)json);
        }
        else
        {
          Serial.print(__func__);
          Serial.println(" JSON create err");
        }
    }
    // Free the doc
    yyjson_mut_doc_free(doc);
    return ret;
}

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
char Stone_HMI_Write_Hex_Ex(const char *path, const char *name, const char *data, unsigned short size)
{ 
    char ret = 0;
    if(path == NULL || name == NULL || data == NULL || size <0)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
		yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE,HMI_CMD_WRITE_HEX_EX);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_CMD_TYPE_SYSTEM);
        yyjson_mut_obj_add_str(doc, root, HMI_WIDGET_PROP_PATH, path);
        yyjson_mut_obj_add_str(doc, root, HMI_WIDGET_PROP_FILE_NAME, name);
        yyjson_mut_val *data_value = yyjson_mut_arr_with_uint8(doc, (const uint8_t *)data, size);
        if(data_value)
        {
            yyjson_mut_obj_add_val(doc, root, HMI_WIDGET_PROP_DATA,data_value);
        }
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* {"cmd_code":"write_hex_ex","type":"system","path":"/user/bin/","file_name":"config.bin","data":[49,50,51,52,53]} */
//            printf("%s JSON: %s\n",__func__ , json);
            ret = Stone_HMI_Cmd_Send(json);
            free((void *)json);
        }
        else
        {
          Serial.print(__func__);
          Serial.println(" JSON create err");
        }
    }
    // Free the doc
    yyjson_mut_doc_free(doc);
    return ret;
}

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
char Stone_HMI_Write_Line_Ex(const char *path, const char *name, const char *data)
{
    char ret = 0;
    if(path == NULL || name == NULL || data == NULL)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE,HMI_CMD_WRITE_LINE_EX);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_CMD_TYPE_SYSTEM);
        yyjson_mut_obj_add_str(doc, root, HMI_WIDGET_PROP_PATH, path);
        yyjson_mut_obj_add_str(doc, root, HMI_WIDGET_PROP_FILE_NAME, name);
        yyjson_mut_obj_add_str(doc, root, HMI_WIDGET_PROP_TEXT, data);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
			/* {"cmd_code":"write_line_ex","type":"system","path":"/user/text/","file_name":"test.txt","text":"2023-10-20 12:56:34 .....l"} */
//            printf("%s JSON: %s\n",__func__ , json);
            ret = Stone_HMI_Cmd_Send(json);
            free((void *)json);
        }
        else
        {
          Serial.print(__func__);
          Serial.println(" JSON create err");
        }
    }
    // Free the doc
    yyjson_mut_doc_free(doc);
    return ret;
}

/*********************************************************************************************************
**  @FunctionName      	Stone_HMI_Write_Text_Ex
**  @Description        Stone HMI Write text numbers to an external disk;not word wrap; System CMD
**  @InputParam         path: path name 
**						          name: file name 
**						          data: write data 
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Write_Text_Ex(const char *path, const char *name, const char *data)
{
    char ret = 0;
    if(path == NULL || name == NULL || data == NULL)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE,HMI_CMD_WRITE_TEXT_EX);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_CMD_TYPE_SYSTEM);
        yyjson_mut_obj_add_str(doc, root, HMI_WIDGET_PROP_PATH, path);
        yyjson_mut_obj_add_str(doc, root, HMI_WIDGET_PROP_FILE_NAME, name);
        yyjson_mut_obj_add_str(doc, root, HMI_WIDGET_PROP_TEXT, data);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
			/* {"cmd_code":"write_text_ex","type":"system","path":"/user/text/","file_name":"log.txt","text":"2023-10-20 12:56:34 .....l\n"} */
//            printf("%s JSON: %s\n",__func__ , json);
            ret = Stone_HMI_Cmd_Send(json);
            free((void *)json);
        }
        else
        {
          Serial.print(__func__);
          Serial.println(" JSON create err");
        }
    }
    // Free the doc
    yyjson_mut_doc_free(doc);
    return ret;
}

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
char Stone_HMI_Read_Hex_Ex(const char *path, const char *name, int offest, int len)
{
    char ret = 0;
    if(path == NULL || name == NULL || offest < 0 || len < 0)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE,HMI_CMD_READ_HEX_EX);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_CMD_TYPE_SYSTEM);
        yyjson_mut_obj_add_str(doc, root, HMI_WIDGET_PROP_PATH, path);
        yyjson_mut_obj_add_str(doc, root, HMI_WIDGET_PROP_FILE_NAME, name);
        yyjson_mut_obj_add_uint(doc, root,HMI_WIDGET_PROP_OFFSET, offest);
        yyjson_mut_obj_add_uint(doc, root,HMI_WIDGET_PROP_LEN, len);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
			/* {"cmd_code":"read_hex_ex","type":"system","path":"/user/bin/","file_name":"config.bin","offset":25,"len":10} */
//            printf("%s JSON: %s\n",__func__ , json);
            ret = Stone_HMI_Cmd_Send(json);
            free((void *)json);
        }
        else
        {
          Serial.print(__func__);
          Serial.println(" JSON create err");
        }
    }
    // Free the doc
    yyjson_mut_doc_free(doc);
    return ret;
}

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
char Stone_HMI_Read_Line_Ex(const char *path, const char *name, int line)
{
    char ret = 0;
    if(path == NULL || name == NULL || line <= 0)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE,HMI_CMD_READ_LINE_EX);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_CMD_TYPE_SYSTEM);
        yyjson_mut_obj_add_str(doc, root, HMI_WIDGET_PROP_PATH, path);
        yyjson_mut_obj_add_str(doc, root, HMI_WIDGET_PROP_FILE_NAME, name);
        yyjson_mut_obj_add_uint(doc, root, HMI_WIDGET_PROP_LINE, line);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
			/* {"cmd_code":"read_line","type":"system","path":"/user/bin/","file_name":"log.txt","line":25} */
//            printf("%s JSON: %s\n",__func__ , json);
            ret = Stone_HMI_Cmd_Send(json);
            free((void *)json);
        }
        else
        {
          Serial.print(__func__);
          Serial.println(" JSON create err");
        }
    }
    // Free the doc
    yyjson_mut_doc_free(doc);
    return ret;
}

/*********************************************************************************************************
**  @FunctionName      	Stone_HMI_Get_File_Size_Ex
**  @Description        Stone HMI get the size of the external disk; System CMD
**  @InputParam         path: path name 
**						          name: file name 
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Get_File_Size_Ex(const char *path, const char *name)
{
    char ret = 0;
    if(path == NULL || name == NULL)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE,HMI_CMD_GET_FILE_SIZE_EX);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_CMD_TYPE_SYSTEM);
        yyjson_mut_obj_add_str(doc, root, HMI_WIDGET_PROP_PATH, path);
        yyjson_mut_obj_add_str(doc, root, HMI_WIDGET_PROP_FILE_NAME, name);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
			/* {"cmd_code":"get_file_size_ex","type":"system","path":"/user/bin/","file_name":"log.txt"} */
//            printf("%s JSON: %s\n",__func__ , json);
            ret = Stone_HMI_Cmd_Send(json);
            free((void *)json);
        }
        else
        {
          Serial.print(__func__);
          Serial.println(" JSON create err");
        }
    }
    // Free the doc
    yyjson_mut_doc_free(doc);
    return ret;
}

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Delete_File_Ex
**  @Description        Stone HMI delete the external disk; System CMD
**  @InputParam         path: path name 
**						          name: file name 
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Delete_File_Ex(const  char *path, const char *name)
{
    char ret = 0;
    if(path == NULL || name == NULL)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE,HMI_CMD_DELETE_FILE_EX);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_CMD_TYPE_SYSTEM);
        yyjson_mut_obj_add_str(doc, root, HMI_WIDGET_PROP_PATH, path);
        yyjson_mut_obj_add_str(doc, root, HMI_WIDGET_PROP_FILE_NAME, name);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
			/* {"cmd_code":"delete_file_ex","type":"system","path":"/user/bin/","file_name":"cfg.bin"} */
//            printf("%s JSON: %s\n",__func__ , json);
            ret = Stone_HMI_Cmd_Send(json);
            free((void *)json);
        }
        else
        {
          Serial.print(__func__);
          Serial.println(" JSON create err");
        }
    }
    // Free the doc
    yyjson_mut_doc_free(doc);
    return ret;
}

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
char Stone_HMI_Set_Rename_Ex(const char *path, const char *oldname, const char *newname)
{
    char ret = 0;
    if(path == NULL || oldname == NULL || newname == NULL)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE,HMI_CMD_SET_RENAME_EX);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_CMD_TYPE_SYSTEM);
        yyjson_mut_obj_add_str(doc, root, HMI_WIDGET_PROP_PATH, path);
        yyjson_mut_obj_add_str(doc, root, HMI_WIDGET_PROP_FILE_NAME, oldname);
        yyjson_mut_obj_add_str(doc, root, HMI_WIDGET_PROP_NEW_NAME, newname);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
			/* {"cmd_code":"set_rename_ex","type":"system","path":"/user/bin/","file_name":"config.bin","new_name":"app.bin"} */
//            printf("%s JSON: %s\n",__func__ , json);
            ret = Stone_HMI_Cmd_Send(json);
            free((void *)json);
        }
        else
        {
          Serial.print(__func__);
          Serial.println(" JSON create err");
        }
    }
    // Free the doc
    yyjson_mut_doc_free(doc);
    return ret;
}

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
char Stone_HMI_Upload_File(const char *path, const char *name, const char *sort)
{
    char ret = 0;
    if(path == NULL || name ==NULL || sort == NULL)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE,HMI_CMD_UPLOAD_FILE);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_CMD_TYPE_SYSTEM);
        yyjson_mut_obj_add_str(doc, root, HMI_WIDGET_PROP_PATH, path);
        yyjson_mut_obj_add_str(doc, root, HMI_WIDGET_PROP_FILE_NAME, name);
        yyjson_mut_obj_add_str(doc, root, HMI_WIDGET_PROP_SORT, sort);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
			/* {"cmd_code":"upload_file","type":"system","path":"/user/bin/","file_name":"config.bin","sort":"user"} */
//            printf("%s JSON: %s\n",__func__ , json);
            ret = Stone_HMI_Cmd_Send(json);
            free((void *)json);
        }
        else
        {
          Serial.print(__func__);
          Serial.println(" JSON create err");
        }
    }
    // Free the doc
    yyjson_mut_doc_free(doc);
    return ret;
}

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
char Stone_HMI_Request_Upgrade_Firmware(const char *url, const char *md5 , const char *version, int port, int size)
{
    char ret = 0;
    if(url == NULL || port <= 0 || size < 0 || md5 == NULL)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_REQUEST_UPGRADE_FIRMWARE);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_CMD_TYPE_SYSTEM);
        yyjson_mut_obj_add_str(doc, root, HMI_WIDGET_PROP_FILE_URL, url);
        yyjson_mut_obj_add_int(doc, root, HMI_WIDGET_PROP_PORT, port);
        yyjson_mut_obj_add_int(doc, root, HMI_WIDGET_PROP_FILE_SIZE, size);
        yyjson_mut_obj_add_str(doc, root, HMI_WIDGET_PROP_FILE_MD5, md5);
        if(version)
        {
            yyjson_mut_obj_add_str(doc, root, HMI_WIDGET_PROP_SOFTWARE_VERSION, version);
        }
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
			/* {"cmd_code":"request_upgrade_firmware","type":"system","file_url":"192.168.1.43/HTTP/STONE/bin/HMI.bin","port":80,"file_size":2266716,"file_md5":"3516BEDF5A4006D9CA4B501C11A176B7","software_version":"V3.0.02.230308RC"} */
//            printf("%s JSON: %s\n",__func__ , json);
            ret = Stone_HMI_Cmd_Send(json);
            free((void *)json);
        }
        else
        {
          Serial.print(__func__);
          Serial.println(" JSON create err");
        }
    }
    // Free the doc
    yyjson_mut_doc_free(doc);
    return ret;
}

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
char Stone_HMI_Request_Upgrade_File(const char *url,  const char *md5, const char *type, int port, int size)
{
    char ret = 0;
    if(url == NULL || port <= 0 || size < 0 || md5 == NULL || type == NULL)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE,HMI_CMD_REQUEST_UPGRADE_FILE);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_CMD_TYPE_SYSTEM);
        yyjson_mut_obj_add_str(doc, root, HMI_WIDGET_PROP_FILE_URL, url);
        yyjson_mut_obj_add_int(doc, root, HMI_WIDGET_PROP_PORT, port);
        yyjson_mut_obj_add_str(doc, root, HMI_WIDGET_PROP_FILE_TYPE, type);
        yyjson_mut_obj_add_int(doc, root, HMI_WIDGET_PROP_FILE_SIZE, size);
        yyjson_mut_obj_add_str(doc, root, HMI_WIDGET_PROP_FILE_MD5, md5);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
			/* {"cmd_code":"request_upgrade_file","type":"system","file_url":"192.168.1.43/HTTP/STONE/project/default/raw/images/xx/1.jpg","port":80,"file_type":"image","file_size":25577,"file_md5":"E2A9EB41A07F04C0AC88F806CE91CEE2"} */
//            printf("%s JSON: %s\n",__func__ , json);
            ret = Stone_HMI_Cmd_Send(json);
            free((void *)json);
        }
        else
        {
          Serial.print(__func__);
          Serial.println(" JSON create err");
        }
    }
    // Free the doc
    yyjson_mut_doc_free(doc);
    return ret;
}

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
char Stone_HMI_Request_Upgrade_Project(const char *url, const char *md5, const char *type, int port,  int size)
{
    char ret = 0;
    if(url == NULL || port <= 0 || size < 0 || md5 == NULL || type == NULL)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE,HMI_CMD_REQUEST_UPGRADE_PROJECT);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_CMD_TYPE_SYSTEM);
        yyjson_mut_obj_add_str(doc, root, HMI_WIDGET_PROP_FILE_URL, url);
        yyjson_mut_obj_add_int(doc, root, HMI_WIDGET_PROP_PORT, port);
        yyjson_mut_obj_add_str(doc, root, HMI_WIDGET_PROP_FILE_TYPE, type);
        yyjson_mut_obj_add_int(doc, root, HMI_WIDGET_PROP_FILE_SIZE, size);
        yyjson_mut_obj_add_str(doc, root, HMI_WIDGET_PROP_FILE_MD5, md5);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
			/* {"cmd_code":"request_upgrade_file","type":"system","file_url":"192.168.1.43/HTTP/STONE/project/default/raw/images/xx/1.jpg","port":80,"file_type":"image","file_size":25577,"file_md5":"E2A9EB41A07F04C0AC88F806CE91CEE2"} */
//            printf("%s JSON: %s\n",__func__ , json);
            ret = Stone_HMI_Cmd_Send(json);
            free((void *)json);
        }
        else
        {
          Serial.print(__func__);
          Serial.println(" JSON create err");
        }
    }
    // Free the doc
    yyjson_mut_doc_free(doc);
    return ret;
}

/* USER CODE END Stone_HMI_CMD */


