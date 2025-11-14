/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : Stone_HMI_CmdGeneral.c
  * Description        : Code for Stone HMI General CMD applications
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


/* USER CODE BEGIN Stone_HMI_CmdGeneral */


/*********************************************************************************************************
**  @FunctionName               Stone_HMI_Set_Enable
**  @Description                Stone HMI Set widget enable or disable
**  @InputParam                 widget_name: widget name
**                              bool_value:	 bool value
**  @OutputParam				        NONE
**  @ReturnValue                0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Enable(const char *widget_name, bool bool_value)
{
    char ret = 0;

    if(widget_name==NULL)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
      yyjson_mut_doc_set_root(doc, root);
      yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_ENABLE);
      yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_CMD_TYPE_WIDGET);
      yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
      yyjson_mut_obj_add_bool(doc, root, HMI_WIDGET_PROP_ENABLE, bool_value);
      const char *json = yyjson_mut_write(doc, 0, NULL);
      if (json)
      {
        /* {"cmd_code":"set_enable","type":"widget","widget":"button1","enable":false} */
        // Serial.print(__func__);
        // Serial.print(" JSON: ");
        // Serial.println(json);
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
**  @FunctionName               Stone_HMI_Set_Visible
**  @Description                Stone HMI Set widget visible
**  @InputParam                 widget_name: widget name
**                              bool_value:	 bool value
**  @OutputParam				        NONE
**  @ReturnValue                0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Visible(const char *widget_name, bool bool_value)
{
    char ret = 0;

    if(widget_name==NULL)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
      yyjson_mut_doc_set_root(doc, root);
      yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_VISIBLE);
      yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_CMD_TYPE_WIDGET);
      yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
      yyjson_mut_obj_add_bool(doc, root, HMI_WIDGET_PROP_VISIBLE, bool_value);
      const char *json = yyjson_mut_write(doc, 0, NULL);
      if (json)
      {
        /* {"cmd_code":"set_visible","type":"widget","widget":"button1","visible":true} */
        // Serial.print(__func__);
        // Serial.print(" JSON: ");
        // Serial.println(json);
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
**  @FunctionName               Stone_HMI_Set_Event
**  @Description                Stone HMI Set widget event triggers
**  @InputParam                 widget_name: widget name
**                              bool_value:	 bool value
**  @OutputParam				NONE
**  @ReturnValue                0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Event(const char *widget_name, bool bool_value)
{
    char ret = 0;

    if(widget_name==NULL)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_EVENT);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_CMD_TYPE_WIDGET);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
        yyjson_mut_obj_add_bool(doc, root, HMI_WIDGET_PROP_ENABLE, bool_value);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* {"cmd_code":"set_event","type":"widget","widget":"button1","enable":true} */
//            printf("%s JSON: %s\n",__func__ , json);
            ret = Stone_HMI_Cmd_Send(json);
            free((void *)json);
        }
        else
        {
            printf("%s JSON create err\n", __func__ );
        }
    }
    // Free the doc
    yyjson_mut_doc_free(doc);
    return ret;
}

/*********************************************************************************************************
**  @FunctionName       Stone_HMI_Set_XY
**  @Description        Stone HMI Set widget coordinates
**  @InputParam         widget_name: widget name
**                 		  x: x coordinate
**						          y: y coordinate
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_XY(const char *widget_name, int x, int y)
{
    char ret = 0;

    if(widget_name == NULL || x < 0 || y  < 0)
    {
      return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
      yyjson_mut_doc_set_root(doc, root);
      yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_XY);
      yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_CMD_TYPE_WIDGET);
      yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
      yyjson_mut_obj_add_int(doc, root, HMI_WIDGET_PROP_X, x);
      yyjson_mut_obj_add_int(doc, root, HMI_WIDGET_PROP_Y, y);
      const char *json = yyjson_mut_write(doc, 0, NULL);
      if (json)
      {
        /* "cmd_code":"set_xy","type":"widget","widget":"slider1","x":-40,"y":240} */
        // Serial.print(__func__);
        // Serial.print(" JSON: ");
        // Serial.println(json);
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
**  @FunctionName       Stone_HMI_Get_XY
**  @Description        Stone HMI Get widget coordinates
**  @InputParam         widget_name: widget name
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Get_XY(const char *widget_name)
{
    char ret = 0;

    if(widget_name == NULL)
    {
      return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
      yyjson_mut_doc_set_root(doc, root);
      yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_GET_XY);
      yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_CMD_TYPE_WIDGET);
      yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
      const char *json = yyjson_mut_write(doc, 0, NULL);
      if (json)
      {
        /* {"cmd_code":"get_xy","type":"widget","widget":"button1"} */
        // Serial.print(__func__);
        // Serial.print(" JSON: ");
        // Serial.println(json);
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
**  @FunctionName       Stone_HMI_Get_WH
**  @Description        Stone HMI Get widget sizes
**  @InputParam         widget_name: widget name		
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Get_WH(const char *widget_name)
{
    char ret = 0;

    if(widget_name == NULL)
    {
      return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
      yyjson_mut_doc_set_root(doc, root);
      yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_GET_WH);
      yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_CMD_TYPE_WIDGET);
      yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
      const char *json = yyjson_mut_write(doc, 0, NULL);
      if (json)
      {
        /* {"cmd_code":"get_wh","type":"widget","widget":"button1"} */
        // Serial.print(__func__);
        // Serial.print(" JSON: ");
        // Serial.println(json);
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
**  @FunctionName               Stone_HMI_Set_State
**  @Description                Stone HMI Set widget state
**  @InputParam                 widget_name: widget name
**                              state:	 widget state
**  @OutputParam				        NONE
**  @ReturnValue                0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_State(const char *widget_name, const char * state)
{
    char ret = 0;

    if(widget_name==NULL || state == NULL)
    {
      return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
      yyjson_mut_doc_set_root(doc, root);
      yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_STATE);
      yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_CMD_TYPE_WIDGET);
      yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
      yyjson_mut_obj_add_str(doc, root, HMI_WIDGET_PROP_STATE, state);
      const char *json = yyjson_mut_write(doc, 0, NULL);
      if (json)
      {
        /* {"cmd_code":"set_state","type":"widget","widget":"button1","state":"pressed"} */
        // Serial.print(__func__);
        // Serial.print(" JSON: ");
        // Serial.println(json);
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
**  @FunctionName       Stone_HMI_Set_Border_Type
**  @Description        Stone HMI Set control border type
**  @InputParam         widget_name: widget name
**                 	    state: widget state
**						          value: border type;  0: not have border 1: left border 2: right border 4: up border 8: down border 15: all border
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Border_Type(const char *widget_name, const char *state, int value)
{
    char ret = 0;

    if(widget_name == NULL || state == NULL || value < 0)
    {
      return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
      yyjson_mut_doc_set_root(doc, root);
      yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_BORDER_TYPE);
      yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_CMD_TYPE_WIDGET);
      yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
      yyjson_mut_obj_add_str(doc, root, HMI_WIDGET_PROP_STATE, state);
      yyjson_mut_obj_add_int(doc, root, HMI_WIDGET_PROP_VALUE,value);
      const char *json = yyjson_mut_write(doc, 0, NULL);
      if (json)
      {
        /* {"cmd_code":"set_border_width","type":"widget","widget":"button1","state":"normal","width":1} */
        // Serial.print(__func__);
        // Serial.print(" JSON: ");
        // Serial.println(json);
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
**  @FunctionName       Stone_HMI_Set_Border_Width
**  @Description        Stone HMI Set control border wide
**  @InputParam         widget_name: widget name
**                 		  state: widget state
**						          value: border wide
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Border_Width(const char *widget_name, const char *state, int width)
{
    char ret = 0;

    if(widget_name == NULL || state == NULL || width < 0)
    {
      return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
      yyjson_mut_doc_set_root(doc, root);
      yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_BORDER_WIDTH);
      yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_CMD_TYPE_WIDGET);
      yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
      yyjson_mut_obj_add_str(doc, root, HMI_WIDGET_PROP_STATE, state);
      yyjson_mut_obj_add_int(doc, root, HMI_WIDGET_PROP_WIDTH,width);
      const char *json = yyjson_mut_write(doc, 0, NULL);
      if (json)
      {
        /* {"cmd_code":"set_border_width","type":"widget","widget":"button1","state":"normal","width":1} */
        // Serial.print(__func__);
        // Serial.print(" JSON: ");
        // Serial.println(json);
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
**  @FunctionName       Stone_HMI_Set_Fg_Image
**  @Description        Stone HMI Set the foreground photo of the widget
**  @InputParam         widget_name: widget name
**                 		  state: widget state
**						          image_name: image name
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Fg_Image(const char *widget_name, const char *state, const char *image_name)
{
    char ret = 0;

    if(widget_name == NULL || state == NULL || image_name == NULL)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_FG_IMAGE);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_CMD_TYPE_WIDGET);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
        yyjson_mut_obj_add_str(doc, root, HMI_WIDGET_PROP_STATE, state);
        yyjson_mut_obj_add_str(doc, root, HMI_WIDGET_PROP_FG_IMAGE,image_name);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
			/* {"cmd_code":"set_fg_image","type":"widget","widget":"button1","state":"normal","fg_image":"01"} */
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
**  @FunctionName       Stone_HMI_Set_Bg_Image
**  @Description        Stone HMI Set the background photo of the widget
**  @InputParam         widget_name: widget name
**                 		  state: widget state
**						          image_name: image name
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Bg_Image(const char *widget_name, const char *state, const char *image_name)
{
    char ret = 0;

    if(widget_name == NULL || state == NULL || image_name == NULL)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_BG_IMAGE);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_CMD_TYPE_WIDGET);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
        yyjson_mut_obj_add_str(doc, root, HMI_WIDGET_PROP_STATE, state);
        yyjson_mut_obj_add_str(doc, root, HMI_WIDGET_PROP_BG_IMAGE,image_name);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
			/* {"cmd_code":"set_bg_image","type":"widget","widget":"button1","state":"normal","bg_image":"01"} */
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
**  @FunctionName       Stone_HMI_Set_Font
**  @Description        Stone HMI Set the widget to change the font
**  @InputParam         widget_name: widget name
**                 		  state: widget state
**						          font_name: font name
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Font(const char *widget_name, const char *state, const char *font_name)
{
    char ret = 0;

    if(widget_name == NULL || state == NULL || font_name == NULL)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_FONT);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_CMD_TYPE_WIDGET);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
        yyjson_mut_obj_add_str(doc, root, HMI_WIDGET_PROP_STATE, state);
        yyjson_mut_obj_add_str(doc, root, HMI_WIDGET_PROP_FONT,font_name);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
			/* {"cmd_code":"set_font","type":"widget","widget":"button1","state":"normal","font":"mysyh"} */
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
**  @FunctionName       Stone_HMI_Set_Font_Size
**  @Description        Stone HMI Set the widget to change the font size
**  @InputParam         widget_name: widget name
**                 		  state: widget state
**						          size: font size
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Font_Size(const char *widget_name, const char *state, int size)
{
    char ret = 0;

    if(widget_name == NULL || state == NULL || size < 0)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_FONT_SIZE);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_CMD_TYPE_WIDGET);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
        yyjson_mut_obj_add_str(doc, root, HMI_WIDGET_PROP_STATE, state);
        yyjson_mut_obj_add_int(doc, root, HMI_WIDGET_PROP_SIZE,size);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
			/* {"cmd_code":"set_font_size","type":"widget","widget":"button1","state":"normal","size":24} */
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
**  @FunctionName       Stone_HMI_Set_Text_Align_H
**  @Description        Stone HMI Set font alignment horizontally
**  @InputParam         widget_name: widget name
**                 		  state: widget state
**						          align: font align 0:not alignment 1:middle alignment 2:left alignment 3:right alignment
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Text_Align_H(const char *widget_name, const char *state, int align)
{
    char ret = 0;

    if(widget_name == NULL || state == NULL || align < 0)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_TEXT_ALIGN_H);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_CMD_TYPE_WIDGET);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
        yyjson_mut_obj_add_str(doc, root, HMI_WIDGET_PROP_STATE, state);
        yyjson_mut_obj_add_int(doc, root, HMI_WIDGET_PROP_ALIGN_H,align);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
			/* {"cmd_code":"set_text_align_h","type":"widget","widget":"button1","state":"normal","align_h":1} */
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
**  @FunctionName       Stone_HMI_Set_Text_Align_V
**  @Description        Stone HMI Set font alignment vertical
**  @InputParam         widget_name: widget name
**                 		  state: widget state
**						          align: font align ;0:not alignment 1:middle alignment 2:up alignment 3:down alignment
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Text_Align_V(const char *widget_name, const char *state, int align)
{
    char ret = 0;

    if(widget_name == NULL || state == NULL || align < 0)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_TEXT_ALIGN_V);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_CMD_TYPE_WIDGET);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
        yyjson_mut_obj_add_str(doc, root, HMI_WIDGET_PROP_STATE, state);
        yyjson_mut_obj_add_int(doc, root, HMI_WIDGET_PROP_ALIGN_V,align);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
			/* {"cmd_code":"set_text_align_v","type":"widget","widget":"button1","state":"normal","align_v":1} */
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
char Stone_HMI_Set_Color(const char *widget_name, const char *object, uint64_t color)
{
    char ret = 0;

    if(widget_name == NULL || object == NULL)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_COLOR);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_CMD_TYPE_WIDGET);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
        yyjson_mut_obj_add_str(doc, root, HMI_WIDGET_PROP_COLOR_OBJECT, object);
        yyjson_mut_obj_add_int(doc, root, HMI_WIDGET_PROP_COLOR, color);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
			/* {"cmd_code":"set_color","type":"widget","widget":"switch","color_object":"normal:bg_color","color":4278190080} */
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
char Stone_HMI_Set_Color_Ex(const char *widget_name, const char *object, uint64_t *color, unsigned short size)
{
    char ret = 0;

    if(widget_name == NULL || object == NULL || color == NULL)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_COLOR);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_CMD_TYPE_WIDGET);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
        yyjson_mut_obj_add_str(doc, root, HMI_WIDGET_PROP_COLOR_OBJECT, object);
        yyjson_mut_val *color_value = yyjson_mut_arr_with_uint(doc, color, size);
        {
            yyjson_mut_obj_add_val(doc, root, HMI_WIDGET_PROP_COLOR, color_value);
        }
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
			/* ST<{"cmd_code":"set_color","type":"widget","widget":"edit1_5","color_object":"normal:text_color","color":[4278190335,4278190080,4294901760,4294967040,4278255360]} */
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
**  @FunctionName       Stone_HMI_Take_Snapshot
**  @Description        Stone HMI Set screenshot
**  @InputParam         widget_name: window name
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Take_Snapshot(const char *window_name)
{
    char ret = 0;

    if(window_name == NULL)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_TAKE_SNAPSHOT);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_CMD_TYPE_WIDGET);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, window_name);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
			/* {"cmd_code":"take_snapshot","type":"widget","widget":"home_page"} */
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
};

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
char Stone_HMI_Set_Text_Array(const char *widget_name[], const char *text[], unsigned short size)
{
    char ret = 0;

    if(widget_name==NULL || text == NULL || size<=0)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_TEXT_ARRAY);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_CMD_TYPE_WIDGET);
        yyjson_mut_val *widget_name_val = yyjson_mut_arr_with_str(doc, widget_name, size);
        if(widget_name_val)
        {
            yyjson_mut_obj_add_val(doc, root, HMI_CMD_WIDGET, widget_name_val);
        }
        yyjson_mut_val *value_text = yyjson_mut_arr_with_str(doc, text, size);
        if(value_text)
        {
            yyjson_mut_obj_add_val(doc, root, HMI_WIDGET_PROP_TEXT, value_text);
        }
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* {"cmd_code":"set_text_array","type":"widget","widget":["label1_copy0","label1_copy1"],"text":["15","14"]} */
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
**  @FunctionName               Stone_HMI_Set_Value_Array
**  @Description                Stone HMI Set widget value by array
**  @InputParam                 widget_name: widget name array
**                              value:	 widget value array
**                              size:	 value array size
**  @OutputParam				        NONE
**  @ReturnValue                0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Value_Array(const char *widget_name[], float *value, unsigned short size)
{
    char ret = 0;

    if(widget_name==NULL || value==NULL || size<=0)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_VALUE_ARRAY);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_CMD_TYPE_WIDGET);
        yyjson_mut_val *widget_name_val = yyjson_mut_arr_with_str(doc, widget_name, size);
        if(widget_name_val)
        {
            yyjson_mut_obj_add_val(doc, root, HMI_CMD_WIDGET, widget_name_val);
        }
        yyjson_mut_val *value_float = yyjson_mut_arr_with_float(doc, (const float *)value, size);
        if(value_float)
        {
            yyjson_mut_obj_add_val(doc, root, HMI_WIDGET_PROP_VALUE, value_float);
        }
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* {"cmd_code":"set_value_array","type":"widget","widget":["label1_copy0","label1_copy1"],"value":[1.0,2.0]} */
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


