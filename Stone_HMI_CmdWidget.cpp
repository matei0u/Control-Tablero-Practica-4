/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : Stone_HMI_CmdWidget.c
  * Description        : Code for Stone HMI Widget CMD applications
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
//#include "yyjson.h"
//#include "usart.h"

/* USER CODE END Includes */


/* USER CODE BEGIN Stone_HMI_CmdWidget */

/*********************************************************************************************************
**  @FunctionName               write_hex_to_float
**  @Description                Hex to float value
**  @InputParam                 buf:	hex data
**  @OutputParam				        NONE
**  @ReturnValue                float value
**
*********************************************************************************************************/
float write_hex_to_float(unsigned char* buf)
{
    unsigned char i = 0;
    unsigned short int dat0;
    unsigned short int dat1;
#if INT_MAX==32767
    long int temp_value = 0;
#elif INT_MAX==2147483647
    unsigned int temp_value = 0;
#endif
    temp_value = buf[i];
    temp_value <<= 8;

    temp_value |= buf[++i];
    temp_value <<= 8;

    temp_value |= buf[++i];
    temp_value <<= 8;

    temp_value |= buf[++i];

    dat0 = temp_value & 0x0000FFFF;
    dat1 = (temp_value >> 16) & 0x0000FFFF;
    HexFloat.data[0] = dat0 & 0xFF;
    HexFloat.data[1] = (dat0 >> 8) & 0xFF;
    HexFloat.data[2] = dat1 & 0xFF;
    HexFloat.data[3] = (dat1 >> 8) & 0xFF;

    float fdata = HexFloat.value_float;
    return fdata;
}

/*********************************************************************************************************
**  @FunctionName               write_hex_to_int
**  @Description                Hex to int value
**  @InputParam                 buf:	hex data
**  @OutputParam				        NONE
**  @ReturnValue                int value
**
*********************************************************************************************************/
int write_hex_to_int(unsigned char* buf)
{
  unsigned char i = 0;
  int temp_value = 0;

  temp_value = buf[i];
  temp_value <<= 8;

  temp_value |= buf[++i];
  temp_value <<= 8;

  temp_value |= buf[++i];
  temp_value <<= 8;

  temp_value |= buf[++i];

  return temp_value;
}

/*********************************************************************************************************
**  @FunctionName               Stone_HMI_Open_Win
**  @Description                Stone HMI open win
**  @InputParam                 win_name:		window name
**  @OutputParam				        NONE
**  @ReturnValue                0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Open_Win(const char *win_name)
{
  char ret = 0;

  if(win_name == NULL)
  {
    return ret;
  }
  // Create a mutable doc
  yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
  yyjson_mut_val *root = yyjson_mut_obj(doc);
  if(root)
  {
    yyjson_mut_doc_set_root(doc, root);
//        yyjson_mut_obj_add_str(doc, root, "cmd_code", "open_win");
//        yyjson_mut_obj_add_str(doc, root, "type", "window");
//        yyjson_mut_obj_add_str(doc, root, "widget", win_name);
    yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_OPEN_WIN);
    yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_WIDGET_TYPE_WINDOW);
    yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, win_name);
    const char *json = yyjson_mut_write(doc, 0, NULL);
    if (json)
    {
        /* {"cmd_code":"open_win","type":"window","widget":"win_name"} */
//            printf("%s JSON: %s\n",__func__, json);
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
**  @FunctionName               Stone_HMI_Close_Win
**  @Description                Stone HMI Close Window
**  @InputParam                 win_name:		window name
**  @OutputParam				        NONE
**  @ReturnValue                0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Close_Win(const char *win_name)
{
    char ret = 0;

    if(win_name == NULL)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_CLOSE_WIN);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_WIDGET_TYPE_WINDOW);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, win_name);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* {"cmd_code":"close_win","type":"window","widget":"label_value"} */
//            printf("%s JSON: %s\n",__func__, json);
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
**  @FunctionName               Stone_HMI_Back_Win
**  @Description                Stone HMI Back Window
**  @InputParam                 NONE
**  @OutputParam				        NONE
**  @ReturnValue                0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Back_Win(void)
{
    char ret = 0;

    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_BACK_WIN);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_WIDGET_TYPE_WINDOW);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* {"cmd_code":"back_win","type":"window"} */
//            printf("%s JSON: %s\n",__func__, json);
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
**  @FunctionName               Stone_HMI_Back_Win_To
**  @Description                Stone HMI Back To Window
**  @InputParam                 win_name:		window name
**  @OutputParam				        NONE
**  @ReturnValue                0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Back_Win_To(const char *win_name)
{
  char ret = 0;

    if(win_name == NULL)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_BACK_WIN_TO);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_WIDGET_TYPE_WINDOW);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, win_name);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* {"cmd_code":"back_win_to","type":"window","widget":"home_page"} */
//            printf("%s JSON: %s\n",__func__, json);
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
**  @FunctionName               Stone_HMI_Back_Home
**  @Description                Stone HMI Back to home_page
**  @InputParam                 NONE
**  @OutputParam				        NONE
**  @ReturnValue                0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Back_Home(void)
{
    char ret = 0;

    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_BACK_HOME);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_WIDGET_TYPE_WINDOW);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* {"cmd_code":"back_home","type":"window"} */
//            printf("%s JSON: %s\n",__func__, json);
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
**  @FunctionName               Stone_HMI_Get_Displayed_Win
**  @Description                Stone HMI get the current displayed main window
**  @InputParam                 NONE
**  @OutputParam				        NONE
**  @ReturnValue                0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Get_Displayed_Win(void)
{
    char ret = 0;

    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_GET_DISPLAYED_WINDOW);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_WIDGET_TYPE_WINDOW);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* {"cmd_code":"get_displayed_window","type":"window"} */
//            printf("%s JSON: %s\n",__func__, json);
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
**  @FunctionName               Stone_HMI_Set_Text
**  @Description                Stone HMI Set widget text
**  @InputParam                 widget_type: widget type
**                              widget_name: widget name
**                              image_name:	 image name
**  @OutputParam				        NONE
**  @ReturnValue                0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Text(const char *widget_type, const char *widget_name, const char *text)
{
    char ret = 0;

    if(widget_type==NULL || widget_name==NULL || text == NULL)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_TEXT);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, widget_type);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
        yyjson_mut_obj_add_str(doc, root, HMI_WIDGET_PROP_TEXT, text);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* ST<{"cmd_code":"set_text","type":"label","widget":"label","text":"Hello Stone"} */
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
char Stone_HMI_Set_Text_Ex(const char *widget_type, const char *widget_name, const char *text[], unsigned short size)
{
    char ret = 0;

    if(widget_type==NULL || widget_name==NULL || text == NULL || size<=0)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_TEXT);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, widget_type);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
        yyjson_mut_val *value_text = yyjson_mut_arr_with_str(doc, text, size);
        if(value_text)
        {
            yyjson_mut_obj_add_val(doc, root, HMI_WIDGET_PROP_TEXT, value_text);
        }
        // Create JSON array for text
        /*
        yyjson_mut_val *value_text = yyjson_mut_arr(doc);
        if(value_text)
        {
            for(unsigned short i = 0; i < size; i++)
            {
                yyjson_mut_arr_add_str(doc, value_text, text[i]);
            }
            yyjson_mut_obj_add_val(doc, root, HMI_WIDGET_PROP_TEXT, value_text);
        }
        */
        
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* {"cmd_code":"set_text","type":"label","widget":"label1_11","text":["2022-07-11\n10:30","10","12"]} */
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
**  @FunctionName               Stone_HMI_Set_Value
**  @Description                Stone HMI Set widget value
**  @InputParam                 widget_type: widget type
**                              widget_name: widget name
**                              format:	 value format
**                              value:	 widget value
**  @OutputParam				        NONE
**  @ReturnValue                0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Value(const char *widget_type, const char *widget_name,const char *format, unsigned int value)
{
    char ret = 0;
    if(widget_type==NULL || widget_name==NULL )
    {
      Serial.println("123456");
      return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_VALUE);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, widget_type);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
        yyjson_mut_obj_add_uint(doc, root, HMI_WIDGET_PROP_VALUE,value);
        if(format != NULL)
        {
            yyjson_mut_obj_add_str(doc, root, HMI_WIDGET_PROP_FORMAT, format);
        }
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* {"cmd_code":"set_value","type":"label","widget":"label2","value":5,"format":"%02d"} */
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
**  @FunctionName               Stone_HMI_Set_Value_Ex
**  @Description                Stone HMI Set widget value by array
**  @InputParam                 widget_type: widget type
**                              widget_name: widget name
**                              value:	 widget value array
**                              size:	 value array size
**  @OutputParam				        NONE
**  @ReturnValue                0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Value_Ex(const char *widget_type, const char *widget_name, float *value, unsigned short size)
{
    char ret = 0;
    if(widget_type==NULL || widget_name==NULL || value==NULL || size<=0)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_VALUE);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, widget_type);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
        yyjson_mut_val *value_float = yyjson_mut_arr_with_float(doc, (const float *)value, size);
        if(value_float)
        {
            yyjson_mut_obj_add_val(doc, root, HMI_WIDGET_PROP_VALUE, value_float);
        }
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* {"cmd_code":"set_value","type":"slider","widget":"slider1_37","value":[10,12,80,15,12,10,12,10,10]} */
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
**  @FunctionName       Stone_HMI_Get_Text
**  @Description        Stone HMI Get widget text 
**  @InputParam         widget_type: widget type
**                      widget_name: widget name
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Get_Text(const char *widget_type, const char *widget_name)
{
    char ret = 0;
    if(widget_type==NULL || widget_name==NULL)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_GET_TEXT);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, widget_type);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* {"cmd_code":"get_text","type":"label","widget":"label1"} */
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
**  @FunctionName       Stone_HMI_Get_Value
**  @Description        Stone HMI Get widget Value
**  @InputParam         widget_type: widget type
**                      widget_name: widget name
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Get_Value(const char *widget_type, const char *widget_name)
{
    char ret = 0;
    if(widget_type==NULL || widget_name==NULL)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_GET_VALUE);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, widget_type);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* ST<{"cmd_code":"get_value","type":"label","widget":"label"} */
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
**  @FunctionName       Stone_HMI_Set_Selected
**  @Description        Stone HMI Sets widget(combo_box/text_selector/...) current selection
**  @InputParam         widget_type: widget type
**						          widget_name: widget name
**						          index: widget selected_index
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Selected(const char *widget_type, const char *widget_name, unsigned short index)
{
    char ret = 0;
    if(widget_type==NULL || widget_name==NULL)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_SELECTED);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, widget_type);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
		    yyjson_mut_obj_add_uint(doc, root, HMI_WIDGET_PROP_SELECTED_INDEX, index);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* ST<{"cmd_code":"set_selected","type":"combo_box_ex","widget":"cbx1","selected_index":2} */
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
**  @FunctionName       Stone_HMI_Set_Option
**  @Description        Stone HMI Set widget(combo_box/text_selector/...) option content
**  @InputParam         widget_type: widget type
**						          widget_name: widget name
**						          options	: combo_box option content
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Option(const char *widget_type, const char *widget_name, const char *options)
{
    char ret = 0;
    if(widget_type==NULL || widget_name==NULL || options==NULL)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_OPTION);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, widget_type);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
		    yyjson_mut_obj_add_str(doc, root, HMI_WIDGET_PROP_OPTIONS, options);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* ST<{"cmd_code":"set_option","type":"combo_box_ex","widget":"cbx1","options":"1:ok;2:cancle;"} */
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
**  @FunctionName       Stone_HMI_Set_Gelected
**  @Description        Stone HMI Get widget(combo_box/text_selector/...) current option
**  @InputParam         idget_type: widget type
**						          widget_name: widget name
**						          index: combo_box selected_index
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Get_Selected(const char *widget_type, const char *widget_name)
{
    char ret = 0;
    if(widget_type == NULL || widget_name==NULL)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_GET_SELECTED);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, widget_type);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* ST<{"cmd_code":"get_selected","type":"combo_box_ex","widget":"combo_box_ex1"} */
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
**  @FunctionName       Stone_HMI_Set_Max
**  @Description        Stone HMI Set widget(progress_bar/progress_circle/slider/...) MAXIMUM
**  @InputParam         widget_type: widget type
**						          widget_name: widget name
**						          max	: widget maximum
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Max(const char *widget_type, const char *widget_name, unsigned int max)
{
    char ret = 0;

    if(widget_type==NULL || widget_name==NULL || max < 0)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_MAX);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, widget_type);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
		    yyjson_mut_obj_add_uint(doc, root, HMI_WIDGET_PROP_MAX, max);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* ST<{"cmd_code":"set_max","type":"progress_bar","widget":"progress_bar","max":100} */
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
**  @FunctionName       Stone_HMI_Set_Min
**  @Description        Stone HMI Set widget(progress_bar/progress_circle/slider/...) MINIMUM
**  @InputParam         widget_type: widget type
**						          widget_name: widget name
**						          max	: widget minimum
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Min(const char *widget_type, const char *widget_name, unsigned short min)
{
    char ret = 0;

    if(widget_type==NULL || widget_name==NULL || min < 0)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_MIN);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, widget_type);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
		    yyjson_mut_obj_add_uint(doc, root,HMI_WIDGET_PROP_MIN,min);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* ST<{"cmd_code":"set_min","type":"progress_bar","widget":"progress_bar","min":0} */
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
**  @FunctionName       Stone_HMI_Set_Show_Text
**  @Description        Stone HMI Sets whether the widget(progress_bar/progress_circle/...) displays text
**  @InputParam         widget_name: widget name
**						          show_text: whether the progress bar displays text (True or False)
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Show_Text(const char *widget_type, const char *widget_name, bool show_text)
{
    char ret = 0;

    if(widget_type==NULL || widget_name==NULL)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SHOW_TEXT);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, widget_type);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
		    yyjson_mut_obj_add_bool(doc, root, HMI_WIDGET_PROP_SHOW_TEXT, show_text);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* ST<{"cmd_code":"set_show_text","type":"progress_bar","widget":"progress_bar","show_text":true} */
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
**  @FunctionName       Stone_HMI_Get_Percent
**  @Description        Stone HMI Gets the widget(progress_bar/progress_circle/...) percentage
**  @InputParam         widget_name: widget name
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Get_Percent(const char *widget_type, const char *widget_name)
{
    char ret = 0;

    if(widget_type==NULL || widget_name==NULL)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_GET_PERCENT);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, widget_type);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* ST<{"cmd_code":"get_percent","type":"progress_bar","widget":"progress_bar"} */
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
**  @FunctionName       Stone_HMI_Set_Loop
**  @Description        Stone HMI Set widget whether to loop
**  @InputParam         widget_type: widget type
**						          widget_name: widget name
**						          loop: whether to loop the hscroll label
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
//char Stone_HMI_Set_Loop(const char *widget_type, const char *widget_name, bool loop)
//{
//    char ret = 0;

//    if(widget_type==NULL || widget_name==NULL)
//    {
//        return ret;
//    }
//    // Create a mutable doc
//    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
//    yyjson_mut_val *root = yyjson_mut_obj(doc);
//    if(root)
//    {
//        yyjson_mut_doc_set_root(doc, root);
//        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_LOOP);
//        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, widget_type);
//        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
//		yyjson_mut_obj_add_bool(doc, root, HMI_WIDGET_PROP_LOOP, loop);
//        const char *json = yyjson_mut_write(doc, 0, NULL);
//        if (json)
//        {
//            /* ST<{"cmd_code":"set_loop","type":"hscroll_label","widget":"hscroll_label1","loop":true} */
////            printf("%s JSON: %s\n",__func__ , json);
//            ret = Stone_HMI_Cmd_Send(json);
//            free((void *)json);
//        }
//        else
//        {
//            Serial.print(__func__);
          // Serial.println(" JSON create err");
//        }
//    }
//    // Free the doc
//    yyjson_mut_doc_free(doc);
//    return ret;
//}

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
char Stone_HMI_Set_Loop(const char *widget_type, const char *widget_name, unsigned short loop)
{
    char ret = 0;

    if(widget_type==NULL || widget_name==NULL)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_LOOP);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, widget_type);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
		    yyjson_mut_obj_add_uint(doc, root, HMI_WIDGET_PROP_LOOP, loop);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* ST<{"cmd_code":"set_loop","type":"hscroll_label","widget":"hscroll_label1","loop":123} */
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
**  @FunctionName       Stone_HMI_Set_Yoyo
**  @Description        Stone HMI Set whether to rool-back the hscroll label
**  @InputParam         widget_name: widget name
**						          loop: whether to rool-back the hscroll label
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Yoyo(const char *widget_name, bool yoyo)
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
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_YOYO);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_WIDGET_TYPE_HSCROLL_LABEL);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
		    yyjson_mut_obj_add_bool(doc, root, HMI_WIDGET_PROP_YOYO, yoyo);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* ST<{"cmd_code":"set_yoyo","type":"hscroll_label","widget":"hscroll_label1","yoyo":true} */
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
**  @FunctionName       Stone_HMI_Set_Direction
**  @Description        Stone HMI Set the scrolling direction of the hscroll label
**  @InputParam         widget_name: widget name
**						          direction: the scrolling direction of the hscroll label
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Direction(const char *widget_name, bool direction)
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
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_DIRECTION);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_WIDGET_TYPE_HSCROLL_LABEL);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
		    yyjson_mut_obj_add_bool(doc, root, HMI_WIDGET_PROP_DIRECTION, direction);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* ST<{"cmd_code":"set_direction","type":"hscroll_label","widget":"hscroll_label1","direction":true} */
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
**  @FunctionName       Stone_HMI_Set_Lull
**  @Description        Stone HMI Set the interval between hscroll label scrolls
**  @InputParam         widget_name: widget name
**						          lull: the interval between hscroll label scrolls
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Lull(const char *widget_name, unsigned int lull)
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
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_LULL);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_WIDGET_TYPE_HSCROLL_LABEL);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
		    yyjson_mut_obj_add_uint(doc, root, HMI_CMD_SET_LULL, lull);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* ST<{"cmd_code":"set_lull","type":"hscroll_label","widget":"hscroll_label1","lull":2000} */
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
**  @FunctionName       Stone_HMI_Set_Duration
**  @Description        Stone HMI Set the time for scrolling the hscroll label once
**  @InputParam         widget_name: widget name
**						          lull: the time for scrolling the hscroll label once
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Duration(const char *widget_name, unsigned int duration)
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
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_DURATION);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_WIDGET_TYPE_HSCROLL_LABEL);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
		    yyjson_mut_obj_add_uint(doc, root, HMI_WIDGET_PROP_DURATION, duration);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* ST<{"cmd_code":"set_duration","type":"hscroll_label","widget":"hscroll_label1","duration":2000} */
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
**  @FunctionName       Stone_HMI_Set_Step
**  @Description        Stone HMI Set the slider step value
**  @InputParam         widget_name: widget name
**						          step: step value
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Step(const char *widget_name, unsigned int step)
{
    char ret = 0;

    if(widget_name==NULL || step<=0)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_STEP);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_WIDGET_TYPE_SLIDER);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
		    yyjson_mut_obj_add_uint(doc, root, HMI_WIDGET_PROP_STEP, step);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* ST<{"cmd_code":"set_step","type":"slider","widget":"slider1","step":1} */
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
**  @FunctionName               Stone_HMI_Set_Image
**  @Description                Stone HMI Set the display image
**  @InputParam                 widget_type: widget type
**                              widget_name: widget name
**                              image_name:	 image name
**  @OutputParam				        NONE
**  @ReturnValue                0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Image(const char *widget_type, const char *widget_name, const char *image_name)
{
    char ret = 0;

    if(widget_type==NULL || widget_name==NULL || image_name == NULL)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_IMAGE);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, widget_type);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
        yyjson_mut_obj_add_str(doc, root, HMI_WIDGET_PROP_IMAGW, image_name);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* {"cmd_code":"set_image","type":"image","widget":"image1","image":"guage_bg"} */
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
**  @FunctionName       Stone_HMI_Set_Draw_Type
**  @Description        Stone HMI Set how the picture is drawn
**  @InputParam         widget_type: widget type
**						          widget_name: widget name
**						          draw_type: drawing mode
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Draw_Type(const char *widget_type, const char *widget_name, unsigned short draw_type)
{
    char ret = 0;

    if(widget_type==NULL || widget_name==NULL || draw_type<0)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_DRAW_TYPE);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, widget_type);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
		    yyjson_mut_obj_add_uint(doc, root, HMI_WIDGET_PROP_DRAW_TYPE, draw_type);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* ST<{"cmd_code":"set_draw_type","type":"image","widget":"image","draw_type":2} */
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
char Stone_HMI_Set_Scale(const char *widget_type, const char *widget_name, float scale_x, float scale_y)
{
    char ret = 0;

    if(widget_type==NULL || widget_name==NULL || scale_x<=0 || scale_y<=0)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_SCALE);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, widget_type);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
        yyjson_mut_obj_add_real(doc, root, HMI_WIDGET_PROP_SCALE_X, scale_x);
        yyjson_mut_obj_add_real(doc, root, HMI_WIDGET_PROP_SCALE_Y, scale_y);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* ST<{"cmd_code":"set_scale","type":"image","widget":"image","scale_x":0.5,"scale_y":0.5} */
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
**  @FunctionName       Stone_HMI_Set_Rotation
**  @Description        Stone HMI Set the rotation Angle of the picture
**  @InputParam         widget_type: widget type
**						          widget_name: widget name
**						          scale_x: rotation
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Rotation(const char *widget_type,const char *widget_name, unsigned short rotation)
{
    char ret = 0;

    if(widget_type==NULL || widget_name==NULL || rotation<0)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_RORATION);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, widget_type);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
		    yyjson_mut_obj_add_uint(doc, root, HMI_WIDGET_PROP_ROTATION, rotation);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* ST<{"cmd_code":"set_rotation","type":"image","widget":"image","rotation":90} */
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
**  @FunctionName       Stone_HMI_Set_Format
**  @Description        Stone HMI Set the format of the control property
**  @InputParam         widget_type: widget type
**						          widget_name: widget name
**						          format: the format of the control property
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Format(const char *widget_type, const char *widget_name, const char *format)
{
    char ret = 0;

    if(widget_type==NULL || widget_name==NULL || format==NULL)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_FORMAT);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, widget_type);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
		    yyjson_mut_obj_add_str(doc, root, HMI_WIDGET_PROP_FORMAT, format);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* ST<{"cmd_code":"set_format","type":"image_value","widget":"image_value","format":""%02.2f"} */
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
**  @FunctionName       Stone_HMI_Set_Play
**  @Description        Stone HMI Set image animation/gif playback
**  @InputParam         widget_type: widget type
**						          widget_name: widget name
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Play(const char *widget_type, const char *widget_name)
{
    char ret = 0;

    if(widget_type==NULL || widget_name==NULL)
    {
      return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_PLAY);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, widget_type);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* ST<{"cmd_code":"set_play","type":"image_animation","widget":"image_ani1"} */
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
**  @FunctionName       Stone_HMI_Set_Pause
**  @Description        Stone HMI Set the image animation/gif to pause
**  @InputParam         widget_type: widget type
**						          widget_name: widget name
**  @OutputParam	      NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Pause(const char *widget_type, const char *widget_name)
{
    char ret = 0;

    if(widget_type==NULL || widget_name==NULL)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_PAUSE);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, widget_type);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* ST<{"cmd_code":"set_pause","type":"image_animation","widget":"image_ani1"} */
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
**  @FunctionName       Stone_HMI_Set_Stop
**  @Description        Stone HMI Set the image animation/gif to stop
**  @InputParam         widget_type: widget type
**						          widget_name: widget name
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Stop(const char *widget_type, const char *widget_name)
{
    char ret = 0;

    if(widget_type==NULL || widget_name==NULL)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_STOP);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, widget_type);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* ST<{"cmd_code":"set_stop","type":"image_animation","widget":"image_ani1"} */
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
**  @FunctionName       Stone_HMI_Set_Interval
**  @Description        Stone HMI Set the image animation interval
**  @InputParam         widget_name: widget name
**						          interval: image animation interval
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Interval(const char *widget_name, unsigned short interval)
{
    char ret = 0;

    if(widget_name==NULL || interval<0)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_INTERVAL);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_WIDGET_TYPE_IMAGE_ANIMATION);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
		    yyjson_mut_obj_add_uint(doc, root, HMI_WIDGET_PROP_INTERVAL, interval);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* ST<{"cmd_code":"set_interval","type":"image_animation","widget":"image_ani1","interval":200} */
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
char Stone_HMI_Set_Range(const char *widget_type,const char *widget_name, unsigned short start_index, unsigned short end_index)
{
    char ret = 0;

    if(widget_type==NULL || widget_name==NULL || start_index<0 || end_index<0)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_RANGE);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, widget_type);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
        yyjson_mut_obj_add_uint(doc, root, HMI_WIDGET_PROP_START_INDEX, start_index);
        yyjson_mut_obj_add_uint(doc, root, HMI_WIDGET_PROP_END_INDEX, end_index);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* ST<{"cmd_code":"set_range","type":"image_animation","widget":"image_ani1","start_index":1,"end_index":9} */
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
**  @FunctionName       Stone_HMI_Set_Frame
**  @Description        Stone HMI Displays a specific frame picture
**  @InputParam         widget_type: widget type
**						          widget_name: widget name
**						          frame: Picture specific frame number
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Frame(const char *widget_type, const char *widget_name, unsigned short frame)
{
    char ret = 0;

    if(widget_type==NULL || widget_name==NULL || frame<0)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_FRAME);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, widget_type);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
		    yyjson_mut_obj_add_uint(doc, root, HMI_WIDGET_PROP_FRAME, frame);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* ST<{"cmd_code":"set_frame","type":"image_animation","widget":"image_ani1","frame":1} */
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
**  @FunctionName       Stone_HMI_Get_Checked
**  @Description        Stone HMI Gets the currently selected radio button
**  @InputParam         widget_name: widget name
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Get_Checked(const char *widget_name)
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
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_GET_CHECKED);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_WIDGET_TYPE_RADIO_BUTTON);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* ST<{"cmd_code":"get_checked","type":"radio_button","widget":"radio_button1"} */
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
**  @FunctionName               Stone_HMI_Set_Date
**  @Description                Stone HMI Set widget Date
**  @InputParam                 widget_name: widget name
**                              date: rtc date
**  @OutputParam				        NONE
**  @ReturnValue                0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Date(const char *widget_name, const char * date)
{
    char ret = 0;

    if(widget_name==NULL || date == NULL)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_DATE);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_WIDGET_TYPE_DIGIT_CLOCK);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
        yyjson_mut_obj_add_str(doc, root, HMI_WIDGET_PROP_DATE, date);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* {"cmd_code":"set_date","type":"digit_clock","widget":"digit_clock","date":"2021-02-26 12:23:46"} */
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
**  @FunctionName               Stone_HMI_Get_Date
**  @Description                Stone HMI Get the RTC Date
**  @InputParam                 widget_name: widget name
**  @OutputParam				        NONE
**  @ReturnValue                0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Get_Date(const char *widget_name)
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
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_GET_DATE);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_WIDGET_TYPE_DIGIT_CLOCK);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* {"cmd_code":"get_date","type":"digit_clock","widget":"digit_clock1"} */
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
**  @FunctionName       Stone_HMI_Set_Angle
**  @Description        Stone HMI Set the rotation Angle of the pointer
**  @InputParam         widget_name: widget name
**						          angle: the rotation Angle of the pointer
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Angle(const char *widget_name, unsigned short angle)
{
    char ret = 0;

    if(widget_name==NULL || angle<0)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_ANGLE);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_WIDGET_TYPE_GAUGE_POINTER);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
		    yyjson_mut_obj_add_uint(doc, root, HMI_WIDGET_PROP_ANGLE, angle);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* ST<{"cmd_code":"set_angle","type":"gauge_pointer","widget":"gp1","angle":60} */
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
char Stone_HMI_Set_Range_Chart_View(const char *widget_type, const char *widget_name, unsigned int min, unsigned int max)
{  
    char ret = 0;

    if(widget_type==NULL || widget_name==NULL || min<0 || max<0)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_RANGE);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, widget_type);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
        yyjson_mut_obj_add_uint(doc, root, HMI_WIDGET_PROP_MIN, min);
        yyjson_mut_obj_add_uint(doc, root, HMI_WIDGET_PROP_MAX, max);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* ST<{"cmd_code":"set_range","type":"image_animation","widget":"image_ani1","min":1,"max":9} */
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
**  @FunctionName       Stone_HMI_Set_Data
**  @Description        Stone HMI Set the curve sequence point data
**  @InputParam         widget_type: widget type
**						          widget_name: widget name
**						          data: Scale display value
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Data(const char *widget_type, const char *widget_name, const char *data)
{
    char ret = 0;

    if(widget_type==NULL || widget_name==NULL || data==NULL)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_DATA);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, widget_type);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
		    yyjson_mut_obj_add_str(doc, root, HMI_WIDGET_PROP_DATA, data);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* ST<{"cmd_code":"set_data","type":"x_axis","widget":"x_axis1","data":"[1,2,3,4,5,6,7,8,9,10]"} */
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
**  @FunctionName       Stone_HMI_Set_Line
**  @Description        Stone HMI Sets whether the curve sequence boundaries are displayed and smooth
**  @InputParam         widget_name: widget name
**						          show: Whether the curve sequence boundary line is displayed
**						          smooth: Whether the boundary line of the curve sequence is smooth
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Line(const char *widget_name, bool show, bool smooth)
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
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_LINE);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_WIDGET_LINE_SERIES);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
        yyjson_mut_obj_add_bool(doc, root, HMI_WIDGET_PROP_SHOW, show);
        yyjson_mut_obj_add_bool(doc, root, HMI_WIDGET_PROP_SMOOTH, smooth);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* ST<{"cmd_code":"set_line","type":"line_series","widget":"line_series1","show":true,"smooth":true} */
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
**  @FunctionName       Stone_HMI_Set_Area
**  @Description        Stone HMI Sets whether the curve sequence area is displayed
**  @InputParam         widget_name: widget name
**						          show: whether the curve sequence area is displayed
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Area(const char *widget_name, bool show)
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
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_AREA);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_WIDGET_LINE_SERIES);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
		    yyjson_mut_obj_add_bool(doc, root, HMI_WIDGET_PROP_SHOW, show);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* ST<{"cmd_code":"set_area","type":"line_series","widget":"line_series1","show":true} */
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
**  @FunctionName       Stone_HMI_Set_Symbol
**  @Description        Stone HMI Sets whether the curve sequence point markers are displayed
**  @InputParam         widget_name: widget name
**						          show: whether the curve sequence point markers are displayed
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Symbol(const char *widget_name, bool show)
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
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_SYMBOL);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_WIDGET_LINE_SERIES);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
		    yyjson_mut_obj_add_bool(doc, root, HMI_WIDGET_PROP_SHOW, show);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* ST<{"cmd_code":"set_symbol","type":"line_series","widget":"line_series1","show":true} */
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
char Stone_HMI_Series_Set_Value(const char *widget_type, const char *widget_name, unsigned int index, unsigned int value)
{
    char ret = 0;

    if(widget_type==NULL || widget_name==NULL || index<0 || value<0)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_VALUE);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, widget_type);
		    yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
        yyjson_mut_obj_add_uint(doc, root, HMI_WIDGET_PROP_INDEX, index);
        yyjson_mut_obj_add_uint(doc, root, HMI_WIDGET_PROP_VALUE, value);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* {"cmd_code":"set_value","type":"line_series","widget":"line_series1","index":4,"value":10} */
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
**  @FunctionName       Stone_HMI_Series_Push_Set_Value
**  @Description        Stone HMI Append data to the end 
**  @InputParam         widget_type: widget type
**                      widget_name: widget name
**                      value: push value
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char STONE_push_series(const char *widget_type, const char *widget_name, unsigned short value){
    char ret = 0;

    if(widget_type==NULL || widget_name==NULL || value<0)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_VALUE);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, widget_type);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
        yyjson_mut_obj_add_str(doc, root, HMI_WIDGET_PROP_MODE, HMI_WIDGET_PROP_PUSH);
        yyjson_mut_obj_add_uint(doc, root, HMI_WIDGET_PROP_VALUE, value);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* {"cmd_code":"set_value","type":"line_series","widget":"line_series1","mode":push,"value":23} */
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

char Stone_HMI_Series_Push_Set_Value(const char *widget_type, const char *widget_name, unsigned short value)
{
    char ret = 0;


    if(widget_type==NULL || widget_name==NULL || value<0)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_VALUE);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, widget_type);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
        yyjson_mut_obj_add_str(doc, root, HMI_WIDGET_PROP_MODE, HMI_WIDGET_PROP_PUSH);
        yyjson_mut_obj_add_uint(doc, root, HMI_WIDGET_PROP_VALUE, value);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* {"cmd_code":"set_value","type":"line_series","widget":"line_series1","mode":push,"value":23} */
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
**  @FunctionName       Stone_HMI_Series_Get_Value
**  @Description        Stone HMI Append data to the end 
**  @InputParam         widget_type: widget type
**                      widget_name: widget name
**                      value: push value
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Series_Get_Value(const char *widget_type, const char *widget_name, unsigned short index)
{
    char ret = 0;

    if(widget_type==NULL || widget_name==NULL || index<0)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_GET_VALUE);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, widget_type);
		    yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
        yyjson_mut_obj_add_uint(doc, root, HMI_WIDGET_PROP_INDEX, index);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* {"cmd_code":"get_value","type":"line_series","widget":"line_series1","index":4} */
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
**  @FunctionName       Stone_HMI_Set_Capacity
**  @Description        Stone HMI Set FIFO capacity for curve sequence points
**  @InputParam         widget_type: widget type
**						          widget_name: widget name
**						          capacity: FIFO capacity
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Capacity(const char *widget_type, const char *widget_name, unsigned short capacity)
{
    char ret = 0;

    if(widget_type==NULL || widget_name==NULL)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_CAPACITY);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, widget_type);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
		    yyjson_mut_obj_add_uint(doc, root, HMI_WIDGET_PROP_CAPACITY, capacity);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* ST<{"cmd_code":"set_capacity","type":"line_series","widget":"line_series1","capacity":15} */
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
**  @FunctionName       Stone_HMI_Get_Capacity
**  @Description        Stone HMI Get FIFO capacity for curve sequence points
**  @InputParam         widget_type: widget type
**						          widget_name: widget name
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Get_Capacity(const char *widget_type, const char *widget_name)
{
    char ret = 0;

    if(widget_type==NULL || widget_name==NULL)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_GET_CAPACITY);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, widget_type);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* ST<{"cmd_code":"get_capacity","type":"line_series","widget":"line_series1"} */
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
**  @FunctionName       Stone_HMI_Set_Start_Angle
**  @Description        Stone HMI Set the pie chart starting Angle
**  @InputParam         widget_name: widget name
**						          angle: the pie chart starting Angle
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Start_Angle(const char *widget_name, int angle)
{
    char ret = 0;

    if(widget_name==NULL || angle<0)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_START_ANGLE);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_WIDGET_PIE_SLICE);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
		    yyjson_mut_obj_add_int(doc, root, HMI_WIDGET_PROP_ANGLE, angle);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* ST<{"cmd_code":"set_start_angle","type":"pie_slice","widget":"pie_slice3","angle":60} */
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
**  @FunctionName       Stone_HMI_Set_Radius
**  @Description        Stone HMI Set the pie circle thickness radius
**  @InputParam         widget_name: widget name
**						          radius: the pie circle thickness radius
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Radius(const char *widget_name, unsigned short radius)
{
    char ret = 0;

    if(widget_name==NULL || radius<0)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_RADIUS);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_WIDGET_PIE_SLICE);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
		    yyjson_mut_obj_add_uint(doc, root, HMI_WIDGET_PROP_RADIUS, radius);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* ST<{"cmd_code":"set_radius","type":"pie_slice","widget":"pie_slice3","radius":60} */
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
**  @FunctionName       Stone_HMI_Set_Size
**  @Description        Stone HMI Set indicator size
**  @InputParam         widget_name: widget name
**						          size: the indicator size
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Size(const char *widget_name, unsigned short size)
{
    char ret = 0;

    if(widget_name==NULL || size<0)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_SIZE);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_WIDGET_SLIDE_INDICATOR);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
		    yyjson_mut_obj_add_uint(doc, root, HMI_WIDGET_PROP_SIZE, size);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* ST<{"cmd_code":"set_size","type":"slide_indicator","widget":"slide_indicator1","size":5} */
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
**  @FunctionName       Stone_HMI_Set_Spacing
**  @Description        Stone HMI Set indicator interval
**  @InputParam         widget_type: widget type
**						          widget_name: widget name
**						          spacing: the indicator interval
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Spacing(const char *widget_type, const char *widget_name, unsigned int spacing)
{
    char ret = 0;

    if(widget_type==NULL || widget_name==NULL || spacing<0)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_SPACING);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, widget_type);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
		    yyjson_mut_obj_add_uint(doc, root, HMI_WIDGET_PROP_SPACING, spacing);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* ST<{"cmd_code":"set_spacing","type":"slide_indicator","widget":"slide_indicator1","spacing":15} */
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
**  @FunctionName       Stone_HMI_Set_View
**  @Description        Stone HMI Set the interface number of the current view
**  @InputParam         widget_name: widget name
**						          index: serial number
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_View(const char *widget_name, unsigned short index)
{
    char ret = 0;

    if(widget_name==NULL || index<0)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_VIEW);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_WIDGET_SLIDE_VIEW);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
		    yyjson_mut_obj_add_real(doc, root, HMI_WIDGET_PROP_INDEX, index);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* ST<{"cmd_code":"set_view","type":"slide_view","widget":"slide_view0","index":0} */
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
**  @FunctionName       Stone_HMI_Set_Auto_Play
**  @Description        Stone HMI Set the current view to play automatically
**  @InputParam         widget_name: widget name
**						          auto_play: Autoplay time
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Auto_Play(const char *widget_name, unsigned short auto_play)
{
    char ret = 0;

    if(widget_name==NULL || auto_play<0)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_AUTO_PLAY);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_WIDGET_SLIDE_VIEW);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
		    yyjson_mut_obj_add_uint(doc, root, HMI_WIDGET_PROP_AUTO_PLAY, auto_play);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* ST<{"cmd_code":"set_auto_play","type":"slide_view","widget":"slide_view0","auto_play":0} */
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
char Stone_HMI_Set_Tab_Button_Value(const char *widget_type, const char *widget_name, bool value)
{
    char ret = 0;

    if(widget_type==NULL || widget_name==NULL )
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_VALUE);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, widget_type);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
        yyjson_mut_obj_add_bool(doc, root, HMI_WIDGET_PROP_VALUE, value);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* {"cmd_code":"set_value","type":"tab_button","widget":"tab_button4","value":true} */
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
**  @FunctionName       Stone_HMI_Get_View
**  @Description        Stone HMI Get the interface number of the current view
**  @InputParam         widget_type: widget type
**						          widget_name: widget name
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Get_View(const char *widget_type, const char *widget_name)
{
    char ret = 0;

    if(widget_type==NULL || widget_name==NULL)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_GET_VIEW);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, widget_type);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* ST<{"cmd_code":"get_view","type":"slide_view","widget":"slide_view0"} */
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
**  @FunctionName       Stone_HMI_Set_Slider_Menu_Scale
**  @Description        Stone HMI Sets the current menu scaling ratio
**  @InputParam         widget_name: widget name
**						          scale: the current menu scaling ratio
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Slide_Menu_Scale(const char *widget_name, float scale)
{
    char ret = 0;

    if(widget_name==NULL || scale<0.5 ||scale>1.0)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_SCALE);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_WIDGET_SLIDE_MENU);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
		    yyjson_mut_obj_add_real(doc, root, HMI_WIDGET_PROP_SCALE, scale);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* ST<{"cmd_code":"set_scale","type":"slide_view","widget":"slide_view0","scale":0.5} */
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
**  @FunctionName       Stone_HMI_Set_Align_V
**  @Description        Stone HMI Set the current menu alignment
**  @InputParam         widget_name: widget name
**						          align_v: the current menu alignment
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Align_V(const char *widget_name, unsigned short align_v)
{
    char ret = 0;

    if(widget_name==NULL || align_v<0 ||align_v>3)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_ALIGN_V);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_WIDGET_SLIDE_MENU);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
		    yyjson_mut_obj_add_uint(doc, root, HMI_WIDGET_PROP_ALIGN_V, align_v);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* ST<{"cmd_code":"set_align_v","type":"slide_menu","widget":"slide_menu0","align_v":1} */
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
**  @FunctionName       Stone_HMI_Set_Xslidable
**  @Description        Stone HMI Set whether to allow x direction sliding
**  @InputParam         widget_name: widget name
**						          value: whether to allow x direction sliding
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Xslidable(const char *widget_name, bool value)
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
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_XSLIDABLE);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_WIDGET_SCROLL_VIEW);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
		    yyjson_mut_obj_add_bool(doc, root, HMI_WIDGET_PROP_VALUE, value);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* ST<{"cmd_code":"set_xslidable","type":"scroll_view","widget":"scroll_view2","value":true} */
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
**  @FunctionName       Stone_HMI_Set_Yslidable
**  @Description        Stone HMI Set whether to allow y direction sliding
**  @InputParam         widget_name: widget name
**						          value: whether to allow y direction sliding
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Yslidable(const char *widget_name, bool value)
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
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_YSLIDABLE);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_WIDGET_SCROLL_VIEW);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
		    yyjson_mut_obj_add_bool(doc, root, HMI_WIDGET_PROP_VALUE, value);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* ST<{"cmd_code":"set_yslidable","type":"scroll_view","widget":"scroll_view2","value":true} */
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
**  @FunctionName       Stone_HMI_Set_Snap_To_Page
**  @Description        Stone HMI Sets whether to align by page when scrolling
**  @InputParam         widget_name: widget name
**						          value: whether to align by page when scrolling
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Snap_To_Page(const char *widget_name, bool value)
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
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_SNAP_TO_PAGE);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_WIDGET_SCROLL_VIEW);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
		    yyjson_mut_obj_add_bool(doc, root, HMI_WIDGET_PROP_VALUE, value);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* ST<{"cmd_code":"set_move_to_page","type":"scroll_view","widget":"scroll_view1","value":true} */
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
**  @FunctionName       Stone_HMI_Set_Move_To_Page
**  @Description        Stone HMI Set whether to scroll one page at a time
**  @InputParam         widget_name: widget name
**						          value: whether to scroll one page at a time
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Move_To_Page(const char *widget_name, bool value)
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
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_MOVE_TO_PAGE);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_WIDGET_SCROLL_VIEW);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
		    yyjson_mut_obj_add_bool(doc, root, HMI_WIDGET_PROP_VALUE, value);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* ST<{"cmd_code":"set_move_to_page","type":"scroll_view","widget":"scroll_view1","value":true} */
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
**  @FunctionName       Stone_HMI_Set_Scroll_To_X
**  @Description        Stone HMI Sets scroll to the specified offset
**  @InputParam         widget_name: widget name
**						          xoffset: Offset in the X direction
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Scroll_To_X(const char *widget_name, int xoffset)
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
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_SCROLL_TO);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_WIDGET_SCROLL_VIEW);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
		    yyjson_mut_obj_add_int(doc, root, HMI_WIDGET_PROP_XOFFSET, xoffset);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* ST<{"cmd_code":"set_scroll_to","type":"scroll_view","widget":"scroll_view1","xoffset":50} */
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
**  @FunctionName       Stone_HMI_Set_Scroll_To_Y
**  @Description        Stone HMI Sets scroll to the specified offset
**  @InputParam         widget_name: widget name
**						          yoffset: Offset in the Y direction
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Scroll_To_Y(const char *widget_name, int yoffset)
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
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_SCROLL_TO);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_WIDGET_SCROLL_VIEW);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
		    yyjson_mut_obj_add_int(doc, root, HMI_WIDGET_PROP_YOFFSET, yoffset);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* ST<{"cmd_code":"set_scroll_to","type":"scroll_view","widget":"scroll_view1","yoffset":50} */
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
**  @FunctionName       Stone_HMI_Set_Scroll_To
**  @Description        Stone HMI Sets scroll to the specified offset
**  @InputParam         widget_name: widget name
**						          xoffset: Offset in the X direction
**						          yoffset: Offset in the Y direction
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Scroll_To(const char *widget_name, int xoffset, int yoffset)
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
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_SCROLL_TO);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_WIDGET_SCROLL_VIEW);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
        yyjson_mut_obj_add_int(doc, root, HMI_WIDGET_PROP_XOFFSET, xoffset);
        yyjson_mut_obj_add_int(doc, root, HMI_WIDGET_PROP_YOFFSET, yoffset);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* ST<{"cmd_code":"set_scroll_to","type":"scroll_view","widget":"scroll_view1","xoffset":50",yoffset":50} */
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
**  @FunctionName       Stone_HMI_Set_Scroll_Delta_To_X
**  @Description        Stone HMI Sets the offset specified by scrolling
**  @InputParam         widget_name: widget name
**						          xoffset: Offset in the X direction
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Scroll_Delta_To_X(const char *widget_name, int xoffset)
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
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_SCROLL_DELTA_TO);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_WIDGET_SCROLL_VIEW);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
		    yyjson_mut_obj_add_int(doc, root, HMI_WIDGET_PROP_XOFFSET, xoffset);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* ST<{"cmd_code":"set_scroll_delta_to","type":"scroll_view","widget":"scroll_view1","xoffset":50} */
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
**  @FunctionName       Stone_HMI_Set_Scroll_Delta_To_Y
**  @Description        Stone HMI Sets the offset specified by scrolling
**  @InputParam         widget_name: widget name
**						          yoffset	: Offset in the Y direction
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Scroll_Delta_To_Y(const char *widget_name, int yoffset)
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
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_SCROLL_DELTA_TO);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_WIDGET_SCROLL_VIEW);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
		    yyjson_mut_obj_add_int(doc, root, HMI_WIDGET_PROP_YOFFSET, yoffset);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* ST<{"cmd_code":"set_scroll_delta_to","type":"scroll_view","widget":"scroll_view1","Yoffset":50} */
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
**  @FunctionName       Stone_HMI_Set_Scroll_Delta_To
**  @Description        Stone HMI Sets scroll to the specified offset
**  @InputParam         widget_name: widget name
**						          xoffset: Offset in the X direction
**						          yoffset: Offset in the Y direction
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Scroll_Delta_To(const char *widget_name, int xoffset, int yoffset)
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
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_SCROLL_DELTA_TO);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_WIDGET_SCROLL_VIEW);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
        yyjson_mut_obj_add_int(doc, root, HMI_WIDGET_PROP_XOFFSET, xoffset);
        yyjson_mut_obj_add_int(doc, root, HMI_WIDGET_PROP_YOFFSET, yoffset);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* ST<{"cmd_code":"set_scroll_delta_to","type":"scroll_view","widget":"scroll_view1","xoffset":50",yoffset":50} */
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
**  @FunctionName       Stone_HMI_Set_Height
**  @Description        Stone HMI Sets the height of the list item
**  @InputParam         widget_name: widget name
**						          height: the height of the list item
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Height(const char *widget_name, unsigned short height)
{
    char ret = 0;

    if(widget_name==NULL || height<=0)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_HEIGHT);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_WIDGET_LIST_VIEW);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
		    yyjson_mut_obj_add_uint(doc, root, HMI_WIDGET_PROP_HEIGHT, height);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* ST<{"cmd_code":"set_height","type":"list_view","widget":"list_view0","height":40"} */
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
**  @FunctionName       Stone_HMI_Set_Width
**  @Description        Stone HMI Sets the width of the list item
**  @InputParam         widget_name: widget name
**						          width: the width of the list item
**  @OutputParam		    NONE
**  @ReturnValue        0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Set_Width(const char *widget_name, unsigned short width)
{
    char ret = 0;

    if(widget_name==NULL || width<=0)
    {
        return ret;
    }
    // Create a mutable doc
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    if(root)
    {
        yyjson_mut_doc_set_root(doc, root);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_CODE, HMI_CMD_SET_WIDTH);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_TYPE, HMI_WIDGET_LIST_VIEW_H);
        yyjson_mut_obj_add_str(doc, root, HMI_CMD_WIDGET, widget_name);
		    yyjson_mut_obj_add_uint(doc, root, HMI_WIDGET_PROP_WIDTH, width);
        const char *json = yyjson_mut_write(doc, 0, NULL);
        if (json)
        {
            /* ST<{"cmd_code":"set_width","type":"list_view_h","widget":"list_view_h3","width":60"} */
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


