/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : Stone_HMI_Send.c
  * Description        : Code for Stone HMI CMD Send  applications
                         Developers who transplant the STONE HMI library generally only need to modify this file;
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
//#include "FreeRTOS.h"
//#include "task.h"
//#include "cmsis_os.h"


/* USER CODE BEGIN Includes */
#include <Arduino.h>
#include "Stone_HMI_Define.h"
// #include "Thread_Define.h"


/* USER CODE END Includes */


/* USER CODE BEGIN Header_Thread_HMI_Send */


/*********************************************************************************************************
**  @FunctionName               Stone_HMI_Cmd_Send
**  @Description                Stone HMI Send CMD
**  @InputParam                 json_buf:		json data
**  @OutputParam				        NONE
**  @ReturnValue                0:Fail; 1:OK
**
*********************************************************************************************************/
char Stone_HMI_Cmd_Send(const char *json_buf)
{
  char ret = 0;
  unsigned short len = 0;

  if(json_buf == NULL)
  {
      return ret;
  }
  len = strlen(json_buf);
  if(len + 6 < HMI_MAX_SEND_LEN)
  {
    /* The maximum sending space is 1024 bytes, and the user can adjust the space size according to their own situation */
    char send_buf[HMI_MAX_SEND_LEN] = {0};
    ret = Stone_HMI_Transmit_Data(send_buf, json_buf, len);
  }
  else
  {
    Serial.println("json use malloc");
    /* If the data length exceeds HMI_MAX_SEND_LEN bytes, apply for dynamic memory to send */
    char *send_buf = (char*)malloc(len + 7);
    if(send_buf)
    {
      send_buf[len + 6] = 0;
      ret = Stone_HMI_Transmit_Data(send_buf, json_buf, len);
      free(send_buf);
    }
  }
  return ret;
}

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
char Stone_HMI_Transmit_Data(char *cmd_buf, const char *json_buf, unsigned short json_len)
{
  char ret = 0;
  char result = 0;
  unsigned short num = 0;
  if(cmd_buf==NULL || json_buf==NULL || json_len ==0)
  {
      return ret;
  }
  cmd_buf[0] = HMI_DATA_HEARD1;   //Add frame header
  cmd_buf[1] = HMI_DATA_HEARD2;
  cmd_buf[2] = HMI_DATA_HEARD3;
  memcpy(&cmd_buf[3], json_buf, json_len);
  cmd_buf[json_len + 3] = HMI_DATA_END1;  //Add frame tail
  cmd_buf[json_len + 4] = HMI_DATA_END2;
  cmd_buf[json_len + 5] = HMI_DATA_END3;
  Serial2.println(cmd_buf);
  return ret;
}


/* USER CODE END Thread_GUI */

