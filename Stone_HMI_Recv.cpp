#include "HardwareSerial.h"
#include <Arduino.h>
#include "Stone_HMI_Define.h"

byte data[200];   
extern HMI_ProtocolMsg hmi_msg;    

/*********************************************************************************************************
**  @FunctionName               calculateCRC16Modbus
**  @Description                Stone HMI Calculate CRC16 checksum
**  @InputParam                 buf:	    data
**                              length:   data len
**  @OutputParam				        NONE
**  @ReturnValue                CRC16 value
**
*********************************************************************************************************/
uint16_t calculateCRC16Modbus(const uint8_t *data, size_t length)
{
  uint16_t crc = 0xFFFF; 
  for (size_t i = 0; i < length; i++) {
    crc ^= (data[i]); 
    for (uint8_t j = 0; j < 8; j++) { 
      if (crc & 0x0001) {
        crc = (crc >> 1) ^ 0xA001; 
      } else {
        crc >>= 1; 
      }
    }
  }
  return crc;
}

/*********************************************************************************************************
**  @FunctionName               Uart_HMI_Data_Receive
**  @Description                Data reception and processing
**  @InputParam                 NONE
**  @OutputParam				        NONE
**  @ReturnValue                CRC16 value
**
*********************************************************************************************************/
void Uart_HMI_Data_Receive()
{
  int index=0;
  unsigned int length =0;   //The length of the complete data frame
  int start = 0;            //This parameter is used to determine the frame header(ST<) length
  int end =0;               //This parameter is used to determine the frame end(>ET) length
  int crc_num = 0;          //Used to determine whether two CRC check bits are received
  
  while(Serial2.available()>0){
    byte receiveByte = Serial2.read();

    if(start==0&&receiveByte==0x53){
      start = 1;
      data[index++]=0x53;
    }
    else if(start==1&&receiveByte==0x54){
      start = 2;
      data[index++]=0x54;
    }
    else if(start==2&&receiveByte==0x3C){
      start = 3;
      data[index++]=0x3C;
    }
    else if(start==3&&receiveByte!=0x3E&&end==0){
      data[index++] = receiveByte;
    }
    
    else if(start==3&&receiveByte==0x3E){
      end = 1;
      data[index++] = 0x3E;
    }
    else if(end==1&&receiveByte==0x45)
    {
      end = 2;
      data[index++]=0x45;
    }
    else if(end==2&&receiveByte==0x54){
        end = 3;
        data[index++]=0x54;
    }
    else if(end==3&&crc_num<=2){
      data[index++] = receiveByte;
      crc_num++;
      if(crc_num==2)
      {
        break;
      }
    }
    else{
      start = 0;
      end = 0;
      index = 0;
    }
    
  }
  //Print the received data
  if (index > 0) {
    length = index;
    for (int i = 0; i < length; i++) {
      //Serial.print(data[i],HEX);
      //Serial.print(" ");
    }
    //Serial.println(); 
  }

  uint16_t expectedCRC = (data[length-2] << 8) | data[length-1];    //The received CRC check bit
  uint16_t calculatedCRC = calculateCRC16Modbus(data, length-2);    //The CRC check bit is calculated from the received data

  //Print data message
  if (length>0 && calculatedCRC == expectedCRC) {
    //Serial.println("CRC Check success！");
    uint32_t num = data[3]<<8 | data[4];            
    //Serial.print("CMD：");
    //Serial.print("0x");
    //Serial.println(num,HEX);
    uint16_t data_length = data[5]<<8 | data[6];    
    //Serial.print("DATA_LEN：");
    //Serial.print("0x");
    //Serial.println(data_length,HEX);
    //Serial.print("HEX_DATA: ");
    for(int i=7;i<data_length+7;i++)
    {
      //Serial.print(data[i],HEX);
      //Serial.print(" ");
    }
    //Serial.println();
    //Serial.print("String_Data: ");
    for(int i=7;i<data_length+7;i++)
    {
      //Serial.print(char(data[i]));
    }
    //Serial.println();
    //Serial.println();

    //Store data to a structure
    hmi_msg.len = data_length;
    hmi_msg.crc = expectedCRC;
    hmi_msg.cmd = num;
    memcpy(hmi_msg.data, &data[7], data_length);

  } 
  else if(length>0 && calculatedCRC != expectedCRC){
    //Serial.println("CRC Check failure！");
    hmi_msg.len = 0;
  }
  else
  {
    hmi_msg.len = 0;
  }
  // Uart_HMI_Data_Analysis(data,length);
}


/* USER CODE BEGIN Stone_HMI_Recv */

#define UART_MAX_SIZE 256
#define UART_BUFFER_MAX_LEN UART_MAX_SIZE*2
uint8_t Uart_RecvTotalBuf[UART_BUFFER_MAX_LEN] = {0};
uint16_t Uart_RevTotalLen = 0;

/*********************************************************************************************************
**  @FunctionName               CRC16_CCITT
**  @Description                Stone HMI Calculate CRC16 checksum
**  @InputParam                 buf:	data
**                              len:    data len
**  @OutputParam				        NONE
**  @ReturnValue                CRC16 value
**
*********************************************************************************************************/
uint16_t CRC16_CCITT(const uint8_t *buf, size_t len)
{
    uint16_t CRC16 = 0xFFFF;
    while (len--)
    {
        CRC16 ^= *buf++;
        for (uint8_t i = 0; i < 8; i++)
        {
            if (CRC16 & 0x0001)
            {
                CRC16 >>= 1;
                CRC16 ^= 0xA001;
            } else
            {
                CRC16 >>= 1;
            }
        }
    }
    return CRC16;
}

/*********************************************************************************************************
**  @FunctionName               Uart_GUI_Data_Analysis
**  @Description                Parsing Uart port related HMI commands
**  @InputParam                 rev:	uart rev data
**                              size:    data len
**  @OutputParam				        NONE
**  @ReturnValue                0:Fail; 1:OK
**
*********************************************************************************************************/
uint8_t Uart_HMI_Data_Analysis(uint8_t *rev, uint16_t size)
{
    uint8_t ret = 0;
//    uint16_t tempCRC = 0;
    uint16_t len = 0;
//    uint32_t temp = 0;
    uint16_t temp_crc = 0;
    uint16_t temp_len = 0;
    uint16_t next_start_index = 0;
    HMI_ProtocolMsg hmi_msg;

    if(rev==NULL || size <= 0)
    {
        return ret;
    }

    /*
    header     CMD     len           data        trailer       CRC
    53 54 3C  10 02   00 06   73 75 6D 36 06 03   3E 45 54   E8 58
    00 01 02  03 04   05 06   07 08 09 10 11 12   13 14 15   16 17
    */
    if(Uart_RevTotalLen+size <= UART_BUFFER_MAX_LEN)
    {
        memcpy(&Uart_RecvTotalBuf[Uart_RevTotalLen],  rev, size);
        Uart_RevTotalLen += size;
    }
    else
    {
        temp_len = abs(Uart_RevTotalLen - next_start_index);
        memcpy(&Uart_RecvTotalBuf[0], &Uart_RecvTotalBuf[next_start_index], temp_len);
        memcpy(&Uart_RecvTotalBuf[temp_len], rev, size);
        Uart_RevTotalLen = temp_len + size;
        next_start_index = 0;
    }
//    printf("Uart_RevTotalLen = %d\n", Uart_RevTotalLen);
    if(Uart_RevTotalLen > 12)/* The minimum length of a frame of data is greater than 12 bytes */
    {
        for(int i=next_start_index; i<Uart_RevTotalLen; i++)
        {
            if(Uart_RecvTotalBuf[i] == HMI_DATA_HEARD1 && (i+12)<Uart_RevTotalLen && Uart_RecvTotalBuf[i+1] == HMI_DATA_HEARD2 && Uart_RecvTotalBuf[i+2] == HMI_DATA_HEARD3) /* find the header */
            {
                len = (Uart_RecvTotalBuf[i+5]<<8) | Uart_RecvTotalBuf[i+6];
                if(Uart_RecvTotalBuf[i+len+7]==HMI_DATA_END1 && (i+len+7+5)<=Uart_RevTotalLen && Uart_RecvTotalBuf[i+len+7+1] == HMI_DATA_END2 && Uart_RecvTotalBuf[i+len+7+2] == HMI_DATA_END3)
                {
                    temp_crc = CRC16_CCITT(&Uart_RecvTotalBuf[i], len+7+3);
                    if(temp_crc == ((Uart_RecvTotalBuf[i+len+7+3]<<8)|Uart_RecvTotalBuf[i+len+7+4]))
                    {
                        hmi_msg.len = len;
//                                hmi_msg.crc = temp_crc;
                        hmi_msg.cmd = (Uart_RecvTotalBuf[i+3]<<8) | Uart_RecvTotalBuf[i+4];
                        memcpy(hmi_msg.data, &Uart_RecvTotalBuf[i+7], len);
                        // if(osMessageQueuePut(Queue_Uart1_DealHandle, &hmi_msg, NULL, 0) != osOK)
                        // {
                        //     printf("Q Send Err\r\n");
                        // }
                        next_start_index = i+len+7+5; /* Next frame data */
                        i = next_start_index-1;
                        ret = 1;
//                        printf("crc ok %04X, len %d, next %d\n",temp_crc, len, next_start_index);
                    }
                }

            }
        }
        if(next_start_index>=Uart_RevTotalLen)
        {
//            printf("data deal done\n");
            next_start_index = 0;
            Uart_RevTotalLen = 0;
        }
        else
        {
            Serial.print("data deal error-------------------------");
            Serial.print(Uart_RevTotalLen);
            Serial.print(", ");
            Serial.println(next_start_index);
        }

    }
    return ret;
}

