/**
  ******************************************************************************
  * File Name          : example.c
  * Description        : This file contains all functions about modbus example.
  ******************************************************************************
  * @attention
  * 
  *
  ******************************************************************************
  */

  
#include "main.h"
#include <stdio.h>
#include "log.h"
#include "usart.h"
#include "modbus.h"
#include "modbus-rtu.h"

uint8_t rxbuffData;
modbus_t *modbus;
/*****************************************************************************
 Function    : Private_Comm_Start_Rec
 Description : None
 Input       : None
 Output      : None
 Return      : None
 *****************************************************************************/
void Private_Comm_Start_Rec(void)
{
    HAL_UART_Receive_IT(&huart2, &rxbuffData, 1);
}
/*****************************************************************************
 Function    : HAL_UART_RxCpltCallback
 Description : None
 Input       : None
 Output      : None
 Return      : None
 *****************************************************************************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart == &huart2)                              
    {
        modbus_receive_msg_with_byte(modbus, rxbuffData, MSG_CONFIRMATION);
        HAL_UART_Receive_IT(&huart2, &rxbuffData, 1);
    }
}
/*****************************************************************************
 Function    : modbus_delayms_callback
 Description : 毫秒延时函数
 Input       : None
 Output      : None
 Return      : None
 *****************************************************************************/
void modbus_delayms_callback(void)
{
    osDelay(1);
}
/*****************************************************************************
 Function    : modbus_send_msg_callback
 Description : None
 Input       : None
 Output      : None
 Return      : 0 成功 -1 失败
 *****************************************************************************/
int modbus_send_msg_callback(modbus_t *mb, const uint8_t *data, 
                                    uint16_t length)
{
    HAL_StatusTypeDef res = HAL_OK;
    res = HAL_UART_Transmit_IT(&huart2, (uint8_t *)data, length);
    if(res == HAL_OK)
        return 0;
    else
    {
        log_err("HAL_UART_Transmit_IT err is %d\n", res);
        return -1;
    }      
}
/*****************************************************************************
 Function    : modbus_malloc_callback
 Description : None
 Input       : None
 Output      : None
 Return      : None
 *****************************************************************************/
void* modbus_malloc_callback(uint32_t size)
{
    return pvPortMalloc(size);
}
/*****************************************************************************
 Function    : modbus_free_callback
 Description : None
 Input       : None
 Output      : None
 Return      : None
 *****************************************************************************/
void modbus_free_callback(void *data)
{
    return vPortFree(data);
}
/*****************************************************************************
 Function    : modbus_task
 Description : None
 Input       : None
 Output      : None
 Return      : None
 *****************************************************************************/
void modbus_slave(void)
{
    int res;
    modbus_mapping_t *mbMap;
    modbus = modbus_new_rtu();
    uint8_t query[256];
    if(modbus == NULL)
        return;
    res = modbus_init(modbus); 
    if(res < 0)
    {
        modbus_free(modbus);
        return;
    }
    res = modbus_set_slave(modbus, 1);
    if(res < 0)
    {
        modbus_free(modbus);
        return;
    }
    mbMap = modbus_mapping_new(32, 32, 32, 32);
    if(mbMap == NULL) 
    {
        log_err("Failed to allocate the mapping\n");
        modbus_free(modbus);
        return;
    }
    while(1)
    {
        res = modbus_receive(modbus, query);
        if(res > 0) 
        {
            modbus_reply(modbus, query, res, mbMap);
        } 
        osDelay(100);
    }

    modbus_mapping_free(mbMap);

    modbus_free(modbus);
}
/*****************************************************************************
 Function    : modbus_task
 Description : None
 Input       : None
 Output      : None
 Return      : None
 *****************************************************************************/
void modbus_master(void)
{
    int res = 0;
    int i;
    uint8_t dest[32];
    uint16_t destReg[32];
    uint8_t src[2] ={ON, ON};
    uint16_t srcReg[2] ={15, 26};
    modbus = modbus_new_rtu();
    if(modbus == NULL)
        return;
    res = modbus_init(modbus); 
    if(res < 0)
    {
        modbus_free(modbus);
        return;
    }
    res = modbus_set_slave(modbus, 1);
    if(res < 0)
    {
        modbus_free(modbus);
        return;
    }
    while(1)
    {
        res = modbus_read_bits(modbus, 0, 2, dest);
        if(res > 0)
        {
            log_none("rev msg:\n");
            for(i = 0; i < res; i++)
                log_none("%02x ", dest[i]);
            log_none("\n");
        }
        res = modbus_read_input_bits(modbus, 0, 2, dest);
        if(res > 0)
        {
            log_none("rev msg:\n");
            for(i = 0; i < res; i++)
                log_none("%02x ", dest[i]);
            log_none("\n");
        }
        res = modbus_read_registers(modbus, 0, 2, destReg);
        if(res > 0)
        {
            log_none("rev msg:\n");
            for(i = 0; i < res; i++)
                log_none("%04x ", destReg[i]);
            log_none("\n");
        }
        res = modbus_read_input_registers(modbus, 0, 2, destReg);
        if(res > 0)
        {
            log_none("rev msg:\n");
            for(i = 0; i < res; i++)
                log_none("%04x ", destReg[i]);
            log_none("\n");
        }
        res = modbus_write_bit(modbus, 0, ON);
        if(res > 0)
        {
            log_none("modbus_write_bit ok\n");
        }
        res = modbus_write_bits(modbus, 0, 2, src);
        if(res > 0)
        {
            log_none("modbus_write_bits ok\n");
        }
        res = modbus_write_register(modbus, 0, 1600);
        if(res > 0)
        {
            log_none("modbus_write_register ok\n");
        }
        res = modbus_write_registers(modbus, 0, 2, srcReg);
        if(res > 0)
        {
            log_none("modbus_write_registers ok\n");
        }
        srcReg[0] = 16;
        srcReg[1] = 27;
        res = modbus_write_and_read_registers(modbus, 0, 2, srcReg, 0, 2, destReg);
        if(res > 0)
        {
            log_none("modbus_write_and_read_registers ok\n");
            log_none("rev msg:\n");
            for(i = 0; i < res; i++)
                log_none("%04x ", destReg[i]);
            log_none("\n");
        }
        res = modbus_mask_write_register(modbus, 0, 0xFFFE, 0x01);
        if(res > 0)
        {
            log_none("modbus_mask_write_register ok\n");
        }
        osDelay(3000);
    }
    modbus_free(modbus);
}
/************************ZXDQ *****END OF FILE********************************/
