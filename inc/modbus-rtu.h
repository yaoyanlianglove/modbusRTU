/**
  ******************************************************************************
  * File Name          : modbus-rtu.h
  * Description        : 
  ******************************************************************************
  * @attention
  
  *
  ******************************************************************************
  */

#ifndef _MODBUS_RTU_H_ 
#define _MODBUS_RTU_H_

#include <stdint.h>
#include <stdlib.h>
#include "modbus.h"

#define _MODBUS_RTU_HEADER_LENGTH      1
#define _MODBUS_RTU_CHECKSUM_LENGTH    2
#define _MODBUS_RTU_PRESET_REQ_LENGTH  6
#define _MODBUS_RTU_PRESET_RSP_LENGTH  2

#define MODBUS_RTU_MAX_ADU_LENGTH  256



typedef struct _modbus_backend
{
    uint16_t headerLength;
    uint16_t checksumLength;
    int (*set_slave) (modbus_t *mb, int slave);
    int (*build_request_basis) (modbus_t *mb, int function, int addr,
                                int nb, uint8_t *req);
    int (*send_msg_pre) (uint8_t *data, int length);
    int (*send) (modbus_t *mb, const uint8_t *data, int length);
    int (*check_integrity) (modbus_t *mb, uint8_t *msg,
                            const int length);
    int (*pre_check_confirmation) (modbus_t *ctx, const uint8_t *req,
                                   const uint8_t *rsp, int rspLength);
    int (*receive) (modbus_t *mb, uint8_t *req);
    int (*build_response_basis) (sft_t *sft, uint8_t *rsp);
    void (*free) (modbus_t *mb);
} modbus_backend_t;


struct _modbus
{
    mosbus_stat_t                stat;
    void                         *uartHandle;                 /* 串口句柄 */
    uint8_t                      *rxBuffer;
    uint8_t                      *txBuffer;
    uint8_t                      rxCount;
    uint8_t                      debug;                       /* 0不打印调试信息 1打印调试信息 */
    uint8_t                      slave;                       /* 从机地址 */
    uint32_t                     errCode;                     /* modbus错误码 */
    modbus_config_t              *cfg;                        /* modbus配置 */
    const modbus_backend_t       *backend;             
};

modbus_t* modbus_new_rtu(void);

int _modbus_receive_msg(modbus_t *mb, uint8_t *msg, msg_type_t msg_type);

void modbus_delayms_callback(void);
int modbus_send_msg_callback(modbus_t *mb, const uint8_t *data, uint16_t length);
void* modbus_malloc_callback(uint32_t size);
void modbus_free_callback(void *data);
void modbus_indication_timeout_callback(modbus_t *mb);

#endif

/**********************************END OF FILE********************************/
