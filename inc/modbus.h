/**
  ******************************************************************************
  * File Name          : modbus.h
  * Description        : 
  ******************************************************************************
  * @attention
  
  *
  ******************************************************************************
  */

#ifndef _MODBUS_H_ 
#define _MODBUS_H_

#include <stdint.h>
#include <stdlib.h>

#ifndef FALSE
#define FALSE 0
#endif

#ifndef TRUE
#define TRUE 1
#endif

#ifndef OFF
#define OFF 0
#endif

#ifndef ON
#define ON 1
#endif

#define _MIN_REQ_LENGTH 12
#define MAX_MESSAGE_LENGTH 260
#define MODBUS_DEFAULT_BYTE_TIMEOUT       (50)       /* 毫秒,用于计算一帧的结束 */
#define MODBUS_DEFAULT_RESPONSE_TIMEOUT   (2000)     /* 毫秒 */
#define MODBUS_DEFAULT_INDICATION_TIMEOUT (30*1000)  /* 30秒 */
#define MODBUS_DEFAULT_REG_SIZE (32)

#define _REPORT_SLAVE_ID 180
#define LIBMODBUS_VERSION_STRING "3.1.7"

/* The size of the MODBUS PDU is limited by the size constraint inherited from
 * the first MODBUS implementation on Serial Line network (max. RS485 ADU = 256
 * bytes). Therefore, MODBUS PDU for serial line communication = 256 - Server
 * address (1 byte) - CRC (2 bytes) = 253 bytes.
 */
#define MODBUS_MAX_PDU_LENGTH              253

/* Modbus function codes */
#define MODBUS_FC_READ_COILS                0x01
#define MODBUS_FC_READ_DISCRETE_INPUTS      0x02
#define MODBUS_FC_READ_HOLDING_REGISTERS    0x03
#define MODBUS_FC_READ_INPUT_REGISTERS      0x04
#define MODBUS_FC_WRITE_SINGLE_COIL         0x05
#define MODBUS_FC_WRITE_SINGLE_REGISTER     0x06
#define MODBUS_FC_READ_EXCEPTION_STATUS     0x07
#define MODBUS_FC_WRITE_MULTIPLE_COILS      0x0F
#define MODBUS_FC_WRITE_MULTIPLE_REGISTERS  0x10
#define MODBUS_FC_MASK_WRITE_REGISTER       0x16
#define MODBUS_FC_WRITE_AND_READ_REGISTERS  0x17

#define MODBUS_BROADCAST_ADDRESS    0

/* Protocol exceptions */
enum 
{
    MODBUS_EXCEPTION_ILLEGAL_FUNCTION = 0x01,
    MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS,
    MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE,
    MODBUS_EXCEPTION_SLAVE_OR_SERVER_FAILURE,
    MODBUS_EXCEPTION_ACKNOWLEDGE,
    MODBUS_EXCEPTION_SLAVE_OR_SERVER_BUSY,
    MODBUS_EXCEPTION_NEGATIVE_ACKNOWLEDGE,
    MODBUS_EXCEPTION_MEMORY_PARITY,
    MODBUS_EXCEPTION_NOT_DEFINED,
    MODBUS_EXCEPTION_GATEWAY_PATH,
    MODBUS_EXCEPTION_GATEWAY_TARGET,
    MODBUS_EXCEPTION_MAX
};

typedef struct _modbus_config
{
    uint16_t  reg1MaxSize;                                   /* 寄存器数量 */
    uint16_t  reg2MaxSize;
    uint16_t  reg3MaxSize;
    uint16_t  reg4MaxSize;
    uint32_t  byteTimeout;                             
    uint32_t  responeTimeout;                                /* 应答超时时间 */
    uint32_t  indicationTimeout;                             /* 未收到指令的超时时间 */
}modbus_config_t;

typedef enum
{
    MODBUS_STAT_IDLE             = 0x01,                      
    MODBUS_STAT_BUSY             = 0x02                    
}mosbus_stat_t;

/* This structure reduces the number of params in functions and so
 * optimizes the speed of execution (~ 37%). */
typedef struct _sft 
{
    int slave;
    int function;
}sft_t;

/* modbus错误信息 */
#define MODBUS_ERR_BAD_FRAME          (-2)                  /* 不完整的帧 */
#define MODBUS_ERR_ILLEGAL            (-3)                  /* 0x80回复 */
#define MODBUS_ERR_FC_DIFF            (-4)                  /* 功能码不一致 */
#define MODBUS_ERR_CRC                (-5)                  /* 校验和错误 */
#define MODBUS_ERR_ADDR               (-6)                  /* 地址错误 */
#define MODBUS_ERR_INDICATION_TIMEOUT (-7)                  /* master指令超时 */
#define MODBUS_ERR_RESPONSE_TIMEOUT   (-8)                  /* 应答超时 */

 /*
 *  ---------- Request     Indication ----------
 *  | Client | ---------------------->| Server |
 *  ---------- Confirmation  Response ----------
 */
typedef enum 
{
    /* Request message on the server side */
    MSG_INDICATION,
    /* Request message on the client side */
    MSG_CONFIRMATION
}msg_type_t;

typedef struct _modbus modbus_t;

typedef struct _modbus_mapping_t 
{
    int nbBits;
    int startBits;
    int nbInputBits;
    int startInputBits;
    int nbInputRegisters;
    int startInputRegisters;
    int nbRegisters;
    int startRegisters;
    uint8_t *tabBits;
    uint8_t *tabInputBits;
    uint16_t *tabInputRegisters;
    uint16_t *tabRegisters;
}modbus_mapping_t;

/* MASTER AND SLAVE */
int modbus_init(modbus_t *mb);
void modbus_free(modbus_t *mb);
/* 在用户数据中断中调用，或者接收数据的循环中调用 */
void modbus_receive_msg_with_byte(modbus_t *mb, uint8_t msgData, msg_type_t msgType);

int modbus_set_slave(modbus_t *mb, int slave);
int modbus_get_slave(modbus_t *mb);

int modbus_set_debug(modbus_t *mb, int flag);

int modbus_get_byte_timeout(modbus_t *mb, uint32_t *msec);
int modbus_set_byte_timeout(modbus_t *mb, uint32_t msec);

/* MASTER */
int modbus_get_response_timeout(modbus_t *mb, uint32_t *msec);
int modbus_set_response_timeout(modbus_t *mb, uint32_t msec);

int modbus_set_regmaxsize(modbus_t *mb,
                          uint16_t reg1Size,
                          uint16_t reg2Size,
                          uint16_t reg3Size,
                          uint16_t reg4Size);

int modbus_read_bits(modbus_t *mb, int addr, int nb, uint8_t *dest);
int modbus_read_input_bits(modbus_t *mb, int addr, int nb, uint8_t *dest);
int modbus_read_registers(modbus_t *mb, int addr, int nb, uint16_t *dest);
int modbus_read_input_registers(modbus_t *mb, int addr, int nb, uint16_t *dest);
int modbus_write_bit(modbus_t *mb, int addr, int status);
int modbus_write_register(modbus_t *mb, int addr, const uint16_t value);
int modbus_write_bits(modbus_t *mb, int addr, int nb, const uint8_t *src);
int modbus_write_registers(modbus_t *mb, int addr, int nb, const uint16_t *src);
int modbus_mask_write_register(modbus_t *mb, int addr, uint16_t andMask, uint16_t orMask);
int modbus_write_and_read_registers(modbus_t *mb,
                                    int writeAddr, int writeNb,
                                    const uint16_t *src,
                                    int readAddr, int readNb,
                                    uint16_t *dest);


/* SLVAVE */
modbus_mapping_t* modbus_mapping_new_start_address(
    unsigned int startBits, unsigned int nbBits,
    unsigned int startInputBits, unsigned int nbInputBits,
    unsigned int startRegisters, unsigned int nbRegisters,
    unsigned int startInputRegisters, unsigned int nbInputRegisters);
modbus_mapping_t* modbus_mapping_new(int nbBits, int nbInputBits,
                                     int nbRegisters, int nbInputRegisters);
void modbus_mapping_free(modbus_mapping_t *mbMapping);
int modbus_send_raw_request(modbus_t *mb, const uint8_t *rawReq, int rawReqLength);
int modbus_receive(modbus_t *mb, uint8_t *req);
int modbus_reply(modbus_t *mb, const uint8_t *req,
                 int reqLength, modbus_mapping_t *mbMapping);
int modbus_receive_confirmation(modbus_t *mb, uint8_t *rsp);
int modbus_reply_exception(modbus_t *mb, const uint8_t *req,
                                      unsigned int exceptionCode);

int modbus_get_indication_timeout(modbus_t *mb, uint32_t *msec);
int modbus_set_indication_timeout(modbus_t *mb, uint32_t msec);

/* tool */
void modbus_set_bits_from_byte(uint8_t *dest, int idx, const uint8_t value);
void modbus_set_bits_from_bytes(uint8_t *dest, int idx, unsigned int nbBits,
                                const uint8_t *tabByte);
uint8_t modbus_get_byte_from_bits(const uint8_t *src, int idx, unsigned int nbBits);
#endif

/**********************************END OF FILE********************************/
