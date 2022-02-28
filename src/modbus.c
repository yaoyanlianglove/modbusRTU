/**
  ******************************************************************************
  * File Name          : modbus.c
  * Description        : This file contains all functions about modbus.
  ******************************************************************************
  * @attention
  * 
  *
  ******************************************************************************
  */

  
#include "main.h"
#include <string.h>
#include "log.h"
#include "modbus.h"
#include "modbus-rtu.h"


/* Internal use */
#define MSG_LENGTH_UNDEFINED -1

/* 打印异常码 */
void modbus_print_errcode(int err)
{
    switch (err)
    {
    case MODBUS_EXCEPTION_ILLEGAL_FUNCTION:
        log_none("MODBUS_EXCEPTION_ILLEGAL_FUNCTION\n");
        break;
    case MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS:
        log_none("MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS\n");
        break;
    case MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE:
        log_none("MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE\n");
        break;
    case MODBUS_EXCEPTION_SLAVE_OR_SERVER_FAILURE:
        log_none("MODBUS_EXCEPTION_SLAVE_OR_SERVER_FAILURE\n");
        break;
    case MODBUS_EXCEPTION_ACKNOWLEDGE:
        log_none("MODBUS_EXCEPTION_ACKNOWLEDGE\n");
        break;
    case MODBUS_EXCEPTION_SLAVE_OR_SERVER_BUSY:
        log_none("MODBUS_EXCEPTION_SLAVE_OR_SERVER_BUSY\n");
        break;
    case MODBUS_EXCEPTION_NEGATIVE_ACKNOWLEDGE:
        log_none("MODBUS_EXCEPTION_NEGATIVE_ACKNOWLEDGE\n");
        break;
    case MODBUS_EXCEPTION_MEMORY_PARITY:
        log_none("MODBUS_EXCEPTION_MEMORY_PARITY\n");
        break;
    case MODBUS_EXCEPTION_NOT_DEFINED:
        log_none("MODBUS_EXCEPTION_NOT_DEFINED\n");
        break;
    case MODBUS_EXCEPTION_GATEWAY_PATH:
        log_none("MODBUS_EXCEPTION_GATEWAY_PATH\n");
        break;
    case MODBUS_EXCEPTION_GATEWAY_TARGET:
        log_none("MODBUS_EXCEPTION_GATEWAY_TARGET\n");
        break;
    default:
        break;
    }
}
/* modbus初始化，必须调用 */
int modbus_init(modbus_t *mb)
{
    if(mb == NULL)
    {
        return -1;
    }
    mb->cfg->reg1MaxSize = MODBUS_DEFAULT_REG_SIZE;
    mb->cfg->reg2MaxSize = MODBUS_DEFAULT_REG_SIZE;
    mb->cfg->reg3MaxSize = MODBUS_DEFAULT_REG_SIZE;
    mb->cfg->reg4MaxSize = MODBUS_DEFAULT_REG_SIZE;

    mb->cfg->byteTimeout = MODBUS_DEFAULT_BYTE_TIMEOUT;
    mb->cfg->responeTimeout = MODBUS_DEFAULT_RESPONSE_TIMEOUT;
    mb->cfg->indicationTimeout = MODBUS_DEFAULT_INDICATION_TIMEOUT;

    mb->debug = TRUE;
    mb->errCode = 0;
    mb->rxCount = 0;

    /* slave必须调用modbus_set_slave进行设置 */
    mb->slave = -1;
    mb->stat  = MODBUS_STAT_IDLE;

    return 0;
}
void modbus_free(modbus_t *mb)
{
    if(mb == NULL)
        return;

    mb->backend->free(mb);
}
int modbus_get_byte_timeout(modbus_t *mb, uint32_t *msec)
{
    if(mb == NULL) 
    {
        return -1;
    }
    *msec = mb->cfg->byteTimeout;
    return 0;
}

int modbus_set_byte_timeout(modbus_t *mb, uint32_t msec)
{
    if(mb == NULL) 
    {
        return -1;
    }
    mb->cfg->byteTimeout = msec;
    return 0;
}
int modbus_set_debug(modbus_t *mb, int flag)
{
    if(mb == NULL) 
    {
        return -1;
    }
    mb->debug = flag;
    return 0;
}

/* 在中断程序中调用，用于发送数据后，接收数据通过stat来通知数据处理线程进行数据处理 */
void modbus_receive_msg_with_byte(modbus_t *mb, uint8_t msgData, msg_type_t msgType)
{
    if(mb == NULL)
    {
        return;
    }
    if(mb->stat == MODBUS_STAT_IDLE)
    {
        mb->rxBuffer[mb->rxCount] = msgData;
        if(mb->rxCount < 255)
            mb->rxCount++;
        else
            mb->rxCount = 0;
    }
}
/* 数据发送 */
static int send_msg(modbus_t *mb, uint8_t *msg, int msgLength)
{
    int res;
    int i;

    msgLength = mb->backend->send_msg_pre(msg, msgLength);

    if(mb->debug) 
    {
        log_none("send msg:\n");
        for (i = 0; i < msgLength; i++)
            log_none("[%.2X]", msg[i]);
        log_none("\n");
    }
    res = mb->backend->send(mb, msg, msgLength);

    /* 发送完成，准备接收数据 */
    mb->rxCount = 0;
    mb->stat = MODBUS_STAT_IDLE;
    
    return res;
}
/* 数据接收，并验证校验和,返回接收数据的字节长度
  返回：
  0 ok
  MODBUS_ERR_RESPONSE_TIMEOUT
  MODBUS_ERR_INDICATION_TIMEOUT
  MODBUS_ERR_ADDR
  MODBUS_ERR_CRC */
int _modbus_receive_msg(modbus_t *mb, uint8_t *msg, msg_type_t msgType)
{
    int res = 0;
    int i;
    int oldLength = 0;
    int length = 0;
    uint32_t cnt = 0;
    uint32_t cntByte = 0;
    uint32_t timeout;
    uint32_t byteTimeout;
    if(mb == NULL)
        return -1;
    if(msgType == MSG_CONFIRMATION)
        timeout = mb->cfg->responeTimeout;
    else if(msgType == MSG_INDICATION)
        timeout = mb->cfg->indicationTimeout;
    byteTimeout = mb->cfg->byteTimeout;
    while(cnt < timeout)
    {
        cnt++;
        /* 接收到第一个字节后开始计时 */
        if(mb->rxCount > 0)
        {
            if(cntByte < byteTimeout)
                cntByte++;
            else
            {
                mb->stat = MODBUS_STAT_BUSY;
                length = mb->rxCount;
                mb->rxCount = 0;
                if(mb->debug)
                {
                    log_none("rev msg:\n");
                }
                for(i = 0; i < length; i++)
                {
                    msg[i] = mb->rxBuffer[i];
                    if(mb->debug)
                        log_none("[%.2X]",msg[i]);
                }
                if(mb->debug)
                {
                    log_none("\n");
                }
                mb->stat = MODBUS_STAT_IDLE;
                return mb->backend->check_integrity(mb, msg, length);
            }
            if(mb->rxCount != oldLength)
            {
                oldLength = mb->rxCount;
                cntByte = 0;
            }
        }
        modbus_delayms_callback();
    }
    mb->rxCount = 0;
    if(cnt >= timeout)
    {
        if(msgType == MSG_CONFIRMATION)
        {
            mb->errCode |= MODBUS_ERR_RESPONSE_TIMEOUT;
            log_warn("Modbus respone timeout\n");
            res = MODBUS_ERR_RESPONSE_TIMEOUT;
        }
        else
        {
            modbus_indication_timeout_callback(mb);
            mb->errCode |= MODBUS_ERR_INDICATION_TIMEOUT;
            log_warn("Modbus indication timeout\n");
            res = MODBUS_ERR_INDICATION_TIMEOUT;
        }
    }
    return res;
}
static unsigned int compute_response_length_from_request(modbus_t *mb, uint8_t *req)
{
    int length;
    const int offset = mb->backend->headerLength;

    switch (req[offset]) 
    {
    case MODBUS_FC_READ_COILS:
    case MODBUS_FC_READ_DISCRETE_INPUTS: 
    {
        /* Header + nb values (code from write_bits) */
        int nb = (req[offset + 3] << 8) | req[offset + 4];
        length = 2 + (nb / 8) + ((nb % 8) ? 1 : 0);
    }
        break;
    case MODBUS_FC_WRITE_AND_READ_REGISTERS:
    case MODBUS_FC_READ_HOLDING_REGISTERS:
    case MODBUS_FC_READ_INPUT_REGISTERS:
        /* Header + 2 * nb values */
        length = 2 + 2 * (req[offset + 3] << 8 | req[offset + 4]);
        break;
    case MODBUS_FC_READ_EXCEPTION_STATUS:
        length = 3;
        break;
    case MODBUS_FC_MASK_WRITE_REGISTER:
        length = 7;
        break;
    default:
        length = 5;
    }

    return offset + length + mb->backend->checksumLength;
}
static int check_confirmation(modbus_t *mb, uint8_t *req,
                              uint8_t *rsp, int rspLength)
{
    int res;
    int rspLengthComputed;
    const int offset = mb->backend->headerLength;
    const int function = rsp[offset];

    if(mb->backend->pre_check_confirmation) 
    {
        res = mb->backend->pre_check_confirmation(mb, req, rsp, rspLength);
        if(res == -1) 
        {
            return -1;
        }
    }

    rspLengthComputed = compute_response_length_from_request(mb, req);

    /* Exception code */
    if(function >= 0x80) 
    {
        if (rspLength == (offset + 2 + (int)mb->backend->checksumLength) &&
            req[offset] == (rsp[offset] - 0x80)) 
        {
            mb->errCode |= MODBUS_ERR_ILLEGAL;
            log_warn("Modbus rev errCode is %d\n", rsp[2]);
            modbus_print_errcode(rsp[2]);
            return -1;
        } 
        else 
        {
            mb->errCode |= MODBUS_ERR_BAD_FRAME;
            log_warn("Modbus rev bad frame\n");/* modbus的帧不完整 */
            return -1;
        }
    }

    /* Check length */
    if ((rspLength == rspLengthComputed ||
         rspLengthComputed == MSG_LENGTH_UNDEFINED) &&
        function < 0x80) 
    {
        int reqNbValue;
        int rspNbValue;

        /* Check function code */
        if(function != req[offset]) 
        {
            if(mb->debug) 
            {
                mb->errCode |= MODBUS_ERR_FC_DIFF;
                log_warn("Modbus rev function is %d, send function is %d\n", req[offset], function);
            }
            return -1;
        }

        /* Check the number of values is corresponding to the request */
        switch(function) 
        {
        case MODBUS_FC_READ_COILS:
        case MODBUS_FC_READ_DISCRETE_INPUTS:
            /* Read functions, 8 values in a byte (nb
             * of values in the request and byte count in
             * the response. */
            reqNbValue = (req[offset + 3] << 8) + req[offset + 4];
            reqNbValue = (reqNbValue / 8) + ((reqNbValue % 8) ? 1 : 0);
            rspNbValue = rsp[offset + 1];
            break;
        case MODBUS_FC_WRITE_AND_READ_REGISTERS:
        case MODBUS_FC_READ_HOLDING_REGISTERS:
        case MODBUS_FC_READ_INPUT_REGISTERS:
            /* Read functions 1 value = 2 bytes */
            reqNbValue = (req[offset + 3] << 8) + req[offset + 4];
            rspNbValue = (rsp[offset + 1] / 2);
            break;
        case MODBUS_FC_WRITE_MULTIPLE_COILS:
        case MODBUS_FC_WRITE_MULTIPLE_REGISTERS:
            /* N Write functions */
            reqNbValue = (req[offset + 3] << 8) + req[offset + 4];
            rspNbValue = (rsp[offset + 3] << 8) | rsp[offset + 4];
            break;
        default:
            /* 1 Write functions & others */
            reqNbValue = rspNbValue = 1;
        }

        if(reqNbValue == rspNbValue) 
        {
            res = rspNbValue;
        } 
        else 
        {
            if(mb->debug) 
            {
                log_warn(
                        "Quantity not corresponding to the request (%d != %d)\n",
                        rspNbValue, reqNbValue);
            }
            res = -1;
        }
    } 
    else 
    {
        if(mb->debug) 
        {
            log_warn(
                    "Message length not corresponding to the computed length (%d != %d)\n",
                    rspLength, rspLengthComputed);
        }
        res = -1;
    }
    return res;
}

static int read_io_status(modbus_t *mb, int function,
                          int addr, int nb, uint8_t *dest)
{
    int res;
    int reqLength;

    uint8_t req[_MIN_REQ_LENGTH];
    uint8_t rsp[MAX_MESSAGE_LENGTH];
    reqLength = mb->backend->build_request_basis(mb, function, addr, nb, req);

    res = send_msg(mb, req, reqLength);
    if(res == 0)
    {
        int i, temp, bit;
        int pos = 0;
        int offset;
        int offset_end;
        res = _modbus_receive_msg(mb, rsp, MSG_CONFIRMATION);
        if(res == -1)
            return -1;
        res = check_confirmation(mb, req, rsp, res);
        if(res == -1)
            return -1;
        offset = mb->backend->headerLength + 2;
        offset_end = offset + res;
        for(i = offset; i < offset_end; i++) 
        {
            /* Shift reg hi_byte to temp */
            temp = rsp[i];
            for(bit = 0x01; (bit & 0xff) && (pos < nb);) 
            {
                dest[pos++] = (temp & bit) ? TRUE : FALSE;
                bit = bit << 1;
            }
        }
    }
    return res;
}

int modbus_read_bits(modbus_t *mb, int addr, int nb, uint8_t *dest)
{
    int res;

    if(mb == NULL)
    {
        return -1;
    }

    if(nb > mb->cfg->reg1MaxSize) 
    {
        if(mb->debug) 
        {
            log_info("ERROR Too many bits requested (%d > %d)\n",
                    nb, mb->cfg->reg1MaxSize);
        }
        return -1;
    }
    res = read_io_status(mb, MODBUS_FC_READ_COILS, addr, nb, dest);
    if(res == -1)
    {
        return -1;
    }
    else
    {
        return nb;
    }
}

int modbus_read_input_bits(modbus_t *mb, int addr, int nb, uint8_t *dest)
{
    int res;

    if(mb == NULL)
    {
        return -1;
    }

    if(nb > mb->cfg->reg2MaxSize) 
    {
        if(mb->debug) 
        {
            log_info("ERROR Too many input bits requested (%d > %d)\n",
                    nb, mb->cfg->reg2MaxSize);
        }
        return -1;
    }
    res = read_io_status(mb, MODBUS_FC_READ_DISCRETE_INPUTS, addr, nb, dest);

    if(res == -1)
    {
        return -1;
    }
    else
    {
        return nb;
    }
}

static int read_registers(modbus_t *mb, int function,
                          int addr, int nb, uint16_t *dest)
{
    int res;
    int reqLength;

    uint8_t req[_MIN_REQ_LENGTH];
    uint8_t rsp[MAX_MESSAGE_LENGTH];

    reqLength = mb->backend->build_request_basis(mb, function, addr, nb, req);

    res = send_msg(mb, req, reqLength);
    if(res == 0)
    {
        int offset;
        int i;
        res = _modbus_receive_msg(mb, rsp, MSG_CONFIRMATION);
        if(res == -1)
            return -1;
        res = check_confirmation(mb, req, rsp, res);
        if(res == -1)
            return -1;

        offset = mb->backend->headerLength;

        for (i = 0; i < res; i++) 
        {
            /* shift reg hi_byte to temp OR with lo_byte */
            dest[i] = (rsp[offset + 2 + (i << 1)] << 8) |
                rsp[offset + 3 + (i << 1)];
        }
    }
    return res;
}

int modbus_read_registers(modbus_t *mb, int addr, int nb, uint16_t *dest)
{
    int res;

    if(mb == NULL) 
    {
        return -1;
    }
    if(nb > mb->cfg->reg3MaxSize) 
    {
        if(mb->debug) 
        {
            log_info("ERROR Too many registers requested (%d > %d)\n",
                    nb, mb->cfg->reg3MaxSize);
        }
        return -1;
    }
    res = read_registers(mb, MODBUS_FC_READ_HOLDING_REGISTERS,
                            addr, nb, dest);
    return res;
}

int modbus_read_input_registers(modbus_t *mb, int addr, int nb, uint16_t *dest)
{
    int res;

    if(mb == NULL) 
    {
        return -1;
    }
    if(nb > mb->cfg->reg4MaxSize) 
    {
        if(mb->debug) 
        {
            log_info("ERROR Too many input registers requested (%d > %d)\n",
                    nb, mb->cfg->reg4MaxSize);
        }
        return -1;
    }
    res = read_registers(mb, MODBUS_FC_READ_INPUT_REGISTERS,
                            addr, nb, dest);
    return res;
}
/* Write a value to the specified register of the remote device.
   Used by write_bit and write_register */
static int write_single(modbus_t *mb, int function, int addr, const uint16_t value)
{
    int res;
    int reqLength;
    uint8_t req[_MIN_REQ_LENGTH];
    uint8_t rsp[MAX_MESSAGE_LENGTH];

    if(mb == NULL) 
    {
        return -1;
    }

    reqLength = mb->backend->build_request_basis(mb, function, addr, (int) value, req);

    res = send_msg(mb, req, reqLength);
    if(res == 0) 
    {
        /* Used by write_bit and write_register */
        res = _modbus_receive_msg(mb, rsp, MSG_CONFIRMATION);
        if(res == -1)
            return -1;

        res = check_confirmation(mb, req, rsp, res);
    }

    return res;
}

/* Turns ON or OFF a single bit of the remote device */
int modbus_write_bit(modbus_t *mb, int addr, int status)
{
    if(mb == NULL) 
    {
        return -1;
    }

    return write_single(mb, MODBUS_FC_WRITE_SINGLE_COIL, addr,
                        status ? 0xFF00 : 0);
}
/* Writes a value in one register of the remote device */
int modbus_write_register(modbus_t *mb, int addr, const uint16_t value)
{
    if(mb == NULL) 
    {
        return -1;
    }

    return write_single(mb, MODBUS_FC_WRITE_SINGLE_REGISTER, addr, value);
}
/* Write the bits of the array in the remote device */
int modbus_write_bits(modbus_t *mb, int addr, int nb, const uint8_t *src)
{
    int res;
    int i;
    int byteCount;
    int reqLength;
    int bitCheck = 0;
    int pos = 0;
    uint8_t req[MAX_MESSAGE_LENGTH];
    uint8_t rsp[MAX_MESSAGE_LENGTH];

    if(mb == NULL) 
    {
        return -1;
    }

    if(nb > mb->cfg->reg1MaxSize) 
    {
        if (mb->debug) 
        {
            log_warn("ERROR Writing too many bits (%d > %d)\n",
                    nb, mb->cfg->reg1MaxSize);
        }
        return -1;
    }

    reqLength = mb->backend->build_request_basis(mb,
                                                   MODBUS_FC_WRITE_MULTIPLE_COILS,
                                                   addr, nb, req);
    byteCount = (nb / 8) + ((nb % 8) ? 1 : 0);
    req[reqLength++] = byteCount;

    for(i = 0; i < byteCount; i++) 
    {
        int bit;
        bit = 0x01;
        req[reqLength] = 0;

        while((bit & 0xFF) && (bitCheck++ < nb)) 
        {
            if (src[pos++])
                req[reqLength] |= bit;
            else
                req[reqLength] &=~ bit;

            bit = bit << 1;
        }
        reqLength++;
    }

    res = send_msg(mb, req, reqLength);
    if(res == 0) 
    {
        res = _modbus_receive_msg(mb, rsp, MSG_CONFIRMATION);
        if(res == -1)
            return -1;

        res = check_confirmation(mb, req, rsp, res);
    }

    return res;
}

/* Write the values from the array to the registers of the remote device */
int modbus_write_registers(modbus_t *mb, int addr, int nb, const uint16_t *src)
{
    int res;
    int i;
    int reqLength;
    int byteCount;
    uint8_t req[MAX_MESSAGE_LENGTH];
    uint8_t rsp[MAX_MESSAGE_LENGTH];

    if(mb == NULL) 
    {
        return -1;
    }

    if(nb > mb->cfg->reg3MaxSize) 
    {
        if(mb->debug)
        {
            log_warn("ERROR Trying to write to too many registers (%d > %d)\n",
                    nb, mb->cfg->reg3MaxSize);
        }
        return -1;
    }

    reqLength = mb->backend->build_request_basis(mb,
                                                   MODBUS_FC_WRITE_MULTIPLE_REGISTERS,
                                                   addr, nb, req);
    byteCount = nb * 2;
    req[reqLength++] = byteCount;

    for (i = 0; i < nb; i++) {
        req[reqLength++] = src[i] >> 8;
        req[reqLength++] = src[i] & 0x00FF;
    }

    res = send_msg(mb, req, reqLength);
    if(res == 0) 
    {
        res = _modbus_receive_msg(mb, rsp, MSG_CONFIRMATION);
        if(res == -1)
            return -1;

        res = check_confirmation(mb, req, rsp, res);
    }

    return res;
}
int modbus_mask_write_register(modbus_t *mb, int addr, uint16_t andMask, uint16_t orMask)
{
    int res;
    int reqLength;
    /* The request length can not exceed _MIN_REQ_LENGTH - 2 and 4 bytes to
     * store the masks. The ugly subtraction is there to remove the 'nb' value
     * (2 bytes) which is not used. */
    uint8_t req[_MIN_REQ_LENGTH + 2];
    uint8_t rsp[MAX_MESSAGE_LENGTH];

    reqLength = mb->backend->build_request_basis(mb,
                                                   MODBUS_FC_MASK_WRITE_REGISTER,
                                                   addr, 0, req);

    /* HACKISH, count is not used */
    reqLength -= 2;

    req[reqLength++] = andMask >> 8;
    req[reqLength++] = andMask & 0x00ff;
    req[reqLength++] = orMask >> 8;
    req[reqLength++] = orMask & 0x00ff;

    res = send_msg(mb, req, reqLength);
    if(res == 0) 
    {
        res = _modbus_receive_msg(mb, rsp, MSG_CONFIRMATION);
        if(res == -1)
            return -1;

        res = check_confirmation(mb, req, rsp, res);
    }

    return res;
}
/* Write multiple registers from src array to remote device and read multiple
   registers from remote device to dest array. */
int modbus_write_and_read_registers(modbus_t *mb,
                                    int writeAddr, int writeNb,
                                    const uint16_t *src,
                                    int readAddr, int readNb,
                                    uint16_t *dest)

{
    int res;
    int reqLength;
    int i;
    int byteCount;
    uint8_t req[MAX_MESSAGE_LENGTH];
    uint8_t rsp[MAX_MESSAGE_LENGTH];

    if(mb == NULL) 
    {
        return -1;
    }

    if(writeNb > mb->cfg->reg3MaxSize) 
    {
        if(mb->debug) 
        {
            log_err("ERROR Too many registers to write (%d > %d)\n",
                    writeNb, mb->cfg->reg3MaxSize);
        }
        return -1;
    }

    if(readNb > mb->cfg->reg4MaxSize) 
    {
        if(mb->debug) 
        {
            log_err("ERROR Too many registers requested (%d > %d)\n",
                    readNb, mb->cfg->reg4MaxSize);
        }
        return -1;
    }
    reqLength = mb->backend->build_request_basis(mb,
                                                   MODBUS_FC_WRITE_AND_READ_REGISTERS,
                                                   readAddr, readNb, req);

    req[reqLength++] = writeAddr >> 8;
    req[reqLength++] = writeAddr & 0x00ff;
    req[reqLength++] = writeNb >> 8;
    req[reqLength++] = writeNb & 0x00ff;
    byteCount = writeNb * 2;
    req[reqLength++] = byteCount;

    for(i = 0; i < writeNb; i++) 
    {
        req[reqLength++] = src[i] >> 8;
        req[reqLength++] = src[i] & 0x00FF;
    }

    res = send_msg(mb, req, reqLength);
    if(res == 0) 
    {
        int offset;

        res = _modbus_receive_msg(mb, rsp, MSG_CONFIRMATION);
        if(res == -1)
            return -1;

        res = check_confirmation(mb, req, rsp, res);
        if (res == -1)
            return -1;

        offset = mb->backend->headerLength;
        for (i = 0; i < res; i++) 
        {
            /* shift reg hi_byte to temp OR with lo_byte */
            dest[i] = (rsp[offset + 2 + (i << 1)] << 8) |
                rsp[offset + 3 + (i << 1)];
        }
    }

    return res;
}
/* Define the slave number */
int modbus_set_slave(modbus_t *mb, int slave)
{
    if (mb == NULL) 
    {
        return -1;
    }

    return mb->backend->set_slave(mb, slave);
}

int modbus_get_slave(modbus_t *mb)
{
    if (mb == NULL) 
    {
        return -1;
    }

    return mb->slave;
}

/* Get the timeout interval used to wait for a response */
int modbus_get_response_timeout(modbus_t *mb, uint32_t *msec)
{
    if(mb == NULL) 
    {
        return -1;
    }
    *msec = mb->cfg->responeTimeout;
    return 0;
}

int modbus_set_response_timeout(modbus_t *mb, uint32_t msec)
{
    if(mb == NULL) 
    {
        return -1;
    }
    mb->cfg->responeTimeout = msec;
    return 0;
}

int modbus_set_regmaxsize(modbus_t *mb,
                           uint16_t reg1Size,
                           uint16_t reg2Size,
                           uint16_t reg3Size,
                           uint16_t reg4Size)
{
    if(mb == NULL) 
    {
        return -1;
    }
    mb->cfg->reg1MaxSize = reg1Size;
    mb->cfg->reg2MaxSize = reg2Size;
    mb->cfg->reg3MaxSize = reg3Size;
    mb->cfg->reg4MaxSize = reg4Size;
    return 0;
}
/* SLAVE */
/* Allocates 4 arrays to store bits, input bits, registers and inputs
   registers. The pointers are stored in modbus_mapping structure.

   The modbus_mapping_new_start_address() function shall return the new allocated
   structure if successful. Otherwise it shall return NULL and set errno to
   ENOMEM. */
modbus_mapping_t* modbus_mapping_new_start_address(
    unsigned int startBits, unsigned int nbBits,
    unsigned int startInputBits, unsigned int nbInputBits,
    unsigned int startRegisters, unsigned int nbRegisters,
    unsigned int startInputRegisters, unsigned int nbInputRegisters)
{
    modbus_mapping_t *mbMapping;

    mbMapping = (modbus_mapping_t *)modbus_malloc_callback(sizeof(modbus_mapping_t));
    if(mbMapping == NULL) 
    {
        return NULL;
    }

    /* 0X */
    mbMapping->nbBits = nbBits;
    mbMapping->startBits = startBits;
    if(nbBits == 0) 
    {
        mbMapping->tabBits = NULL;
    } 
    else 
    {
        /* Negative number raises a POSIX error */
        mbMapping->tabBits =
            (uint8_t *) modbus_malloc_callback(nbBits * sizeof(uint8_t));
        if(mbMapping->tabBits == NULL) 
        {
            modbus_free_callback(mbMapping);
            return NULL;
        }
        memset(mbMapping->tabBits, 0, nbBits * sizeof(uint8_t));
    }

    /* 1X */
    mbMapping->nbInputBits = nbInputBits;
    mbMapping->startInputBits = startInputBits;
    if(nbInputBits == 0) 
    {
        mbMapping->tabInputBits = NULL;
    } 
    else
    {
        mbMapping->tabInputBits =
            (uint8_t *) malloc(nbInputBits * sizeof(uint8_t));
        if (mbMapping->tabInputBits == NULL) {
            modbus_free_callback(mbMapping->tabBits);
            modbus_free_callback(mbMapping);
            return NULL;
        }
        memset(mbMapping->tabInputBits, 0, nbInputBits * sizeof(uint8_t));
    }

    /* 4X */
    mbMapping->nbRegisters = nbRegisters;
    mbMapping->startRegisters = startRegisters;
    if(nbRegisters == 0) 
    {
        mbMapping->tabRegisters = NULL;
    } 
    else 
    {
        mbMapping->tabRegisters =
            (uint16_t *) malloc(nbRegisters * sizeof(uint16_t));
        if(mbMapping->tabRegisters == NULL) 
        {
            free(mbMapping->tabInputBits);
            free(mbMapping->tabBits);
            free(mbMapping);
            return NULL;
        }
        memset(mbMapping->tabRegisters, 0, nbRegisters * sizeof(uint16_t));
    }

    /* 3X */
    mbMapping->nbInputRegisters = nbInputRegisters;
    mbMapping->startInputRegisters = startInputRegisters;
    if (nbInputRegisters == 0) {
        mbMapping->tabInputRegisters = NULL;
    } else {
        mbMapping->tabInputRegisters =
            (uint16_t *) malloc(nbInputRegisters * sizeof(uint16_t));
        if (mbMapping->tabInputRegisters == NULL) {
            free(mbMapping->tabRegisters);
            free(mbMapping->tabInputBits);
            free(mbMapping->tabBits);
            free(mbMapping);
            return NULL;
        }
        memset(mbMapping->tabInputRegisters, 0,
               nbInputRegisters * sizeof(uint16_t));
    }

    return mbMapping;
}
modbus_mapping_t* modbus_mapping_new(int nbBits, int nbInputBits,
                                     int nbRegisters, int nbInputRegisters)
{
    return modbus_mapping_new_start_address(
        0, nbBits, 0, nbInputBits, 0, nbRegisters, 0, nbInputRegisters);
}
void modbus_mapping_free(modbus_mapping_t *mbMapping)
{
    if(mbMapping == NULL) 
    {
        return;
    }

    free(mbMapping->tabInputRegisters);
    free(mbMapping->tabRegisters);
    free(mbMapping->tabInputBits);
    free(mbMapping->tabBits);
    free(mbMapping);
}

/* Receive the request from a modbus master, return length of request*/
int modbus_receive(modbus_t *mb, uint8_t *req)
{
    int res;
    if(mb == NULL) 
    {
        return -1;
    }
    res = mb->backend->receive(mb, req);
    return res;
}
/* Build the exception response */
static int response_exception(modbus_t *mb, sft_t *sft,
                              int exceptionCode, uint8_t *rsp)
{
    int rsplength;

    /* Build exception response */
    sft->function = sft->function + 0x80;
    rsplength = mb->backend->build_response_basis(sft, rsp);
    rsp[rsplength++] = exceptionCode;

    return rsplength;
}
static int response_io_status(uint8_t *tabIoStatus,
                              int address, int nb,
                              uint8_t *rsp, int offset)
{
    int shift = 0;
    /* Instead of byte (not allowed in Win32) */
    int oneByte = 0;
    int i;

    for (i = address; i < address + nb; i++) 
    {
        oneByte |= tabIoStatus[i] << shift;
        if(shift == 7) 
        {
            /* Byte is full */
            rsp[offset++] = oneByte;
            oneByte = shift = 0;
        } 
        else 
        {
            shift++;
        }
    }

    if(shift != 0)
        rsp[offset++] = oneByte;

    return offset;
}
/* Send a response to the received request.
   Analyses the request and constructs a response.

   If an error occurs, this function construct the response
   accordingly.
*/
int modbus_reply(modbus_t *mb, const uint8_t *req,
                 int reqLength, modbus_mapping_t *mbMapping)
{
    int offset;
    int slave;
    int function;
    uint16_t address;
    uint8_t rsp[MAX_MESSAGE_LENGTH];
    int rspLength = 0;
    sft_t sft;

    if(mb == NULL) 
    {
        return -1;
    }

    offset = mb->backend->headerLength;
    slave = req[offset - 1];
    function = req[offset];
    address = (req[offset + 1] << 8) + req[offset + 2];

    sft.slave = slave;
    sft.function = function;

    /* Data are flushed on illegal number of values errors. */
    switch(function) 
    {
    case MODBUS_FC_READ_COILS:
    case MODBUS_FC_READ_DISCRETE_INPUTS: 
    {
        unsigned int isInput = (function == MODBUS_FC_READ_DISCRETE_INPUTS);
        int startBits = isInput ? mbMapping->startInputBits : mbMapping->startBits;
        int nbBits = isInput ? mbMapping->nbInputBits : mbMapping->nbBits;
        uint8_t *tabBits = isInput ? mbMapping->tabInputBits : mbMapping->tabBits;
        const char * const name = isInput ? "read_input_bits" : "read_bits";
        int nb = (req[offset + 3] << 8) + req[offset + 4];
        /* The mapping can be shifted to reduce memory consumption and it
           doesn't always start at address zero. */
        int mappingAddress = address - startBits;
        uint16_t maxSize = isInput ? mb->cfg->reg2MaxSize : mb->cfg->reg1MaxSize;

        if(nb < 1 ||maxSize < nb) 
        {
            rspLength = response_exception(
                mb, &sft, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE, rsp);
            if(mb->debug) 
            {
                log_err("Illegal nb of values %d in %s (max %d)\n", nb, name, maxSize);
            }
            
        } 
        else if(mappingAddress < 0 || (mappingAddress + nb) > nbBits) 
        {
            rspLength = response_exception(
                mb, &sft,
                MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS, rsp);
            if(mb->debug) 
            {
                log_err("Illegal data address 0x%0X in %s\n",mappingAddress < 0 ? address : address + nb, name);
            }
        } 
        else 
        {
            rspLength = mb->backend->build_response_basis(&sft, rsp);
            rsp[rspLength++] = (nb / 8) + ((nb % 8) ? 1 : 0);
            rspLength = response_io_status(tabBits, mappingAddress, nb,
                                            rsp, rspLength);
        }
    }
        break;
    case MODBUS_FC_READ_HOLDING_REGISTERS:
    case MODBUS_FC_READ_INPUT_REGISTERS: 
    {
        unsigned int isInput = (function == MODBUS_FC_READ_INPUT_REGISTERS);
        int startRegisters = isInput ? mbMapping->startInputRegisters : mbMapping->startRegisters;
        int nbRegisters = isInput ? mbMapping->nbInputRegisters : mbMapping->nbRegisters;
        uint16_t *tabRegisters = isInput ? mbMapping->tabInputRegisters : mbMapping->tabRegisters;
        const char * const name = isInput ? "read_input_registers" : "read_registers";
        int nb = (req[offset + 3] << 8) + req[offset + 4];
        /* The mapping can be shifted to reduce memory consumption and it
           doesn't always start at address zero. */
        int mappingAddress = address - startRegisters;
        uint16_t maxSize = isInput ? mb->cfg->reg3MaxSize : mb->cfg->reg4MaxSize;

        if(nb < 1 || maxSize < nb) 
        {
            rspLength = response_exception(
                mb, &sft, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE, rsp);
            if(mb->debug) 
            {
                log_err("Illegal nb of values %d in %s (max %d)\n", nb, name, maxSize);
            }
        }
        else if(mappingAddress < 0 || (mappingAddress + nb) > nbRegisters) 
        {
            rspLength = response_exception(
                mb, &sft, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS, rsp);
            if(mb->debug) 
            {
                log_err("Illegal data address 0x%0X in %s\n",  mappingAddress < 0 ? address : address + nb, name);
            }
        } 
        else 
        {
            int i;

            rspLength = mb->backend->build_response_basis(&sft, rsp);
            rsp[rspLength++] = nb << 1;
            for (i = mappingAddress; i < mappingAddress + nb; i++) {
                rsp[rspLength++] = tabRegisters[i] >> 8;
                rsp[rspLength++] = tabRegisters[i] & 0xFF;
            }
        }
    }
        break;
    case MODBUS_FC_WRITE_SINGLE_COIL: 
    {
        int mappingAddress = address - mbMapping->startBits;

        if(mappingAddress < 0 || mappingAddress >= mbMapping->nbBits) 
        {
            rspLength = response_exception(
                mb, &sft, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS, rsp);
            if(mb->debug) 
            {
                log_err("Illegal data address 0x%0X in write_bit\n", address);
            } 
        } 
        else 
        {
            int data = (req[offset + 3] << 8) + req[offset + 4];

            if(data == 0xFF00 || data == 0x0) 
            {
                mbMapping->tabBits[mappingAddress] = data ? ON : OFF;
                memcpy(rsp, req, reqLength);
                rspLength = reqLength;
            } 
            else
            {
                rspLength = response_exception(
                    mb, &sft,
                    MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE, rsp);
                if(mb->debug) 
                {
                    log_err("Illegal data value 0x%0X in write_bit request at address %0X\n",
                        data, address);
                }
            }
        }
    }
        break;
    case MODBUS_FC_WRITE_SINGLE_REGISTER: 
    {
        int mappingAddress = address - mbMapping->startRegisters;

        if(mappingAddress < 0 || mappingAddress >= mbMapping->nbRegisters) 
        {
            rspLength = response_exception(
                mb, &sft,
                MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS, rsp);
            if(mb->debug) 
            {
                log_err("Illegal data address 0x%0X in write_register\n",
                address);
            }   
        } 
        else 
        {
            int data = (req[offset + 3] << 8) + req[offset + 4];

            mbMapping->tabRegisters[mappingAddress] = data;
            memcpy(rsp, req, reqLength);
            rspLength = reqLength;
        }
    }
        break;
    case MODBUS_FC_WRITE_MULTIPLE_COILS: {
        int nb = (req[offset + 3] << 8) + req[offset + 4];
        int nbBits = req[offset + 5];
        int mappingAddress = address - mbMapping->startBits;
        uint16_t maxSize = mb->cfg->reg1MaxSize;
        if(nb < 1 || maxSize < nb || nbBits * 8 < nb) {
            /* May be the indication has been truncated on reading because of
             * invalid address (eg. nb is 0 but the request contains values to
             * write) so it's necessary to flush. */
            rspLength = response_exception(
                mb, &sft, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE, rsp);
            if(mb->debug) 
            {
                log_err("Illegal number of values %d in write_bits (max %d)\n",
                nb, maxSize);
            } 
        } 
        else if(mappingAddress < 0 ||
                   (mappingAddress + nb) > mbMapping->nbBits) 
        {
            rspLength = response_exception(
                mb, &sft,
                MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS, rsp);
            if(mb->debug) 
            {
                log_err("Illegal data address 0x%0X in write_bits\n",
                mappingAddress < 0 ? address : address + nb);
            } 
        } 
        else
        {
            /* 6 = byte count */
            modbus_set_bits_from_bytes(mbMapping->tabBits, mappingAddress, nb,
                                       &req[offset + 6]);

            rspLength = mb->backend->build_response_basis(&sft, rsp);
            /* 4 to copy the bit address (2) and the quantity of bits */
            memcpy(rsp + rspLength, req + rspLength, 4);
            rspLength += 4;
        }
    }
        break;
    case MODBUS_FC_WRITE_MULTIPLE_REGISTERS: {
        int nb = (req[offset + 3] << 8) + req[offset + 4];
        int nbBytes = req[offset + 5];
        int mappingAddress = address - mbMapping->startRegisters;
        uint16_t maxSize = mb->cfg->reg3MaxSize;
        if (nb < 1 || maxSize < nb || nbBytes != nb * 2) {
            rspLength = response_exception(
                mb, &sft, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE, rsp);
            if(mb->debug) 
            {
                log_err("Illegal number of values %d in write_registers (max %d)\n",
                nb, maxSize);
            } 
        } 
        else if(mappingAddress < 0 ||
                   (mappingAddress + nb) > mbMapping->nbRegisters) 
        {
            rspLength = response_exception(
                mb, &sft, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS, rsp);
            if(mb->debug) 
            {
                log_err("Illegal data address 0x%0X in write_registers\n",
                mappingAddress < 0 ? address : address + nb);
            } 
        } 
        else 
        {
            int i, j;
            for(i = mappingAddress, j = 6; i < mappingAddress + nb; i++, j += 2) 
            {
                /* 6 and 7 = first value */
                mbMapping->tabRegisters[i] =
                    (req[offset + j] << 8) + req[offset + j + 1];
            }

            rspLength = mb->backend->build_response_basis(&sft, rsp);
            /* 4 to copy the address (2) and the no. of registers */
            memcpy(rsp + rspLength, req + rspLength, 4);
            rspLength += 4;
        }
    }
        break;
    case MODBUS_FC_READ_EXCEPTION_STATUS:
        if(mb->debug) 
        {
            log_err("FIXME Not implemented\n");
        }
        return -1;
        break;
    case MODBUS_FC_MASK_WRITE_REGISTER: 
    {
        int mappingAddress = address - mbMapping->startRegisters;

        if(mappingAddress < 0 || mappingAddress >= mbMapping->nbRegisters) 
        {
            rspLength = response_exception(
                mb, &sft, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS, rsp);
            if(mb->debug) 
            {
                log_err("Illegal data address 0x%0X in write_register\n",
                address);
            } 
        } 
        else 
        {
            uint16_t data = mbMapping->tabRegisters[mappingAddress];
            uint16_t and = (req[offset + 3] << 8) + req[offset + 4];
            uint16_t or = (req[offset + 5] << 8) + req[offset + 6];

            data = (data & and) | (or & (~and));
            mbMapping->tabRegisters[mappingAddress] = data;
            memcpy(rsp, req, reqLength);
            rspLength = reqLength;
        }
    }
        break;
    case MODBUS_FC_WRITE_AND_READ_REGISTERS: 
    {
        int nb = (req[offset + 3] << 8) + req[offset + 4];
        uint16_t address_write = (req[offset + 5] << 8) + req[offset + 6];
        int nbWrite = (req[offset + 7] << 8) + req[offset + 8];
        int nbWrite_bytes = req[offset + 9];
        int mappingAddress = address - mbMapping->startRegisters;
        int mappingAddress_write = address_write - mbMapping->startRegisters;

        if(nbWrite < 1 || mb->cfg->reg3MaxSize < nbWrite ||
            nb < 1 || mb->cfg->reg4MaxSize < nb ||
            nbWrite_bytes != nbWrite * 2) 
        {
            rspLength = response_exception(
                mb, &sft, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE, rsp);
            if(mb->debug) 
            {
                log_err("Illegal nb of values (W%d, R%d) in write_and_read_registers (max W%d, R%d)\n",
                nbWrite, nb, mb->cfg->reg3MaxSize, mb->cfg->reg4MaxSize);
            }   
        } 
        else if(mappingAddress < 0 ||
                   (mappingAddress + nb) > mbMapping->nbRegisters ||
                   mappingAddress_write < 0 ||
                   (mappingAddress_write + nbWrite) > mbMapping->nbRegisters)
        {
            rspLength = response_exception(
                mb, &sft, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS, rsp);
            if(mb->debug) 
            {
                log_err("Illegal data read address 0x%0X or write address 0x%0X write_and_read_registers\n",
                mappingAddress < 0 ? address : address + nb,
                mappingAddress_write < 0 ? address_write : address_write + nbWrite);
            }  
        } 
        else 
        {
            int i, j;
            rspLength = mb->backend->build_response_basis(&sft, rsp);
            rsp[rspLength++] = nb << 1;

            /* Write first.
               10 and 11 are the offset of the first values to write */
            for(i = mappingAddress_write, j = 10;
                 i < mappingAddress_write + nbWrite; i++, j += 2) 
            {
                mbMapping->tabRegisters[i] =
                    (req[offset + j] << 8) + req[offset + j + 1];
            }
            /* and read the data for the response */
            for(i = mappingAddress; i < mappingAddress + nb; i++) 
            {
                rsp[rspLength++] = mbMapping->tabRegisters[i] >> 8;
                rsp[rspLength++] = mbMapping->tabRegisters[i] & 0xFF;
            }
        }
    }
        break;

    default:
        rspLength = response_exception(
            mb, &sft, MODBUS_EXCEPTION_ILLEGAL_FUNCTION, rsp);
        if(mb->debug) 
        {
            log_err("Unknown Modbus function code: 0x%0X\n", function);
        } 
        break;
    }

    /* Suppress any responses when the request was a broadcast */
    return (slave == MODBUS_BROADCAST_ADDRESS) ? 0 : send_msg(mb, rsp, rspLength);
}
int modbus_send_rawRequest(modbus_t *mb, const uint8_t *rawReq, int rawReqLength)
{
    sft_t sft;
    uint8_t req[MAX_MESSAGE_LENGTH];
    int reqLength;

    if(mb == NULL) 
    {
        return -1;
    }

    if(rawReqLength < 2 || rawReqLength > (MODBUS_MAX_PDU_LENGTH + 1)) 
    {
        /* The raw request must contain function and slave at least and
           must not be longer than the maximum pdu length plus the slave
           address. */
        return -1;
    }

    sft.slave = rawReq[0];
    sft.function = rawReq[1];
    /* This response function only set the header so it's convenient here */
    reqLength = mb->backend->build_response_basis(&sft, req);

    if(rawReqLength > 2) 
    {
        /* Copy data after function code */
        memcpy(req + reqLength, rawReq + 2, rawReqLength - 2);
        reqLength += rawReqLength - 2;
    }

    return send_msg(mb, req, reqLength);
}
/* Receives the confirmation.

   The function shall store the read response in rsp and return the number of
   values (bits or words). Otherwise, its shall return -1 and errno is set.

   The function doesn't check the confirmation is the expected response to the
   initial request.
*/
int modbus_receive_confirmation(modbus_t *mb, uint8_t *rsp)
{
    if(mb == NULL) 
    {
        return -1;
    }
    return _modbus_receive_msg(mb, rsp, MSG_CONFIRMATION);
}
int modbus_reply_exception(modbus_t *mb, const uint8_t *req,
                           unsigned int exceptionCcode)
{
    int offset;
    int slave;
    int function;
    uint8_t rsp[MAX_MESSAGE_LENGTH];
    int rspLength;
    sft_t sft;

    if(mb == NULL) 
    {
        return -1;
    }

    offset = mb->backend->headerLength;
    slave = req[offset - 1];
    function = req[offset];

    sft.slave = slave;
    sft.function = function + 0x80;
    rspLength = mb->backend->build_response_basis(&sft, rsp);

    /* Positive exception code */
    if(exceptionCcode < MODBUS_EXCEPTION_MAX) 
    {
        rsp[rspLength++] = exceptionCcode;
        return send_msg(mb, rsp, rspLength);
    } 
    else 
    {
        return -1;
    }
}

/* Get the timeout interval used to wait for a indication */
int modbus_get_indication_timeout(modbus_t *mb, uint32_t *msec)
{
    if(mb == NULL) 
    {
        return -1;
    }
    *msec = mb->cfg->indicationTimeout;
    return 0;
}

int modbus_set_indication_timeout(modbus_t *mb, uint32_t msec)
{
    if(mb == NULL) 
    {
        return -1;
    }
    mb->cfg->indicationTimeout = msec;
    return 0;
}

/* Sets many bits from a single byte value (all 8 bits of the byte value are
   set) */
void modbus_set_bits_from_byte(uint8_t *dest, int idx, const uint8_t value)
{
    int i;

    for(i=0; i < 8; i++) 
    {
        dest[idx+i] = (value & (1 << i)) ? 1 : 0;
    }
}
/* Sets many bits from a table of bytes (only the bits between idx and
   idx + nb_bits are set) */
void modbus_set_bits_from_bytes(uint8_t *dest, int idx, unsigned int nbBits,
                                const uint8_t *tabByte)
{
    unsigned int i;
    int shift = 0;

    for(i = idx; i < idx + nbBits; i++) 
    {
        dest[i] = tabByte[(i - idx) / 8] & (1 << shift) ? 1 : 0;
        /* gcc doesn't like: shift = (++shift) % 8; */
        shift++;
        shift %= 8;
    }
}
/* Gets the byte value from many bits.
   To obtain a full byte, set nb_bits to 8. */
uint8_t modbus_get_byte_from_bits(const uint8_t *src, int idx,
                                  unsigned int nbBits)
{
    unsigned int i;
    uint8_t value = 0;

    if(nbBits > 8) 
    {
        /* Assert is ignored if NDEBUG is set */
        assert(nbBits < 8);
        nbBits = 8;
    }

    for(i=0; i < nbBits; i++) 
    {
        value |= (src[idx+i] << i);
    }

    return value;
}

/*********************************END OF FILE*********************************/
