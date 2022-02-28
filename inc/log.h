/**
  ******************************************************************************
  * File Name          : log.h
  * Description        : This file contains all functions about log.
  ******************************************************************************
  * @attention
  
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __log_H
#define __log_H
#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include "cmsis_os.h"

extern osMutexId mutexPrintfHandle;

#define __DEBUG    //日志模块总开关，注释掉将关闭日志输出

#ifdef __DEBUG
    #define DEBUG_T(format, ...) \
        do { \
            osMutexWait(mutexPrintfHandle, osWaitForever); \
            printf(format, ##__VA_ARGS__); \
            osMutexRelease(mutexPrintfHandle); \
        } while(0)
#else
    #define DEBUG_T(format, ...)
#endif

#define log_fatal(format, ...) \
    do { \
         DEBUG_T("##FATAL @ FUNC:%s FILE:%s LINE:%d \r\n" format,\
                 __func__, __FILE__, __LINE__, ##__VA_ARGS__ );\
    } while (0)

#define log_err(format, ...) \
    do { \
         DEBUG_T("##ERR   @ FUNC:%s FILE:%s LINE:%d \r\n" format,\
                 __func__, __FILE__, __LINE__, ##__VA_ARGS__ );\
    } while (0)

#define log_warn(format, ...) \
    do { \
        DEBUG_T("##WARN  @ FUNC:%s \r\n" format, __func__, ##__VA_ARGS__ );\
    } while (0)

#define log_info(format, ...) \
    do { \
        DEBUG_T("\n##INFO:  \r\n"format, ##__VA_ARGS__ );\
    } while (0)

#define log_none(format, ...) \
    do { \
        DEBUG_T(format, ##__VA_ARGS__ );\
    } while (0)
#ifdef __cplusplus
}
#endif

#endif

/*********************************END OF FILE*********************************/
  
