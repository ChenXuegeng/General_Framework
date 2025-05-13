#ifndef HWT101CT_H
#define HWT101CT_H

#ifdef __cplusplus
extern "C"{
#endif

/*----------------------------------include-----------------------------------*/
/* rtos层接口 */
#include "rtos_interface.h"
/* bsp层接口 */
#include "bsp_usart.h"
#include "bsp_log.h"
#include "data_type.h"
/*-----------------------------------macro------------------------------------*/
#define FRAME_HEADER_1 0x55
#define FRAME_HEADER_2 0x53
#define HWT101CT_DATA_NUM 12 // HWT101CT 数据包的字节数

/*----------------------------------typedef-----------------------------------*/
/**
 * @brief HWT101CT 数据帧结构体
 */
typedef struct
{
    uint8_t reserved[4]; // 前4个保留字节
    uint8_t YawL;        // 偏航角低字节
    uint8_t YawH;        // 偏航角高字节
    uint8_t VL;          // 版本号低字节
    uint8_t VH;          // 版本号高字节
    uint8_t checksum;    // 校验和
} HWT101CT_Frame_t;

/**
 * @brief HWT101CT 模块实例结构体
 */
typedef struct
{
    rtos_for_module_t *rtos_for_hwt;          // FreeRTOS 接口
    Uart_Instance_t *hwt_uart_instance;      // 串口设备实例
    HWT101CT_Frame_t *hwt_msgs;              // 存放接收数据的结构体

    uint8_t (*hwt_get_data)(uint8_t *, HWT101CT_Frame_t *); // 获取 HWT101CT 返回的值
    uint8_t (*hwt_task)(void *hwt_instance);               // HWT101CT 任务函数
    uint8_t (*hwt_deinit)(void *hwt_instance);             // HWT101CT 注销函数
} HWT_Instance_t;

/*----------------------------------function----------------------------------*/
/**
 * @brief HWT101CT 模块初始化函数
 * 
 * @param hwt_uart 串口设备实例
 * @param queue_length FreeRTOS 队列长度
 * @return HWT_Instance_t* 返回 HWT101CT 模块实例指针
 */
HWT_Instance_t* HWT_Init(Uart_Instance_t *hwt_uart, uint32_t queue_length);

/**
 * @brief HWT101CT 任务函数，可以直接放置在 FreeRTOS 的一个线程中
 * 
 * @param hwt_instance HWT101CT 模块实例
 * @return uint8_t 返回任务执行状态
 */
uint8_t HWT_Task(void *hwt_instance);

/**
 * @brief 解析 HWT101CT 数据帧
 * 
 * @param data 接收到的原始数据
 * @param hwt_msgs 存储解析后的数据帧
 * @return uint8_t 返回解析状态
 */
uint8_t HWT_GetData(uint8_t *data, HWT101CT_Frame_t *hwt_msgs);

/**
 * @brief HWT101CT 串口接收回调函数
 * 
 * @param hwt_instance HWT101CT 模块实例
 * @param data_len 接收到的数据长度
 * @return uint8_t 返回回调执行状态
 */
uint8_t HWT_RxCallback_Fun(void *hwt_instance, uint16_t data_len);

/**
 * @brief HWT101CT 模块注销函数
 * 
 * @param hwt_instance HWT101CT 模块实例
 * @return uint8_t 返回注销状态
 */
uint8_t HWT_Deinit(void *hwt_instance);

#ifdef __cplusplus
}
#endif

#endif /* HWT101CT_H */