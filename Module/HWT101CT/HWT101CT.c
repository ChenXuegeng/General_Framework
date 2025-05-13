/**
 * @file 
 * @author 
 * @brief00
 * @version 0.1
 * @date 2025-1-13
 *
 * @copyright Copyright (c) 2025
 *
 * @attention :   
 * 
 * 
 * @note :
 * @versioninfo :
 */
#include "HWT101CT.h"
#include "bsp_usart.h"
#include "FreeRTOS.h"
#include "queue.h"

extern osThreadId_t ins_SensorTaskHandle;
extern osThreadId_t MotorTaskHandle;

static uint8_t HWT_Rtos_Init(HWT_Instance_t *hwt_instance, uint32_t queue_length);

HWT_Instance_t* HWT_Init(Uart_Instance_t *hwt_uart, uint32_t queue_length)
{
    if (hwt_uart == NULL)
    {
        LOGERROR("HWT UART is not prepared!");
        return NULL;
    }

    // 分配 HWT101CT 实例内存
    HWT_Instance_t *hwt_instance = (HWT_Instance_t *)pvPortMalloc(sizeof(HWT_Instance_t));
    if (hwt_instance == NULL)
    {
        LOGERROR("Failed to allocate memory for HWT_Instance!");
        return NULL;
    }

    // 分配数据帧存储内存
    hwt_instance->hwt_msgs = (HWT101CT_Frame_t *)pvPortMalloc(sizeof(HWT101CT_Frame_t));
    if (hwt_instance->hwt_msgs == NULL)
    {
        LOGERROR("Failed to allocate memory for hwt_msgs!");
        vPortFree(hwt_instance);
        return NULL;
    }
    memset(hwt_instance->hwt_msgs, 0, sizeof(HWT101CT_Frame_t));

    // 初始化 FreeRTOS 队列
    if (HWT_Rtos_Init(hwt_instance, queue_length) != 1)
    {
        LOGERROR("RTOS initialization failed!");
        vPortFree(hwt_instance->hwt_msgs);
        vPortFree(hwt_instance);
        return NULL;
    }

    // 挂载 UART 实例
    hwt_instance->hwt_uart_instance = hwt_uart;
    hwt_uart->device = hwt_instance;

    // 挂载函数指针
    hwt_instance->hwt_get_data = HWT_GetData;
    hwt_instance->hwt_task = HWT_Task;
    hwt_instance->hwt_deinit = HWT_Deinit;

    LOGINFO("HWT101CT is initialized!");
    return hwt_instance;
}

static uint8_t HWT_Rtos_Init(HWT_Instance_t *hwt_instance, uint32_t queue_length)
{
    if (hwt_instance == NULL)
    {
        LOGERROR("HWT RTOS init failed!");
        return 0;
    }

    rtos_for_module_t *rtos_interface = (rtos_for_module_t *)pvPortMalloc(sizeof(rtos_for_module_t));
    if (rtos_interface == NULL)
    {
        LOGERROR("Failed to allocate memory for RTOS interface!");
        return 0;
    }
    memset(rtos_interface, 0, sizeof(rtos_for_module_t));

    // 创建队列
    QueueHandle_t queue = xQueueCreate(queue_length, sizeof(UART_TxMsg));
    if (queue == NULL)
    {
        LOGERROR("Failed to create queue for HWT!");
        vPortFree(rtos_interface);
        return 0;
    }
    rtos_interface->xQueue = queue;
    rtos_interface->queue_send = queue_send_wrapper;
    rtos_interface->queue_receive = xQueueReceive;

    hwt_instance->rtos_for_hwt = rtos_interface;
    LOGINFO("HWT RTOS init success!");
    return 1;
}

uint8_t HWT_Task(void *hwt_instance_ptr)
{
    UART_TxMsg Msg;
    HWT_Instance_t *temp_hwt_instance = (HWT_Instance_t *)hwt_instance_ptr;

    // 接收方设置为 portMAX_DELAY，在未收到数据时阻塞，减少 CPU 占用
    if (temp_hwt_instance->rtos_for_hwt->queue_receive(temp_hwt_instance->rtos_for_hwt->xQueue, &Msg, portMAX_DELAY) == pdPASS)
    {
#ifdef DEBUG_FOR_HWT
        LOGINFO("HWT task is running!");
#endif
        temp_hwt_instance->hwt_get_data(Msg.data_addr, temp_hwt_instance->hwt_msgs);
        return 1;
    }
#ifdef DEBUG_FOR_HWT
    LOGERROR("HWT task is not running!");
#endif
    return 0;
}
uint8_t HWT_GetData(uint8_t *data, HWT101CT_Frame_t *hwt_msgs)
{
    if (data == NULL)
    {
        LOGWARNING("HWT data is NULL!");
        return 0;
    }

    // 检查包头
    if (data[0] != FRAME_HEADER_1 || data[1] != FRAME_HEADER_2)
    {
        LOGWARNING("HWT UART head error!");
        return 0;
    }

    // 解析保留字节
    for (int i = 0; i < 4; i++)
    {
        hwt_msgs->reserved[i] = data[2 + i];
    }

    // 解析偏航角低字节和高字节
    hwt_msgs->YawL = data[6];
    hwt_msgs->YawH = data[7];

    // 解析版本号低字节和高字节
    hwt_msgs->VL = data[8];
    hwt_msgs->VH = data[9];

    // 解析校验和
    hwt_msgs->checksum = data[10];

    // 校验和验证
    uint8_t calculated_checksum = FRAME_HEADER_1 + FRAME_HEADER_2;
    for (int i = 0; i < 4; i++)
    {
        calculated_checksum += hwt_msgs->reserved[i];
    }
    calculated_checksum += hwt_msgs->YawL + hwt_msgs->YawH + hwt_msgs->VL + hwt_msgs->VH;

    if (calculated_checksum != hwt_msgs->checksum)
    {
        LOGWARNING("HWT checksum error!");
        return 0;
    }

    LOGINFO("HWT data received successfully!");
    return 1;
}
uint8_t HWT_RxCallback_Fun(void *hwt_instance, uint16_t data_len)
{
    UART_TxMsg Msg;
    if (hwt_instance == NULL)
    {
        LOGERROR("HWT instance is NULL!");
        return 0;
    }

    Uart_Instance_t *temp_uart_instance = (Uart_Instance_t *)hwt_instance;
    HWT_Instance_t *temp_hwt_instance = temp_uart_instance->device;

    if (temp_hwt_instance->rtos_for_hwt->xQueue != NULL && temp_hwt_instance->hwt_uart_instance != NULL)
    {
        Msg.data_addr = temp_hwt_instance->hwt_uart_instance->uart_package.rx_buffer;
        Msg.len = data_len;
        Msg.huart = temp_hwt_instance->hwt_uart_instance->uart_package.uart_handle;

        if (Msg.data_addr != NULL) // 注意发送不能阻塞！
        {
            temp_hwt_instance->rtos_for_hwt->queue_send(temp_hwt_instance->rtos_for_hwt->xQueue, &Msg, 0);
            return 1;
        }
    }
    return 0;
}

uint8_t HWT_Deinit(void *hwt_instance)
{
    if (hwt_instance == NULL)
    {
        LOGERROR("HWT instance is NULL!");
        return 0;
    }

    HWT_Instance_t *temp_hwt_instance = (HWT_Instance_t *)hwt_instance;

    // 释放数据帧内存
    if (temp_hwt_instance->hwt_msgs != NULL)
    {
        vPortFree(temp_hwt_instance->hwt_msgs);
        temp_hwt_instance->hwt_msgs = NULL;
    }

    // 删除队列
    if (temp_hwt_instance->rtos_for_hwt != NULL)
    {
        if (temp_hwt_instance->rtos_for_hwt->xQueue != NULL)
        {
            vQueueDelete(temp_hwt_instance->rtos_for_hwt->xQueue);
            temp_hwt_instance->rtos_for_hwt->xQueue = NULL;
        }
        vPortFree(temp_hwt_instance->rtos_for_hwt);
        temp_hwt_instance->rtos_for_hwt = NULL;
    }

    // 释放实例内存
    vPortFree(temp_hwt_instance);
    LOGINFO("HWT101CT is deinitialized!");
    return 1;
}