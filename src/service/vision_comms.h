#ifndef _service_vision_comms_h_
#define _service_vision_comms_h_

/**
 * @file vision_comms.h
 * @brief 云台相机（气泡水平仪视觉）串口接收服务接口
 *
 * 云台相机（MaixCam）按 115200 8N1 输出 ASCII 行：
 *   status,x_deg,y_deg\n
 *
 *   status: 1=已水平  0=未水平(x_deg/y_deg 为需施加的修正角，已反号)  2=本帧检测失败
 *
 * 上电还会输出一行 "spirit_level ready, ..."（本服务自动忽略）。
 * 这是纯文本行协议，不复用二进制帧解析器（binary_frame/protocol_parser）。
 */

#include <stdbool.h>
#include <stdint.h>

// ! ========================= 类 型 声 明 ========================= ! //

typedef enum {
    VISION_COMMS_STATUS_OK = 0,
    VISION_COMMS_STATUS_INVALID_PARAM,
} VisionCommsStatus;

typedef struct {
    uint32_t (*now_ms)(void);
} VisionCommsPortOps;

typedef struct {
    VisionCommsPortOps port_ops;
} VisionCommsConfig;

/**
 * @brief 水平状态（由相机 status 字段映射而来）
 */
typedef enum {
    VISION_LEVEL_STATE_NONE = 0,      ///< 尚无有效数据
    VISION_LEVEL_STATE_LEVEL = 1,     ///< status=1 已水平
    VISION_LEVEL_STATE_NOT_LEVEL = 2, ///< status=0 未水平
    VISION_LEVEL_STATE_FAIL = 3,      ///< status=2 本帧检测失败
} VisionLevelState;

/**
 * @brief 最近一帧水平检测结果
 */
typedef struct {
    VisionLevelState state;
    float corr_x_deg;   ///< 需施加的修正角（相机已反号）
    float corr_y_deg;
    uint32_t stamp_ms;  ///< 收到该帧时的毫秒时间戳
} VisionLevelResult;

typedef struct {
    uint32_t rx_line_count;
    uint32_t rx_parse_fail_count;
    uint32_t last_rx_ms;
} VisionCommsStats;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 初始化云台相机接收服务
 * @param config 通信配置，port_ops.now_ms 必须提供
 * @return VisionCommsStatus 初始化结果
 */
VisionCommsStatus vision_comms_init(const VisionCommsConfig* config);

/**
 * @brief 向接收服务喂入一个接收字节（在 UART 接收中断回调中调用）
 * @param data 本次收到的字节
 */
void vision_comms_on_rx_byte(uint8_t data);

/**
 * @brief 轮询接收缓存并解析完整的文本行
 * @details 建议在主循环（或 100Hz 调度点）调用
 */
void vision_comms_process(void);

/**
 * @brief 判断相机链路是否在线（最近是否收到过有效数据）
 * @return bool `true` 表示链路正常
 */
bool vision_comms_is_online(void);

/**
 * @brief 判断最近一帧结果是否仍然新鲜
 * @param timeout_ms 超时阈值，单位 ms
 * @return bool `true` 表示结果在超时窗口内
 */
bool vision_comms_result_is_fresh(uint32_t timeout_ms);

/**
 * @brief 获取最近一帧水平检测结果
 * @param result 输出结果
 * @return bool `true` 表示存在有效结果
 */
bool vision_comms_get_result(VisionLevelResult* result);

/**
 * @brief 读取接收统计信息
 * @param stats 输出统计信息
 * @return bool `true` 表示读取成功
 */
bool vision_comms_get_stats(VisionCommsStats* stats);

#endif
