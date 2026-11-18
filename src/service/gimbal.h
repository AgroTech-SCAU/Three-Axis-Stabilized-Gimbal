#ifndef _gimbal_h_
#define _gimbal_h_

/** @file gimbal.h
 * @brief 云台服务：启动、自动运行与安全停止
 */

#include "attitude.h"
#include "gimbal_control.h"

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

#define gimbal gimbal_interface

/** @brief 云台服务统一状态码表 */
#define GIMBAL_STATUS_TABLE               \
    X(OK, "OK")                           \
    X(INVALID_PARAM, "Invalid parameter") \
    X(NOT_INITIALIZED, "Not initialized") \
    X(NOT_READY, "Not ready")             \
    X(COMMAND_REJECTED, "Command rejected")

#define X(name, str) GIMBAL_STATUS_##name,
typedef enum {
    GIMBAL_STATUS_TABLE
} GimbalStatus;
#undef X

/** @brief 故障原因；只有运行中的安全退出需要人工确认 */
typedef enum {
    GIMBAL_FAULT_NONE = 0,
    GIMBAL_FAULT_IMU,
    GIMBAL_FAULT_HOLD_COMMAND,
    GIMBAL_FAULT_CONTROL,
    GIMBAL_FAULT_RUN_COMMAND,
} GimbalFault;

/** @brief 事件快照中的单轴原因；年龄始终另行保存，不把未就绪混同为超龄 */
typedef enum {
    GIMBAL_MOTOR_DIAG_OK = 0,
    GIMBAL_MOTOR_DIAG_NOT_READY,
    GIMBAL_MOTOR_DIAG_STALE,
    GIMBAL_MOTOR_DIAG_NONFINITE,
    GIMBAL_MOTOR_DIAG_LIMIT,
    GIMBAL_MOTOR_DIAG_COMMAND,
} GimbalMotorDiagnosticReason;

typedef struct {
    uint32_t age_ms;
    uint32_t limit_ms;
    GimbalMotorDiagnosticReason reason;
    bool ready;
} GimbalMotorDiagnostic;

/** @brief 一次输入检查的证据；不会因下一帧反馈恢复而改变 */
typedef struct {
    uint32_t time_ms;
    uint32_t since_good_ms;
    GimbalControlStatus blocker;
    uint8_t axis_mask;
    GimbalMotorDiagnostic motor[GIMBAL_AXIS_COUNT];
} GimbalInputDiagnostic;

/** @brief 最近安全停止的历史；确认后保留，下一次停止才覆盖 */
typedef struct {
    GimbalFault reason;
    GimbalControlStatus blocker;
    uint8_t axis_mask;
    bool valid;
    GimbalInputDiagnostic first_bad;
    GimbalInputDiagnostic stopped;
} GimbalStopDiagnostic;

/** @brief 只包装当前工程已有接口，不修改 DM/HT 的协议或生命周期 */
typedef struct {
    void (*process_motor_bus)(void);
    bool (*read_motor)(GimbalAxis axis, GimbalMotorFeedback* feedback);
    bool (*set_motor_target)(GimbalAxis axis, float position, float speed, uint32_t timeout_ms);
    void (*clear_motor_target)(GimbalAxis axis);
    bool (*read_console_byte)(uint8_t* byte);
    bool (*take_console_rx_reset)(void);
    bool (*write_console)(const char* data, uint32_t length);
    void (*format_led_diagnostics)(char* text, uint32_t capacity); /**< 可选；写入含结尾的短文本，不带换行 */
} GimbalPortOps;

typedef struct {
    const GimbalPortOps* ops;
    GimbalControlConfig control;
    uint16_t motor_id[GIMBAL_AXIS_COUNT];
    float hold_speed;
    uint32_t hold_period_ms;
    uint32_t hold_timeout_ms;
    uint32_t prepare_feedback_max_age_ms; /**< 准备保持与残余零偏的反馈时效 */
    uint32_t run_recovery_ms;             /**< 自最后完整健康时刻计的总恢复上限；另预留控制周期及 1ms 给指令看门狗 */
    uint32_t status_period_ms;
    uint32_t raw_log_period_ms;
} GimbalConfig;

typedef struct {
    AttitudeSnapshot attitude_snapshot;
    GimbalMotorFeedback motor[GIMBAL_AXIS_COUNT];
    GimbalControl control;
    GimbalControlStatus blocker;
    uint32_t now_ms;
    uint32_t max_loop_us;
    uint32_t telemetry_dropped;
    uint32_t command_retry_count[GIMBAL_AXIS_COUNT];
    uint32_t imu_error_count;
    uint32_t run_wait_count;
    GimbalFault fault_reason;
    GimbalAxis fault_axis; /**< GIMBAL_AXIS_COUNT 表示无具体电机轴 */
    GimbalStopDiagnostic stop_diag;
    uint8_t hold_pending_mask;
    bool initialized;
    bool running;
    bool ready;
    bool fault;
    bool fault_latched; /**< 安全停止锁存；PA15 确认后等待自动运行 */
    bool run_waiting;   /**< 暂停新控制目标，等待新鲜输入或限频重试；不重新捕获姿态 */
} GimbalState;

#define X(name, str) GimbalStatus name;
extern const struct GimbalInterface {
    struct {
        GIMBAL_STATUS_TABLE
    };
    GimbalStatus (*init)(const GimbalConfig* config);
    void (*process)(void);
    bool (*acknowledge_fault)(void); /**< 仅安全锁存时清锁，准备与运行时返回 false */
    const GimbalState* (*get_state)(void);
    bool (*is_ready)(void);
    const char* (*status_str)(GimbalStatus status);
} gimbal_interface;
#undef X

// ! ========================= 接 口 函 数 声 明 ========================= ! //

GimbalStatus gimbal_init(const GimbalConfig* config);
void gimbal_process(void);
bool gimbal_acknowledge_fault(void);
const GimbalState* gimbal_get_state(void);
bool gimbal_is_ready(void);
const char* gimbal_status_str(GimbalStatus status);

#endif
