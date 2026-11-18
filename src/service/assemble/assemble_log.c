#include "assemble.h"

#include "log.h"
#include "stm32_hal_uart.h"

#include <string.h>

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static bool project_log_write(const char* data, uint32_t len);

// ! ========================= 变 量 声 明 ========================= ! //

static const LogPortOps log_ops = {
    .write = project_log_write,
};

// ! ========================= 接 口 函 数 实 现 ========================= ! //

SystemStatus assemble_log(void) {
    LogConfig log_config = {
        .ops = &log_ops,
        .level = LOG_LEVEL_INFO,
        .enable_color = false,
        .async_write = false,
    };

    if(log_init(&log_config) != LOG_STATUS_OK) {
        return SYSTEM_STATUS_ERROR;
    }

    return SYSTEM_STATUS_OK;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

/**
 * @brief 过滤重复 DM 周期反馈，其余日志交给 UART 队列
 * @details ANSI 颜色已关闭，固定 INFO 前缀匹配后返回 true 表示已处理
 */
static bool project_log_write(const char* data, uint32_t len) {
    static const char duplicate_prefix[] = "[INFO] DM fb ";

    if(data != NULL && len >= sizeof(duplicate_prefix) - 1u &&
       memcmp(data, duplicate_prefix, sizeof(duplicate_prefix) - 1u) == 0) {
        return true;
    }
    return uart1_write_it(data, len);
}
