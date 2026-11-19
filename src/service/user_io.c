/**
 * @file user_io.c
 * @brief 非阻塞按键消抖与低亮度 RGB 状态显示
 */

#include "user_io.h"

#include "rgb_led/rgb_led.h"

#include <stddef.h>
#include <string.h>

// ! ========================= 变 量 声 明 ========================= ! //

#define USER_IO_BUTTON_POLL_MS 5u
#define USER_IO_BUTTON_DEBOUNCE_MS 25u
#define USER_IO_LED_RETRY_MS 20u
#define USER_IO_LED_REFRESH_MS 1000u
#define USER_IO_WAIT_HALF_PERIOD_MS 500u
#define USER_IO_FAULT_HALF_PERIOD_MS 125u
#define USER_IO_MAX_BRIGHTNESS 32u

static UserIoPortOps s_ops;
static bool s_button_active_high;
static uint8_t s_brightness;
static bool s_initialized;
static bool s_button_sampled;
static bool s_button_candidate;
static bool s_button_stable;
static bool s_button_stable_known;
static bool s_button_armed;
static bool s_button_press_pending;
static uint32_t s_button_poll_ms;
static uint32_t s_button_candidate_ms;
static UserIoState s_state;
static bool s_led_phase_initialized;
static bool s_led_phase_on;
static uint32_t s_led_phase_ms;
static bool s_led_attempted;
static uint32_t s_led_attempt_ms;
static bool s_led_sent;
static uint32_t s_led_sent_ms;
static RgbLedColor s_led_sent_color;

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static UserIoStatus user_io_init(const UserIoConfig* config);
static const char* user_io_status_str(UserIoStatus status);
static UserIoStatus user_io_process(uint32_t now_ms);
static bool user_io_take_button_press(void);
static UserIoStatus user_io_set_indicator(UserIoIndicator indicator);
static const UserIoState* user_io_get_state(void);
static void user_io_process_button(uint32_t now_ms);
static void user_io_refresh_led_transport(void);
static UserIoStatus user_io_process_indicator(uint32_t now_ms);
static RgbLedColor user_io_indicator_color(void);

// ! ========================= 接 口 变 量 定 义 ========================= ! //

const UserIoInterface user_io = {
    .init = user_io_init,
    .status_str = user_io_status_str,
    .process = user_io_process,
    .take_button_press = user_io_take_button_press,
    .set_indicator = user_io_set_indicator,
    .get_state = user_io_get_state,
};

// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 初始化服务，等待稳定释放后才允许产生按下事件
 * @param config 按键极性与 LED 亮度配置
 * @return 参数有效时返回 USER_IO_STATUS_OK
 */
static UserIoStatus user_io_init(const UserIoConfig* config) {
    if(config == NULL || config->ops == NULL || config->ops->read_button_level == NULL ||
       config->brightness == 0u || config->brightness > USER_IO_MAX_BRIGHTNESS) {
        return USER_IO_STATUS_INVALID_PARAM;
    }

    s_ops = *config->ops;
    s_button_active_high = config->button_active_high;
    s_brightness = config->brightness;
    s_button_sampled = false;
    s_button_candidate = false;
    s_button_stable = false;
    s_button_stable_known = false;
    s_button_armed = false;
    s_button_press_pending = false;
    s_button_poll_ms = 0u;
    s_button_candidate_ms = 0u;
    memset(&s_state, 0, sizeof(s_state));
    s_state.indicator = USER_IO_INDICATOR_WAIT;
    s_state.led_init_ok = config->led_init_ok;
    s_state.initialized = true;
    s_led_phase_initialized = false;
    s_led_phase_on = true;
    s_led_phase_ms = 0u;
    s_led_attempted = false;
    s_led_attempt_ms = 0u;
    s_led_sent = false;
    s_led_sent_ms = 0u;
    s_led_sent_color = (RgbLedColor){ 0 };
    s_initialized = true;
    user_io_refresh_led_transport();
    return USER_IO_STATUS_OK;
}

/**
 * @brief 获取状态码的静态名称
 * @param status 用户交互服务状态码
 * @return 不需要释放的字符串
 */
static const char* user_io_status_str(UserIoStatus status) {
    switch(status) {
        case USER_IO_STATUS_OK:
            return "OK";
        case USER_IO_STATUS_INVALID_PARAM:
            return "INVALID_PARAM";
        case USER_IO_STATUS_NOT_INITIALIZED:
            return "NOT_INITIALIZED";
        case USER_IO_STATUS_LED_BUSY:
            return "LED_BUSY";
        case USER_IO_STATUS_LED_ERROR:
            return "LED_ERROR";
        default:
            return "UNKNOWN";
    }
}

/**
 * @brief 先采样按键，再尝试刷新指示灯；LED 忙或出错不影响按键推进
 * @param now_ms 当前毫秒时间戳
 * @return 本轮灯态更新结果
 */
static UserIoStatus user_io_process(uint32_t now_ms) {
    if(!s_initialized) {
        return USER_IO_STATUS_NOT_INITIALIZED;
    }

    user_io_process_button(now_ms);
    user_io_refresh_led_transport();
    return user_io_process_indicator(now_ms);
}

/**
 * @brief 取走一次按下事件，调用方应在主循环每轮消费
 * @return 存在未消费事件时返回 true；一次稳定长按只产生一次事件
 */
static bool user_io_take_button_press(void) {
    bool pending = s_initialized && s_button_press_pending;

    s_button_press_pending = false;
    return pending;
}

/**
 * @brief 更新实际运行状态对应的灯态，相同状态不重置闪烁相位
 * @param indicator 需要显示的云台状态
 * @return 状态有效时返回 USER_IO_STATUS_OK
 */
static UserIoStatus user_io_set_indicator(UserIoIndicator indicator) {
    if(!s_initialized) {
        return USER_IO_STATUS_NOT_INITIALIZED;
    }
    if(indicator < USER_IO_INDICATOR_WAIT || indicator > USER_IO_INDICATOR_MANUAL) {
        return USER_IO_STATUS_INVALID_PARAM;
    }
    if(indicator != s_state.indicator) {
        s_state.indicator = indicator;
        s_led_phase_initialized = false;
    }
    return USER_IO_STATUS_OK;
}

/** @brief 返回最近一次主循环更新的诊断；不存在物理颜色确认 */
static const UserIoState* user_io_get_state(void) {
    return &s_state;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

/** @brief 异步错误使颜色缓存待重发；传输错误与成功完成分别保留 */
static void user_io_refresh_led_transport(void) {
    if(s_ops.get_led_tx_stats != NULL) {
        UserIoLedTxStats next = { 0 };
        s_ops.get_led_tx_stats(&next);
        if(next.error_count != s_state.tx.error_count) {
            s_led_sent = false;
        }
        s_state.tx = next;
    }
}

/**
 * @brief 以 5 ms 最小周期读取原始电平，连续稳定 25 ms 后才接受边沿
 * @param now_ms 当前毫秒时间戳
 */
static void user_io_process_button(uint32_t now_ms) {
    bool level;

    if(s_button_sampled && (uint32_t)(now_ms - s_button_poll_ms) < USER_IO_BUTTON_POLL_MS) {
        return;
    }
    s_button_poll_ms = now_ms;
    level = s_ops.read_button_level();

    if(!s_button_sampled || level != s_button_candidate) {
        s_button_sampled = true;
        s_button_candidate = level;
        s_button_candidate_ms = now_ms;
        return;
    }
    if((uint32_t)(now_ms - s_button_candidate_ms) < USER_IO_BUTTON_DEBOUNCE_MS) {
        return;
    }
    if(s_button_stable_known && s_button_stable == level) {
        return;
    }

    s_button_stable = level;
    s_button_stable_known = true;
    if(level != s_button_active_high) {
        s_button_armed = true;
    }
    else if(s_button_armed) {
        s_button_press_pending = true;
        s_button_armed = false;
    }
}

/**
 * @brief 根据当前状态和闪烁相位生成低亮度颜色
 * @return 需要写入 RGB 缓存的颜色
 */
static RgbLedColor user_io_indicator_color(void) {
    RgbLedColor color = { 0 };

    if(!s_led_phase_on) {
        return color;
    }
    switch(s_state.indicator) {
        case USER_IO_INDICATOR_WAIT:
            color.r = s_brightness;
            color.g = s_brightness;
            break;
        case USER_IO_INDICATOR_FAULT:
            color.r = s_brightness;
            break;
        case USER_IO_INDICATOR_RUN:
            color.b = s_brightness;
            break;
        case USER_IO_INDICATOR_MANUAL:
            color.g = s_brightness;
            break;
        default:
            break;
    }
    return color;
}

/**
 * @brief 非阻塞刷新指示灯，发送失败或忙时至少间隔 20 ms 再尝试
 * @param now_ms 当前毫秒时间戳
 * @return RGB 本次接受发送的结果；不等待 DMA 完成
 */
static UserIoStatus user_io_process_indicator(uint32_t now_ms) {
    uint32_t half_period_ms = 0u;
    RgbLedColor color;
    RgbLedStatus status;

    if(!s_led_phase_initialized) {
        s_led_phase_initialized = true;
        s_led_phase_on = true;
        s_led_phase_ms = now_ms;
    }
    if(s_state.indicator == USER_IO_INDICATOR_WAIT) {
        half_period_ms = USER_IO_WAIT_HALF_PERIOD_MS;
    }
    else if(s_state.indicator == USER_IO_INDICATOR_FAULT) {
        half_period_ms = USER_IO_FAULT_HALF_PERIOD_MS;
    }
    if(half_period_ms != 0u) {
        uint32_t steps = (uint32_t)(now_ms - s_led_phase_ms) / half_period_ms;
        if(steps != 0u) {
            if((steps & 1u) != 0u) {
                s_led_phase_on = !s_led_phase_on;
            }
            s_led_phase_ms += steps * half_period_ms;
        }
    }

    color = user_io_indicator_color();
    if(s_led_sent && color.r == s_led_sent_color.r && color.g == s_led_sent_color.g &&
       color.b == s_led_sent_color.b && (uint32_t)(now_ms - s_led_sent_ms) < USER_IO_LED_REFRESH_MS) {
        return USER_IO_STATUS_OK;
    }
    if(s_led_attempted && (uint32_t)(now_ms - s_led_attempt_ms) < USER_IO_LED_RETRY_MS) {
        return USER_IO_STATUS_OK;
    }

    s_led_attempted = true;
    s_led_attempt_ms = now_ms;
    s_state.submit_attempted = true;
    status = rgb_led_fill(color.r, color.g, color.b);
    if(status == RGB_LED_STATUS_OK) {
        status = rgb_led_show();
    }
    if(status == RGB_LED_STATUS_BUSY) {
        s_state.last_submit_status = USER_IO_STATUS_LED_BUSY;
        ++s_state.busy_count;
        user_io_refresh_led_transport();
        return USER_IO_STATUS_LED_BUSY;
    }
    if(status != RGB_LED_STATUS_OK) {
        s_state.last_submit_status = USER_IO_STATUS_LED_ERROR;
        ++s_state.error_count;
        s_led_sent = false;
        user_io_refresh_led_transport();
        return USER_IO_STATUS_LED_ERROR;
    }

    /* 每秒刷新，异步错误后按最小间隔重试 */
    s_led_sent = true;
    s_led_sent_ms = now_ms;
    s_led_sent_color = color;
    s_state.last_submit_status = USER_IO_STATUS_OK;
    user_io_refresh_led_transport();
    return USER_IO_STATUS_OK;
}
