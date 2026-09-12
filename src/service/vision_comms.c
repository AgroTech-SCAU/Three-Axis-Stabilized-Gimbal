/**
 * @file vision_comms.c
 * @brief 云台相机（气泡水平仪视觉）串口接收服务实现
 *
 * 接收字节 -> 环形缓冲 -> 按行切分 -> 解析 status,x_deg,y_deg -> 缓存结果。
 * 解析使用 strtol/strtof，不依赖 scanf 浮点支持。
 */

#include "vision_comms.h"

#include "delay.h"
#include "log.h"
#include "protocol_parser.h"

#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// ! ========================= 宏 定 义 声 明 ========================= ! //

#define VISION_COMMS_RX_RING_SIZE 128u
#define VISION_COMMS_LINE_BUF_SIZE 64u
#define VISION_COMMS_ONLINE_TIMEOUT_MS 1000u
#define VISION_COMMS_WARN_LOG_PERIOD_MS 1000u

// ! ========================= 变 量 声 明 ========================= ! //

static uint8_t s_vision_comms_rx_ring_buf[VISION_COMMS_RX_RING_SIZE] = { 0 };
static RingBuf s_vision_comms_rx_ring = { 0 };
static char s_vision_comms_line_buf[VISION_COMMS_LINE_BUF_SIZE] = { 0 };
static uint16_t s_vision_comms_line_len = 0u;
static bool s_vision_comms_line_overflow = false;
static VisionCommsConfig s_vision_comms_config = { 0 };
static VisionLevelResult s_vision_comms_result = { 0 };
static bool s_vision_comms_result_valid = false;
static VisionCommsStats s_vision_comms_stats = { 0 };
static uint32_t s_vision_comms_warn_last_ms = 0u;

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static uint32_t vision_comms_now_ms(void);
static void vision_comms_warn_limited(const char* message);
static void vision_comms_handle_line(const char* line);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

VisionCommsStatus vision_comms_init(const VisionCommsConfig* config) {
    if(config == NULL || config->port_ops.now_ms == NULL) {
        return VISION_COMMS_STATUS_INVALID_PARAM;
    }

    s_vision_comms_config = *config;
    if(ring_buf_create(&s_vision_comms_rx_ring,
                       s_vision_comms_rx_ring_buf,
                       VISION_COMMS_RX_RING_SIZE,
                       true) != RING_BUF_SUCCESS) {
        return VISION_COMMS_STATUS_INVALID_PARAM;
    }
    s_vision_comms_line_len = 0u;
    s_vision_comms_line_overflow = false;
    memset(&s_vision_comms_result, 0, sizeof(s_vision_comms_result));
    s_vision_comms_result_valid = false;
    memset(&s_vision_comms_stats, 0, sizeof(s_vision_comms_stats));
    s_vision_comms_warn_last_ms = 0u;
    return VISION_COMMS_STATUS_OK;
}

void vision_comms_on_rx_byte(uint8_t data) {
    (void)ring_buf_write(&s_vision_comms_rx_ring, data);
}

void vision_comms_process(void) {
    uint8_t byte;

    while(ring_buf_read(&s_vision_comms_rx_ring, &byte) == RING_BUF_SUCCESS) {
        if(byte == '\n') {
            s_vision_comms_line_buf[s_vision_comms_line_len] = '\0';
            if(!s_vision_comms_line_overflow) {
                vision_comms_handle_line(s_vision_comms_line_buf);
            }
            s_vision_comms_line_len = 0u;
            s_vision_comms_line_overflow = false;
        }
        else if(byte == '\r') {
            /* 忽略 \r，兼容 \r\n 行结束 */
        }
        else {
            if(s_vision_comms_line_overflow) {
                continue; /* 行过长，丢弃直到换行 */
            }

            if(s_vision_comms_line_len < (VISION_COMMS_LINE_BUF_SIZE - 1u)) {
                s_vision_comms_line_buf[s_vision_comms_line_len++] = (char)byte;
            }
            else {
                s_vision_comms_line_overflow = true;
                s_vision_comms_line_len = 0u;
            }
        }
    }
}

bool vision_comms_is_online(void) {
    return s_vision_comms_stats.last_rx_ms != 0u &&
           (vision_comms_now_ms() - s_vision_comms_stats.last_rx_ms) <= VISION_COMMS_ONLINE_TIMEOUT_MS;
}

bool vision_comms_result_is_fresh(uint32_t timeout_ms) {
    if(!s_vision_comms_result_valid) {
        return false;
    }

    return (vision_comms_now_ms() - s_vision_comms_result.stamp_ms) <= timeout_ms;
}

bool vision_comms_get_result(VisionLevelResult* result) {
    if(result == NULL || !s_vision_comms_result_valid) {
        return false;
    }

    *result = s_vision_comms_result;
    return true;
}

bool vision_comms_get_stats(VisionCommsStats* stats) {
    if(stats == NULL) {
        return false;
    }

    *stats = s_vision_comms_stats;
    return true;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static uint32_t vision_comms_now_ms(void) {
    return s_vision_comms_config.port_ops.now_ms();
}

static void vision_comms_warn_limited(const char* message) {
    const uint32_t now_ms = vision_comms_now_ms();

    if(s_vision_comms_warn_last_ms == 0u || (now_ms - s_vision_comms_warn_last_ms) >= VISION_COMMS_WARN_LOG_PERIOD_MS) {
        s_vision_comms_warn_last_ms = now_ms;
        log_warn("%s", message);
    }
}

static void vision_comms_handle_line(const char* line) {
    char* end = NULL;
    long status;
    float x_deg;
    float y_deg;
    uint32_t now_ms = vision_comms_now_ms();

    /* 格式: status,x_deg,y_deg */
    status = strtol(line, &end, 10);
    if(end == line || *end != ',') {
        /* 非目标行（例如开机 "spirit_level ready, ..." 行），静默忽略 */
        return;
    }

    {
        char* value_start = end + 1;
        x_deg = strtof(value_start, &end);
        if(end == value_start || *end != ',' || !isfinite(x_deg)) {
            s_vision_comms_stats.rx_parse_fail_count++;
            vision_comms_warn_limited("VISION_COMMS line dropped: bad x_deg");
            return;
        }
    }

    {
        char* value_start = end + 1;
        y_deg = strtof(value_start, &end);
        if(end == value_start || !isfinite(y_deg)) {
            s_vision_comms_stats.rx_parse_fail_count++;
            vision_comms_warn_limited("VISION_COMMS line dropped: bad y_deg");
            return;
        }
        while(*end != '\0' && isspace((unsigned char)*end)) {
            ++end;
        }
        if(*end != '\0') {
            s_vision_comms_stats.rx_parse_fail_count++;
            vision_comms_warn_limited("VISION_COMMS line dropped: trailing garbage");
            return;
        }
    }

    switch(status) {
        case 1:
            s_vision_comms_result.state = VISION_LEVEL_STATE_LEVEL;
            break;

        case 0:
            s_vision_comms_result.state = VISION_LEVEL_STATE_NOT_LEVEL;
            break;

        case 2:
            s_vision_comms_result.state = VISION_LEVEL_STATE_FAIL;
            break;

        default:
            s_vision_comms_stats.rx_parse_fail_count++;
            vision_comms_warn_limited("VISION_COMMS line dropped: bad status");
            return;
    }

    s_vision_comms_result.corr_x_deg = x_deg;
    s_vision_comms_result.corr_y_deg = y_deg;
    s_vision_comms_result.stamp_ms = now_ms;
    s_vision_comms_result_valid = true;

    s_vision_comms_stats.rx_line_count++;
    s_vision_comms_stats.last_rx_ms = now_ms;
}
