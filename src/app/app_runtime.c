#include "app_runtime.h"

#include "assemble/assemble.h"
#include "gimbal.h"
#include "user_io.h"

// ! ========================= 变 量 声 明 ========================= ! //

static bool initialized;

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static UserIoIndicator app_runtime_indicator(const GimbalState* state);

// ! ========================= 接 口 变 量 定 义 ========================= ! //

const struct AppRuntimeInterface app_runtime_interface = {
    .init = app_runtime_init,
    .process = app_runtime_process,
};

// ! ========================= 接 口 函 数 实 现 ========================= ! //

void app_runtime_init(void) {
    initialized = assemble_gimbal() == SYSTEM_STATUS_OK;
    (void)user_io.set_indicator(initialized ? USER_IO_INDICATOR_WAIT : USER_IO_INDICATOR_FAULT);
}

void app_runtime_process(void) {
    gimbal.process();
    const GimbalState* state = gimbal.get_state();
    (void)user_io.set_indicator(app_runtime_indicator(state));
    /* LED 失败不阻断按键，初始化失败也继续推进闪烁 */
    (void)user_io.process(state->now_ms);
    if(user_io.take_button_press() && initialized && state->fault_latched) {
        (void)gimbal.acknowledge_fault();
    }
    (void)user_io.set_indicator(app_runtime_indicator(state));
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static UserIoIndicator app_runtime_indicator(const GimbalState* state) {
    if(!initialized || state->fault) {
        return USER_IO_INDICATOR_FAULT;
    }
    if(state->running && !state->run_waiting) {
        return USER_IO_INDICATOR_RUN;
    }
    return USER_IO_INDICATOR_WAIT;
}
