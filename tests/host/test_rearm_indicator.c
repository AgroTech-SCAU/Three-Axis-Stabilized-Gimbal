#include "app_runtime.h"
#include "assemble/assemble.h"
#include "gimbal.h"
#include "user_io.h"

#include <assert.h>
#include <stdio.h>

static GimbalState test_state;
static UserIoIndicator last_indicator = USER_IO_INDICATOR_WAIT;
static UserIoState user_io_state;

static void mock_gimbal_process(void) {}
static const GimbalState* mock_gimbal_get_state(void) { return &test_state; }
static bool mock_gimbal_acknowledge(void) { return true; }
static bool mock_gimbal_toggle(void) { return true; }
static bool mock_gimbal_ready(void) { return test_state.ready; }
static GimbalStatus mock_gimbal_init(const GimbalConfig* config) { (void)config; return GIMBAL_STATUS_OK; }
static const char* mock_gimbal_status(GimbalStatus status) { (void)status; return "OK"; }

const struct GimbalInterface gimbal_interface = {
    .init = mock_gimbal_init,
    .process = mock_gimbal_process,
    .acknowledge_fault = mock_gimbal_acknowledge,
    .toggle_target_mode = mock_gimbal_toggle,
    .get_state = mock_gimbal_get_state,
    .is_ready = mock_gimbal_ready,
    .status_str = mock_gimbal_status,
};

static SystemStatus mock_assemble_gimbal(void) { return SYSTEM_STATUS_OK; }
SystemStatus assemble_gimbal(void) { return mock_assemble_gimbal(); }

static UserIoStatus mock_user_io_process(uint32_t now_ms) { (void)now_ms; return USER_IO_STATUS_OK; }
static bool mock_button_press(void) { return false; }
static UserIoStatus mock_set_indicator(UserIoIndicator indicator) {
    last_indicator = indicator;
    user_io_state.indicator = indicator;
    return USER_IO_STATUS_OK;
}
static const UserIoState* mock_user_io_get_state(void) { return &user_io_state; }

const UserIoInterface user_io = {
    .process = mock_user_io_process,
    .take_button_press = mock_button_press,
    .set_indicator = mock_set_indicator,
    .get_state = mock_user_io_get_state,
};

int main(void) {
    app_runtime_init();

    test_state = (GimbalState){ .rearming = true, .fault = true, .fault_latched = false };
    app_runtime_process();
    assert(last_indicator == USER_IO_INDICATOR_WAIT);

    test_state.fault_latched = true;
    app_runtime_process();
    assert(last_indicator == USER_IO_INDICATOR_FAULT);

    test_state = (GimbalState){ .manual = true, .motors_released = true };
    app_runtime_process();
    assert(last_indicator == USER_IO_INDICATOR_MANUAL);

    test_state = (GimbalState){ .running = true };
    app_runtime_process();
    assert(last_indicator == USER_IO_INDICATOR_RUN);

    puts("Re-arm indicator test passed");
    return 0;
}
