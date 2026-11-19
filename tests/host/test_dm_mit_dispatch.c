#include "bus_motor/bus_motor.h"
#include "bus_motor/dm_motor.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static uint32_t mock_time_ms;
static uint32_t last_can_id;
static uint8_t last_can_data[8];
static uint8_t last_can_len;
static uint32_t sent_frames;

static bool mock_send(uint32_t id, const uint8_t* data, uint8_t len) {
    last_can_id = id;
    last_can_len = len;
    memset(last_can_data, 0, sizeof(last_can_data));
    memcpy(last_can_data, data, len > sizeof(last_can_data) ? sizeof(last_can_data) : len);
    ++sent_frames;

    if(id == 0x7FFu && len == 8u && data[2] == 0x55u && data[3] == 10u) {
        uint8_t ack[8];
        memcpy(ack, data, sizeof(ack));
        assert(dm_motor_parse_parameter_frame(0u, ack));
    }
    return true;
}

static uint32_t mock_now_ms(void) { return mock_time_ms; }
static void mock_delay_ms(uint32_t ms) { mock_time_ms += ms; }

int main(void) {
    DmMotorInstance instances[1];
    const BusMotorPortOps ops = {
        .send = mock_send,
        .read = NULL,
        .now_ms = mock_now_ms,
        .delay_ms = mock_delay_ms,
        .flush_rx = NULL,
    };
    const DmMotorConfig config = {
        .can_id = 1u,
        .master_id = 0u,
        .model = DM_MOTOR_MODEL_DMG6220,
        .firmware = { .major = DM_MOTOR_FIRMWARE_V4 },
        .default_mode = DM_MOTOR_MODE_POS_VEL,
    };
    const BusMotorImpedanceCommand command = {
        .position = 0.25f,
        .velocity = -0.5f,
        .kp = 4.0f,
        .kd = 0.04f,
        .torque = 0.0f,
    };

    assert(bus_motor.init() == MOTOR_STATUS_OK);
    assert(dm_motor_init(&ops, instances, 1u) == MOTOR_STATUS_OK);
    assert(dm_motor_bind(1u, &config) == MOTOR_STATUS_OK);
    assert(bus_motor.profile.activate(1u, BUS_MOTOR_PROFILE_IMPEDANCE) == MOTOR_STATUS_OK);
    assert(bus_motor.profile.current(1u) == BUS_MOTOR_PROFILE_IMPEDANCE);
    assert(instances[0].mode == DM_MOTOR_MODE_MIT);

    assert(bus_motor.imp(1u, &command) == MOTOR_STATUS_OK);
    assert(last_can_id == 1u);
    assert(last_can_len == 8u);
    assert(sent_frames >= 2u);

    puts("DM MIT dispatch test passed");
    return 0;
}
