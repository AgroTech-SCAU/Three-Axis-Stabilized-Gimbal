#include "bus_motor/dm_motor.h"
#include "bus_motor/dm_motor/dm_motor_protocol.h"
#include "bus_motor/dm_motor/dm_motor_core.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static float decode(uint16_t value, float lower, float upper, uint8_t bits) {
    return dm_motor_codec_uint_to_f32(value, lower, upper, bits);
}

int main(void) {
    DmMotorInstance motor = { 0 };
    DmMotorProtocolFrame frame = { 0 };
    uint16_t position_bits;
    uint16_t velocity_bits;
    uint16_t kp_bits;
    uint16_t kd_bits;
    uint16_t torque_bits;

    assert(dm_motor_protocol_v4.supports_mode(DM_MOTOR_MODE_MIT));

    motor.can_id = 0x01u;
    motor.mode = DM_MOTOR_MODE_MIT;
    motor.limits = (DmMotorLimitParam){ 12.5f, 45.0f, 10.0f };
    motor.position = 0.5f;
    motor.velocity = -1.25f;
    motor.kp = 4.0f;
    motor.kd = 0.04f;
    motor.torque = 0.0f;

    assert(dm_motor_protocol_v4.build_control(&motor, &frame) == MOTOR_STATUS_OK);
    assert(frame.id == 0x01u);
    assert(frame.len == 8u);

    position_bits = (uint16_t)(((uint16_t)frame.data[0] << 8) | frame.data[1]);
    velocity_bits = (uint16_t)(((uint16_t)frame.data[2] << 4) | (frame.data[3] >> 4));
    kp_bits = (uint16_t)(((uint16_t)(frame.data[3] & 0x0Fu) << 8) | frame.data[4]);
    kd_bits = (uint16_t)(((uint16_t)frame.data[5] << 4) | (frame.data[6] >> 4));
    torque_bits = (uint16_t)(((uint16_t)(frame.data[6] & 0x0Fu) << 8) | frame.data[7]);

    assert(fabsf(decode(position_bits, -12.5f, 12.5f, 16u) - motor.position) < 0.001f);
    assert(fabsf(decode(velocity_bits, -45.0f, 45.0f, 12u) - motor.velocity) < 0.03f);
    assert(fabsf(decode(kp_bits, 0.0f, 500.0f, 12u) - motor.kp) < 0.2f);
    assert(fabsf(decode(kd_bits, 0.0f, 5.0f, 12u) - motor.kd) < 0.01f);
    assert(fabsf(decode(torque_bits, -10.0f, 10.0f, 12u) - motor.torque) < 0.01f);

    motor.mode = DM_MOTOR_MODE_POS_VEL;
    assert(dm_motor_protocol_v4.build_control(&motor, &frame) == MOTOR_STATUS_OK);
    assert(frame.id == 0x101u);

    puts("DM MIT frame test passed");
    return 0;
}
