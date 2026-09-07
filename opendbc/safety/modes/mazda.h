#pragma once

#include "opendbc/safety/declarations.h"

// CAN msgs we care about
#define MAZDA_LKAS          0x243U
#define MAZDA_LKAS_HUD      0x440U
#define MAZDA_CRZ_INFO      0x21bU
#define MAZDA_CRZ_CTRL      0x21cU
#define MAZDA_CRZ_BTNS      0x09dU
#define MAZDA_RADAR_STATIC  0x499U
#define MAZDA_RADAR_TRACK_1 0x361U
#define MAZDA_RADAR_TRACK_2 0x362U
#define MAZDA_RADAR_TRACK_3 0x363U
#define MAZDA_RADAR_TRACK_4 0x364U
#define MAZDA_RADAR_TRACK_5 0x365U
#define MAZDA_RADAR_TRACK_6 0x366U
#define MAZDA_STEER_TORQUE  0x240U
#define MAZDA_ENGINE_DATA   0x202U
#define MAZDA_PEDALS        0x165U
#define MAZDA_RADAR_DIAG    0x764U

// Independent opt-in permissions. Bit 0 is already deployed for the BM
// low-speed steering envelope; direct longitudinal must never alias it.
#define MAZDA_PARAM_BM_LOW_SPEED_STEER 1U
#define MAZDA_PARAM_VISION_ONLY_RADAR  2U

// CAN bus numbers
#define MAZDA_MAIN 0
#define MAZDA_CAM  2

static bool mazda_vision_only_radar = false;
static bool mazda_bm_low_speed_steer = false;
static int mazda_desired_accel_last[2] = {0, 0};

static const TorqueSteeringLimits MAZDA_STEERING_LIMITS = {
  .max_torque = 800,
  .max_rate_up = 10,
  .max_rate_down = 25,
  .max_rt_delta = 300,
  .driver_torque_multiplier = 1,
  .driver_torque_allowance = 15,
  .type = TorqueDriverLimited,
};

static const TorqueSteeringLimits MAZDA_BM_STEERING_LIMITS = {
  .max_torque = 1500,
  .dynamic_max_torque = false,
  .max_rate_up = 16,
  .max_rate_down = 25,
  .max_rt_delta = 700,
  .driver_torque_multiplier = 1,
  .driver_torque_allowance = 22,
  .type = TorqueDriverLimited,
};

// CRZ_INFO.ACCEL_CMD is a 13-bit Motorola signal spanning data[2:4]. The DBC
// used by the proven BM drive encodes it at 0.001 m/s^2 with an offset of
// -4.096 m/s^2, so removing the raw offset yields milli-m/s^2 directly.
static int mazda_get_desired_accel(const CANPacket_t *msg) {
  return ((((int)msg->data[2] & 0x3) << 11) |
          (((int)msg->data[3]) << 3) |
          (((int)msg->data[4]) >> 5)) - 4096;
}

static bool mazda_crz_info_checksum_valid(const CANPacket_t *msg) {
  uint8_t checksum = 0U;
  for (int i = 0; i < 7; i++) {
    checksum += msg->data[i];
  }
  checksum = 0xFFU - checksum;
  return msg->data[7] == checksum;
}

static bool mazda_crz_info_standby_valid(const CANPacket_t *msg) {
  // Stock main-off pattern. The low nibble of data[6] is the rolling counter,
  // so its checksum is pinned as a function of that byte.
  return (msg->data[0] == 0x01U) && (msg->data[1] == 0xFFU) &&
         (msg->data[2] == 0xE3U) && (msg->data[3] == 0xFFU) &&
         (msg->data[4] == 0xC0U) && (msg->data[5] == 0x00U) &&
         ((msg->data[6] & 0xF0U) == 0x00U) &&
         (msg->data[7] == ((0x5DU - msg->data[6]) & 0xFFU));
}

static bool mazda_crz_info_layout_valid(const CANPacket_t *msg) {
  // Ordinary cruise/follow frame only. STOPPING, STOPPING_2 and
  // RESUME_UNLATCHING remain blocked for the first vision-only release.
  return (msg->data[0] == 0x01U) && (msg->data[1] == 0xFFU) &&
         ((msg->data[2] & 0xFCU) == 0xE0U) &&
         ((msg->data[4] & 0x1DU) == 0x04U) &&
         (msg->data[5] == 0x80U) &&
         ((msg->data[6] & 0xF0U) == 0x00U) &&
         mazda_crz_info_checksum_valid(msg);
}

static bool mazda_crz_ctrl_layout_valid(const CANPacket_t *msg) {
  const bool cruise_active = GET_BIT(msg, 3U);
  const unsigned int stop_go_phase = msg->data[3] >> 5;

  // Permit only fields emitted by create_crz_ctrl for ordinary cruise/follow.
  // The proven active template (0a010b2000001000 for gap 2) uses phase 1 and
  // ACC_ACTIVE_2. Inactive frames use phase 0 and clear ACC_ACTIVE_2. A radar
  // target is never synthesized in this vision-only first release.
  return ((msg->data[0] & 0xF5U) == 0x00U) && ((msg->data[0] & 0x02U) == 0x02U) &&
         (msg->data[1] == 0x01U) &&
         ((msg->data[2] & 0xE1U) == 0x01U) &&
         ((msg->data[3] & 0x1FU) == 0x00U) &&
         (stop_go_phase == (cruise_active ? 1U : 0U)) &&
         (msg->data[4] == 0x00U) && (msg->data[5] == 0x00U) &&
         (msg->data[6] == (cruise_active ? 0x10U : 0x00U)) &&
         (msg->data[7] == 0x00U);
}

static bool mazda_empty_radar_static_valid(const CANPacket_t *msg) {
  return (msg->data[0] == 0x00U) && (msg->data[1] == 0x08U) &&
         (msg->data[2] == 0xC0U) && (msg->data[3] == 0x00U) &&
         (msg->data[4] == 0x00U) && (msg->data[5] == 0x00U) &&
         (msg->data[6] == 0x00U) && (msg->data[7] == 0x00U);
}

static bool mazda_empty_radar_track_valid(const CANPacket_t *msg) {
  bool valid = false;

  if (msg->addr == MAZDA_RADAR_TRACK_1) {
    valid = (msg->data[0] == 0xFFU) && (msg->data[1] == 0xF7U) &&
            (msg->data[2] == 0xFEU) && (msg->data[3] == 0xFEU) &&
            (msg->data[4] == 0x1FU) && (msg->data[5] == 0xC0U) &&
            (msg->data[6] == 0x00U) && ((msg->data[7] & 0xF0U) == 0x80U);
  } else if (msg->addr == MAZDA_RADAR_TRACK_2) {
    valid = (msg->data[0] == 0xFFU) && (msg->data[1] == 0xF7U) &&
            (msg->data[2] == 0xFEU) && (msg->data[3] == 0xFEU) &&
            (msg->data[4] == 0x1FU) && (msg->data[5] == 0xC7U) &&
            (msg->data[6] == 0x8CU) && ((msg->data[7] & 0xF0U) == 0x80U);
  } else if ((msg->addr == MAZDA_RADAR_TRACK_3) || (msg->addr == MAZDA_RADAR_TRACK_4)) {
    valid = (msg->data[0] == 0xFFU) && (msg->data[1] == 0xF7U) &&
            (msg->data[2] == 0xFEU) && (msg->data[3] == 0xFEU) &&
            (msg->data[4] == 0x1FU) && (msg->data[5] == 0xC0U) &&
            (msg->data[6] == 0x00U) && ((msg->data[7] & 0xF0U) == 0x00U);
  } else if ((msg->addr == MAZDA_RADAR_TRACK_5) || (msg->addr == MAZDA_RADAR_TRACK_6)) {
    valid = (msg->data[0] == 0xFFU) && (msg->data[1] == 0xF7U) &&
            (msg->data[2] == 0xFEU) && (msg->data[3] == 0x7FU) &&
            (msg->data[4] == 0xFBU) && (msg->data[5] == 0xFFU) &&
            (msg->data[6] == 0x3FU) && ((msg->data[7] & 0xF0U) == 0xC0U);
  } else {
  }

  // Only the rolling counter in the low nibble of data[7] is unconstrained.
  // In particular, occupied/synthetic target payloads are never accepted.
  return valid;
}

// track msgs coming from OP so that we know what CAM msgs to drop and what to forward
static void mazda_rx_hook(const CANPacket_t *msg) {
  if ((int)msg->bus == MAZDA_MAIN) {
    if (msg->addr == MAZDA_ENGINE_DATA) {
      // sample speed: scale by 0.01 to get kph
      int speed = (msg->data[2] << 8) | msg->data[3];
      vehicle_moving = speed > 10; // moving when speed > 0.1 kph
      UPDATE_VEHICLE_SPEED(speed * 0.01 * KPH_TO_MS);
    }

    if (msg->addr == MAZDA_STEER_TORQUE) {
      int torque_driver_new = msg->data[0] - 127U;
      // update array of samples
      update_sample(&torque_driver, torque_driver_new);
    }

    // enter controls on rising edge of ACC, exit controls on ACC off
    if ((msg->addr == MAZDA_CRZ_CTRL) && !mazda_vision_only_radar) {
      bool cruise_engaged = msg->data[0] & 0x8U;
      pcm_cruise_check(cruise_engaged);
      // SOURCE: sunnypilot/opendbc mazda.h + vendor 4f5c464 mazda.h (DRIVE-MVP-001B MADS acc_main)
      acc_main_on = GET_BIT(msg, 17U);
    }

    if (msg->addr == MAZDA_ENGINE_DATA) {
      gas_pressed = (msg->data[4] || (msg->data[5] & 0xF0U));
    }

    if ((msg->addr == MAZDA_CRZ_BTNS) && mazda_vision_only_radar) {
      // Keep the physical cancel button authoritative after CRZ_CTRL vanishes.
      if (GET_BIT(msg, 0U)) {
        controls_allowed = false;
      }
    }

    if (msg->addr == MAZDA_PEDALS) {
      bool brake = (msg->data[0] & 0x10U);
      if (mazda_vision_only_radar) {
        // Radar silence removes stock CRZ_CTRL. PEDALS retains the independent
        // ACC_OFF/ACC_ACTIVE state used by the vehicle during the proven drive.
        bool cruise_engaged = GET_BIT(msg, 3U);
        bool acc_main = GET_BIT(msg, 2U) || cruise_engaged;
        bool stable_pedal_state = !brake && !brake_pressed_prev;

        // During a brake press both state bits can briefly be low. Longitudinal
        // controls must still disengage immediately, but ACC MAIN must remain
        // latched so MADS can apply its configured brake policy instead of
        // always dropping lateral control.
        if (acc_main || stable_pedal_state) {
          acc_main_on = acc_main;
        }
        if (cruise_engaged || cruise_engaged_prev || stable_pedal_state) {
          pcm_cruise_check(cruise_engaged);
        }
      }
      brake_pressed = brake;
    }
  }
}

static bool mazda_tx_hook(const CANPacket_t *msg) {
  const LongitudinalLimits MAZDA_VISION_ONLY_LONG_LIMITS = {
    // milli-m/s^2 after removing the CRZ_INFO raw offset
    .max_accel = 600,       // +0.60 m/s^2
    .min_accel = -1500,     // -1.50 m/s^2
    .inactive_accel = 0,
  };

  bool tx = true;
  const bool long_replacement_bus = (msg->bus == (unsigned char)MAZDA_MAIN) ||
                                    (msg->bus == (unsigned char)MAZDA_CAM);

  if (mazda_vision_only_radar && long_replacement_bus && (msg->addr == MAZDA_CRZ_INFO)) {
    const bool standby = mazda_crz_info_standby_valid(msg);
    if (!standby) {
      const int desired_accel = mazda_get_desired_accel(msg);
      const bool acc_active = GET_BIT(msg, 33U);
      const bool gas_override = gas_pressed_prev;

      bool violation = !mazda_crz_info_layout_valid(msg);
      violation |= longitudinal_accel_checks(desired_accel, MAZDA_VISION_ONLY_LONG_LIMITS);
      // Brake always drops longitudinal authority. A gas override may retain
      // ACC_ACTIVE only with a strict zero command, matching the proven Mazda
      // behavior and avoiding an active-bit transition lurch/rev flare.
      violation |= acc_active && brake_pressed_prev;
      violation |= acc_active && !gas_override && !controls_allowed;
      violation |= acc_active && gas_override && (desired_accel != 0);
      violation |= !acc_active && (desired_accel != MAZDA_VISION_ONLY_LONG_LIMITS.inactive_accel);

      if (!violation && acc_active && !gas_override) {
        // Commands are emitted at 50 Hz on bus 0 and duplicated on bus 2.
        // +10/-30 raw per unique command corresponds to +0.50/-1.50 m/s^3.
        const int bus_index = (msg->bus == (unsigned char)MAZDA_CAM) ? 1 : 0;
        const int accel_delta = desired_accel - mazda_desired_accel_last[bus_index];
        violation |= (accel_delta > 10) || (accel_delta < -30);
      }

      if (violation) {
        tx = false;
      } else {
        const int bus_index = (msg->bus == (unsigned char)MAZDA_CAM) ? 1 : 0;
        mazda_desired_accel_last[bus_index] = acc_active ? desired_accel : 0;
      }
    } else {
      const int bus_index = (msg->bus == (unsigned char)MAZDA_CAM) ? 1 : 0;
      mazda_desired_accel_last[bus_index] = 0;
    }
  }

  if (mazda_vision_only_radar && long_replacement_bus && (msg->addr == MAZDA_CRZ_CTRL)) {
    const bool cruise_active = GET_BIT(msg, 3U);
    const bool cruise_available = GET_BIT(msg, 17U);

    if (!mazda_crz_ctrl_layout_valid(msg) ||
        (cruise_active && (!cruise_available || brake_pressed_prev ||
                           (!controls_allowed && !gas_pressed_prev)))) {
      tx = false;
    }
  }

  if (mazda_vision_only_radar && long_replacement_bus && (msg->addr == MAZDA_RADAR_STATIC)) {
    if (!mazda_empty_radar_static_valid(msg)) {
      tx = false;
    }
  }

  if (mazda_vision_only_radar && long_replacement_bus &&
      (msg->addr >= MAZDA_RADAR_TRACK_1) && (msg->addr <= MAZDA_RADAR_TRACK_6)) {
    if (!mazda_empty_radar_track_valid(msg)) {
      tx = false;
    }
  }

  // Check if msg is sent on the main BUS
  if (msg->bus == (unsigned char)MAZDA_MAIN) {
    // steer cmd checks
    if (msg->addr == MAZDA_LKAS) {
      int desired_torque = (((msg->data[0] & 0x0FU) << 8) | msg->data[1]) - 2048U;

      const TorqueSteeringLimits limits = mazda_bm_low_speed_steer ? MAZDA_BM_STEERING_LIMITS : MAZDA_STEERING_LIMITS;
      if (steer_torque_cmd_checks(desired_torque, -1, limits)) {
        tx = false;
      }
    }

    // cruise buttons check
    if (msg->addr == MAZDA_CRZ_BTNS) {
      // allow resume spamming while controls allowed, but
      // only allow cancel while controls not allowed
      bool cancel_cmd = (msg->data[0] == 0x1U);
      if (!controls_allowed && !cancel_cmd) {
        tx = false;
      }
    }

    if (msg->addr == MAZDA_RADAR_DIAG) {
      const bool programming_session = (GET_BYTES(msg, 0, 4) == 0x00021002U) && (GET_BYTES(msg, 4, 4) == 0x0U);
      const bool default_session = (GET_BYTES(msg, 0, 4) == 0x00011002U) && (GET_BYTES(msg, 4, 4) == 0x0U);
      const bool tester_present = (GET_BYTES(msg, 0, 4) == 0x00803E02U) && (GET_BYTES(msg, 4, 4) == 0x0U);

      // Taking ownership is additionally pinned to standstill in Safety, not
      // only in CarController. Hand-back may restore the stock radar after a
      // fault while moving, but only after longitudinal controls disengage.
      const bool takeover_allowed = !controls_allowed && !vehicle_moving && programming_session;
      const bool handback_allowed = !controls_allowed && default_session;
      if (!mazda_vision_only_radar || !(tester_present || takeover_allowed || handback_allowed)) {
        tx = false;
      }
    }
  }

  return tx;
}

static safety_config mazda_init(uint16_t param) {
  static const CanMsg MAZDA_TX_MSGS[] = {{MAZDA_LKAS, 0, 8, .check_relay = true}, {MAZDA_CRZ_BTNS, 0, 8, .check_relay = false}, {MAZDA_LKAS_HUD, 0, 8, .check_relay = true}};
  static const CanMsg MAZDA_VISION_ONLY_RADAR_TX_MSGS[] = {
    {MAZDA_LKAS, 0, 8, .check_relay = true},
    {MAZDA_CRZ_BTNS, 0, 8, .check_relay = false},
    {MAZDA_LKAS_HUD, 0, 8, .check_relay = true},
    {MAZDA_RADAR_DIAG, 0, 8, .check_relay = false},
    {MAZDA_CRZ_INFO, MAZDA_MAIN, 8, .check_relay = false},
    {MAZDA_CRZ_CTRL, MAZDA_MAIN, 8, .check_relay = false},
    {MAZDA_CRZ_INFO, MAZDA_CAM, 8, .check_relay = false},
    {MAZDA_CRZ_CTRL, MAZDA_CAM, 8, .check_relay = false},
    {MAZDA_RADAR_STATIC, MAZDA_MAIN, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_1, MAZDA_MAIN, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_2, MAZDA_MAIN, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_3, MAZDA_MAIN, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_4, MAZDA_MAIN, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_5, MAZDA_MAIN, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_6, MAZDA_MAIN, 8, .check_relay = false},
    {MAZDA_RADAR_STATIC, MAZDA_CAM, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_1, MAZDA_CAM, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_2, MAZDA_CAM, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_3, MAZDA_CAM, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_4, MAZDA_CAM, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_5, MAZDA_CAM, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_6, MAZDA_CAM, 8, .check_relay = false},
  };

  static RxCheck mazda_rx_checks[] = {
    {.msg = {{MAZDA_CRZ_CTRL,     0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_CRZ_BTNS,     0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_STEER_TORQUE, 0, 8, 83U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_ENGINE_DATA,  0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_PEDALS,       0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };

  // Stock CRZ_CTRL disappears once the radar enters its diagnostic session.
  static RxCheck mazda_vision_only_rx_checks[] = {
    {.msg = {{MAZDA_CRZ_BTNS,     0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_STEER_TORQUE, 0, 8, 83U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_ENGINE_DATA,  0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_PEDALS,       0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };

  mazda_bm_low_speed_steer = GET_FLAG(param, MAZDA_PARAM_BM_LOW_SPEED_STEER);
  mazda_vision_only_radar = GET_FLAG(param, MAZDA_PARAM_VISION_ONLY_RADAR);
  mazda_desired_accel_last[0] = 0;
  mazda_desired_accel_last[1] = 0;
  acc_main_on = false;
  if (mazda_vision_only_radar) {
    return BUILD_SAFETY_CFG(mazda_vision_only_rx_checks, MAZDA_VISION_ONLY_RADAR_TX_MSGS);
  } else {
    return BUILD_SAFETY_CFG(mazda_rx_checks, MAZDA_TX_MSGS);
  }
}

const safety_hooks mazda_hooks = {
  .init = mazda_init,
  .rx = mazda_rx_hook,
  .tx = mazda_tx_hook,
};
