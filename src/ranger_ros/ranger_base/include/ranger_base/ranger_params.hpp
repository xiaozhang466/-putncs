/**
 * @file ranger_params.hpp
 * @date 2021-04-20
 * @brief 
 * 
 # @copyright Copyright (c) 2021 AgileX Robotics
 * @copyright Copyright (c) 2023 Weston Robot Pte. Ltd.
 */

#ifndef RANGER_PARAMS_HPP
#define RANGER_PARAMS_HPP

namespace westonrobot {
struct RangerParams {
  static constexpr double track = 0.56
      /*0.56*/;  // in meter (left & right wheel distance) //ranger-mini 0.364
  static constexpr double wheelbase = 0.90
      /*0.89*/;  // in meter (front & rear wheel distance) //ranger-mini 0.494

  static constexpr double max_linear_speed = 2.7;            // in m/s
  static constexpr double max_angular_speed = 0.7853;        // in rad/s
  static constexpr double max_speed_cmd = 10.0;              // in rad/s
  static constexpr double max_steer_angle_ackermann = 0.6981; // 40 degree
  static constexpr double max_steer_angle_parallel = 1.570;   // 180 degree
  static constexpr double max_round_angle = 1.0057;
  static constexpr double min_turn_radius = 0.7837;
};

struct RangerMiniV1Params {
  static constexpr double track =
      0.36;  // in meter (left & right wheel distance) //ranger-mini 0.364
  static constexpr double wheelbase =
      0.36;  // in meter (front & rear wheel distance) //ranger-mini 0.364

  static constexpr double max_linear_speed = 1.5;   // in m/s
  static constexpr double max_angular_speed = 0.3;  // in rad/s
  static constexpr double max_speed_cmd = 5;        // in rad/s

  static constexpr double max_steer_angle_ackermann = 0.6981;   // 40 degree
  static constexpr double max_steer_angle_parallel = 0.6981;    // 40 degree
  static constexpr double max_round_angle = 0.935671;
  static constexpr double min_turn_radius = 0.39457;
};

struct RangerMiniV2Params {
  static constexpr double track =
      0.364;  // in meter (left & right wheel distance) //ranger-mini 0.364
  static constexpr double wheelbase =
      0.494;  // in meter (front & rear wheel distance) //ranger-mini 0.494

  static constexpr double max_linear_speed = 1.5;      // in m/s
  static constexpr double max_angular_speed = 4.8;  // in rad/s
  static constexpr double max_speed_cmd = 10.0;        // in rad/s

  static constexpr double max_steer_angle_ackermann = 0.6981;   // 40 degree
  static constexpr double max_steer_angle_parallel = 1.570;     // 180 degree
  static constexpr double max_round_angle = 0.935671;
  static constexpr double min_turn_radius = 0.47644;
};
}  // namespace westonrobot

#endif  // RANGER_PARAMS_HPP
