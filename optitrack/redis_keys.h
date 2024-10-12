/**
 * @file redis_keys.h
 * @author William Chong (williamchong@stanford.edu)
 * @brief 
 * @version 0.1
 * @date 2022-04-30
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#pragma once 

// one CONTROLLER keys
const std::string TORO_JOINT_ANGLES_KEY = "sai2::sim::toro::sensors::q";
const std::string TORO_JOINT_VELOCITIES_KEY = "sai2::sim::toro::sensors::dq";
const std::string TORO_JOINT_TORQUES_COMMANDED_KEY = "sai2::sim::toro::actuators::fgc";

// Hannah's keys
const std::string HANNAH_TORO_JOINT_ANGLES_KEY = "sai2::sim::hannah::toro::sensors::q";
const std::string HANNAH_TORO_JOINT_VELOCITIES_KEY = "sai2::sim::hannah::toro::sensors::dq";
const std::string HANNAH_TORO_JOINT_TORQUES_COMMANDED_KEY = "sai2::sim::hannah::toro::actuators::fgc";

// Tracy's keys
const std::string TRACY_TORO_JOINT_ANGLES_KEY = "sai2::sim::tracy::toro::sensors::q";
const std::string TRACY_TORO_JOINT_VELOCITIES_KEY = "sai2::sim::tracy::toro::sensors::dq";
const std::string TRACY_TORO_JOINT_TORQUES_COMMANDED_KEY = "sai2::sim::tracy::toro::actuators::fgc";

const std::vector<std::string> MULTI_TORO_JOINT_ANGLES_KEY = {HANNAH_TORO_JOINT_ANGLES_KEY, TRACY_TORO_JOINT_ANGLES_KEY, TORO_JOINT_ANGLES_KEY};
const std::vector<std::string> MULTI_TORO_JOINT_VELOCITIES_KEY = {HANNAH_TORO_JOINT_VELOCITIES_KEY, TRACY_TORO_JOINT_VELOCITIES_KEY, TORO_JOINT_VELOCITIES_KEY};
const std::vector<std::string> MULTI_TORO_JOINT_TORQUES_COMMANDED_KEY = {HANNAH_TORO_JOINT_TORQUES_COMMANDED_KEY, TRACY_TORO_JOINT_TORQUES_COMMANDED_KEY, TORO_JOINT_TORQUES_COMMANDED_KEY};

//const std::string TORO_CONTROLLER_RUNNING_KEY = "sai2::sim::toro::controller";

const std::string POSITION_FROM_CAMERA = "sai2::animation::camera::position";
const std::string RESET_BUTTON = "sai2::animation::reset";
const std::string READY_BUTTON = "sai2::animation::user_ready";
const std::string OPTITRACK_POS_KEY = "sai2::optitrack::pos_rigid_bodies";
const std::string OPTITRACK_ORI_KEY = "sai2::optitrack::ori_rigid_bodies";
const std::string USER_READY_KEY = "sai2::optitrack::user_ready";

const std::vector<std::string> MULTI_USER_READY_KEY = {"sai2::optitrack::user_1_ready", "sai2::optitrack::user_2_ready", "sai2::optitrack::user_ready"};

const std::string HEAD_POS = "sai2::sim::toro::head_pos";
const std::string HEAD_VERT_AXIS = "sai2::sim::toro::head_vert_axis";
const std::string HEAD_LOOK_AT = "sai2::sim::toro::head_look_at";

const std::string GLOBAL_CAM = "sai2::sim::toro::global_cam";
const std::string CLAP_KEY = "sai2::sim::toro::clap_count";
const std::string HANDS_ABOVE_HEAD_KEY = "sai2::sim::toro::hands_above_head";
const double CLAP_THRESHOLD = 0.43; // Threshold for clapping detection
const double HANDS_ABOVE_HEAD_THRESHOLD = 0.4; 
const std::string LAGRANGIAN = "sai2::sim::toro::lagrangian";
const std::string KINETIC = "sai2::sim::toro::kinetic";
const std::string POTENTIAL = "sai2::sim::toro::potential";
const std::string GROUND = "sai2::optitrack::ground";

const std::string HANNAH_LAGRANGIAN = "sai2::sim::toro::lagrangian";
const std::string HANNAH_KINETIC = "sai2::sim::toro::kinetic";
const std::string HANNAH_POTENTIAL = "sai2::sim::toro::potential";

const std::string TRACY_LAGRANGIAN = "sai2::sim::toro::lagrangian";
const std::string TRACY_KINETIC = "sai2::sim::toro::kinetic";
const std::string TRACY_POTENTIAL = "sai2::sim::toro::potential";

const std::string TEST_POS = "test::pos";
const std::string TEST_ORI = "test::ori";

const std::string Z_VIEWING_ANGLE = "sai2::sim::viewing_angle";
const std::string RESET_ROBOT_KEY = "sai2::sim::reset";

const std::vector<std::string> MULTI_RESET_ROBOT_KEY = {"sai2::sim::reset_1", "sai2::sim::reset_2"};