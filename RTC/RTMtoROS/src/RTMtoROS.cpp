// -*- C++ -*-
/*!
 * @file  RTMtoROS.cpp
 * @brief ModuleDescription
 * @date $Date$
 *
 * $Id$
 */

#include "RTMtoROS.h"
#include <numeric> // for accumulate
#include <cmath>   // for abs
#include <thread>  // for sleep

// Module specification
static const char* rtmtoros_spec[] =
  {
    "implementation_id", "RTMtoROS",
    "type_name",         "RTMtoROS",
    "description",       "ModuleDescription",
    "version",           "1.0.0",
    "vendor",            "rsdlab",
    "category",          "Converter",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    ""
  };

RTMtoROS::RTMtoROS(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    m_ManipulatorCommonInterface_CommonPort("ManipulatorCommonInterface_Common"),
    m_ManipulatorCommonInterface_MiddlePort("ManipulatorCommonInterface_Middle")
{
    nh = nullptr;
    stop_mode = false;
}

RTMtoROS::~RTMtoROS()
{
    if(nh) delete nh;
}

RTC::ReturnCode_t RTMtoROS::onInitialize()
{
  m_ManipulatorCommonInterface_CommonPort.registerProvider("ManipulatorCommonInterface_Common", "JARA_ARM::ManipulatorCommonInterface_Common", m_ManipulatorCommonInterface_Common);
  m_ManipulatorCommonInterface_MiddlePort.registerProvider("ManipulatorCommonInterface_Middle", "JARA_ARM::ManipulatorCommonInterface_Middle", m_ManipulatorCommonInterface_Middle);

  addPort(m_ManipulatorCommonInterface_CommonPort);
  addPort(m_ManipulatorCommonInterface_MiddlePort);

  nh = nullptr;
  return RTC::RTC_OK;
}

// 現在の関節角度を常に更新するコールバック
void RTMtoROS::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    for(size_t i = 0; i < msg->name.size(); ++i) {
        current_joint_map[msg->name[i]] = msg->position[i];
    }
}

RTC::ReturnCode_t RTMtoROS::onActivated(RTC::UniqueId ec_id)
{
  int argc = 0;
  char** argv = nullptr;
  if (!ros::isInitialized()) {
      ros::init(argc, argv, "rtm_to_ros_bridge", ros::init_options::NoSigintHandler);
  }

  if (!ros::master::check()) {
      std::cout << "\033[31m[ERROR] roscore not found! Please start roscore before activating.\033[0m" << std::endl;
      return RTC::RTC_ERROR; 
  }

  if (!nh) {
      nh = new ros::NodeHandle();
      ROS_INFO("Connecting to /mikata_arm/goal_joint_space_path service...");
      joint_client = nh->serviceClient<open_manipulator_msgs::SetJointPosition>("/mikata_arm/goal_joint_space_path");
      joint_state_sub = nh->subscribe("/joint_states", 10, &RTMtoROS::jointStateCallback, this);
      ROS_INFO("ROS Communication Initialized.");
  }

  stop_mode = false;
  last_positions.clear();
  return RTC::RTC_OK;
}

RTC::ReturnCode_t RTMtoROS::onDeactivated(RTC::UniqueId ec_id)
{
  if (nh) {
      ROS_INFO("Disconnecting ROS...");
      nh->shutdown();
      delete nh;
      nh = nullptr;
  }
  return RTC::RTC_OK;
}

RTC::ReturnCode_t RTMtoROS::onExecute(RTC::UniqueId ec_id)
{
  if (!nh || !ros::ok()) return RTC::RTC_OK;

  ros::spinOnce();

  std::vector<double> current_cmd;
  for(int i=0; i<6; i++) {
      current_cmd.push_back(m_ManipulatorCommonInterface_Middle.targetJointPos[i]);
  }

  // ==========================================
  // 停止信号 (999) チェック
  // ==========================================
  if (current_cmd[0] > 900) {
      if (!stop_mode) {
          ROS_WARN("EMERGENCY STOP (999) RECEIVED! Analyzing stable stop position...");
          stopRobot(); 
          stop_mode = true;
          last_positions.clear();
      }
      return RTC::RTC_OK;
  }
  
  if (stop_mode) {
      ROS_INFO("Resume Signal Received.");
      stop_mode = false;
  }

  // ==========================================
  // 通常移動処理
  // ==========================================
  if (last_positions == current_cmd) {
      return RTC::RTC_OK;
  }
  last_positions = current_cmd;

  open_manipulator_msgs::SetJointPosition srv;
  srv.request.planning_group = "arm";
  srv.request.joint_position.joint_name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
  srv.request.joint_position.position = current_cmd; 
  srv.request.path_time = 2.0;

  if (joint_client.exists()) {
      if (joint_client.call(srv)) {
          // ROS_INFO("Sent Goal: %f, %f ...", current_cmd[0], current_cmd[1]);
      } else {
          ROS_WARN("Failed to call service (call returned false).");
      }
  }

  return RTC::RTC_OK;
}

// 【改良版】安全停止処理
// 0.3秒間データを集め、異常値を排除し、最新に近い平均値で止める
void RTMtoROS::stopRobot() {
    if (!nh) return;

    std::vector<std::string> target_joints = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
    std::map<std::string, std::vector<double>> position_buffer;

    ROS_INFO("Sampling joint states for 0.3 seconds...");

    // 【修正点】0.3秒間データを収集 (10ms * 30回)
    for(int i = 0; i < 30; i++) {
        ros::spinOnce();
        if (!current_joint_map.empty()) {
            for(const auto& name : target_joints) {
                if (current_joint_map.count(name) > 0) {
                    position_buffer[name].push_back(current_joint_map[name]);
                }
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (position_buffer.empty() || position_buffer["joint1"].empty()) {
        ROS_WARN("Cannot stop safely: No joint states received.");
        return;
    }

    std::vector<double> stop_positions;

    // 各関節ごとに値を決定
    for(const auto& name : target_joints) {
        std::vector<double>& vals = position_buffer[name];
        
        // 1. 外れ値の除去（簡易的：平均から大きく外れたものを除外）
        double sum = std::accumulate(vals.begin(), vals.end(), 0.0);
        double mean = sum / vals.size();
        
        std::vector<double> filtered_vals;
        for(double v : vals) {
            // 平均から0.2rad (約11度) 以上離れていなければ採用
            if (std::abs(v - mean) < 0.2) {
                filtered_vals.push_back(v);
            }
        }

        if (filtered_vals.empty()) {
            // 全部外れ値なら元の平均を使う（緊急時）
            filtered_vals = vals;
        }

        // 2. 「最後に取得したデータに寄せる」ための加重平均
        double weighted_sum = 0.0;
        double weight_total = 0.0;
        int n = filtered_vals.size();
        
        for(int i = 0; i < n; i++) {
            double w = (double)(i + 1); // 重み 1, 2, 3 ... N
            weighted_sum += filtered_vals[i] * w;
            weight_total += w;
        }

        double final_pos = weighted_sum / weight_total;
        stop_positions.push_back(final_pos);
    }

    // 停止指令送信
    open_manipulator_msgs::SetJointPosition srv;
    srv.request.planning_group = "arm";
    srv.request.joint_position.joint_name = target_joints;
    srv.request.joint_position.position = stop_positions;
    srv.request.path_time = 0.5; // なめらかに止めるために少し時間をかける

    if (joint_client.exists() && joint_client.call(srv)) {
        ROS_INFO("SAFE STOP command sent. Position calculated from weighted average (0.3s buffer).");
    } else {
        ROS_ERROR("Failed to send STOP command.");
    }
}

extern "C"
{
  void RTMtoROSInit(RTC::Manager* manager)
  {
    coil::Properties profile(rtmtoros_spec);
    manager->registerFactory(profile,
                             RTC::Create<RTMtoROS>,
                             RTC::Delete<RTMtoROS>);
  }
};
