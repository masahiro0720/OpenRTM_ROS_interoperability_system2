// -*- C++ -*-
/*!
 * @file  RTMtoROS.cpp
 * @brief ModuleDescription
 * @date $Date$
 *
 * $Id$
 */

#include "RTMtoROS.h"

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
  // CORBAポート登録
  m_ManipulatorCommonInterface_CommonPort.registerProvider("ManipulatorCommonInterface_Common", "JARA_ARM::ManipulatorCommonInterface_Common", m_ManipulatorCommonInterface_Common);
  m_ManipulatorCommonInterface_MiddlePort.registerProvider("ManipulatorCommonInterface_Middle", "JARA_ARM::ManipulatorCommonInterface_Middle", m_ManipulatorCommonInterface_Middle);

  addPort(m_ManipulatorCommonInterface_CommonPort);
  addPort(m_ManipulatorCommonInterface_MiddlePort);

  // --- ROS初期化 ---
  int argc = 0;
  char** argv = nullptr;
  if (!ros::isInitialized()) {
      ros::init(argc, argv, "rtm_to_ros_bridge", ros::init_options::NoSigintHandler);
  }
  
  nh = new ros::NodeHandle();
  
  // サービス待機
  ROS_INFO("Waiting for /mikata_arm/goal_joint_space_path service...");
  joint_client = nh->serviceClient<open_manipulator_msgs::SetJointPosition>("/mikata_arm/goal_joint_space_path");

  // 現在位置取得のためのSubscriber (追加)
  joint_state_sub = nh->subscribe("/joint_states", 10, &RTMtoROS::jointStateCallback, this);

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
  return RTC::RTC_OK;
}

RTC::ReturnCode_t RTMtoROS::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}

RTC::ReturnCode_t RTMtoROS::onExecute(RTC::UniqueId ec_id)
{
  // ROSのコールバックを処理するために必要
  ros::spinOnce();

  // データを取得
  std::vector<double> current_cmd;
  for(int i=0; i<6; i++) {
      current_cmd.push_back(m_ManipulatorCommonInterface_Middle.targetJointPos[i]);
  }

  // ==========================================
  // 停止信号 (999) チェック
  // ==========================================
  if (current_cmd[0] > 900) {
      // まだ停止モードに入っていない場合のみ、停止処理を行う（1回だけ実行）
      if (!stop_mode) {
          ROS_WARN("EMERGENCY STOP (999) RECEIVED! Stopping Robot.");
          stopRobot(); 
          stop_mode = true;
          last_positions.clear(); // 再開時に確実に送れるよう履歴を消す
      }
      // 既に停止モードなら何もしない（これで振動を防ぐ）
      return RTC::RTC_OK;
  }
  
  // 999以外のデータが来たら停止モード解除
  if (stop_mode) {
      ROS_INFO("Resume Signal Received.");
      stop_mode = false;
  }

  // ==========================================
  // 通常移動処理
  // ==========================================
  
  // 重複送信防止
  if (last_positions == current_cmd) {
      return RTC::RTC_OK;
  }
  last_positions = current_cmd;

  // サービス呼び出し
  open_manipulator_msgs::SetJointPosition srv;
  srv.request.planning_group = "arm";
  srv.request.joint_position.joint_name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
  srv.request.joint_position.position = current_cmd; 
  srv.request.path_time = 2.0;

  if (joint_client.call(srv)) {
      ROS_INFO("Sent Goal: %f, %f ...", current_cmd[0], current_cmd[1]);
  } 

  return RTC::RTC_OK;
}

// 急停止処理
void RTMtoROS::stopRobot() {
    // 現在位置を取得できていなければ何もしない
    if (current_joint_map.empty()) {
        ROS_WARN("Cannot stop: No joint states received yet.");
        return;
    }

    std::vector<std::string> target_joints = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
    std::vector<double> stop_positions;

    // 現在の角度リストを作成
    for(const auto& name : target_joints) {
        if (current_joint_map.count(name) > 0) {
            stop_positions.push_back(current_joint_map[name]);
        } else {
            stop_positions.push_back(0.0);
        }
    }

    // 自分自身に「現在地に0.1秒で移動せよ」という指令を送る
    open_manipulator_msgs::SetJointPosition srv;
    srv.request.planning_group = "arm";
    srv.request.joint_position.joint_name = target_joints;
    srv.request.joint_position.position = stop_positions;
    srv.request.path_time = 0.1; // 即座に止めるための短い時間

    if (joint_client.call(srv)) {
        ROS_INFO("STOP command sent (Holding current position).");
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
