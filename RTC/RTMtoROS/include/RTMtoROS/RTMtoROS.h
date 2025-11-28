// -*- C++ -*-
/*!
 * @file  RTMtoROS.h
 * @brief ModuleDescription
 * @date  $Date$
 *
 * $Id$
 */

#ifndef RTMTOROS_H
#define RTMTOROS_H

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <rtm/idl/InterfaceDataTypesSkel.h>

// Service implementation headers
#include "ManipulatorCommonInterface_CommonSVC_impl.h"
#include "ManipulatorCommonInterface_MiddleSVC_impl.h"

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>

// --- ROS Headers ---
#include <ros/ros.h>
#include <open_manipulator_msgs/SetJointPosition.h>
#include <sensor_msgs/JointState.h> // 追加
#include <map>
#include <string>

class RTMtoROS
  : public RTC::DataFlowComponentBase
{
 public:
  RTMtoROS(RTC::Manager* manager);
  ~RTMtoROS();

  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

 protected:
  // CORBA Port
  RTC::CorbaPort m_ManipulatorCommonInterface_CommonPort;
  RTC::CorbaPort m_ManipulatorCommonInterface_MiddlePort;

  // Service Provider
  JARA_ARM_ManipulatorCommonInterface_CommonSVC_impl m_ManipulatorCommonInterface_Common;
  JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl m_ManipulatorCommonInterface_Middle;

 private:
  // --- ROS関連の変数 ---
  ros::NodeHandle* nh;
  ros::ServiceClient joint_client;
  ros::Subscriber joint_state_sub; // 追加: 現在地取得用

  std::vector<double> last_positions;
  bool stop_mode;

  // 現在の関節角度を保存するマップ {関節名: 角度}
  std::map<std::string, double> current_joint_map;

  // 停止処理用関数
  void stopRobot();
  
  // JointStateコールバック関数
  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
};

extern "C"
{
  DLL_EXPORT void RTMtoROSInit(RTC::Manager* manager);
};

#endif // RTMTOROS_H
