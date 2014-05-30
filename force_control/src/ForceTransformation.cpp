#include <rtt/Component.hpp>

#include "ForceTransformation.h"
#include "eigen_conversions/eigen_msg.h"

ForceTransformation::ForceTransformation(const std::string& name)
    : RTT::TaskContext(name, PreOperational) {

  this->ports()->addPort("CurrentWristPose", port_current_wrist_pose_);

  this->ports()->addPort("CurrentSensorWrench", port_current_sensor_wrench_);
  this->ports()->addPort("OutputWristWrench", port_output_wrist_wrench_);
  this->ports()->addPort("OutputEndEffectorWrench",
                         port_output_end_effector_wrench_);
  this->ports()->addPort("Tool", port_tool_);

}

ForceTransformation::~ForceTransformation() {

}

bool ForceTransformation::configureHook() {

  // docelowo odczyt z konfiguracji
  sensor_frame_ = KDL::Frame(KDL::Rotation::RotZ(M_PI),
                             KDL::Vector(0.0, 0.0, 0.09));

  // ustalenie skretnosci wektora z odczytami z czujnika
  is_right_turn_frame_ = true;

  return true;
}

bool ForceTransformation::startHook() {

  // read current force ad set as an offset force
  geometry_msgs::Wrench current_wrench;
  if (port_current_sensor_wrench_.read(current_wrench) == RTT::NoData) {
    return false;
  }

  tf::wrenchMsgToKDL(current_wrench, force_offset_);

  // read current wrist pose
  geometry_msgs::Pose current_wrist_pose;
  if (port_current_wrist_pose_.read(current_wrist_pose) == RTT::NoData) {
    return false;
  }

  // set tool and compute reaction force
  tool_weight_ = 10.8;

  gravity_arm_in_wrist_ = KDL::Vector(0.004, 0.0, 0.156);

  gravity_force_torque_in_base_ = KDL::Wrench(
      KDL::Vector(0.0, 0.0, -tool_weight_), KDL::Vector(0.0, 0.0, 0.0));

  KDL::Frame current_frame;

  tf::poseMsgToKDL(current_wrist_pose, current_frame);

// sila reakcji w ukladzie czujnika z orientacja bazy
  KDL::Wrench gravity_force_torque_in_sensor = current_frame.M.Inverse()
      * gravity_force_torque_in_base_;

// macierz narzedzia wzgledem nadgarstka
  tool_mass_center_translation_ = KDL::Frame(KDL::Rotation(),
                                             gravity_arm_in_wrist_);

// sila reakcji w ukladzie nadgarstka z orientacja bazy
  reaction_force_torque_in_wrist_ = -(tool_mass_center_translation_
      * gravity_force_torque_in_sensor);

  return true;
}

void ForceTransformation::updateHook() {
  geometry_msgs::Pose current_wrist_pose;
  port_current_wrist_pose_.read(current_wrist_pose);
  KDL::Frame current_wrist_pose_kdl;
  tf::poseMsgToKDL(current_wrist_pose, current_wrist_pose_kdl);

  // odczyt sily
  geometry_msgs::Wrench current_wrench;
  port_current_sensor_wrench_.read(current_wrench);
  KDL::Wrench input_force;
  tf::wrenchMsgToKDL(current_wrench, input_force);

  // offset level removal
  KDL::Wrench biased_force = input_force - force_offset_;

  if (!is_right_turn_frame_) {

    biased_force[2] = -biased_force[2];
    biased_force[5] = -biased_force[5];
  }

  // sprowadzenie wejsciowych, zmierzonych sil i momentow sil z ukladu czujnika do ukladu nadgarstka
  biased_force = sensor_frame_ * biased_force;

  // sprowadzenie odczytow sil do ukladu czujnika przy zalozeniu ze uklad chwytaka ma te sama orientacje
  // co uklad narzedzia
  KDL::Wrench gravity_force_torque_in_sensor =
      (current_wrist_pose_kdl.Inverse()).M * gravity_force_torque_in_base_;

  // finalne przeksztalcenie (3.30 z doktoratu TW)
  KDL::Wrench computed_force = biased_force
      - tool_mass_center_translation_ * gravity_force_torque_in_sensor
      - reaction_force_torque_in_wrist_;

  // sprowadzenie sily w ukladzie nadgarstka do orientacji ukladu bazowego
  computed_force = current_wrist_pose_kdl.M * (-computed_force);

  geometry_msgs::Wrench output_wrist_wrench;
  tf::wrenchKDLToMsg(computed_force, output_wrist_wrench);

  port_output_wrist_wrench_.write(output_wrist_wrench);

  //tool determination
  geometry_msgs::Pose tool_msgs;
  port_tool_.read(tool_msgs);
  KDL::Frame tool_kdl;
  tf::poseMsgToKDL(tool_msgs, tool_kdl);

  KDL::Wrench computed_ef_force = tool_kdl.Inverse()
      * (current_wrist_pose_kdl.M.Inverse() * computed_force);

  geometry_msgs::Wrench output_end_effector_wrench;
  tf::wrenchKDLToMsg(computed_ef_force, output_end_effector_wrench);

  port_output_end_effector_wrench_.write(output_end_effector_wrench);

}

ORO_CREATE_COMPONENT(ForceTransformation)

