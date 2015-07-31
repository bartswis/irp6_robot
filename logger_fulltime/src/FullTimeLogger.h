/*
 * Copyright (c) 2014-2015, Robot Control and Pattern Recognition Group, Warsaw University of Technology.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Robot Control and Pattern Recognition Group,
 *       Warsaw University of Technology nor the names of its contributors may
 *       be used to endorse or promote products derived from this software
 *       without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef MOTORSTATEEXPORT_H_
#define MOTORSTATEEXPORT_H_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <std_msgs/Float64.h>
#include <std_msgs/Int64.h>
#include <rtt/Component.hpp>
#include <std_srvs/Empty.h>
#include <ros/ros.h>

#include <fstream> // NOLINT
#include <string>
#include <ctime>

class FullTimeLogger : public RTT::TaskContext {
 public:
  explicit FullTimeLogger(const std::string& name);
  ~FullTimeLogger();

  void reset();

 private:
  bool configureHook();
  void updateHook();
  void write();

  RTT::InputPort<double> port_desired_position_;
  RTT::InputPort<double> port_motor_position_;
  RTT::InputPort<double> port_motor_increment_;
  RTT::InputPort<double> port_motor_current_;

  RTT::InputPort<bool> synchro_state_in_;

  double desiredData;
  double positionData;
  double incrementData;
  double currentData;

  bool synchro_state_old_, synchro_state_new_;

  int64_t update_hook_iteration_number_;
  int64_t new_position_iteration_number_;

  // Properties
  int reg_number_;
  bool pre_syn_export_;
  std::string filename_;
  bool debug_;

  std::ofstream file;
};

#endif  // MOTORSTATEEXPORT_H_
