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

#include <string>
#include "MotorStateExport.h"
#include "../../hardware_interface/src/string_colors.h"

const int MAX_PWM = 190;

MotorStateExport::MotorStateExport(const std::string& name)
    : TaskContext(name),
      port_motor_position_("MotorPositionIn"),
      port_motor_increment_("MotorIncrementIn"),
      port_motor_current_("MotorCurrentIn"),
      synchro_state_in_("SynchroStateIn"),
      positionData(0.0),
      incrementData(0.0),
      currentData(0.0) {

  this->addEventPort(port_motor_position_).doc("");
  this->addPort(port_motor_increment_).doc("");
  this->addPort(port_motor_current_).doc("");

  this->addPort(synchro_state_in_).doc("Synchro State from HardwareInterface");

  this->addProperty("reg_number", reg_number_).doc("");
  this->addProperty("debug", debug_).doc("");
  this->addProperty("pre_syn_export", pre_syn_export_).doc("");
  this->addProperty("filename", filename_).doc("");
}

MotorStateExport::~MotorStateExport() {
  file.close();
}

bool MotorStateExport::configureHook() {
  reset();

  char cCurrentPath[FILENAME_MAX];

  if (!getcwd(cCurrentPath, sizeof(cCurrentPath))) {
    return errno;
  }

  cCurrentPath[sizeof(cCurrentPath) - 1] = '\0';

  time_t now = time(0);
  struct tm newtime;
  char buf[80];
  localtime_r(&now, &newtime);
  strftime(buf, sizeof(buf), "_%Y-%m-%d_%X", &newtime);

  std::string filename = std::string(cCurrentPath);
  int ros = filename.find(".ros");
  filename = filename.erase(ros) + filename_ + std::string(buf);

  file.open((filename + ".csv").c_str());

  file << reg_number_ << "_position;";
  file << reg_number_ << "_increment;";
  file << reg_number_ << "_current;";

  file << std::endl;

  return true;
}

void MotorStateExport::updateHook() {
  if (RTT::NewData == synchro_state_in_.read(synchro_state_new_)) {
    if (synchro_state_new_ != synchro_state_old_) {
      synchro_state_old_ = synchro_state_new_;
    }
  }

  if (RTT::NewData == port_motor_position_.read(positionData)
      && RTT::NewData == port_motor_increment_.read(incrementData)
      && RTT::NewData == port_motor_current_.read(currentData)) {
    if (!synchro_state_old_ && pre_syn_export_) {
    } else {
      write();
    }
  }
}

void MotorStateExport::write() {
  file << positionData << ";";
  file << incrementData << ";";
  file << currentData << ";";
  file << std::endl;

  if (debug_) {
    std::cout << "  position: " << positionData;
    std::cout << "  increment: " << incrementData;
    std::cout << "  current: " << currentData;
    std::cout << std::endl;
  }
}

void MotorStateExport::reset() {
}

ORO_CREATE_COMPONENT(MotorStateExport)
