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

#include "Logger.h"

#include <string>
#include "../../hardware_interface/src/string_colors.h"

const int MAX_PWM = 190;

Logger::Logger(const std::string& name)
: TaskContext(name),
  port_desired_position_("DesiredPositionIn"),
  port_desired_velocity_("DesiredVelocityIn"),
  port_motor_position_("MotorPositionIn"),
  port_motor_increment_("MotorIncrementIn"),
  port_motor_current_("MotorCurrentIn"),
  hardware_panic_in_("HardwarePanicIn"),
  synchro_state_in_("SynchroStateIn"),
  desiredPosData(0.0),
  desiredVelData(0.0),
  positionData(0.0),
  incrementData(0.0),
  currentData(0.0),
  log(0),
  writed(false)  {
  this->addEventPort(port_desired_position_).doc("");
  this->addPort(port_desired_velocity_).doc("");
  this->addPort(port_motor_position_).doc("");
  this->addPort(port_motor_increment_).doc("");
  this->addPort(port_motor_current_).doc("");

  this->addPort(hardware_panic_in_).doc("Hardware Panic from HardwareInterface");
  this->addPort(synchro_state_in_).doc("Synchro State from HardwareInterface");

  this->addProperty("reg_number", reg_number_).doc("");
  this->addProperty("debug", debug_).doc("");
  this->addProperty("pre_syn_export", pre_syn_export_).doc("");
  this->addProperty("filename", filename_).doc("");
  this->addProperty("max_log", max_log_).doc("");
  this->addProperty("full_log", full_log_).doc("");
}

Logger::~Logger() {
  if (full_log_) {
    file.close();
  }
}

bool Logger::configureHook() {
  reset();

  if (full_log_) {
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
    filename = filename.erase(ros) + "log_" + filename_ + std::string(buf);

    file.open((filename + ".csv").c_str());

    file << reg_number_ << "_time;";
    file << reg_number_ << "_pos_desired;";
    file << reg_number_ << "_vel_desired;";
    file << reg_number_ << "_position;";
    file << reg_number_ << "_increment;";
    file << reg_number_ << "_current;";

    file << std::endl;
  }

  buffer.resize(max_log_);
  hardware_panic = false;

  return true;
}

void Logger::updateHook() {
  if (RTT::NewData == synchro_state_in_.read(synchro_state_new_)) {
    if (synchro_state_new_ != synchro_state_old_) {
      synchro_state_old_ = synchro_state_new_;
    }
  }

  if (RTT::NewData == hardware_panic_in_.read(hardware_panic)) {
    // std::cout << "hardware_panic " << hardware_panic  << std::endl;
  }

  if (RTT::NewData == port_desired_position_.read(desiredPosData)) {
    // std::cout << "  desired: " << desiredData;
  }
  if (RTT::NewData == port_desired_velocity_.read(desiredVelData)) {
    // std::cout << "  desired: " << desiredData;
  }
  if (RTT::NewData == port_motor_position_.read(positionData)) {
    // std::cout << "  position: " << positionData;
  }
  if (RTT::NewData == port_motor_increment_.read(incrementData)) {
    // std::cout << "  increment: " << incrementData;
  }
  if (RTT::NewData == port_motor_current_.read(currentData)) {
    // std::cout << "  current: " << currentData;
  }
  // std::cout << std::endl;


  struct timeval tp;
  gettimeofday(&tp, NULL);
  int64_t ms = tp.tv_sec * 1000 + tp.tv_usec / 1000;

  std::ostringstream output;
  output << ms << ";" << desiredPosData << ";" << desiredVelData << ";" << positionData << ";"
      << incrementData << ";" << currentData << ";\n";

  if (debug_) {
    std::cout << log;
    std::cout << "  time: " << ms;
    std::cout << "  desired_pos: " << desiredPosData;
    std::cout << "  desired_vel: " << desiredVelData;
    std::cout << "  position: " << positionData;
    std::cout << "  increment: " << incrementData;
    std::cout << "  current: " << currentData;
    std::cout << std::endl;
  }

  buffer[log] = output.str();
  ++log;
  if (log >= max_log_) log = 0;

  if (hardware_panic) {
    panic_write();
  }

  if (synchro_state_old_ || !pre_syn_export_) {
    if (full_log_) write();
  }
}

void Logger::write() {
  struct timeval tp;
  gettimeofday(&tp, NULL);
  int64_t ms = tp.tv_sec * 1000 + tp.tv_usec / 1000;

  std::ostringstream output;
  output << ms << ";" << desiredPosData << ";" << desiredVelData << ";" << positionData << ";"
      << incrementData << ";" << currentData << ";\n";
  file << output.str();
}

void Logger::panic_write() {
  if (!writed) {
    std::ofstream panic_file;
    char cCurrentPath[FILENAME_MAX];

    if (!getcwd(cCurrentPath, sizeof(cCurrentPath))) {
      return;
    }

    cCurrentPath[sizeof(cCurrentPath) - 1] = '\0';

    time_t now = time(0);
    struct tm newtime;
    char buf[80];
    localtime_r(&now, &newtime);
    strftime(buf, sizeof(buf), "_%Y-%m-%d_%X", &newtime);

    std::string filename = std::string(cCurrentPath);
    int ros = filename.find(".ros");
    filename = filename.erase(ros) + "panic_log_" + filename_ + std::string(buf);

    panic_file.open((filename + ".csv").c_str());

    panic_file << reg_number_ << "_time;";
    panic_file << reg_number_ << "_desired;";
    panic_file << reg_number_ << "_position;";
    panic_file << reg_number_ << "_increment;";
    panic_file << reg_number_ << "_current;";

    panic_file << std::endl;

    for (int i = log; i < max_log_; ++i) {
      panic_file << buffer[i];
    }
    for (int i = 0; i < log; ++i) {
      panic_file << buffer[i];
    }
    panic_file.close();

    writed = true;
  }
}

void Logger::reset() {
}

ORO_CREATE_COMPONENT(Logger)
