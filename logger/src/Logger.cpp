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
#include "../../hardware_interface/src/string_colors.h"
#include "Logger.h"

const int MAX_PWM = 190;

Logger::Logger(const std::string& name)
: TaskContext(name),
  port_desired_position_("DesiredPositionIn"),
  port_motor_position_("MotorPositionIn"),
  port_motor_increment_("MotorIncrementIn"),
  port_motor_current_("MotorCurrentIn"),
  hardware_panic_in_("HardwarePanicIn"),
  desiredData(0.0),
  positionData(0.0),
  incrementData(0.0),
  currentData(0.0),
  log(0),
  writed(false) {
  this->addEventPort(port_desired_position_).doc("");
  this->addPort(port_motor_position_).doc("");
  this->addPort(port_motor_increment_).doc("");
  this->addPort(port_motor_current_).doc("");

  this->addPort(hardware_panic_in_).doc("Hardware Panic from HardwareInterface");

  this->addProperty("reg_number", reg_number_).doc("");
  this->addProperty("debug", debug_).doc("");
  this->addProperty("filename", filename_).doc("");
  this->addProperty("max_log", max_log_).doc("");
}

Logger::~Logger() {
  write();
}

bool Logger::configureHook() {
  reset();

  buffer.resize(max_log_);
  hardware_panic_old_ = false;

  return true;
}

void Logger::updateHook() {
  if (RTT::NewData == hardware_panic_in_.read(hardware_panic_new_)) {
    if (hardware_panic_new_ != hardware_panic_old_) {
      hardware_panic_old_ = hardware_panic_new_;
    }
  }

  if (RTT::NewData == port_desired_position_.read(desiredData)) {
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
  output << ms << ";" << desiredData << ";" << positionData << ";"
      << incrementData << ";" << currentData << ";\n";

  if (debug_) {
    std::cout << log;
    std::cout << "  time: " << ms;
    std::cout << "  desired: " << desiredData;
    std::cout << "  position: " << positionData;
    std::cout << "  increment: " << incrementData;
    std::cout << "  current: " << currentData;
    std::cout << std::endl;
  }

  buffer[log] = output.str();
  ++log;
  if (log >= max_log_) log = 0;

  std::cout << "hardware_panic " << hardware_panic_old_  << std::endl;

  if (hardware_panic_old_) {
    write();
  }
}

void Logger::write() {
  if (!writed) {
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
    filename = filename.erase(ros) + filename_ + std::string(buf);

    file.open((filename + ".csv").c_str());

    file << reg_number_ << "_time;";
    file << reg_number_ << "_desired;";
    file << reg_number_ << "_position;";
    file << reg_number_ << "_increment;";
    file << reg_number_ << "_current;";

    file << std::endl;

    for (int i = log; i < max_log_; ++i) {
      file << buffer[i];
    }
    for (int i = 0; i < log; ++i) {
      file << buffer[i];
    }
    file.close();

    writed = true;
  }
}

void Logger::reset() {
}

ORO_CREATE_COMPONENT(Logger)
