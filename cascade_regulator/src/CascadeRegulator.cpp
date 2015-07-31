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

#include "CascadeRegulator.h"
#include "../../hardware_interface/src/string_colors.h"
const int MAX_PWM = 190;

CascadeRegulator::CascadeRegulator(const std::string& name)
    : TaskContext(name),
      desired_position_("DesiredPosition"),
      measured_position_("MeasuredPosition"),
      deltaInc_in("deltaInc_in"),
      computedPwm_out("computedPwm_out"),
      synchro_state_in_("SynchroStateIn"),
      emergency_stop_out_("EmergencyStopOut"),
      current_mode_(true),
      kp_pos(0.0),
      Ti_pos(0.0),
      Td_pos(0.0),
      kp_inc(0.0),
      Ti_inc(0.0),
      Td_inc(0.0),
      output_value(0.0),
      position_err_new(0.0),
      position_err_old(0.0),
      position_err_very_old(0.0),
      position_set_value_new(0.0),
      position_set_value_old(0.0),
      increment_err_new(0.0),
      increment_err_old(0.0),
      increment_err_very_old(0.0),
      increment_set_value_new(0.0),
      increment_set_value_old(0.0),
      update_hook_iteration_number_(0),
      new_position_iteration_number_(0),
      max_desired_increment_(0.0),
      desired_position_old_(0.0),
      desired_position_new_(0.0),
      measured_increment_new_(0.0),
      measured_position_new_(0.0),
      a_(0.0),
      b0_(0.0),
      b1_(0.0),
      delta_eint_old(0.0),
      delta_eint(0.0),
      desired_position_increment_(0.0),
      position_increment_new(0.0),
      position_increment_old(0.0),
      set_value_new(0.0),
      set_value_old(0.0),
      set_value_very_old(0.0),
      step_new(0.0),
      step_old(0.0),
      step_old_pulse(0.0),
      synchro_state_old_(false),
      synchro_state_new_(false) {
  this->addEventPort(deltaInc_in).doc("");
  this->addPort(desired_position_).doc("");
  this->addPort(measured_position_).doc("");
  this->addPort(computedPwm_out).doc("");
  this->addPort(synchro_state_in_).doc("Synchro State from HardwareInterface");
  this->addPort(emergency_stop_out_).doc("Emergency Stop Out");

  this->addProperty("KP_POS", KP_POS_).doc("");
  this->addProperty("TI_POS", TI_POS_).doc("");
  this->addProperty("TD_POS", TD_POS_).doc("");
  this->addProperty("KP_INC", KP_INC_).doc("");
  this->addProperty("TI_INC", TI_INC_).doc("");
  this->addProperty("TD_INC", TD_INC_).doc("");
  this->addProperty("max_output_current", max_output_current_).doc("");
  this->addProperty("current_reg_kp", current_reg_kp_).doc("");
  this->addProperty("reg_number", reg_number_).doc("");
  this->addProperty("debug", debug_).doc("");
  this->addProperty("max_desired_increment", max_desired_increment_).doc("");
  this->addProperty("enc_res", enc_res_).doc("");
  this->addProperty("A", A_).doc("");
  this->addProperty("BB0", BB0_).doc("");
  this->addProperty("BB1", BB1_).doc("");
  this->addProperty("current_mode", current_mode_).doc("");
  this->addProperty("eint_dif", eint_dif_).doc("");
}

CascadeRegulator::~CascadeRegulator() {
}

bool CascadeRegulator::configureHook() {
  reset();

  kp_pos = KP_POS_;
  Ti_pos = TI_POS_;
  Td_pos = TD_POS_;
  kp_inc = KP_INC_;
  Ti_inc = TI_INC_;
  Td_inc = TD_INC_;

  r0_pos = kp_pos * (1+ 1/(2*Ti_pos) + Td_pos);
  r1_pos = kp_pos * (1/(2*Ti_pos) - 2 * Td_pos - 1);
  r2_pos = kp_pos * Td_pos;

  r0_inc = kp_inc * (1+ 1/(2*Ti_inc) + Td_inc);
  r1_inc = kp_inc * (1/(2*Ti_inc) - 2 * Td_inc - 1);
  r2_inc = kp_inc * Td_inc;

  desired_position_old_ = desired_position_new_ = 0.0;

  a_ = A_;
  b0_ = BB0_;
  b1_ = BB1_;

  return true;
}

void CascadeRegulator::updateHook() {
  if (RTT::NewData == deltaInc_in.read(measured_increment_new_)) {
    update_hook_iteration_number_++;
    // std::cout << "  increment: " << measured_increment_new_;
    if (update_hook_iteration_number_ <= 1) {
      measured_increment_new_ = 0.0;
    }

    if (RTT::NewData == desired_position_.read(desired_position_new_)) {
      new_position_iteration_number_++;
      // std::cout << "  desired: " << desired_position_new_;

      if (new_position_iteration_number_ <= 1) {
        desired_position_old_ = desired_position_new_ = measured_position_new_;
        measured_position_old_ = measured_position_new_;
      }
    }

    if (RTT::NewData == measured_position_.read(measured_position_new_)) {
      // std::cout << "  position: " << measured_position_new_;
    }

    // std::cout << std::endl;

    if (RTT::NewData == synchro_state_in_.read(synchro_state_new_)) {
      if (synchro_state_new_ != synchro_state_old_) {
        desired_position_old_ = desired_position_new_;
        measured_position_old_ = measured_position_new_;
        synchro_state_old_ = synchro_state_new_;
        reset();
      }
    }

    if (synchro_state_old_) {
      int output = doServo_cas(desired_position_new_, measured_position_new_, measured_increment_new_);
      computedPwm_out.write(output);
    } else {
      desired_position_increment_ =
          (desired_position_new_ - desired_position_old_)
        * (enc_res_ / (2.0 * M_PI));

      if (fabs(desired_position_increment_) > max_desired_increment_) {
        std::cout << "very high pos_inc_: " << reg_number_ << " pos_inc: "
                  << desired_position_increment_ << std::endl;

        emergency_stop_out_.write(true);
      }
      desired_position_old_ = desired_position_new_;

      int output = doServo(desired_position_increment_, measured_increment_new_);
      computedPwm_out.write(output);
    }
  } else {
     computedPwm_out.write(0.0);
  }
}

int CascadeRegulator::doServo(double step_new, int pos_inc) {
// algorytm regulacji dla serwomechanizmu
// position_increment_old - przedostatnio odczytany przyrost polozenie
//                         (delta y[k-2] -- mierzone w impulsach)
// position_increment_new - ostatnio odczytany przyrost polozenie
//                         (delta y[k-1] -- mierzone w impulsach)
// step_old_pulse               - poprzednia wartosc zadana dla jednego kroku
//                         regulacji (przyrost wartosci zadanej polozenia --
//                         delta r[k-2] -- mierzone w impulsach)
// step_new               - nastepna wartosc zadana dla jednego kroku
//                         regulacji (przyrost wartosci zadanej polozenia --
//                         delta r[k-1] -- mierzone w radianach)
// set_value_new          - wielkosc kroku do realizacji przez HIP
//                         (wypelnienie PWM -- u[k]): czas trwania jedynki
// set_value_old          - wielkosc kroku do realizacji przez HIP
//                         (wypelnienie PWM -- u[k-1]): czas trwania jedynki
// set_value_very_old     - wielkosc kroku do realizacji przez HIP
//                         (wypelnienie PWM -- u[k-2]): czas trwania jedynki

  double step_new_pulse;  // nastepna wartosc zadana dla jednego kroku regulacji

  step_new_pulse = step_new;
  position_increment_new = pos_inc;

// Przyrost calki uchybu
  delta_eint = delta_eint_old
      + (1.0 + eint_dif_) * (step_new_pulse - position_increment_new)
      - (1.0 - eint_dif_) * (step_old_pulse - position_increment_old);

// std::cout << "POS INCREMENT NEW: " << position_increment_new <<  std::endl;

// obliczenie nowej wartosci wypelnienia PWM algorytm PD + I
  set_value_new = (1 + a_) * set_value_old - a_ * set_value_very_old
      + b0_ * delta_eint - b1_ * delta_eint_old;

  std::cout << "PWM VALUE (" << position_increment_old << " to " << position_increment_new << ") IS ";
  std::cout << static_cast<int>(set_value_new) << std::endl;

// ograniczenie na sterowanie
  if (set_value_new > MAX_PWM)
    set_value_new = MAX_PWM;
  if (set_value_new < -MAX_PWM)
    set_value_new = -MAX_PWM;

  if (current_mode_) {
    output_value = set_value_new * current_reg_kp_;
    if (output_value > max_output_current_) {
      output_value = max_output_current_;
    } else if (output_value < -max_output_current_) {
      output_value = -max_output_current_;
    }
  } else {
    output_value = set_value_new;
  }

  // przepisanie nowych wartosci zmiennych
  // do zmiennych przechowujacych wartosci poprzednie
  position_increment_old = position_increment_new;
  delta_eint_old = delta_eint;
  step_old_pulse = step_new_pulse;
  set_value_very_old = set_value_old;
  set_value_old = set_value_new;

  return (static_cast<int>(output_value));
}

int CascadeRegulator::doServo_cas(double position_desired, double position, int increment) {
  // algorytm regulacji

  position_err_new = position_desired - position;

  position_set_value_new = position_set_value_old + position_err_new * r0_pos +
      + position_err_old * r1_pos + position_err_very_old * r2_pos;

  double increment_desired = position_set_value_new;

  increment_err_new = increment_desired - increment;

  increment_set_value_new = increment_set_value_old + increment_err_new * r0_inc +
      + increment_err_old * r1_inc + increment_err_very_old * r2_inc;

  // ograniczenie na sterowanie
  if (increment_set_value_new > MAX_PWM)
    increment_set_value_new = MAX_PWM;
  if (increment_set_value_new < -MAX_PWM)
    increment_set_value_new = -MAX_PWM;

  if (current_mode_) {
    output_value = increment_set_value_new * current_reg_kp_;
    if (output_value > max_output_current_) {
      output_value = max_output_current_;
    } else if (output_value < -max_output_current_) {
      output_value = -max_output_current_;
    }
  } else {
    output_value = increment_set_value_new;
  }

  if (debug_) {
    std::cout << position_err_new <<";"<< position_set_value_new <<";"<< increment_err_new <<";"
        << increment_set_value_new <<";"<<  output_value << std::endl;
  }

  position_err_very_old = position_err_old;
  position_err_old = position_err_new;
  position_set_value_old = position_set_value_new;

  increment_err_very_old = increment_err_old;
  increment_err_old = increment_err_new;
  increment_set_value_old = increment_set_value_new;

  return (static_cast<int>(output_value));
}

void CascadeRegulator::reset() {
  position_err_new = 0.0;
  position_err_old = 0.0;
  position_err_very_old = 0.0;
  position_set_value_new = 0.0;
  position_set_value_old = 0.0;
  increment_err_new = 0.0;
  increment_err_old = 0.0;
  increment_err_very_old = 0.0;
  increment_set_value_new = 0.0;
  increment_set_value_old = 0.0;

  position_increment_old = 0.0;
  position_increment_new = 0.0;
  step_old_pulse = 0.0;
  step_new = 0.0;
  set_value_new = 0.0;
  set_value_old = 0.0;
  set_value_very_old = 0.0;
  delta_eint = 0.0;
  delta_eint_old = 0.0;
}

ORO_CREATE_COMPONENT(CascadeRegulator)
