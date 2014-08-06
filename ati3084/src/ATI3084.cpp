#include <ocl/Component.hpp>

#include "ATI3084.h"

#define MUX0 0
#define MUX1 1
#define MUX2 2

#define DOSD 3

void WrenchKDLToMsg(const KDL::Wrench &in, geometry_msgs::Wrench &out) {
  out.force.x = in[0];
  out.force.y = in[1];
  out.force.z = in[2];

  out.torque.x = in[3];
  out.torque.y = in[4];
  out.torque.z = in[5];
}

ATI3084::ATI3084(const std::string &name)
    : RTT::TaskContext(name, PreOperational),
      wrench_port_("Wrench"),
      device_prop_("device", "DAQ device to use", "/dev/comedi0"),
      offset_prop_("offset", "sensor zero offset", KDL::Wrench::Zero()),
      device_(NULL){
  this->addPort(wrench_port_);
  this->addProperty(device_prop_);
  this->addProperty(offset_prop_);
  
  	// Initialize conversion matrix
  conversion_matrix << -0.000022, 0.001325, -0.035134, 0.640126, 0.051951, -0.641909, 0.017570, -0.743414, -0.016234, 0.372558, -0.032329, 0.366082, -1.184654, -0.012028, -1.165485, -0.014266, -1.174821, 0.002540, 0.007847, -0.144965, 0.552931, 0.079813, -0.571950, 0.071877, -0.661215, -0.007048, 0.337836, -0.125610, 0.315335, 0.132327, -0.010556, 0.346443, -0.009666, 0.344562, -0.031572, 0.339944;

  conversion_scale << -20.4, -20.4, -20.4, -1.23, -1.23, -1.23;
}

bool ATI3084::configureHook() {
  return initSensor();
}

bool ATI3084::startHook() {
  readData();

  bias_ = voltage_ADC_;

  return true;
}

void ATI3084::updateHook() {
  geometry_msgs::Wrench wrenchMsg;

  readData();
  voltage2FT();

  //wrench_ -= offset_prop_.value();

  WrenchKDLToMsg(wrench_, wrenchMsg);
  wrench_port_.write(wrenchMsg);
}

void ATI3084::stopHook() {

}

bool ATI3084::initSensor() {
  device_ = comedi_open(device_prop_.value().c_str());
  if (!device_) {
    RTT::log(RTT::Error) << "Unable to open device [" << device_prop_.value()
                         << "]" << RTT::endlog();
    return false;
  }

  //comedi_dio_config(device_, DOSD, 0, COMEDI_OUTPUT);
  //comedi_dio_config(device_, DOSD, 1, COMEDI_OUTPUT);
  //comedi_dio_config(device_, DOSD, 2, COMEDI_OUTPUT);

  maxdata_ = comedi_get_maxdata(device_, 0, 0);
  rangetype_ = comedi_get_range(device_, 0, 0, 0);
  
  return true;
}

void ATI3084::readData() {

  comedi_dio_write(device_, DOSD, MUX0, 0);
  comedi_dio_write(device_, DOSD, MUX1, 0);
  comedi_dio_write(device_, DOSD, MUX2, 0);

  usleep(100);
  comedi_data_read(device_, 0, 0, 0, AREF_DIFF, &raw_ADC_[0]);
  ////// G1

  comedi_dio_write(device_, DOSD, MUX0, 1);
  usleep(100);
  comedi_data_read(device_, 0, 0, 0, AREF_DIFF, &raw_ADC_[1]);
  ////// G2

  comedi_dio_write(device_, DOSD, MUX1, 1);
  usleep(100);
  comedi_data_read(device_, 0, 0, 0, AREF_DIFF, &raw_ADC_[2]);
  ////// G3

  comedi_dio_write(device_, DOSD, MUX0, 0);
  usleep(100);
  comedi_data_read(device_, 0, 0, 0, AREF_DIFF, &raw_ADC_[3]);
  ////// G4

  comedi_dio_write(device_, DOSD, MUX2, 1);
  usleep(100);
  comedi_data_read(device_, 0, 0, 0, AREF_DIFF, &raw_ADC_[4]);
  ////// G5

  comedi_dio_write(device_, DOSD, MUX0, 1);
  usleep(100);
  comedi_data_read(device_, 0, 0, 0, AREF_DIFF, &raw_ADC_[5]);

  for (int i = 0; i < 6; i++) {
    voltage_ADC_(i) = comedi_to_phys(raw_ADC_[i], rangetype_, maxdata_);
  }

  //for(int i = 0; i < 6; i++)
  //  voltage_ADC_[i] /= 5;
}

void ATI3084::voltage2FT() {
  SetToZero(wrench_);
  
	Vector6d result_voltage = voltage_ADC_ - bias_;
	
  Vector6d force = conversion_matrix * result_voltage;
	force = force.array() * conversion_scale.array();
	
	for (int i = 0; i < 6; i++) {
    wrench_[i] = force(i);
  }
	
  /*

  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      wrench_[i] += (voltage_ADC_(j) - bias_(j)) * conversion_matrix(i, j);
    }

    wrench_(i) /= conversion_scale(i);
  }*/
}

ORO_CREATE_COMPONENT(ATI3084)
