#pragma once

#include <memory>
#include <unistd.h>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <cmath>

#include "vehicle_core/Devices/I2C/I2CDevice.hpp"

class BNO055 : public I2CDevice {
public:
  using I2CDevice::I2CDevice;

  BNO055(std::shared_ptr<I2CBus> bus, uint8_t addr = 0x29);

  // ---- Init sequence
  bool init_ndof(bool ext_xtal = true);

  // ---- Reads
  bool read_quat(double& w, double& x, double& y, double& z);
  bool read_gyro_rad_s(double& gx, double& gy, double& gz);
  bool read_lin_acc_ms2(double& ax, double& ay, double& az);
  bool read_mag_uT(double& mx, double& my, double& mz);

  // calibration (0..3 each)
  bool read_calib(uint8_t& sys, uint8_t& gyro, uint8_t& accel, uint8_t& mag);

private:
};
