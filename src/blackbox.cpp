#include "main.h"
#include "entropy.hpp"

FILE* outfile = nullptr;
bool toggle_blackbox() {
  if(!outfile) {
    outfile = fopen(("/usd/black_box_" + std::to_string(get_random())).c_str(), "w");
    return outfile;
  } else {
    fclose(outfile);
    return false;
  }
}

typedef enum {
  kDeviceTypeNoSensor        = 0,
  kDeviceTypeMotorSensor     = 2,
  kDeviceTypeLedSensor       = 3,
  kDeviceTypeAbsEncSensor    = 4,
  kDeviceTypeBumperSensor    = 5,
  kDeviceTypeImuSensor       = 6,
  kDeviceTypeRangeSensor     = 7,
  kDeviceTypeRadioSensor     = 8,
  kDeviceTypeTetherSensor    = 9,
  kDeviceTypeBrainSensor     = 10,
  kDeviceTypeVisionSensor    = 11,
  kDeviceTypeAdiSensor       = 12,
  kDeviceTypeGyroSensor      = 0x46,
  kDeviceTypeSonarSensor     = 0x47,
  kDeviceTypeGenericSensor   = 128,
  kDeviceTypeGenericSerial   = 129,
  kDeviceTypeUndefinedSensor = 255
} V5_DeviceType;

extern "C" {
  int32_t               vexDeviceGetStatus( V5_DeviceType *buffer );
}

std::string accelText(pros::c::imu_accel_s_t it) {
  return "accel(" + std::to_string(it.x) + ", " + std::to_string(it.y) + ", " + std::to_string(it.z) + ")";
}

std::string gyroText(pros::c::imu_gyro_s_t it) {
  return "gyro(" + std::to_string(it.x) + ", " + std::to_string(it.y) + ", " + std::to_string(it.z) + ")";
}

std::string attitudeText(pros::c::euler_s_t it) {
  return "attitude(" + std::to_string(it.pitch) + ", " + std::to_string(it.roll) + ", " + std::to_string(it.yaw) + ")";
}

void make_blackbox_entry() {
  if(outfile) {
    auto text = "@ " + std::to_string(pros::millis()) + "ms:\n";
    V5_DeviceType device_types[32];
    vexDeviceGetStatus(device_types);
    for(int i = 0; i < 32; i++) {
      if(device_types[i] == kDeviceTypeMotorSensor) {
        auto mtr = pros::Motor(i + 1);
        text += "  " + std::to_string(i + 1) + "[Motor "
          + std::to_string(mtr.get_position()) + "enc, "
          + std::to_string(mtr.get_actual_velocity()) + "rpm, "
          + std::to_string(mtr.get_current_draw()) + "mA, "
          + std::to_string(mtr.get_temperature()) + "temp"
          + "]\n";
      }
      if(device_types[i] == kDeviceTypeImuSensor) {
        auto imu = pros::Imu(i + 1);
        text += "  " + std::to_string(i + 1) + "[IMU "
          + accelText(imu.get_accel()) + ", " + gyroText(imu.get_gyro_rate()) + ", " + attitudeText(imu.get_euler())
          + "]\n";
      }
    }
    fwrite(text.c_str(), text.size(), 1, outfile);
  }
}

void init_blackbox() {
  pros::Task([]{
    while(true) {
      make_blackbox_entry();
      pros::delay(10);
    }
  });
}