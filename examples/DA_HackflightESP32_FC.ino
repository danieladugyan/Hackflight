/*
   Hackflight sketch for TinyPICO with Ultimate Sensor Fusion Solution IMU and DSMX receiver

   Additional libraries needed:

       https://github.com/simondlevy/USFS
       https://github.com/simondlevy/CrossPlatformDataBus
       https://github.com/simondlevy/SpektrumDSM

       https://github.com/plerup/espsoftwareserial


   Copyright (c) 2019 Simon D. Levy

   This file is part of Hackflight.
   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "hackflight.hpp"
#include "boards/realboards/espdev.hpp"
#include "actuators/mixers/quadxcf.hpp"
#include "pidcontrollers/rate.hpp"
#include "pidcontrollers/level.hpp"
#include "imus/usfs.hpp"
//#include "imus/softquats/nxp.hpp"

#include "receivers/dualshock.hpp"
//#include "receivers/espnow.hpp"

hf::Hackflight h;

// init Dualshock 4 - MAC: DOIT DEVKIT V1
// might need to erase flash before using
hf::DualshockReceiver rc = hf::DualshockReceiver("30:AE:A4:8F:BA:28");
//hf::ESPNOWReceiver rc;

hf::MixerQuadXCF mixer;

hf::RatePid ratePid = hf::RatePid(0.05f, 0.00f, 0.00f, 0.10f, 0.01f);

hf::LevelPid levelPid = hf::LevelPid(0.20f);

//hf::NXPSoftwareQuaternionIMU imu;
hf::USFS imu;

/*#include "motors/esp32dshot600.hpp"
#include "motors/dshotStandard.hpp"
hf::Esp32DShot600 DshotMotors;
hf::DshotMotor motor1(13, &DshotMotors, 0);
hf::DshotMotor motor2(25, &DshotMotors, 1);
hf::DshotMotor motor3(26, &DshotMotors, 2);
hf::DshotMotor motor4(27, &DshotMotors, 3);*/

/*
    4cw   2ccw
       \ /
        ^
       / \
    3ccw  1cw
*/

#include "motors/multishot.hpp"
hf::MultiShotMotor motor1(13); // Huzzah32 A12, GPIO13, M1
hf::MultiShotMotor motor2(27); // Huzzah32 A10, GPIO27, M2
hf::MultiShotMotor motor3(26); // Huzzah32  A0, GPIO26, M3
hf::MultiShotMotor motor4(25); // Huzzah32  A1, GPIO25, M4
hf::Motor * motors[4] = { &motor1, &motor2, &motor3, &motor4 };

/*uint8_t inThrottle = 0, inYaw = 90, inPitch = 180, inRoll = 220;

typedef struct messageIn
{ // data sent from CONTROLLER
  uint8_t throttle;
  uint8_t yaw;
  uint8_t pitch;
  uint8_t roll;
} messageIn;

messageIn dataController;

void onDataRecv(const uint8_t * mac, const uint8_t * dataIn, int len) {
  memcpy(&dataController, dataIn, sizeof(dataController));
  rc.updateVals(dataController.throttle,dataController.yaw,dataController.pitch, dataController.roll);
}*/

void setup() {
  // Initialize Hackflight firmware
  h.init(new hf::ESPDEV(), &imu, &rc, &mixer, motors);
  //esp_now_register_recv_cb(onDataRecv);

  //DshotMotors.printInfo(); // test, check motor count, add print func in esp32dshot600.hpp instead

  // Add Rate and Level PID controllers (Stabilize mode)
  h.addPidController(&levelPid);
  h.addPidController(&ratePid);
}

void loop() {
  h.update();
}
