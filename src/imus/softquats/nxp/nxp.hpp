/*
   Support for USFS IMU

   Copyright (c) 2018 Simon D. Levy

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

#pragma once

#include <Wire.h>

#include <Adafruit_Sensor_Calibration.h>
#include <Adafruit_AHRS.h>
#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>

#include "imu.hpp"

namespace hf
{
    class NXP : public IMU {
    
    private:
        // Tunable NXP parameters
        static const uint8_t UPDATE_RATE = 100; // Hz

        Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;
        //#include "NXP_FXOS_FXAS.h"
        #include <NXP_FXOS_FXAS.h>
        Adafruit_Mahony filter;
        Adafruit_Sensor_Calibration_EEPROM cal;

        sensors_event_t accel, gyro, mag;
        float gx, gy, gz;

        void checkEventStatus(void)
        {
            accelerometer->getEvent(&accel);
            gyroscope->getEvent(&gyro);
            magnetometer->getEvent(&mag);

            cal.calibrate(mag);
            cal.calibrate(accel);
            cal.calibrate(gyro);
        }
    protected:

        virtual void adjustGyrometer(float & gx, float & gy, float & gz) { (void)gx; (void)gy; (void)gz;  }
        virtual void adjustQuaternion(float & qw, float & qx, float & qy, float & qz) { (void)qw; (void)qx; (void)qy; (void)qz;  }
    
    public:

        virtual bool getGyrometer(float & gx, float & gy, float & gz) override
        {
            checkEventStatus();

            gx = gyro.gyro.x;
            gy = gyro.gyro.y;
            gz = gyro.gyro.z;

            adjustGyrometer(gx, gy, gz);
            return true;
        }

        virtual bool getQuaternion(float &qw, float &qx, float &qy, float &qz, float time) override
        {
            (void)time;

            filter.update(gx, gy, gz, 
                accel.acceleration.x, accel.acceleration.y, accel.acceleration.z, 
                mag.magnetic.x, mag.magnetic.y, mag.magnetic.z);
            
            filter.getQuaternion(&qw, &qx, &qy, &qz);

            adjustQuaternion(qw, qx, qy, qz);
            return true;
        }

        virtual void begin(void) override
        {
            // Start the NXP in master mode, no interrupt
            if (!cal.begin())
            { // init calibration
                Serial.println("Failed to initialize calibration helper");
            }
            else if (!cal.loadCalibration())
            {
                Serial.println("No calibration loaded/found");
            }

            if (!init_sensors())
            { // init sensors
                Serial.println("Failed to find sensors");
                while (1)
                    delay(10);
            }

            setup_sensors();
            filter.begin(UPDATE_RATE);
        }
    };
}