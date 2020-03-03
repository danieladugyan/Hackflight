#pragma once

#include "imus/softquat9dof.hpp"
#include <Adafruit_Sensor.h>
#include <Adafruit_Sensor_Calibration.h>
#include <Adafruit_FXOS8700.h>
#include <Adafruit_FXAS21002C.h>

namespace hf {
    Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);
    Adafruit_FXOS8700 accelmagmeter = Adafruit_FXOS8700(0x8700A, 0x8700B);
    Adafruit_Sensor_Calibration_EEPROM cal;

    class NXPSoftwareQuaternionIMU : public SoftwareQuaternion9DoFIMU {
        protected:
            virtual void begin(void) override
            {
                if (!cal.begin()) {
                    Serial.println("Failed to initialize calibration helper");
                } else if (! cal.loadCalibration()) {
                    Serial.println("No calibration loaded/found");
                }
                
                if (!gyro.begin()) {
                    /* There was a problem detecting the FXAS21002C ... check your connections */
                    Serial.println("Ooops, no FXAS21002C detected ... Check your wiring!");
                } // doesn't crash

                if (!accelmagmeter.begin(ACCEL_RANGE_2G)) {
                    /* There was a problem detecting the FXOS8700 ... check your connections */
                    Serial.println("Ooops, no FXOS8700 detected ... Check your wiring!");
                } // doesn't crash
            }

            virtual bool imuReady(void) override
            {
                return true;
            }

            virtual void imuReadAccelGyroMag(float & ax, float & ay, float & az, float & gx, float & gy, float &gz, float & mx, float & my, float & mz) override
            {
                sensors_event_t gyevent;
                sensors_event_t accevent;
                sensors_event_t magevent;

                /* Get a new gyro sensor event (rad/s) */
                gyro.getEvent(&gyevent);
                cal.calibrate(gyevent);

                gx = gyevent.gyro.x;
                gy = gyevent.gyro.y;
                gz = gyevent.gyro.z;
                
                /* Get new accelerometer/magnetometer sensor events (m/s^2 / uT) */
                accelmagmeter.getEvent(&accevent, &magevent);
                cal.calibrate(accevent);
                cal.calibrate(magevent);

                ax = accevent.acceleration.x;
                ay = accevent.acceleration.y;
                az = accevent.acceleration.z;

                mx = magevent.magnetic.x;
                my = magevent.magnetic.y;
                mz = magevent.magnetic.z;
            }
    };
}