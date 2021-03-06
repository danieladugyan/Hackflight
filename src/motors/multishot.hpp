/*
   Arduino code for brushless motors running on MultiShot ESC

   Copyright (c) 2018 Juan Gallostra Acin, Simon D. Levy, Pep Martí Saumell

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

#include "motor.hpp"

namespace hf {

    class MultiShotMotor : public Motor {

        private:

            const uint16_t _channel = 1;
            uint16_t _pin;

            // Min, max PWM values, experimental values, should be 819-4095?
            const uint16_t PWM_MIN = 400;
            const uint16_t PWM_MAX = 2000;

        public:

            MultiShotMotor(uint8_t pin) 
                : Motor(pin)
            {
                _pin = pin;
            }

            virtual void init(void) override
            {
                ledcSetup(_channel, 20000, 13); // 20 kHz, 13-bit resolution
                ledcAttachPin(_pin, _channel);
                ledcWrite(_channel, PWM_MIN);
            }

            virtual void write(float value) override
            {
                value = Filter::round2(value); // round to two decimal places
                ledcWrite(_channel, (uint16_t)(PWM_MIN+value*(PWM_MAX-PWM_MIN)));
            }

    }; // class MultiShotMotor

} // namespace hf
