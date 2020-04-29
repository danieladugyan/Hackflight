/*
   Arduino code for brushless motor running on standard ESC

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
#include "motors/esp32dshot600.hpp"

namespace hf {

    class DshotMotor : public Motor {

        private:
            Esp32DShot600 * _motors;
            uint8_t _index;

        public:

            DshotMotor(uint8_t pin, Esp32DShot600 * motors, uint8_t index) 
                : Motor(pin)
            {
                _motors = motors;
                _motors->addMotor(pin);
                _index = index;
            }

            virtual void init(void) override
            {
                pinMode(_pin, OUTPUT);
                _motors->begin();
            }

            virtual void write(float value) override
            { 
                _motors->writeMotor(_index, value); // value is in range [0, 1]
            }

    }; // class DshotMotor

} // namespace hf
