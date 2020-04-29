/*
   "Mock" receiver subclass for prototyping

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

#include "filters.hpp"
#include "receiver.hpp"
#include <PS4Controller.h>

static constexpr uint8_t DEFAULT_MAP[6] = {0,1,2,3,4,5};
static constexpr float   DEFAULT_DEMAND_SCALE = 1.0f;

namespace hf {

    class DualshockReceiver : public Receiver {
        private:
            char _macAddress[18];
            
            const float _alpha = 0.3f;
            ExpMovAvg _tCor = ExpMovAvg(_alpha);
            ExpMovAvg _rCor = ExpMovAvg(_alpha);
            ExpMovAvg _pCor = ExpMovAvg(_alpha);
            ExpMovAvg _yCor = ExpMovAvg(_alpha);

        protected:
        volatile uint16_t _rcValue[MAXCHAN];

            void begin(void)
            {
                if (!PS4.begin(_macAddress)) {
                    Serial.println("Error initializing DS4 bluetooth");
                }
            }

            virtual bool gotNewFrame(void) override
            {
                if (PS4.isConnected()) {
                    return true;
                }
            }

            void readRawvals(void)
            {
                rawvals[CHANNEL_THROTTLE] = (_tCor.update(PS4.data.analog.button.r2)/127.f)-1.f;
                rawvals[CHANNEL_ROLL] = _rCor.update(PS4.data.analog.stick.lx)/127.f;
                rawvals[CHANNEL_PITCH] = _pCor.update(PS4.data.analog.stick.ly)/127.f;
                rawvals[CHANNEL_YAW] = _yCor.update(PS4.data.analog.stick.rx)/127.f;
                
                //Serial.println(rawvals[CHANNEL_THROTTLE]);
                if (PS4.data.button.cross) {
                    rawvals[CHANNEL_AUX1] = !rawvals[CHANNEL_AUX1];
                    delay(200); // INCREDIBLY stupid debounce
                }
                if (PS4.data.button.circle) {
                    rawvals[CHANNEL_AUX2] = !rawvals[CHANNEL_AUX2];
                    delay(200); // INCREDIBLY stupid debounce
                }
            }

            bool lostSignal(void)
            {
                if (PS4.isConnected()) {
                    return false;
                }
                
                return true;
            }

        public:

            DualshockReceiver(char * mac) 
                : Receiver(DEFAULT_MAP, DEFAULT_DEMAND_SCALE)
            {
                memcpy(_macAddress, mac, 18);
            }

    }; // class MockReceiver

} // namespace hf
