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
#include <esp_now.h>
#include <WiFi.h>

static constexpr uint8_t DEFAULT_MAP[6] = {0, 1, 2, 3, 4, 5};
static constexpr float DEFAULT_DEMAND_SCALE = 1.0f;

namespace hf {
    class ESPNOWReceiver : public Receiver {
    
    public:    
        void updateVals(uint8_t throttle, uint8_t roll, uint8_t pitch, uint8_t yaw) {
            rawvals[CHANNEL_THROTTLE] = (_tCor.update(throttle)/127.f)-1.f;
            rawvals[CHANNEL_ROLL] = _rCor.update(roll)/127.f;
            rawvals[CHANNEL_PITCH] = _pCor.update(pitch)/127.f;
            rawvals[CHANNEL_YAW] = _yCor.update(yaw)/127.f;
            /*rawvals[CHANNEL_AUX1] = 0;
            rawvals[CHANNEL_AUX2] = 0;*/
        }

    protected:
        uint8_t _broadcastAddress[6] = {0x30, 0xAE, 0xA4, 0x28, 0x74, 0x4C}; // Controller MAC address

        const float _alpha = 0.3f;
        ExpMovAvg _tCor = ExpMovAvg(_alpha);
        ExpMovAvg _rCor = ExpMovAvg(_alpha);
        ExpMovAvg _pCor = ExpMovAvg(_alpha);
        ExpMovAvg _yCor = ExpMovAvg(_alpha);

        void begin(void)
        {
            WiFi.mode(WIFI_STA); // set device as a Wi-Fi Station

            if (esp_now_init() != ESP_OK)
            { // init esp-now
                Serial.println("Error initializing ESP-NOW");
                return;
            }

            esp_now_peer_info_t peerInfo;
            memcpy(peerInfo.peer_addr, _broadcastAddress, 6);
            peerInfo.channel = 0;
            peerInfo.encrypt = false;

            // add drone
            if (esp_now_add_peer(&peerInfo) != ESP_OK)
            {
                Serial.println("Failed to add peer");
                return;
            }
        }

        virtual bool gotNewFrame(void) override
        {
            return true;
        }

        void readRawvals(void)
        {
            return;
        }

        bool lostSignal(void)
        {
            return false;
        }

    public:
        ESPNOWReceiver(void)
            : Receiver(DEFAULT_MAP, DEFAULT_DEMAND_SCALE)
        {
        }

    }; // class ESPNOW
} // namespace hf
