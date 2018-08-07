/*
main.h 

Copyright (C) 2017 Simon D. Levy 

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

extern "C" {

#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>

void ledSet(bool value);

unsigned long micros(void);
unsigned long millis(void);
void delay(unsigned long);

void reset(void);
void resetToBootloader(void);

void setup(void);
void loop(void);

} // extern "C"