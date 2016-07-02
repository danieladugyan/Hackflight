#!/usr/bin/env python
'''
   companion.py : Companion-board Python code

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
'''

import cv2
import struct

from socket_server import serve_socket

client = serve_socket(5000)

while True:

    data = client.recv(4)
    imgsize = struct.unpack('i', data)[0]
    print(imgsize)

