#!/usr/bin/env python3
#
# Copyright (C) 2017 Hector Garcia de Marina <hgdemarina@gmail.com>
#                    Gautier Hattenberger <gautier.hattenberger@enac.fr>
#
# This file is part of paparazzi.
#
# paparazzi is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2, or (at your option)
# any later version.
#
# paparazzi is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with paparazzi; see the file COPYING.  If not, see
# <http://www.gnu.org/licenses/>.
#

'''
Moving base simulator
'''

# too many things here
from __future__ import print_function
import sys
import numpy as np
import json
from time import sleep
from os import path, getenv
import time
import pymap3d as pm
import math as m

PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
#PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '~/paparazzi'))) #TEST
sys.path.append(PPRZ_HOME + "/var/lib/python/")
from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage

from scipy import linalg as la

class UAV:
    def __init__(self, ac_id):
        self.initialized = False
        self.id = ac_id
        self.lat = 0
        self.lon = 0
        self.alt = 0
        self.timeout = 0

class Base:
    def __init__(self, freq=10, use_ground_ref=False, ignore_geo_fence=False, verbose=False):
        self.step = 1. / freq
        self.use_ground_ref = use_ground_ref
        self.enabled = True # run sim by default
        self.verbose = verbose
        self.ids = [203] # originaly 14, but did not work, did not match A/C in Paparazzi
        self.uavs = [UAV(i) for i in self.ids]
        self.time = time.mktime(time.gmtime())
        self.speed = 1.5 # m/s
        self.course = 90 # deg (moving directon of platform)
        self.heading = 45 # deg (orientation of platform)
        print("lat0,long0,alt0 position = Valkenburg")
        self.lat0 = 52.167102 #deg
        self.lon0 = 4.413168 #deg
        self.alt0 = 5.0 # starts from 1 m high

        # Start IVY interface
        self._interface = IvyMessagesInterface("Moving Base")

        # bind to GPS_INT message
        def ins_cb(ac_id, msg):
            if ac_id in self.ids and msg.name == "GPS_INT":
                uav = self.uavs[self.ids.index(ac_id)]
                i2p = 1. / 2**8     # integer to position
                i2v = 1. / 2**19    # integer to velocity
                uav.lat[0] = float(msg['lat']) / 1e7
                uav.lon[1] = float(msg['lon']) / 1e7
                uav.alt[2] = float(msg['alt']) / 100
                uav.timeout = 0
                uav.initialized = True
                print("GPS_INT msg received\n") #TEST
        if not self.use_ground_ref:
            self._interface.subscribe(ins_cb, PprzMessage("telemetry", "INS"))

        # bind to GROUND_REF message
        def ground_ref_cb(ground_id, msg):
            ac_id = int(msg['ac_id'])
            if ac_id in self.ids:
                uav = self.uavs[self.ids.index(ac_id)]
                # X and V in NED
                uav.X[0] = float(msg['pos'][1])
                uav.X[1] = float(msg['pos'][0])
                uav.X[2] = -float(msg['pos'][2])
                uav.V[0] = float(msg['speed'][1])
                uav.V[1] = float(msg['speed'][0])
                uav.V[2] = -float(msg['speed'][2])
                uav.timeout = 0
                uav.initialized = True
                print("GPS_INT msg received\n") #TEST
        if self.use_ground_ref:
            self._interface.subscribe(ground_ref_cb, PprzMessage("ground", "GROUND_REF"))

    def __del__(self):
        self.stop()

    def stop(self):
        # Stop IVY interface
        if self._interface is not None:
            self._interface.shutdown()

    # Adds the NED coordinates to the GROUND_REF to get new LLA coordinate of ship
    def move_base(self, north, east, down=0):
        # [lat,lon,h] = ned2geodetic(xNorth,yEast,zDown,lat0,lon0,h0,spheroid) 
        # transforms the local north-east-down (NED) Cartesian coordinates specified by xNorth, yEast, and zDown 
        # to the geodetic coordinates specified by lat, lon, and h. 
        # Specify the origin of the local NED system with the geodetic coordinates lat0, lon0, and h0. 
        # Each coordinate input argument must match the others in size or be scalar. 
        # Specify spheroid as the reference spheroid for the geodetic coordinates. 
        out = pm.ned2geodetic(north, east, down, self.lat0, self.lon0, self.alt0) 
        self.lat0 = out[0]
        self.lon0 = out[1]
        print(out)

    def send_pos(self):
        '''
        Send position of base sation
        '''
        ready = True
        for uav in self.uavs:
            if not uav.initialized:
                if self.verbose:
                    print("Waiting for state of rotorcraft ", uav.id)
                    sys.stdout.flush()
                ready = False
            if uav.timeout > 0.5:
                if self.verbose:
                    print("TIMEOUT ERROR: The state msg of rotorcraft ", uav.id, ": stopped")
                    sys.stdout.flush()
                ready = False

        # if not ready:
        #     return


        if self.verbose:
            E = 0 #dummy value (TODO: ERROR SHOULD BE CALCULATED BY RTK)
            print("Error distances: " + str(E).replace('[','').replace(']',''))
            sys.stdout.flush()

        for ac in self.uavs:
            # send ship location/speed information to drone
            msg = PprzMessage("datalink", "TARGET_POS")
            msg['ac_id'] = ac.id
            msg['target_id'] = ac.id
            msg['lat'] = int(self.lat * 1e7)
            msg['lon'] = int(self.lon * 1e7)
            msg['alt'] = int(self.altitude *1000)
            msg['speed'] = self.speed
            msg['climb'] = 0
            msg['course'] = self.course
            msg['heading'] = self.heading
            self._interface.send(msg)

    def run(self):
        try:
            # The main loop
            print("\nmoving base loop started!\n")
            while True:
                # TODO: make better frequency managing
                sleep(self.step)

                for uav in self.uavs:
                    uav.timeout = uav.timeout + self.step

                # Send base position
                if self.enabled:
                    dn = self.speed*m.cos(self.course/180.0*m.pi) # DELTA ?
                    de = self.speed*m.sin(self.course/180.0*m.pi) # DELTA ?
                    self.move_base(self.step*dn,self.step*de)
                    self.send_pos()

        except KeyboardInterrupt:
            self.stop()


if __name__ == '__main__':
    import argparse


    base = Base()
    base.run()

