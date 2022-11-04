#!/usr/bin/env python3
#
# Copyright (C) 2017 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
Forward rigid body position from NatNet (Optitrack positioning system)
to the IVY bus as a REMOTE_GPS_LOCAL message

As the NatNetClient is only compatible with Python 3.x, the Ivy python
should be installed for this version, eventually by hand as paparazzi
packages are only providing an install for Python 2.x (although the
source code itself is compatile for both version)

Manual installation of Ivy:
    1. git clone https://gitlab.com/ivybus/ivy-python.git
    2. cd ivy-python
    3. sudo python3 setup.py install
Otherwise, you can use PYTHONPATH if you don't want to install the code
in your system



##########################
###### Axes systems ######
##########################

The following assumes you calibrated the Optitrack ground plane as shown below,
with the long edge of the ground plane calibration triangle towards the right 
wall.
For correct heading information, define your rigid body in Motive with the nose
of the vehicle (body-x) pointing away from you and towards the far edge.
+──────────────────────────+
│                          │
│                          │
│            .             │
│            |             │
│            .────.        │
│                          │
│                          │
│                          │
+──────────────────────────+
│        Observers         │
└──────────────────────────┘


1. Optitrack axis system
   Without making changes in Motive, this should be transmitted via NatNet
+──────────────────────────+
│                          │
│                          │
│             x            │
│             ▲            │
│             │            │
│             ⊙──►z        │
│           y              │
│                          │
│                          │
+──────────────────────────+
│        Observers         │
└──────────────────────────┘


2. Cyberzoo axis system: DEFAULT OUTPUT
   Per default, this program will rotate the axis to match the system below. The
   advantage is that gps_datalink.c will understand the axes such that the North
   of the ENU axes is pointed away from the observer, which is intuitive.
   Rotation can be represented by a 2pi/3 rotation around the axis [1, 1, 1]
   in the Optitrack system.
+──────────────────────────+
│                          │
│                          │
│             y            │
│             ▲            │
│             │            │
│             ⊙──►x        │
│           z              │
│                          │
│                          │
+──────────────────────────+
│        Observers         │
└──────────────────────────┘


3. Geographic system: 
   Here, y points towards the geographic North, making the resulting ENU system 
   in the drone correspond to the true magnetic directions.
   The argument --north_angle <angle> should be used to specifcy the right-hand
   rotation around z from the Cyberzoo axis system to the Geographic system, if
   the user wants to use it.
+──────────────────────────+
│                          │
│               y          │
│              ▲           │
│             /            │
│            /             │
│         z ⊙ \            │
│              \► x        │
│                          │
│                          │
│                          │
+──────────────────────────+
│        Observers         │
└──────────────────────────┘

'''


from __future__ import print_function

import sys
from os import path, getenv
from time import time, sleep
import numpy as np
from pyquaternion import Quaternion as Quat
from collections import deque
import argparse

# import NatNet client
from NatNetClient import NatNetClient

# if PAPARAZZI_HOME not set, then assume the tree containing this
# file is a reasonable substitute
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_HOME + "/var/lib/python")
from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage

# parse args
parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
parser.add_argument('-le', '--long_edge', required=True, dest='long_edge', choices=['left', 'far', 'right', 'near'], help="Side of the test area that the long edge of the calibration tool was pointing at")
group = parser.add_mutually_exclusive_group(required=True)
group.add_argument('-xs', '--x_side', dest='x_side', choices=['left', 'far', 'right', 'near'], help="Side that the x/east axis of the resulting ENU frame should point to.")
group.add_argument('-xa', '--x_angle_from_left', dest='x_angle', type=float, help="Right-hand rotation in degrees of the x/east-axis around the up-axis, where 0.0 means x/east is pointing to the left")
parser.add_argument('-an', '--ac_nose', required=True, dest='ac_nose', choices=['left', 'far', 'right', 'near'], help="Side of the test area that the nose of the drone was pointing at when definition the rigid body")
parser.add_argument('-ac', action='append', nargs=2,
                    metavar=('rigid_id','ac_id'), required=True, help='pair of rigid body and A/C id (multiple possible)')

parser.add_argument('-b', '--ivy_bus', dest='ivy_bus', help="Ivy bus address and port")
parser.add_argument('-s', '--server', dest='server', default="127.0.0.1", help="NatNet server IP address")
parser.add_argument('-m', '--multicast_addr', dest='multicast', default="239.255.42.99", help="NatNet server multicast address")
parser.add_argument('-dp', '--data_port', dest='data_port', type=int, default=1511, help="NatNet server data socket UDP port")
parser.add_argument('-cp', '--command_port', dest='command_port', type=int, default=1510, help="NatNet server command socket UDP port")
parser.add_argument('-v', '--verbose', dest='verbose', action='store_true', help="display debug messages")
parser.add_argument('-f', '--freq', dest='freq', default=10, type=int, help="transmit frequency")
parser.add_argument('-gr', '--ground_ref', dest='ground_ref', action='store_true', help="also send the GROUND_REF message")
parser.add_argument('-vs', '--vel_samples', dest='vel_samples', default=4, type=int, help="amount of samples to compute velocity (should be greater than 2)")
parser.add_argument('-rg', '--remote_gps', dest='rgl_msg', action='store_true', help="use the old REMOTE_GPS_LOCAL message")
parser.add_argument('-sm', '--small', dest='small_msg', action='store_true', help="enable the EXTERNAL_POSE_SMALL message instead of the full")
parser.add_argument('-o', '--old_natnet', dest='old_natnet', action='store_true', help="Change the NatNet version to 2.9")

args = parser.parse_args()

if args.ac is None:
    print("At least one pair of rigid boby / AC id must be declared")
    exit()

# dictionary of ID associations
id_dict = dict(args.ac)

# initial time per AC
timestamp = dict([(ac_id, None) for ac_id in id_dict.keys()])
period = 1. / args.freq

# initial track per AC
track = dict([(ac_id, deque()) for ac_id in id_dict.keys()])

# axes rotation definitions
edge_correction = {'left': -90., 'far': 180., 'right': 90., 'near': 0.}
q_edge_correction = Quat(
    axis=[0., 1., 0.],
    angle=np.deg2rad(edge_correction[args.long_edge]))

if args.x_side is not None:
    x_correction = {'left': 0., 'far': -90., 'right': 180., 'near': 90.}
    x_angle = x_correction[args.x_side]
else:
    x_angle = args.x_angle

q_x_correction = Quat(
    axis=[0., 0., 1.],
    angle=np.deg2rad(x_angle)
)
q_edge_near_to_x_left = Quat(axis=[0., 1., 1.], angle=np.pi)
    
q_total = q_x_correction * q_edge_near_to_x_left * q_edge_correction
nose_correction = {'left': 0., 'far': -90., 'right': 180., 'near': 90.}
q_nose_correction = Quat(
    axis=[0., 0., 1.],
    angle=np.deg2rad(nose_correction[args.ac_nose] - x_angle)
)

# start ivy interface
if args.ivy_bus is not None:
    ivy = IvyMessagesInterface("natnet2ivy", ivy_bus=args.ivy_bus)
else:
    ivy = IvyMessagesInterface("natnet2ivy")

# store track function
def store_track(ac_id, pos, t):
    if ac_id in id_dict.keys():
        track[ac_id].append((pos, t))
        if len(track[ac_id]) > args.vel_samples:
            track[ac_id].popleft()

# compute velocity from track
# returns zero if not enough samples
def compute_velocity(ac_id):
    vel = [ 0., 0., 0. ]
    if len(track[ac_id]) >= args.vel_samples:
        nb = -1
        for (p2, t2) in track[ac_id]:
            nb = nb + 1
            if nb == 0:
                p1 = p2
                t1 = t2
            else:
                dt = t2 - t1
                if dt < 1e-5:
                    continue
                vel[0] += (p2[0] - p1[0]) / dt
                vel[1] += (p2[1] - p1[1]) / dt
                vel[2] += (p2[2] - p1[2]) / dt
                p1 = p2
                t1 = t2
        if nb > 0:
            vel[0] /= nb
            vel[1] /= nb
            vel[2] /= nb
    return vel

def receiveRigidBodyList( rigidBodyList, stamp ):
    for (ac_id, pos, quat, valid) in rigidBodyList:
        if not valid:
            # skip if rigid body is not valid
            continue

        i = str(ac_id)
        if i not in id_dict.keys():
            continue

        store_track(i, pos, stamp)
        if timestamp[i] is None or abs(stamp - timestamp[i]) < period:
            if timestamp[i] is None:
                timestamp[i] = stamp
            continue # too early for next message
        timestamp[i] = stamp

        vel = compute_velocity(i)

        # Rotate position and velocity according to the quaternions found above
        pos = q_total.rotate([pos[0], pos[1], pos[2]]).tolist()
        vel = q_total.rotate([vel[0], vel[1], vel[2]]).tolist()

        # Rotate the attitude delta to the new frame
        quat = Quat(real=quat[3], imaginary=[quat[0], quat[1], quat[2]])
        quat = q_nose_correction * (q_total * quat * q_total.inverse)
        quat = quat.elements[[3, 0, 1, 2]].tolist()

        # Check which message to send
        if args.rgl_msg:
            msg = PprzMessage("datalink", "REMOTE_GPS_LOCAL")
            msg['ac_id'] = id_dict[i]
            msg['pad'] = 0
            msg['enu_x'] = pos[0]
            msg['enu_y'] = pos[1]
            msg['enu_z'] = pos[2]
            msg['enu_xd'] = vel[0]
            msg['enu_yd'] = vel[1]
            msg['enu_zd'] = vel[2]
            msg['tow'] = int(1000. * stamp) # TODO convert to GPS itow ?
            # convert quaternion to psi euler angle
            dcm_0_0 = 1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2])
            dcm_1_0 = 2.0 * (quat[0] * quat[1] - quat[3] * quat[2])
            msg['course'] = 180. * np.arctan2(dcm_1_0, dcm_0_0) / 3.14
            ivy.send(msg)
        elif args.small_msg:
            # First check if position fits in message
            if abs(pos[0]*100.0) > pow(2, 10) or abs(pos[1]*100.0) > pow(2, 10) or pos[2]*100.0 > pow(2, 10) or pos[2] < 0.:
                print("Position out of range for small message")
                continue

            # TODO calculate everything
            msg = PprzMessage("datalink", "EXTERNAL_POSE_SMALL")
            msg['ac_id'] = id_dict[i]
            msg['timestamp'] = int(1000. * stamp) # Time in ms
            ivy.send(msg)
        else:
            msg = PprzMessage("datalink", "EXTERNAL_POSE")
            msg['ac_id'] = id_dict[i]
            msg['timestamp'] = int(1000. * stamp) # Time in ms
            msg['enu_x'] = pos[0]
            msg['enu_y'] = pos[1]
            msg['enu_z'] = pos[2]
            msg['enu_xd'] = vel[0]
            msg['enu_yd'] = vel[1]
            msg['enu_zd'] = vel[2]
            msg['body_qi'] = quat[3]
            msg['body_qx'] = quat[0]
            msg['body_qy'] = quat[1]
            msg['body_qz'] = quat[2]
            ivy.send(msg)

        # send GROUND_REF message if needed
        if args.ground_ref:
            gr = PprzMessage("ground", "GROUND_REF")
            gr['ac_id'] = str(id_dict[i])
            gr['frame'] = "LTP_ENU"
            gr['pos'] = pos
            gr['speed'] = vel
            gr['quat'] = [quat[3], quat[0], quat[1], quat[2]]
            gr['rate'] = [ 0., 0., 0. ]
            gr['timestamp'] = stamp
            ivy.send(gr)



# start natnet interface
natnet_version = (3,0,0,0)
if args.old_natnet:
    natnet_version = (2,9,0,0)
natnet = NatNetClient(
        server=args.server,
        rigidBodyListListener=receiveRigidBodyList,
        dataPort=args.data_port,
        commandPort=args.command_port,
        verbose=args.verbose,
        version=natnet_version)


print("Starting Natnet3.x to Ivy interface at %s" % (args.server))
try:
    # Start up the streaming client.
    # This will run perpetually, and operate on a separate thread.
    natnet.run()
    while True:
        sleep(1)
except (KeyboardInterrupt, SystemExit):
    print("Shutting down ivy and natnet interfaces...")
    natnet.stop()
    ivy.shutdown()
except OSError:
    print("Natnet connection error")
    natnet.stop()
    ivy.stop()
    exit(-1)

