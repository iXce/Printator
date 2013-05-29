#!/usr/bin/env python

# This file is copied from Printator.
#
# Printator is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# Printator is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with Printator.  If not, see <http://www.gnu.org/licenses/>.

from Printrun.printrun import gcoder
from Printrun.printrun import gcview
from Printrun.printrun.libtatlin import actors

import sys
import os
import time
import traceback
import math

import threading
from Queue import Queue

import serial
import subprocess
import tempfile

import socket

import wx

import argparse

serial_baudrate = 115200
socket_host = "127.0.0.1"
socket_port = 12421
build_dimensions = [200, 200, 100, 0, 0, 0, 0, 0, 0]

parser = argparse.ArgumentParser(description = "3D printer simulator", add_help = False)
parser.add_argument('--help', help = "Show this help message and quit", action = "store_true")
parser.add_argument('-f', '--fast', help = "Process commands without reallistically waiting", action = "store_true")
parser.add_argument('-s', '--serial', help = "Simulator serial port")
parser.add_argument('-n', '--network', help = "Use networking instead of serial communications", action = "store_true")
parser.add_argument('-h', '--host', help = "Host to bind", default = str(socket_host))
parser.add_argument('-p', '--port', help = "Port to bind", default = str(socket_port))
args = parser.parse_args()
if args.help:
    parser.print_help()
    raise SystemExit
use_serial = not args.network
fast_mode = args.fast
serial_port = args.serial

class PrinterSimulator(object):

    cur_x = 0
    cur_y = 0
    cur_z = 0
    cur_e = 0
    cur_f = 0

    acceleration = 1500.0 #mm/s/s  ASSUMING THE DEFAULT FROM SPRINTER !!!!
    xy_homing_feedrate = 50.
    z_homing_feedrate = 4.

    def __init__(self, path, port, gline_cb = None, debug = False):
        self.path = path
        self.port = port
        self.stop_threads = False
        self.read_thread = None
        self.process_thread = None
        self.gcoder = None
        self.gline_cb = None
        self.debug = debug
        self.command_buffer = Queue(20 if not fast_mode else 400)
        self.glframe = None

    def log(self, message):
        if self.debug: print "???", message

    def start(self, frame):
        self.gcoder = gcoder.GCode([])
        self.glframe = frame
        self.init_glmodel()
        self.stop_threads = False
        self.read_thread = threading.Thread(target = self.reader)
        self.read_thread.start()
        self.process_thread = threading.Thread(target = self.processor)
        self.process_thread.start()

    def stop(self):
        self.stop_threads = True
        self.command_buffer.put(None)
        if self.read_thread:
            self.read_thread.join()
            self.read_thread = None
        if self.process_thread:
            self.process_thread.join()
            self.process_thread = None

    def compute_duration(self, x, y, z, f):
        currenttravel = math.hypot(x - self.cur_x, y - self.cur_y)
        if currenttravel > 0:
            distance = 2 * abs(((self.cur_f + f) * (f - self.cur_f) * 0.5) / self.acceleration)  # multiply by 2 because we have to accelerate and decelerate
            if distance <= currenttravel and self.cur_f + f != 0 and f != 0:
                # Unsure about this formula -- iXce reviewing this code
                moveduration = 2 * distance / (self.cur_f + f)
                currenttravel -= distance
                moveduration += currenttravel/f
            else:
                moveduration = math.sqrt(2 * distance / self.acceleration) # probably buggy : not taking actual travel into account
        else:
            moveduration = 0
        if z != self.cur_z:
            distance = abs(self.cur_z - z)
            moveduration += distance / f
        return moveduration

    def process_gline_nong(self, gline):
        # These unbuffered commands must be acked manually
        if gline.command == "M114":
            self.write("ok X:%.02fY:%.02fZ:%.02fE:%.02f Count:" % (self.cur_x, self.cur_y, self.cur_z, self.cur_e))
        else:
            self.write("ok")
        if self.gline_cb:
            self.gline_cb(gline)

    def process_gline(self, gline):
        if not gline.command.startswith("G"): # unbuffered
            return self.process_gline_nong(gline)
        line_duration = 0
        if gline.is_move:
            new_x = self.cur_x
            new_y = self.cur_y
            new_z = self.cur_z
            new_e = self.cur_e
            new_f = self.cur_f
            if gline.relative:
                if gline.x is not None: new_x += gline.x
                if gline.y is not None: new_y += gline.y
                if gline.z is not None: new_z += gline.z
            else:
                if gline.x is not None: new_x = gline.x
                if gline.y is not None: new_y = gline.y
                if gline.z is not None: new_z = gline.z

            if gline.e is not None:
                if gline.relative_e:
                    new_e += gline.e
                else:
                    new_e = gline.e

            if gline.f is not None: new_f = gline.f / 60.0

            line_duration = self.compute_duration(new_x, new_y, new_z, new_f)

            wx.CallAfter(self.add_glmove, gline, self.cur_x, self.cur_y, self.cur_z, new_x, new_y, new_z)
            self.cur_x = new_x
            self.cur_y = new_y
            self.cur_z = new_z
            self.cur_e = new_e
            self.cur_f = new_f
        elif gline.command == "G4":
            line_duration = gline.p
        elif gline.command == "G28":
            new_x = 0 if "X" in gline.raw else self.cur_x
            new_y = 0 if "Y" in gline.raw else self.cur_y
            new_z = 0 if "Z" in gline.raw else self.cur_z
            line_duration = self.compute_duration(new_x, new_y, self.cur_z, self.xy_homing_feedrate)
            line_duration += self.compute_duration(self.cur_x, self.cur_y, new_z, self.z_homing_feedrate)
            self.cur_x = new_x
            self.cur_y = new_y
            self.cur_z = new_z
        elif gline.command == "G92":
            if gline.x: self.cur_x = gline.x
            if gline.y: self.cur_y = gline.y
            if gline.z: self.cur_z = gline.z
            if gline.e: self.cur_e = gline.e
        if not fast_mode and line_duration and line_duration > 0:
            self.log("sleeping for %ss" % line_duration)
            time.sleep(line_duration)
        if self.gline_cb:
            self.gline_cb(gline)

    def processor(self):
        while not self.stop_threads:
            gline = self.command_buffer.get()
            if gline == None:
                self.command_buffer.task_done()
                return
            try:
                self.process_gline(gline)
            except:
                print "Exception caught while processing command %s" % gline.raw
                traceback.print_exc()
            self.command_buffer.task_done()

    def write(self, data):
        if self.debug: print ">>>", data
        self.port.write(data + "\n")

    def reader(self):
        print "Simulator listening on %s" % self.path
        while not self.stop_threads:
            line = self.port.readline().strip()
            if not line:
                continue
            if self.debug: print "<<<", line
            gline = self.gcoder.append(line)
            if not gline.command.startswith("G"): # unbuffered (okai, all G-commands are not buffered, but that's a light move from reality)
                while not self.command_buffer.empty():
                    time.sleep(0.05)
                self.command_buffer.put(gline)
                while not self.command_buffer.empty():
                    time.sleep(0.05)
            else: # buffered
                self.command_buffer.put(gline)
                self.write("ok")

    def init_glmodel(self):
        self.glmodel = actors.GcodeModel()
        self.glmodel.load_data(self.gcoder)
        self.glmodel.nvertices = 0
        self.glmodel.layer_stops[-1] = self.glmodel.nvertices
        self.glmodel.use_vbos = False
        self.glmodel.loaded = True
        self.glmodel.initialized = False
        self.glframe.objects[-1].model = self.glmodel
        self.refresh_timer = wx.CallLater(100, self.glframe.Refresh)

    def add_glmove(self, gline, prev_x, prev_y, prev_z, cur_x, cur_y, cur_z):
        if self.glmodel.nvertices + 2 > len(self.glmodel.vertices):
            self.glmodel.colors.resize((2 * len(self.glmodel.vertices) + 2, 4))
            self.glmodel.vertices.resize((2 * len(self.glmodel.vertices) + 2, 3))
        self.glmodel.vertices[self.glmodel.nvertices] = (prev_x, prev_y, prev_z)
        self.glmodel.vertices[self.glmodel.nvertices + 1] = (cur_x, cur_y, cur_z)
        color = self.glmodel.movement_color(gline)
        self.glmodel.colors[self.glmodel.nvertices] = color
        self.glmodel.colors[self.glmodel.nvertices + 1] = color
        self.glmodel.nvertices += 2
        self.glmodel.layer_stops[-1] = self.glmodel.nvertices
        self.glmodel.initialized = False
        if not self.refresh_timer.IsRunning():
            self.refresh_timer.Start()

if use_serial:
    privend = tempfile.mktemp(prefix = "simpriv_", dir = os.getcwd())
    if serial_port:
        pubend = serial_port
    else:
        pubend = tempfile.mktemp(prefix = "printer_", dir = os.getcwd())
    socat_p = subprocess.Popen(["socat","PTY,link=%s" % privend, "PTY,link=%s" % pubend])
    while not os.path.exists(privend) or not os.path.exists(pubend):
        time.sleep(0.1)
    sport = serial.Serial(privend, baudrate = serial_baudrate, timeout = 0.25)
    simulator = PrinterSimulator(pubend, sport)
else:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.bind((socket_host, socket_port))
    sock.listen(1)

app = wx.App(redirect = False)
frame = gcview.GcodeViewFrame(None, wx.ID_ANY, '3D printer simulator', size = (400, 400), build_dimensions = build_dimensions)
frame.Show(True)
simulator.start(frame)
app.MainLoop()
app.Destroy()

simulator.stop()

if use_serial:
    socat_p.terminate()
else:
    pass
