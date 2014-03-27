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

import sys
import os
import argparse
import time
import traceback
import math
import numpy

import threading
from Queue import Queue

import serial
import subprocess
import tempfile
import socket

import wx

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "Printrun"))

from printrun import gcoder
from printrun import gcview
from printrun.gl.libtatlin import actors

com_timeout = 0.25
serial_baudrate = 115200
socket_host = "127.0.0.1"
socket_port = 12421
build_dimensions = [200, 200, 100, 0, 0, 0, 0, 0, 0]

parser = argparse.ArgumentParser(description = "3D printer simulator", add_help = False)
parser.add_argument('--help', help = "Show this help message and quit", action = "store_true")
parser.add_argument('-f', '--fast', help = "Process commands without reallistically waiting", action = "store_true")
parser.add_argument('--speed', type = float, help = "Speed factor (> 1.0 is faster)", default = 1.0)
parser.add_argument('-d', '--debug', help = "Display debug messages", action = "store_true")
parser.add_argument('-s', '--serial', help = "Simulator serial port")
parser.add_argument('-n', '--network', help = "Use networking instead of serial communications", action = "store_true")
parser.add_argument('-h', '--host', help = "Host to bind", default = str(socket_host))
parser.add_argument('-p', '--port', help = "Port to bind", default = str(socket_port))
args = parser.parse_args()
if args.help:
    parser.print_help()
    raise SystemExit
use_serial = not args.network
debug_mode = args.debug
fast_mode = args.fast
serial_port = args.serial
speed_factor = max(args.speed, 0.001)

class MoveUpdater(threading.Thread):
    def __init__(self, parent, gline, totalduration, orig, vec):
        super(MoveUpdater, self).__init__()
        self.parent = parent
        self.gline = gline
        self.totalduration = totalduration
        self.orig = numpy.array(orig)
        self.vec = numpy.array(vec)

    def run(self):
        starttime = time.time()
        timestep = 0.1
        orig = self.orig
        vec = self.vec
        wx.CallAfter(self.parent.add_glmove, self.gline, *(list(orig) + list(orig)))
        while True:
            prop = (time.time() - starttime) / self.totalduration
            if prop > 0.99: break
            wx.CallAfter(self.parent.update_glmove, self.gline, *list(orig + prop * vec))
            time.sleep(timestep)
        wx.CallAfter(self.parent.update_glmove, self.gline, *list(orig + vec))

class PrinterSimulator(object):

    cur_x = 0
    cur_y = 0
    cur_z = 0
    cur_e = 0
    cur_f = 0

    acceleration = 1500.0  # mm/s/s  ASSUMING THE DEFAULT FROM SPRINTER !!!!
    xy_homing_feedrate = 50.
    z_homing_feedrate = 4.

    def __init__(self, path, port, gline_cb = None, debug = False, server = None):
        self.path = path
        self.port = port
        self.stop_threads = False
        self.read_thread = None
        self.process_thread = None
        self.gcoder = None
        self.gline_cb = None
        self.debug = debug
        self.server = server
        self.command_buffer = Queue(20 if not fast_mode else 400)
        self.glframe = None
        self.sd_upload = False

    def log(self, message):
        if self.debug: print "???", message

    def start(self, frame):
        self.gcoder = gcoder.GCode([])
        self.glframe = frame
        self.init_glmodel()
        self.init_glhead()
        self.stop_threads = False
        self.read_thread = threading.Thread(target = self.reader)
        self.read_thread.start()
        self.process_thread = threading.Thread(target = self.processor)
        self.process_thread.start()

    def stop(self):
        self.command_buffer.put(None)
        self.stop_threads = True
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
                moveduration += currenttravel / f
            else:
                moveduration = math.sqrt(2 * distance / self.acceleration)  # probably buggy : not taking actual travel into account
        else:
            moveduration = 0
        if z != self.cur_z:
            distance = abs(self.cur_z - z)
            moveduration += distance / f
        return moveduration / speed_factor

    def process_gline_nong(self, gline):
        # These unbuffered commands must be acked manually
        if gline.command == "M114":
            self.write("ok X:%.02fY:%.02fZ:%.02fE:%.02f Count:" % (self.cur_x, self.cur_y, self.cur_z, self.cur_e))
        elif gline.command == "M105":
            self.write("ok T:100.0/225.0 B:98.0 /110.0 T0:228.0/220.0 T1:150.0/185")
        elif gline.command == "M115":
            self.write("ok PROTOCOL_VERSION:0.1 FIRMWARE_NAME:FiveD FIRMWARE_URL:http%3A//reprap.org MACHINE_TYPE:Mendel EXTRUDER_COUNT:3")
        elif gline.command == "M190":
            time.sleep(10)
            self.write("ok")
        elif gline.command == "M28":
            self.sd_upload = True
            self.write("ok Writing to file")
        elif gline.command == "M29":
            self.sd_upload = False
            self.write("ok")
        else:
            self.write("ok")
        if self.gline_cb:
            self.gline_cb(gline)

    def process_gline(self, gline):
        if not gline.command.startswith("G"):  # unbuffered
            return self.process_gline_nong(gline)
        if self.sd_upload:
            return
        line_duration = 0
        timer = None
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

            if not fast_mode and line_duration > 0.5:
                vec = (new_x - self.cur_x, new_y - self.cur_y, new_z - self.cur_z)
                timer = MoveUpdater(self, gline, line_duration, (self.cur_x, self.cur_y, self.cur_z), vec)
            else:
                wx.CallAfter(self.add_glmove, gline, self.cur_x, self.cur_y, self.cur_z, new_x, new_y, new_z)
            self.cur_x = new_x
            self.cur_y = new_y
            self.cur_z = new_z
            self.cur_e = new_e
            self.cur_f = new_f
        elif gline.command == "G4":
            line_duration = gcoder.P(gline)
        elif gline.command == "G28":
            new_x = 0 if "X" in gline.raw else self.cur_x
            new_y = 0 if "Y" in gline.raw else self.cur_y
            new_z = 0 if "Z" in gline.raw else self.cur_z
            line_duration = self.compute_duration(new_x, new_y, self.cur_z, self.xy_homing_feedrate)
            line_duration += self.compute_duration(self.cur_x, self.cur_y, new_z, self.z_homing_feedrate)
            self.cur_x = new_x
            self.cur_y = new_y
            self.cur_z = new_z
            wx.CallAfter(self.move_head, gline, self.cur_x, self.cur_y, self.cur_z)
        elif gline.command == "G92":
            if gline.x is not None: self.cur_x = gline.x
            if gline.y is not None: self.cur_y = gline.y
            if gline.z is not None: self.cur_z = gline.z
            if gline.e is not None: self.cur_e = gline.e
            wx.CallAfter(self.move_head, gline, self.cur_x, self.cur_y, self.cur_z)
        if not fast_mode and line_duration and line_duration > 0:
            self.log("sleeping for %ss" % line_duration)
            if timer: timer.start()
            time.sleep(line_duration)
            if timer: timer.join()
        if self.gline_cb:
            self.gline_cb(gline)

    def processor(self):
        while not self.stop_threads or not self.command_buffer.empty():
            gline = self.command_buffer.get()
            if gline is None:
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
        try:
            self.port.write(data + "\n")
            self.port.flush()
        except socket.error:
            pass  # Don't do anything : reader thread will pick it up

    def reader(self):
        print "Simulator listening on %s" % self.path
        while not self.stop_threads:
            if not self.port and self.server:
                try:
                    self.conn, self.remote_addr = self.server.accept()
                    print "TCP connection from %s:%s" % self.remote_addr
                    self.conn.settimeout(com_timeout)
                    self.port = self.conn.makefile()
                except socket.timeout:
                    continue
            try:
                line = self.port.readline()
            except socket.timeout:
                continue
            except socket.error:
                line = ""
            if self.server and not line:  # empty line returned from the socket: this is EOF
                print "Lost connection from %s:%s" % self.remote_addr
                self.port = None
                continue
            line = line.strip()
            if not line:
                continue
            if self.debug: print "<<<", line
            try:
                gline = self.gcoder.append(line)
                if not gline.command.startswith("G"):  # unbuffered (okai, all G-commands are not buffered, but that's a light move from reality)
                    while not self.command_buffer.empty():
                        time.sleep(0.05)
                    self.command_buffer.put(gline)
                    while not self.command_buffer.empty():
                        time.sleep(0.05)
                else:  # buffered
                    self.command_buffer.put(gline)
                    self.write("ok")
            except ValueError:
                # Failed to parse the G-Code, probably a custom command
                self.write("ok")

    def init_glmodel(self):
        self.glmodel = actors.GcodeModelLight()
        self.glmodel.load_data(self.gcoder)
        self.glmodel.nvertices = 0
        self.glmodel.layer_stops[-1] = self.glmodel.nvertices
        self.glmodel.num_layers_to_draw = self.glmodel.max_layers + 1
        self.glmodel.use_vbos = False
        self.glmodel.loaded = True
        self.glmodel.initialized = False
        self.glframe.objects[-1].model = self.glmodel
        self.refresh_timer = wx.CallLater(100, self.glframe.Refresh)

    def init_glhead(self):
        self.printhead = gcview.GCObject(actors.PrintHead())
        self.glframe.objects.insert(1, self.printhead)

    def move_head(self, gline, cur_x, cur_y, cur_z):
        self.printhead.offsets[0] = cur_x
        self.printhead.offsets[1] = cur_y
        self.printhead.offsets[2] = cur_z
        if not self.refresh_timer.IsRunning():
            self.refresh_timer.Start()

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
        self.move_head(gline, cur_x, cur_y, cur_z)

    def update_glmove(self, gline, cur_x, cur_y, cur_z):
        self.glmodel.vertices[self.glmodel.nvertices - 1] = (cur_x, cur_y, cur_z)
        self.glmodel.initialized = False
        self.move_head(gline, cur_x, cur_y, cur_z)

if use_serial:
    privend = tempfile.mktemp(prefix = "simpriv_", dir = os.getcwd())
    if serial_port:
        pubend = serial_port
    else:
        pubend = tempfile.mktemp(prefix = "printer_", dir = os.getcwd())
    socat_p = subprocess.Popen(["socat", "PTY,link=%s" % privend, "PTY,link=%s" % pubend])
    while not os.path.exists(privend) or not os.path.exists(pubend):
        time.sleep(0.1)
    sport = serial.Serial(privend, baudrate = serial_baudrate, timeout = com_timeout)
    simulator = PrinterSimulator(pubend, sport, debug = debug_mode)
else:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((socket_host, socket_port))
    sock.settimeout(com_timeout)
    sock.listen(1)
    simulator = PrinterSimulator("%s:%s" % (socket_host, socket_port), None, server = sock, debug = debug_mode)

app = wx.App(redirect = False)
frame = gcview.GcodeViewFrame(None, wx.ID_ANY, '3D printer simulator', size = (400, 400), build_dimensions = build_dimensions)
frame.Bind(wx.EVT_CLOSE, lambda event: frame.Destroy())
frame.Show(True)
simulator.start(frame)
app.MainLoop()

simulator.stop()

app.Destroy()

if use_serial:
    socat_p.terminate()
