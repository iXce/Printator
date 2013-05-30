Printator is a simple 3D printer simulator based on a subpart of Printrun.

At the time of writing this, the simulator in itself fits in 232 lines of
Python code (as reported by sloccount). Most of the non-trivial things come
from Printrun's gcoder and gcview modules, which respectively provide gcode
parsing and the 3D viewer.

# Dependencies

To use Printator, you will need:

  * python (ideally 2.6.x or 2.7.x),
  * pyserial (or python-serial on ubuntu/debian)
  * pyglet
  * argparse (installed by default with python >= 2.7)
  * wxPython
  * socat (external binary dependency, used to create virtual serial ports)

# Initial setup

You first need to fetch Printrun through the `git submodule` mechanism:

    git submodule init
    git submodule update

Once this is done, you will just need to rerun the second command to update
Printrun to the latest version when you want to.

# Running

Running Printator is as easy as

    python2 printator.py

It will create a serial port with a random file name and start the simulator.

You can specify the serial port path with the `-s` option:

    python2 printator.py -s `pwd`/sim

You can also disable delays (i.e. process gcode as fast as possible, without
realistically waiting) using the `-f` option.

# Known issues

- Custom baudrates are not supported for serial-based communications. TCP
  support shouldn't have any speed limit, though.

# Planned features

- Checksum/line number verification
- Computing an STL file from the print results
- Temperature support
