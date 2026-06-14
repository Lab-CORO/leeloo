#!/bin/bash
# The Azure Kinect depth engine requires an OpenGL 4.4 context via GLX.
# On headless Jetson (JetPack 6.x, no OpenCL), it uses Mesa LLVMpipe via Xvfb.
#
# Strategy: always start an internal Xvfb on :99 for the depth engine.
# If the host DISPLAY socket is not reachable, redirect everything to :99.

Xvfb :99 -screen 0 1280x720x24 +extension GLX &>/dev/null &
export DISPLAY=:99
export LIBGL_ALWAYS_SOFTWARE=1

exec "$@"
