#!/bin/bash
echo ' / __\ \ / / __| |  / _ \| \| | __|                     '
echo '| (__ \ V / (__| |_| (_) | .` | _|                      '
echo ' \___|_|_|_\___|____\___/|_|\_|___|___ ___   _   __  __ '
echo '|   \| __| _ ) | | |/ __/ __|_   _| _ \ __| /_\ |  \/  |'
echo '| |) | _|| _ \ |_| | (_ \__ \ | | |   / _| / _ \| |\/| |'
echo '|___/|___|___/\___/ \___|___/ |_| |_|_\___/_/ \_\_|  |_|'

gst-launch-1.0 -vvv udpsrc port=9999 ! "application/x-rtp, media=video, clock-rate=90000,payload=96,a-framerate=30,encoding-name=VP8-DRAFT-IETF-01" ! rtpvp8depay ! vp8dec ! glimagesink
