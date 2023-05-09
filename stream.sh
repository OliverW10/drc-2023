#! /usr/bin/bash
rm ./*.ts
rm playlist.m3u8
gst-launch-1.0 v4l2src ! videoconvert ! x264enc tune=zerolatency ! hlssink2 playlist-root=http://localhost:8000 location=/home/oliver/car/colors-test/segment_%06d.ts target-duration=1 max-files=5 &
python3 -m http.server