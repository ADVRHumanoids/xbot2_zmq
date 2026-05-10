# xbot2_zmq - No proto version

A XBot2 plugin exporting a ZMQ-based API.

It communicates over 3 different channels:

* The current robot state (Joint States and IMU) is streamed over a raw bytes connection
* Joint commands are received over a second raw bytes connection
* A third connection is dedicated to a reply/request channel where info is sent as YAML messages


## How to load in XBot
(TBD)
```yaml
zmq_io:
  type: zmq_io
  thread: nrt_main
  parameters:
    autostart: true
```


## Python interface

A python interface is available at [pyxbot](https://github.com/ADVRHumanoids/pyxbot).

