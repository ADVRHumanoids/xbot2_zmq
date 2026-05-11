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
    raw_pub_bind_addr: tcp://*:5559
    cmd_sub_addr: tcp://*:5558
    rep_bind_addr: tcp://*:5557
```


## Python interface

A python interface is available as a pip-installable package in the folder pyxbot.
You can install it with:

```
uv pip install git+https://github.com/ADVRHumanoids/xbot2_zmq.git@crzz-dev#subdirectory=pyxbot
```

You can find more info in the package [README](pyxbot/README.md)


## Installation

You can use the build script build_and_install_systemwide.sh to build and install, you can pass a
path with --install-dir to specify the installation folder.


