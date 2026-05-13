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
    protocol: ipc # either tcp or ipc
    tcp_state_port: 5559
    tcp_cmd_port: 5558
    tcp_service_port: 5557 
    ipc_state_port: /tmp/xbot2_zmq_pub.ipc
    ipc_cmd_port: /tmp/xbot2_zmq_cmd.ipc
    ipc_service_port: /tmp/xbot2_zmq_rep.ipc
    
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


