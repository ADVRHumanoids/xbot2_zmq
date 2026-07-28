# XBot2_zmq & Pyxbot

This repo provides the tools to communicate with XBot without using ROS. This is done through two components:
* **zmq_io**: a XBot2 plugin which exposes a ZMQ-based robot API
* **[pyxbot](pyxbot/README.md)**: a python package providing a friendly python interface for zmq_io


The underlying communication happens over 3 different channels:
* The current robot state (Joint States and IMU) is streamed over a raw bytes connection
* Joint commands are received over a second raw bytes connection
* A third connection is dedicated to a reply/request channel where info is sent as YAML messages


## Python interface

A python interface is available as a pip-installable package in the pyxbot subfolder.
You can install it with:

```
uv pip install git+https://github.com/ADVRHumanoids/xbot2_zmq.git@crzz-dev#subdirectory=pyxbot
```

You can find more info in the [README](pyxbot/README.md)


## How to load the XBot plugin

The XBot plugin name is zmq_io.
After building and installing it you can load it in XBot by adding it to the config as follows.

```yaml
zmq_io:
  type: zmq_io
  thread: nrt_main
  params:
    autostart: true
    protocol: tcp # either tcp or ipc
    tcp_state_port: 5559
    tcp_cmd_port: 5558
    tcp_service_port: 5557 
    ipc_state_port: /tmp/xbot2_zmq_pub.ipc
    ipc_cmd_port: /tmp/xbot2_zmq_cmd.ipc
    ipc_service_port: /tmp/xbot2_zmq_rep.ipc
    
```

## Installation

The XBot plugin can be built with CMake.
You can also use the standalone build script [build_and_install_systemwide.sh](build_and_install_systemwide.sh) to build and install, you can pass a
path with --install-dir to specify the installation folder (e.g. a forest install directory).


