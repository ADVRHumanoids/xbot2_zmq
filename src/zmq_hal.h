#ifndef XBOT2_ZMQ_HAL_H
#define XBOT2_ZMQ_HAL_H

#include <xbot2/hal/device.h>
#include <xbot2/hal/dev_joint.h>
#include <xbot2/ipc/pipe.h>
#include <sys/un.h>

namespace XBot {
namespace Hal {

class JointDriver : public DeviceDriverTpl<joint_rx, joint_tx>,
                    private Journal
{

public:

    XBOT2_DECLARE_SMART_PTR(JointDriver)

    JointDriver(DeviceInfo dinfo, const Device::CommonParams& params);



private:

    JointSafety _safety;


    // DeviceDriverTpl interface
private:

    bool sense_impl() override;
    bool move_impl() override;
    void on_tx_recv(const TxType &msg) override;

    bool _init_done = false;
    TxType _tx_tmp;
    double _safe_kp, _safe_kd;
};

class ZmqDeviceContainer : public DeviceContainerBase
{

public:

    ZmqDeviceContainer(std::vector<DeviceInfo> devinfo,
                       const Device::CommonParams& params);


    bool sense_all() override;
    void run_all() override;
    bool move_all() override;

    bool send_string(const std::string& msg);
    bool recv_string(std::string& msg, bool blocking = true);

private:

    // socket
    int _socket_fd;
    sockaddr_un _socket_local_addr;
    sockaddr_un _socket_remote_addr;

    // devs
    std::vector<JointDriver::Ptr> _joints;


};

class ZmqClientContainer : public DeviceContainer<JointClient>
{

public:

    ZmqClientContainer(std::vector<DeviceInfo> devinfo,
                       const Device::CommonParams& params);

};

}
}


#endif // ZMQ_HAL_H
