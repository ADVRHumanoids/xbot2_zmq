#include <xbot2/rt_plugin/control_plugin.h>
#include <zmq.hpp>

#include "generic_rx_msg.pb.h"
#include "jointstate.pb.h"
#include "jointcmd.pb.h"
#include "imu.pb.h"

namespace XBot {


class ZmqIO : public ControlPlugin {

public:

    using ControlPlugin::ControlPlugin;

    bool on_initialize() override;
    void run() override;

private:

    void publish_js();
    void publish_imu(ImuSensor::ConstPtr imu);
    void recv_cmd();
    void handle_request_response();

    std::unique_ptr<zmq::context_t> context;
    std::unique_ptr<zmq::socket_t> publisher, subscriber, responder;

    GenericRxMsg rx_msg;
    uint32_t seq = 0;
    JointCommand joint_cmd;
    JointNameMap jmap;

    chrono::steady_clock::time_point cmd_timeout;

};

}