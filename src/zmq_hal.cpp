#include "zmq_hal.h"
#include <yaml-cpp/yaml.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

#include <xbot2_interface/xbotinterface2.h>

XBot::Hal::ZmqDeviceContainer::ZmqDeviceContainer(std::vector<DeviceInfo> devinfo,
                                                  const Device::CommonParams &params)
    : DeviceContainerBase()
{
    Journal j("zmq_hal");

    auto& pm = Context().paramManager();

    _socket_fd = socket(AF_UNIX, SOCK_DGRAM, 0);
    if(_socket_fd == -1)
    {
        throw DeviceUnavailable("Error opening socket: " + std::string(strerror(errno)));
    }

    // setup remote address
    memset(&_socket_remote_addr, 0, sizeof (_socket_remote_addr));
    _socket_remote_addr.sun_family = AF_UNIX;
    std::string remote_name = "/tmp/.xbot2_isaac/xbot2_isaac_server.sock";
    pm.getParam("/xbot/hal/zmq_hal/remote_sock_addr", remote_name);
    strncpy(_socket_remote_addr.sun_path,
            remote_name.c_str(),
            remote_name.size());

    // setup local address
    memset(&_socket_local_addr, 0, sizeof (_socket_local_addr));
    _socket_local_addr.sun_family = AF_UNIX;
    std::string local_name = remote_name + ".client." + std::to_string(getpid());
    strncpy(_socket_local_addr.sun_path,
            local_name.c_str(),
            sizeof(_socket_local_addr.sun_path));

    if(local_name.size() > sizeof(_socket_local_addr.sun_path))
    {
        throw std::out_of_range("Local socket name too long ('" + local_name + "')");
    }

    // bind local address
    unlink(local_name.c_str());
    int bind_ret = bind(_socket_fd,
                        (struct sockaddr*)&_socket_local_addr,
                        local_name.size());
    if(bind_ret == -1)
    {
        throw std::runtime_error("Error binding local socket '" + local_name + "': " + std::string(strerror(errno)));
    }


    // discovery
    YAML::Node discovery_msg;
    discovery_msg["type"] = "discovery";
    YAML::Emitter out;
    out << discovery_msg;
    out.SetMapFormat(YAML::Flow);
    while(true)
    {
        try
        {
            if(send_string(out.c_str()))
            {
                break;
            }
        }
        catch(std::runtime_error& e)
        {
            j.jwarn("discovery send failed: {}", e.what());
        }

        j.jinfo("waiting for xbot2 isaac server on socket '{}'", remote_name);
        usleep(666000);
    }

    j.jinfo("sent discovery message to xbot2 isaac serve, waiting for reply...");

    // wait for response
    std::string response_str(40960, '\0');
    recv_string(response_str);
    auto response = YAML::Load(response_str);
    j.jinfo("...got response");


    // get urdf
    auto urdf_str = response["urdf"].as<std::string>();
    XBot::ConfigOptions xb_ifc_cfg;
    xb_ifc_cfg.set_urdf(urdf_str);
    xb_ifc_cfg.set_srdf("<robot name=\"robot\"/>");


    // change framework to xbot2rt
    xb_ifc_cfg.set_parameter<std::string>("robot_type", "xbot2rt");

    // load robot interface cfg object to param manager
    pm.setParam("/xbot/hal/robot_ifc_cfg", xb_ifc_cfg);

    // make an xbi to get info about hal
    auto xbi = ModelInterface::getModel(xb_ifc_cfg);
    xbi->print(std::cout);

    // upload urdf to internal params (used for safety limits)
    pm.setParam("/xbot/robot_description", xbi->getUrdfString());
    pm.setParam<urdf::ModelInterface>("/xbot/urdf_model", *xbi->getUrdf());

    // construct joints
    auto joint_names = response["joint_names"].as<std::vector<std::string>>();

    for(auto jname : joint_names)
    {
        DeviceInfo dinfo;
        dinfo.name = jname;
        dinfo.type = "joint_zmq";
        dinfo.id = -1;
        auto dev = std::make_shared<JointDriver>(dinfo, params);
        addDevice(dev);
        _joints.push_back(dev);
        j.jinfo("added joint '{}'", jname);
    }

}

bool XBot::Hal::ZmqDeviceContainer::sense_all()
{

    // wait for message
    std::string response_str(40960, '\0');

    if(!recv_string(response_str, false))
    {
        return false;
    }

    while(recv_string(response_str, false))
    {
        // keep only most recent
    }

    auto response = YAML::Load(response_str);

    auto type = response["type"].as<std::string>();

    if(type == "state")
    {
        auto q = response["q"].as<std::vector<double>>();
        auto dq = response["dq"].as<std::vector<double>>();
        auto tau = response["tau"].as<std::vector<double>>();
        auto k = response["k"].as<std::vector<double>>();
        auto d = response["d"].as<std::vector<double>>();
        auto qref = response["qref"].as<std::vector<double>>();
        auto vref = response["vref"].as<std::vector<double>>();
        auto tauref = response["tauref"].as<std::vector<double>>();

        for(int i = 0; i < _joints.size(); i++)
        {
            auto& rx = _joints[i]->rx();
            rx.link_pos = rx.motor_pos = q[i];
            rx.link_vel = rx.motor_vel = dq[i];
            rx.torque = tau[i];
            rx.gain_kp = k[i];
            rx.gain_kd = d[i];
            rx.pos_ref = qref[i];
            rx.vel_ref = vref[i];
            rx.tor_ref = tauref[i];
        }
    }

    DeviceContainerBase::sense_all();

    return true;
}

void XBot::Hal::ZmqDeviceContainer::run_all()
{

}

bool XBot::Hal::ZmqDeviceContainer::move_all()
{
    DeviceContainerBase::move_all();

    // prepare command message
    YAML::Node command_msg;
    command_msg["type"] = "control";

    auto q = YAML::Node(YAML::NodeType::Sequence);
    command_msg["q"] = q;

    auto dq = YAML::Node(YAML::NodeType::Sequence);
    command_msg["dq"] = dq;

    for(auto& j : _joints)
    {
        auto& tx = j->tx();
        q.push_back(tx.pos_ref);
        dq.push_back(tx.vel_ref);
    }

    YAML::Emitter out;
    out << command_msg;
    out.SetMapFormat(YAML::Flow);
    out.SetDoublePrecision(4);
    out.SetFloatPrecision(4);
    if(!send_string(out.c_str()))
    {
        return false;
    }

    return true;
}

bool XBot::Hal::ZmqDeviceContainer::send_string(const std::string &msg)
{
    int ret = sendto(_socket_fd,
               msg.data(), msg.size(), // do not send null termination
               0,
               reinterpret_cast<const sockaddr*>(&_socket_remote_addr),
               sizeof(_socket_remote_addr));

    if(ret < 0)
    {
        throw std::runtime_error("Error sendto: " + std::string(strerror(errno)));
    }

    return ret == msg.size();
}

bool XBot::Hal::ZmqDeviceContainer::recv_string(std::string &msg, bool blocking)
{
    // msg.resize(4096);
    int ret = recvfrom(_socket_fd,
                       msg.data(), msg.size(),
                       blocking ? 0 : MSG_DONTWAIT,
                       nullptr, nullptr);
    if(ret > 0)
    {
        msg.resize(ret);
        return true;
    }
    else
    {
        return false;
    }
}



XBot::Hal::ZmqClientContainer::ZmqClientContainer(std::vector<DeviceInfo> devinfo,
                                                  const Device::CommonParams &params):
    DeviceContainer(devinfo, params)
{

}


XBot::Hal::JointDriver::JointDriver(DeviceInfo dinfo, const CommonParams &params)
    :
    DeviceDriverTpl<joint_rx, joint_tx>(dinfo, params),
    Journal(Journal::no_publish, dinfo.name),
    _safety(dinfo, get_period_sec(), JointSafety::safety_not_required)
{
    // declare available resources
    uint32_t mask = ~0;
    uint8_t _pos_mask = JointBase::Resource::Mask::Position;
    uint8_t _vel_mask = JointBase::Resource::Mask::Velocity;
    uint8_t _tor_mask = JointBase::Resource::Mask::Effort;
    uint8_t _imp_mask = JointBase::Resource::Mask::Impedance;


    if(mask & _pos_mask)
    {
        XBOT2_ASSERT_THROW(
            register_resource(JointBase::Resource::Position, _pos_mask)
            );
    }

    if(mask & _vel_mask)
    {
        XBOT2_ASSERT_THROW(
            register_resource(JointBase::Resource::Velocity, _vel_mask)
            );
    }

    if(mask & _tor_mask)
    {
        XBOT2_ASSERT_THROW(
            register_resource(JointBase::Resource::Effort, _tor_mask)
            );
    }

    if(mask & _imp_mask)
    {
        XBOT2_ASSERT_THROW(
            register_resource(JointBase::Resource::Impedance, _imp_mask)
            );

        XBOT2_ASSERT_THROW(
            register_resource(JointBase::Resource::Stiffness,
                              JointBase::Resource::Mask::Stiffness)
            );

        XBOT2_ASSERT_THROW(
            register_resource(JointBase::Resource::Damping,
                              JointBase::Resource::Mask::Damping)
            );
    }

    // customize safety reactions
    _safety.set_on_safety_triggered([this](joint_tx& tx)
                                    {
                                        if(_rx.gain_kp < _safe_kp)
                                        {
                                            tx.gain_kp = _safe_kp;
                                            tx.gain_kd = _safe_kd;
                                            tx.pos_ref = _rx.motor_pos;
                                            jwarn("setting safe impedance: kp = {}  kd = {}",
                                                     _safe_kp, _safe_kd);
                                        }
                                    });
}

bool XBot::Hal::JointDriver::sense_impl()
{
    if(!_init_done)
    {
        // initialize tx and safety from received rx
        _safety.initialize(_rx);
        _tx_tmp.reset(_rx);

        // safe gains are defined as the first received gains
        // this is reasonable at least in simulation
        _safe_kp = _rx.gain_kp;
        _safe_kd = _rx.gain_kd;

        _init_done = true;
    }

    return true;
}

bool XBot::Hal::JointDriver::move_impl()
{
    // turn _tmp_tx into a safe tx from safety filter,
    // and save it to _tx

    if(!_safety.enforce(_tx_tmp, _tx))
    {
        // note: returning false means "unable to
        // communicate with the robot"
        // so, we don't

        // reset _tx_tmp (which is unsafe) to be safe
        _tx_tmp = _tx;
    }

    // note: reset mask before next tx msg received
    _tx_tmp.mask = 0;

    return true;
}

void XBot::Hal::JointDriver::on_tx_recv(const TxType &msg)
{
    _tx_tmp.apply(msg);
}


XBOT2_REGISTER_DEVICE(XBot::Hal::ZmqDeviceContainer, XBot::Hal::ZmqClientContainer, zmq_hal)
