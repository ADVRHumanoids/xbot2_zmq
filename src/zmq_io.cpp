#include "zmq_io.h"

using namespace XBot;

bool ZmqIO::on_initialize() 
{
    // bind publisher
    auto pub_bind_addr = "tcp://*:5556";
    getParam("~pub_bind_addr", pub_bind_addr);
    jinfo("Binding PUB socket to {}", pub_bind_addr);

    context = std::make_unique<zmq::context_t>(1);
    publisher = std::make_unique<zmq::socket_t>(*context, ZMQ_PUB);
    publisher->bind(pub_bind_addr);

    // bind command subscriber
    auto cmd_sub_addr = "tcp://*:5558";
    getParam("~cmd_sub_addr", cmd_sub_addr);
    jinfo("Binding CMD socket to {}", cmd_sub_addr);
    subscriber = std::make_unique<zmq::socket_t>(*context, ZMQ_SUB);
    subscriber->bind(cmd_sub_addr);
    subscriber->set(zmq::sockopt::subscribe, "");
    subscriber->set(zmq::sockopt::conflate, 1);

    // add REP socket for request/response
    auto rep_bind_addr = "tcp://*:5557";
    getParam("~rep_bind_addr", rep_bind_addr);
    jinfo("Binding REP socket to {}", rep_bind_addr);
    responder = std::make_unique<zmq::socket_t>(*context, ZMQ_REP);
    responder->bind(rep_bind_addr);

    return true;
}

void ZmqIO::publish_js()
{
    _robot->sense(false);
    Eigen::VectorXd buffer, buffer2;
    auto js_msg = rx_msg.mutable_js();
    
    _robot->getJointPosition(buffer2);
    _robot->positionToMinimal(buffer2, buffer);
    auto* repeated_field = js_msg->mutable_linkpos();
    repeated_field->Clear();
    repeated_field->Add(buffer.data(), buffer.data() + buffer.size());
    
    _robot->getPositionReferenceFeedback(buffer2);
    _robot->positionToMinimal(buffer2, buffer);
    repeated_field = js_msg->mutable_posref();
    repeated_field->Clear();
    repeated_field->Add(buffer.data(), buffer.data() + buffer.size());
    
    _robot->getMotorVelocity(buffer);
    repeated_field = js_msg->mutable_motvel();
    repeated_field->Clear();
    repeated_field->Add(buffer.data(), buffer.data() + buffer.size());
    
    _robot->getJointEffort(buffer);
    repeated_field = js_msg->mutable_tor();
    repeated_field->Clear();
    repeated_field->Add(buffer.data(), buffer.data() + buffer.size());
    
    rx_msg.set_seq(seq++);
    rx_msg.set_stamp(chrono::wall_clock::now().time_since_epoch().count() * 1e-9);
        
    std::string out;
    rx_msg.SerializeToString(&out);
    publisher->send(zmq::buffer(out), zmq::send_flags::none);
}

void ZmqIO::publish_imu(ImuSensor::ConstPtr imu)
{
    Eigen::Vector3d buffer;
    auto imu_msg = rx_msg.mutable_imu();

    imu_msg->set_name(imu->getName());

    imu->getLinearAcceleration(buffer);
    imu_msg->set_linear_acceleration_x(buffer(0));
    imu_msg->set_linear_acceleration_y(buffer(1));
    imu_msg->set_linear_acceleration_z(buffer(2));

    imu->getAngularVelocity(buffer);
    imu_msg->set_angular_velocity_x(buffer(0));
    imu_msg->set_angular_velocity_y(buffer(1));
    imu_msg->set_angular_velocity_z(buffer(2));

    Eigen::Quaterniond qbuffer;
    imu->getOrientation(qbuffer);
    imu_msg->set_orientation_x(qbuffer.x());
    imu_msg->set_orientation_y(qbuffer.y());
    imu_msg->set_orientation_z(qbuffer.z());
    imu_msg->set_orientation_w(qbuffer.w());

    rx_msg.set_seq(seq++);
    rx_msg.set_stamp(chrono::wall_clock::now().time_since_epoch().count());

    std::string out;
    rx_msg.SerializeToString(&out);
    publisher->send(zmq::buffer(out), zmq::send_flags::none);
}

void ZmqIO::handle_request_response()
{
    zmq::message_t request;

    if (responder->recv(request, zmq::recv_flags::dontwait)) 
    {
        // request string
        std::string req_str(static_cast<char*>(request.data()), request.size());

        // parse it as yaml
        YAML::Node req_yaml = YAML::Load(req_str);

        // prepare response
        YAML::Node resp_yaml;
        resp_yaml["success"] = false;
        resp_yaml["message"] = "";

        // get mandatory 'type' field from request
        std::string req_type;
        if(req_yaml["type"] && req_yaml["type"].IsScalar()) 
        {
            req_type = req_yaml["type"].as<std::string>();
        }
        else 
        {
            req_type = "";
        }

        if(req_type.empty()) 
        {
            jerror("missing 'type' field in request");
            resp_yaml["message"] = "missing 'type' field in request";
        }
        else if (req_type == "urdf") {
            resp_yaml["success"] = true;
            resp_yaml["data"] = _robot->getUrdfString();
        }
        else if(req_type == "srdf") {
            resp_yaml["success"] = true;
            resp_yaml["data"] = _robot->getSrdfString();
        }
        else if(req_type == "joint_names") {
            resp_yaml["success"] = true;
            resp_yaml["data"] = _robot->getJointNames();
        }
        else if(req_type == "set_filter_frequency_hz")
        {
            bool enabled = false;
            double cutoff_hz = 0.0;
            if(req_yaml["enabled"] && req_yaml["enabled"].IsScalar()) {
                enabled = req_yaml["enabled"].as<bool>();
            } else {
                resp_yaml["message"] = "missing or invalid 'enabled' field";
                responder->send(zmq::buffer(YAML::Dump(resp_yaml)), zmq::send_flags::none);
                return;
            }
            if(req_yaml["cutoff_hz"] && req_yaml["cutoff_hz"].IsScalar()) {
                cutoff_hz = req_yaml["cutoff_hz"].as<double>();
            } else {
                resp_yaml["message"] = "missing or invalid 'cutoff_hz' field";
                responder->send(zmq::buffer(YAML::Dump(resp_yaml)), zmq::send_flags::none);
                return;
            }
            resp_yaml["success"] = Hal::JointSafety::enable_filter(enabled, cutoff_hz);
        }
        else if(req_type == "start_plugin")
        {
            std::string plugin_name;
            if(req_yaml["plugin"] && req_yaml["plugin"].IsScalar()) {
                plugin_name = req_yaml["plugin"].as<std::string>();
            } else {
                resp_yaml["message"] = "missing or invalid 'plugin' field";
                responder->send(zmq::buffer(YAML::Dump(resp_yaml)), zmq::send_flags::none);
                return;
            }
            resp_yaml["success"] = sendCommand(plugin_name, Runnable::Command::Start);
        }
        else {
            jerror("unknown request type: {}", req_type);
            resp_yaml["message"] = "unknown request type: " + req_type;
        }
        std::string resp_str = YAML::Dump(resp_yaml);
        responder->send(zmq::buffer(resp_str), zmq::send_flags::none);
    }
}

void ZmqIO::recv_cmd()
{
    // currently not used
    zmq::message_t cmd;

    if (subscriber->recv(cmd, zmq::recv_flags::dontwait)) 
    {
        // consume any pending messages, keep only the last one
        while(subscriber->recv(cmd, zmq::recv_flags::dontwait));

        std::string cmd_str(static_cast<char*>(cmd.data()), cmd.size());
        rx_msg.ParseFromString(cmd_str);
        joint_cmd = rx_msg.cmd();

        for (int i = 0; i < joint_cmd.name_size(); ++i) {
            
            const std::string& joint_name = joint_cmd.name(i);
            
            auto j = _robot->getJoint(joint_name);
            
            if(!j) 
            {
                jerror("unknown joint '{}'", joint_name);
                continue;
            }
            
            if(joint_cmd.posref_size() != joint_cmd.name_size() ||
               joint_cmd.velref_size() != joint_cmd.name_size() ||
               joint_cmd.torref_size() != joint_cmd.name_size() ||
               joint_cmd.k_size() != joint_cmd.name_size() ||
               joint_cmd.d_size() != joint_cmd.name_size() ||
               joint_cmd.ctrl_size() != joint_cmd.name_size()) 
            {
                jerror("inconsistent command sizes");
                jinfo("name_size: {}, posref_size: {}, velref_size: {}, torref_size: {}, k_size: {}, d_size: {}, ctrl_size: {}",
                    joint_cmd.name_size(),
                    joint_cmd.posref_size(),
                    joint_cmd.velref_size(),
                    joint_cmd.torref_size(),
                    joint_cmd.k_size(),
                    joint_cmd.d_size(),
                    joint_cmd.ctrl_size());
                break;
            }

            j->setPositionReferenceMinimal(Eigen::Scalard(joint_cmd.posref(i)));
            j->setVelocityReference(Eigen::Scalard(joint_cmd.velref(i)));
            j->setEffortReference(Eigen::Scalard(joint_cmd.torref(i)));
            j->setStiffness(Eigen::Scalard(joint_cmd.k(i)));
            j->setDamping(Eigen::Scalard(joint_cmd.d(i)));  
            j->setControlMode(static_cast<ControlMode::Type>(joint_cmd.ctrl(i)));
        }

        cmd_timeout = chrono::steady_clock::now() + 1s;

        _robot->move();
    }

    // if no command received for 1s, release all control mode
    if(cmd_timeout.time_since_epoch().count() != 0 && 
        chrono::steady_clock::now() > cmd_timeout) 
    {
        jinfo("timeout expired, releasing resources");
        _robot->releaseResources();
        cmd_timeout = decltype(cmd_timeout)();
    }

}

void ZmqIO::run()
{
    recv_cmd();

    rx_msg.Clear();

    publish_js();

    for(auto [name, imu] : _robot->getImu()) 
    {
        publish_imu(imu);
    }

    std::string out;
    rx_msg.set_seq(seq++);
    rx_msg.SerializeToString(&out);
    publisher->send(zmq::buffer(out), zmq::send_flags::none);

    handle_request_response();
}



XBOT2_REGISTER_PLUGIN(ZmqIO, zmq_io)
