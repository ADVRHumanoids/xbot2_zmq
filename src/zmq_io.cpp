#include "zmq_io.h"

using namespace XBot;


bool ZmqIO::on_initialize() 
{
    // bind publisher
    auto raw_pub_bind_addr = "tcp://*:5559";
    getParam("~raw_pub_bind_addr", raw_pub_bind_addr);
    jinfo("Binding RAW PUB socket to {}", raw_pub_bind_addr);

    context = std::make_unique<zmq::context_t>(1);
    raw_publisher = std::make_unique<zmq::socket_t>(*context, ZMQ_PUB);
    raw_publisher->bind(raw_pub_bind_addr);

    // bind command subscriber
    auto cmd_sub_addr = "tcp://*:5558";
    getParam("~cmd_sub_addr", cmd_sub_addr);
    jinfo("Binding CMD socket to {}", cmd_sub_addr);
    cmd_subscriber = std::make_unique<zmq::socket_t>(*context, ZMQ_SUB);
    cmd_subscriber->bind(cmd_sub_addr);
    cmd_subscriber->set(zmq::sockopt::subscribe, "");
    cmd_subscriber->set(zmq::sockopt::conflate, 1);

    // add REP socket for request/response
    auto rep_bind_addr = "tcp://*:5557";
    getParam("~rep_bind_addr", rep_bind_addr);
    jinfo("Binding REP socket to {}", rep_bind_addr);
    req_resp_socket = std::make_unique<zmq::socket_t>(*context, ZMQ_REP);
    req_resp_socket->bind(rep_bind_addr);

    return true;
}

void ZmqIO::getJointPosition(Eigen::Ref<Eigen::VectorXd> out) const
{
    Eigen::VectorXd tmp_buffer;
    Eigen::VectorXd tmp_buffer2;

    int joints_num = _robot->getJointNum();
    if(_robot->isFloatingBase())
        joints_num -= 1;

    _robot->getJointPosition(tmp_buffer);
    _robot->positionToMinimal(tmp_buffer, tmp_buffer2);
    tmp_buffer2 = tmp_buffer2.tail(joints_num); // Remove floating base joint if present
    out = tmp_buffer2;
}

void ZmqIO::getMotorPosition(Eigen::Ref<Eigen::VectorXd> out) const
{
    Eigen::VectorXd tmp_buffer;
    Eigen::VectorXd tmp_buffer2;

    int joints_num = _robot->getJointNum();
    if(_robot->isFloatingBase())
        joints_num -= 1;

    _robot->getMotorPosition(tmp_buffer);
    _robot->positionToMinimal(tmp_buffer, tmp_buffer2);
    tmp_buffer2 = tmp_buffer2.tail(joints_num); // Remove floating base joint if present
    out = tmp_buffer2;
}

void ZmqIO::getJointPositionReference(Eigen::Ref<Eigen::VectorXd> out) const
{
    Eigen::VectorXd tmp_buffer;
    Eigen::VectorXd tmp_buffer2;

    int joints_num = _robot->getJointNum();
    if(_robot->isFloatingBase())
        joints_num -= 1;

    _robot->getPositionReferenceFeedback(tmp_buffer);
    _robot->positionToMinimal(tmp_buffer, tmp_buffer2);
    tmp_buffer2 = tmp_buffer2.tail(joints_num); // Remove floating base joint if present
    out = tmp_buffer2;
}

/**
 * Builds a raw state message string from the provided data. The output is structured as follows:
 * - All data is in 64-bit double precision for floating-point values and 32-bit integers for integer values.
 * - First integer is the sequence number (seq).
 * - Second double is the timestamp (stamp).
 * - Next comes the number of IMUs (imus_num) as an integer.
 * - Next comes the number of joints (joints_num) as an integer.
 * - Then follows the joint states matrix (joints_state) serialized in row-major order, with dimensions (joints_num x 12).
 * - Finally, the IMU states matrix (imus_state) serialized in row-major order, with dimensions (imus_num x 10).
 * @param imu_names List of IMU names.
 * @param joints_state Matrix containing joint states.
 * @param joints_num Number of joints.
 * @param imus_state Matrix containing IMU states.
 * @return The serialized raw bytes message
 */
std::vector<uint8_t> ZmqIO::build_state_msg_raw(std::vector<std::string> imu_names,
                                    Eigen::Ref<Eigen::MatrixXd> joints_state,
                                    int joints_num,
                                    Eigen::Ref<Eigen::MatrixXd> imus_state)
{
    // ensure row-major for numpy compatibility
    Eigen::Matrix<DoubleType, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> joints_state_rm = joints_state;
    Eigen::Matrix<DoubleType, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> imus_state_rm = imus_state;

    IntType seq_val = static_cast<IntType>(seq++);
    DoubleType stamp = chrono::wall_clock::now().time_since_epoch().count() * 1e-9;
    IntType imus_num = static_cast<IntType>(imu_names.size());
    IntType joints_num_val = static_cast<IntType>(joints_num);

    size_t total = sizeof(IntType) + sizeof(DoubleType) + sizeof(IntType) + sizeof(IntType)
                 + joints_state_rm.size() * sizeof(DoubleType)
                 + imus_state_rm.size() * sizeof(DoubleType);

    std::vector<uint8_t> out;
    out.reserve(total);

    auto append = [&](const void* data, size_t n) {
        const auto* p = reinterpret_cast<const uint8_t*>(data);
        out.insert(out.end(), p, p + n);
    };

    append(&seq_val,       sizeof(IntType));
    append(&stamp,         sizeof(DoubleType));
    append(&imus_num,      sizeof(IntType));
    append(&joints_num_val, sizeof(IntType));
    append(joints_state_rm.data(), joints_state_rm.size() * sizeof(DoubleType));
    append(imus_state_rm.data(),   imus_state_rm.size()   * sizeof(DoubleType));

    return out;
}

void ZmqIO::publish_state()
{
    _robot->sense(false);
    Eigen::VectorXd tmp_buffer;
    Eigen::VectorXd tmp_buffer2;

    int joints_num = _robot->getJointNum();
    if(_robot->isFloatingBase())
        joints_num -= 1;
    Eigen::Matrix<DoubleType, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor> joints_state(joints_num, 12);
    getJointPosition(joints_state.col(0));
    getMotorPosition(joints_state.col(1));
    joints_state.col(2) = _robot->getJointVelocity().tail(joints_num);
    joints_state.col(3) = _robot->getMotorVelocity().tail(joints_num);
    joints_state.col(4) = _robot->getJointEffort().tail(joints_num);
    // _robot->getTemperatureMotor(joints_state.col(5));
    // _robot->getTemperatureBoard(joints_state.col(6));
    getJointPositionReference(joints_state.col(7));
    joints_state.col(8) = _robot->getVelocityReferenceFeedback().tail(joints_num);
    joints_state.col(9) = _robot->getEffortReferenceFeedback().tail(joints_num);
    joints_state.col(10) = _robot->getStiffness().tail(joints_num);
    joints_state.col(11) = _robot->getDamping().tail(joints_num);


    
    int imus_num = _robot->getImu().size();
    Eigen::Matrix<DoubleType, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor> imus_state(imus_num, 10);
    int i = 0;
    Eigen::Vector3d lin_acc, ang_vel;
    Eigen::Quaterniond quat;
    std::vector<std::string> imu_names;
    for(auto [name, imu] : _robot->getImu())
    {
        imu->getLinearAcceleration(lin_acc);
        imu->getAngularVelocity(ang_vel);
        imu->getOrientation(quat);
        imus_state.row(i).segment(0,3) = lin_acc;
        imus_state.row(i).segment(3,3) = ang_vel;
        imus_state.row(i).segment(6,4) = Eigen::Vector4d(quat.x(), quat.y(), quat.z(), quat.w());
        imu_names.push_back(name);
        i++;
    }

    std::vector<uint8_t> raw_msg = build_state_msg_raw(imu_names, joints_state, joints_num, imus_state);
    raw_publisher->send(zmq::buffer(raw_msg), zmq::send_flags::none);
}

void ZmqIO::handle_request_response()
{
    zmq::message_t request;

    if (req_resp_socket->recv(request, zmq::recv_flags::dontwait)) 
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
        else if(req_type == "imu_names") {
            resp_yaml["success"] = true;
            std::vector<std::string> imu_names;
            for(auto [name, imu] : _robot->getImu())
                imu_names.push_back(name);
            resp_yaml["data"] = imu_names;
        }
        else if(req_type == "set_filter_frequency_hz")
        {
            bool enabled = false;
            double cutoff_hz = 0.0;
            if(req_yaml["enabled"] && req_yaml["enabled"].IsScalar()) {
                enabled = req_yaml["enabled"].as<bool>();
            } else {
                resp_yaml["message"] = "missing or invalid 'enabled' field";
                req_resp_socket->send(zmq::buffer(YAML::Dump(resp_yaml)), zmq::send_flags::none);
                return;
            }
            if(req_yaml["cutoff_hz"] && req_yaml["cutoff_hz"].IsScalar()) {
                cutoff_hz = req_yaml["cutoff_hz"].as<double>();
            } else {
                resp_yaml["message"] = "missing or invalid 'cutoff_hz' field";
                req_resp_socket->send(zmq::buffer(YAML::Dump(resp_yaml)), zmq::send_flags::none);
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
                req_resp_socket->send(zmq::buffer(YAML::Dump(resp_yaml)), zmq::send_flags::none);
                return;
            }
            resp_yaml["success"] = sendCommand(plugin_name, Runnable::Command::Start);
        }
        else {
            jerror("unknown request type: {}", req_type);
            resp_yaml["message"] = "unknown request type: " + req_type;
        }
        std::string resp_str = YAML::Dump(resp_yaml);
        req_resp_socket->send(zmq::buffer(resp_str), zmq::send_flags::none);
    }
}

void ZmqIO::readToMat(const std::string& data_str, Eigen::Ref<Eigen::MatrixXd> out, 
                      int rows, int cols)
{
    if(data_str.size() != rows * cols * sizeof(DoubleType)) 
    {
        jerror("invalid data_str size: expected {}, got {}", 
                rows * cols * sizeof(DoubleType), 
                data_str.size());
        throw std::runtime_error("invalid data_str size in readToMat:"+
                std::to_string(rows * cols * sizeof(DoubleType))+
                " vs "+std::to_string(data_str.size()));
    }
    Eigen::Map<const Eigen::Matrix<DoubleType, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> data_map(
        reinterpret_cast<const DoubleType*>(data_str.data()), rows, cols);
    out = data_map;
}

void ZmqIO::readToMat(const std::string& data_str, Eigen::Ref<Eigen::MatrixXi> out, 
                      int rows, int cols)
{
    if(data_str.size() != rows * cols * sizeof(IntType)) 
    {
        jerror("invalid data_str size: expected {}, got {}", 
                rows * cols * sizeof(IntType), 
                data_str.size());
        throw std::runtime_error("invalid data_str size in readToMat:"+
                std::to_string(rows * cols * sizeof(IntType))+
                " vs "+std::to_string(data_str.size()));
    }
    if(out.rows() != rows || out.cols() != cols) 
    {
        jerror("output matrix has invalid size: expected {}x{}, got {}x{}", 
                rows, cols, out.rows(), out.cols());
        throw std::runtime_error("invalid output matrix size in readToMat:"+
                std::to_string(rows)+"x"+std::to_string(cols)+
                " vs "+std::to_string(out.rows())+"x"+std::to_string(out.cols()));
    }
    // std::cout<<"data_str = "<<data_str<<std::endl;
    Eigen::Map<const Eigen::Matrix<IntType, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> data_map(
        reinterpret_cast<const IntType*>(data_str.data()), rows, cols);
    // std::cout<<"data_map = "<<data_map<<std::endl;
    out = data_map;
}

void ZmqIO::readToMat(const DoubleType* data, size_t byte_size, Eigen::Ref<Eigen::MatrixXd> out,
                      int rows, int cols)
{
    size_t expected = rows * cols * sizeof(DoubleType);
    if(byte_size != expected)
    {
        jerror("invalid data size: expected {}, got {}", expected, byte_size);
        throw std::runtime_error("invalid data size in readToMat: " +
                std::to_string(expected) + " vs " + std::to_string(byte_size));
    }
    Eigen::Map<const Eigen::Matrix<DoubleType, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> data_map(
        data, rows, cols);
    out = data_map;
}

void ZmqIO::readToMat(const IntType* data, size_t byte_size, Eigen::Ref<Eigen::MatrixXi> out,
                      int rows, int cols)
{
    size_t expected = rows * cols * sizeof(IntType);
    if(byte_size != expected)
    {
        jerror("invalid data size: expected {}, got {}", expected, byte_size);
        throw std::runtime_error("invalid data size in readToMat: " +
                std::to_string(expected) + " vs " + std::to_string(byte_size));
    }
    Eigen::Map<const Eigen::Matrix<IntType, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> data_map(
        data, rows, cols);
    out = data_map;
}

/**
 * Receives a command message in the raw bytes format, parses it, and applies the commands to the robot's joints. The expected command message format is as follows:
 * - A message sequence number (seq) as a 32-bit integer.                               1 x int32
 * - A timestamp (stamp) as a 64-bit double.                                            1 x float64
 * - The number of commanded joints (joints_num) as a 32-bit integer.                   1 x int32
 * - Then we have a list of the id of the commanded joints, as integers (joint_ids).    joints_num x int32
 * - Then we have a matrix of position, velocity, effort, stiffness and damping
 *   references (joints_pvesd), as doubles, in row-major order.                         joints_num x 5 x float64
 * - Then we have a matrix of control modes (joints_ctrl), in row-major order.          joints_num x int32
 */
void ZmqIO::recv_cmd_v3()
{
    // currently not used
    zmq::message_t cmd;

    if (cmd_subscriber->recv(cmd, zmq::recv_flags::dontwait))
    {
        try
        {
            size_t header_size = sizeof(uint32_t) + sizeof(DoubleType) + sizeof(uint32_t);
            if (cmd.size() < header_size)
            {
                jerror("invalid command size: expected at least {}, got {}, SKIPPING COMMAND.", header_size, cmd.size());
                return;
            }
            const char* ptr = static_cast<const char*>(cmd.data());
            uint32_t seq            = *reinterpret_cast<const uint32_t*>(ptr);
            DoubleType stamp        = *reinterpret_cast<const DoubleType*>(ptr + sizeof(uint32_t));
            uint32_t cmd_joints_num = *reinterpret_cast<const uint32_t*>(ptr + sizeof(uint32_t) + sizeof(DoubleType));

            size_t joint_ids_size    = cmd_joints_num * sizeof(int32_t);
            size_t joints_pvesd_size = cmd_joints_num * 5 * sizeof(DoubleType);
            size_t joints_ctrl_size  = cmd_joints_num * sizeof(int32_t);
            size_t expected_size     = header_size + joint_ids_size + joints_pvesd_size + joints_ctrl_size;
            if(cmd.size() != expected_size)
            {
                jerror("invalid command size: expected {}, got {}, SKIPPING COMMAND.", expected_size, cmd.size());
                return;
            }

            // jinfo("Received command seq: {}, stamp: {}, joints_num: {}", seq, stamp, cmd_joints_num);

            std::vector<std::string> joint_names(cmd_joints_num);
            const int32_t* joint_ids = reinterpret_cast<const int32_t*>(ptr + header_size);
            std::vector<std::string> all_joint_names = _robot->getJointNames();
            for(int i = 0; i < cmd_joints_num; ++i)
            {
                int32_t idx = joint_ids[i];
                if(idx < 0 || idx >= all_joint_names.size())
                {
                    jerror("invalid joint id {} at index {}, joints_number: {}, SKIPPING COMMAND.", idx, i, all_joint_names.size());
                    return;
                }
                joint_names[i] = all_joint_names.at(idx);
            }

            const DoubleType* pvesd_raw = reinterpret_cast<const DoubleType*>(ptr + header_size + joint_ids_size);
            Eigen::Matrix<DoubleType, Eigen::Dynamic, Eigen::Dynamic> joints_pvesd(cmd_joints_num, 5);
            readToMat(pvesd_raw, joints_pvesd_size, joints_pvesd, cmd_joints_num, 5);

            const IntType* ctrl_raw = reinterpret_cast<const IntType*>(ptr + header_size + joint_ids_size + joints_pvesd_size);
            Eigen::Matrix<IntType, Eigen::Dynamic, Eigen::Dynamic> joints_ctrl(cmd_joints_num, 1);
            readToMat(ctrl_raw, joints_ctrl_size, joints_ctrl, cmd_joints_num, 1);

            for (int i = 0; i < joint_names.size(); ++i)
            {
                const std::string& joint_name = joint_names[i];
                auto j = _robot->getJoint(joint_name);

                if(!j)
                {
                    jerror("unknown joint '{}'", joint_name);
                    continue;
                }
                auto joint_cmd_vec   = joints_pvesd.row(i);
                auto joints_ctrl_vec = joints_ctrl.row(i);
                // std::cout << "Joint '" << joint_name << "' cmd: " << joint_cmd_vec.transpose() << " ctrl: " << joints_ctrl_vec.transpose() << std::endl;
                j->setPositionReferenceMinimal(Eigen::Scalard(joint_cmd_vec[0]));
                j->setVelocityReference(Eigen::Scalard(joint_cmd_vec[1]));
                j->setEffortReference(Eigen::Scalard(joint_cmd_vec[2]));
                j->setStiffness(Eigen::Scalard(joint_cmd_vec[3]));
                j->setDamping(Eigen::Scalard(joint_cmd_vec[4]));
                j->setControlMode(static_cast<ControlMode::Type>(joints_ctrl_vec[0]));
            }

            cmd_timeout = chrono::steady_clock::now() + 1s;

            _robot->move();
        }
        catch(const std::exception& e)
        {
            jerror("exception while processing command, SKIPPING: {}", e.what());
        }
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
    recv_cmd_v3();

    publish_state();

    handle_request_response();
}



XBOT2_REGISTER_PLUGIN(ZmqIO, zmq_io)
