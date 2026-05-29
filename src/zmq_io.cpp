#include "zmq_io.h"
#include <time.h>
#include <cstdint>
#include <atomic>
#include <chrono>

uint64_t monotonic_ns() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts); // Use this specific clock to try to use the same time here and in C++, so at least on the same machine things should match
    return static_cast<uint64_t>(ts.tv_sec) * 1'000'000'000ULL + ts.tv_nsec;
}

using namespace XBot;

void ClientDelayStats::update(int64_t delay_ns, uint32_t seq) {
    uint64_t now_ns = monotonic_ns();
    int64_t ipt_ns = last_update_ns == 0 ? 0 : static_cast<int64_t>(now_ns - last_update_ns);
    last_update_ns = now_ns;
    int packets_since_last = seq - last_seq;
    if (packets_since_last > 1)
        std::cout << "Warning: Missed " << packets_since_last - 1 << " packets from client." << std::endl;
    last_seq = seq;

    sum -= delays_ns[head];
    ipt_sum -= inter_packet_ns[head];
    delays_ns[head] = delay_ns;
    inter_packet_ns[head] = ipt_ns;
    sum += delay_ns;
    ipt_sum += ipt_ns;
    head = (head + 1) % WINDOW;
    if (count < WINDOW)
        ++count;

    avg_delay_ns = sum / static_cast<double>(count);
    avg_inter_packet_ns = ipt_sum / static_cast<double>(count);

    double var_delay = 0.0, var_ipt = 0.0;
    for (size_t i = 0; i < count; ++i)
    {
        double dd = delays_ns[i] - avg_delay_ns;
        var_delay += dd * dd;
        double di = inter_packet_ns[i] - avg_inter_packet_ns;
        var_ipt += di * di;
    }
    jitter_ns = std::sqrt(var_delay / static_cast<double>(count));
    inter_packet_jitter_ns = std::sqrt(var_ipt / static_cast<double>(count));

    double max_std_deviation = 5;
    if (std::abs(delay_ns - avg_delay_ns) > max_std_deviation * jitter_ns)
        std::cout << "Abnormal client delay: " << delay_ns * 1e-6 << " ms (avg: " << avg_delay_ns * 1e-6 << " ms, jitter: " << jitter_ns * 1e-6 << " ms, pkgs since last: " << packets_since_last << ")" << std::endl;
    if (last_update_ns != 0 && std::abs(ipt_ns - avg_inter_packet_ns) > max_std_deviation * inter_packet_jitter_ns)
        std::cout << "Abnormal inter-packet time: " << ipt_ns * 1e-6 << " ms (avg: " << avg_inter_packet_ns * 1e-6 << " ms, jitter: " << inter_packet_jitter_ns * 1e-6 << " ms)" << std::endl;
    // std::cout << "Client delay: " << delay_ns * 1e-6 << " ms (avg: " << avg_delay_ns * 1e-6 << " ms, jitter: " << jitter_ns * 1e-6 << " ms), inter-packet: " << ipt_ns * 1e-6 << " ms (avg: " << avg_inter_packet_ns * 1e-6 << " ms, jitter: " << inter_packet_jitter_ns * 1e-6 << " ms), skipped: " << packets_since_last - 1 << std::endl;
}

void ClientDelayStats::reset(uint32_t initial_seq)
{   
    delays_ns.fill(0);
    inter_packet_ns.fill(0);
    head = 0;
    count = 0;
    sum = 0;
    ipt_sum = 0;
    avg_delay_ns = 0.0;
    jitter_ns = 0.0;
    avg_inter_packet_ns = 0.0;
    inter_packet_jitter_ns = 0.0;
    last_seq = initial_seq;
    last_update_ns = 0;
}


bool ZmqIO::on_initialize()
{
    std::string protocol = "ipc";
    getParam("~protocol", protocol);

    std::string pub_bind_addr, cmd_sub_addr, service_bind_addr;

    if (protocol == "tcp")
    {
        int state_port = 5559, cmd_port = 5558, service_port = 5557;
        getParam("~tcp_state_port", state_port);
        getParam("~tcp_cmd_port", cmd_port);
        getParam("~tcp_service_port", service_port);
        pub_bind_addr = "tcp://*:" + std::to_string(state_port);
        cmd_sub_addr      = "tcp://*:" + std::to_string(cmd_port);
        service_bind_addr     = "tcp://*:" + std::to_string(service_port);
    }
    else if (protocol == "ipc")
    {
        std::string pub_path = "/tmp/xbot2_zmq_pub.ipc";
        std::string cmd_path = "/tmp/xbot2_zmq_cmd.ipc";
        std::string rep_path = "/tmp/xbot2_zmq_rep.ipc";
        getParam("~ipc_state_path", pub_path);
        getParam("~ipc_cmd_path", cmd_path);
        getParam("~ipc_service_path", rep_path);
        pub_bind_addr     = "ipc://" + pub_path;
        cmd_sub_addr      = "ipc://" + cmd_path;
        service_bind_addr = "ipc://" + rep_path;
    }
    else
    {
        jerror("Unknown protocol '{}', expected 'tcp' or 'ipc'", protocol);
        return false;
    }

    context = std::make_unique<zmq::context_t>(1);

    jinfo("Binding RAW PUB socket to {}", pub_bind_addr);
    raw_publisher = std::make_unique<zmq::socket_t>(*context, ZMQ_PUB);
    raw_publisher->bind(pub_bind_addr);

    jinfo("Binding CMD socket to {}", cmd_sub_addr);
    cmd_subscriber = std::make_unique<zmq::socket_t>(*context, ZMQ_SUB);
    cmd_subscriber->bind(cmd_sub_addr);
    cmd_subscriber->set(zmq::sockopt::subscribe, "");
    cmd_subscriber->set(zmq::sockopt::conflate, 1);

    jinfo("Binding REP socket to {}", service_bind_addr);
    req_resp_socket = std::make_unique<zmq::socket_t>(*context, ZMQ_REP);
    req_resp_socket->bind(service_bind_addr);

    _safety_flag = Hal::JointSafety::get_shared_safety_flag();

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
    _last_state_seq = seq - 1;
    _last_state_publish_ns = monotonic_ns();
    _state_publish_count++;
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
        else if(req_type == "plugin_status")
        {
            std::string plugin_name;
            if(req_yaml["plugin"] && req_yaml["plugin"].IsScalar()) {
                plugin_name = req_yaml["plugin"].as<std::string>();
            } else {
                resp_yaml["message"] = "missing or invalid 'plugin' field";
                req_resp_socket->send(zmq::buffer(YAML::Dump(resp_yaml)), zmq::send_flags::none);
                return;
            }
            Runnable::State plugin_state;
            const bool status_ok = getPluginState(plugin_name, plugin_state);
            resp_yaml["success"] = status_ok;
            if(status_ok)
            {
                resp_yaml["data"]["state"] = Runnable::StateAsString(plugin_state);
            }
            else
            {
                resp_yaml["message"] = "failed to read plugin state for '" + plugin_name + "'";
            }
        }
        else if(req_type == "plugin_command")
        {
            std::string plugin_name;
            std::string command_name;
            if(req_yaml["plugin"] && req_yaml["plugin"].IsScalar()) {
                plugin_name = req_yaml["plugin"].as<std::string>();
            } else {
                resp_yaml["message"] = "missing or invalid 'plugin' field";
                req_resp_socket->send(zmq::buffer(YAML::Dump(resp_yaml)), zmq::send_flags::none);
                return;
            }
            if(req_yaml["command"] && req_yaml["command"].IsScalar()) {
                command_name = req_yaml["command"].as<std::string>();
            } else {
                resp_yaml["message"] = "missing or invalid 'command' field";
                req_resp_socket->send(zmq::buffer(YAML::Dump(resp_yaml)), zmq::send_flags::none);
                return;
            }

            Runnable::Command command;
            if(command_name == "start") {
                command = Runnable::Command::Start;
            } else if(command_name == "stop") {
                command = Runnable::Command::Stop;
            } else if(command_name == "abort") {
                command = Runnable::Command::Abort;
            } else {
                resp_yaml["message"] = "invalid plugin command '" + command_name + "'";
                req_resp_socket->send(zmq::buffer(YAML::Dump(resp_yaml)), zmq::send_flags::none);
                return;
            }

            const bool command_ok = sendCommand(plugin_name, command);
            resp_yaml["success"] = command_ok;
            if(!command_ok)
            {
                resp_yaml["message"] = "failed to send '" + command_name + "' to plugin '" + plugin_name + "'";
            }
        }
        else if(req_type == "safety_status")
        {
            auto safety_status = Hal::JointSafety::status();
            resp_yaml["success"] = true;
            resp_yaml["data"]["safety_enabled"] = safety_status.safety_enabled;
            resp_yaml["data"]["filter_enabled"] = safety_status.filter_enabled;
            resp_yaml["data"]["cutoff_hz"] = safety_status.cutoff_hz;
            resp_yaml["data"]["safety_triggered"] = _safety_flag && _safety_flag->load(std::memory_order_relaxed);
        }
        else if(req_type == "safety_restore")
        {
            const bool restore_ok = Hal::JointSafety::restore();
            resp_yaml["success"] = restore_ok;
            if(!restore_ok)
            {
                resp_yaml["message"] = "failed to restore XBot joint safety";
            }
        }
        else if(req_type == "state_stats")
        {
            uint64_t now_ns = monotonic_ns();
            resp_yaml["success"] = true;
            resp_yaml["data"]["last_seq"] = _last_state_seq;
            resp_yaml["data"]["publish_count"] = static_cast<unsigned long long>(_state_publish_count);
            resp_yaml["data"]["last_publish_monotonic_ns"] = static_cast<unsigned long long>(_last_state_publish_ns);
            resp_yaml["data"]["last_publish_age_s"] = _last_state_publish_ns > 0 ?
                static_cast<double>(now_ns - _last_state_publish_ns) * 1e-9 : -1.0;
        }
        else if(req_type == "cmd_stats")
        {
            uint64_t now_ns = monotonic_ns();
            const bool timeout_active = cmd_timeout.time_since_epoch().count() != 0;
            resp_yaml["success"] = true;
            resp_yaml["data"]["last_seq"] = _last_cmd_seq;
            resp_yaml["data"]["last_session_id"] = static_cast<unsigned long long>(_last_cmd_session_id);
            resp_yaml["data"]["last_recv_monotonic_ns"] = static_cast<unsigned long long>(_last_cmd_recv_ns);
            resp_yaml["data"]["last_recv_age_s"] = _last_cmd_recv_ns > 0 ?
                static_cast<double>(now_ns - _last_cmd_recv_ns) * 1e-9 : -1.0;
            resp_yaml["data"]["consecutive_steps"] = cmd_consecutive_steps;
            resp_yaml["data"]["timeout_active"] = timeout_active;
            resp_yaml["data"]["timeout_remaining_s"] = timeout_active ?
                std::chrono::duration<double>(cmd_timeout - chrono::steady_clock::now()).count() : 0.0;
        }
        else if(req_type == "health")
        {
            uint64_t now_ns = monotonic_ns();
            auto safety_status = Hal::JointSafety::status();
            Runnable::State plugin_state;
            const bool plugin_state_ok = getPluginState("zmq_io", plugin_state);
            resp_yaml["success"] = true;
            resp_yaml["data"]["zmq_io_state_ok"] = plugin_state_ok;
            resp_yaml["data"]["zmq_io_state"] = plugin_state_ok ? Runnable::StateAsString(plugin_state) : std::string();
            resp_yaml["data"]["safety_enabled"] = safety_status.safety_enabled;
            resp_yaml["data"]["filter_enabled"] = safety_status.filter_enabled;
            resp_yaml["data"]["filter_cutoff_hz"] = safety_status.cutoff_hz;
            resp_yaml["data"]["safety_triggered"] = _safety_flag && _safety_flag->load(std::memory_order_relaxed);
            resp_yaml["data"]["state_last_seq"] = _last_state_seq;
            resp_yaml["data"]["state_publish_count"] = static_cast<unsigned long long>(_state_publish_count);
            resp_yaml["data"]["state_last_publish_age_s"] = _last_state_publish_ns > 0 ?
                static_cast<double>(now_ns - _last_state_publish_ns) * 1e-9 : -1.0;
            resp_yaml["data"]["cmd_last_seq"] = _last_cmd_seq;
            resp_yaml["data"]["cmd_last_session_id"] = static_cast<unsigned long long>(_last_cmd_session_id);
            resp_yaml["data"]["cmd_last_recv_age_s"] = _last_cmd_recv_ns > 0 ?
                static_cast<double>(now_ns - _last_cmd_recv_ns) * 1e-9 : -1.0;
            resp_yaml["data"]["cmd_timeout_active"] = cmd_timeout.time_since_epoch().count() != 0;
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
 * - A message sequence number (seq) as a 32-bit integer.                               1 x uint32
 * - A timestamp (stamp_ns) as a 64-bit unsigned integer.                               1 x uint64
 * - The number of commanded joints (joints_num) as a 32-bit integer.                   1 x uint32
 * - Client session ID                                                                  1 x uint64
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
        if(_safety_flag && _safety_flag->load(std::memory_order_relaxed))
        {
            if(cmd_consecutive_steps > 0)
            {
                jerror("joint safety is triggered, releasing resources and ignoring ZMQ commands");
                _robot->releaseResources();
                cmd_timeout = decltype(cmd_timeout)();
                cmd_consecutive_steps = 0;
            }
            return;
        }

        try
        {
            size_t header_size = sizeof(uint32_t) + sizeof(uint64_t) + sizeof(uint32_t) + sizeof(uint64_t);
            if (cmd.size() < header_size)
            {
                jerror("invalid command size: expected at least {}, got {}, SKIPPING COMMAND.", header_size, cmd.size());
                return;
            }
            const char* ptr = static_cast<const char*>(cmd.data());
            uint32_t seq               = *reinterpret_cast<const uint32_t*>(ptr);
            uint64_t stamp_ns          = *reinterpret_cast<const uint64_t*>(ptr + sizeof(uint32_t));
            uint32_t cmd_joints_num    = *reinterpret_cast<const uint32_t*>(ptr + sizeof(uint32_t) + sizeof(uint64_t));
            uint64_t client_session_id = *reinterpret_cast<const uint64_t*>(ptr + sizeof(uint32_t) + sizeof(uint64_t) + sizeof(uint32_t));

            uint64_t now_ns = monotonic_ns();
            if(cmd_consecutive_steps > 0)
                _client_delay_stats[client_session_id].update(static_cast<int64_t>((now_ns - stamp_ns)), seq);
            else
                _client_delay_stats[client_session_id].reset(seq);

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

            _last_cmd_seq = seq;
            _last_cmd_session_id = client_session_id;
            _last_cmd_recv_ns = now_ns;

            cmd_timeout = chrono::steady_clock::now() + 1s;
            cmd_consecutive_steps++;

            _robot->move();
            if(_safety_flag && _safety_flag->load(std::memory_order_relaxed))
            {
                jerror("joint safety triggered while applying ZMQ command, releasing resources");
                _robot->releaseResources();
                cmd_timeout = decltype(cmd_timeout)();
                cmd_consecutive_steps = 0;
            }
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
        cmd_consecutive_steps = 0;
    }

}



void ZmqIO::run()
{
    recv_cmd_v3();

    publish_state();

    handle_request_response();
}



XBOT2_REGISTER_PLUGIN(ZmqIO, zmq_io)
