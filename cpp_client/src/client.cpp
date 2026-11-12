#include "xbot2_zmq/client.hpp"

#include <time.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <random>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <zmq.hpp>

namespace xbot2_zmq {

namespace {

const std::size_t kStateHeaderSize = 3 * sizeof(std::int32_t) + sizeof(double);
const std::size_t kCommandHeaderSize = 2 * sizeof(std::uint32_t) +
                                       2 * sizeof(std::uint64_t);

template <typename T>
void append(std::vector<std::uint8_t>& out, std::size_t& offset, const T* data,
            std::size_t count) {
  const std::size_t bytes = count * sizeof(T);
  if (bytes != 0) std::memcpy(out.data() + offset, data, bytes);
  offset += bytes;
}

template <typename T>
T read(const std::uint8_t* data, std::size_t& offset) {
  T value;
  std::memcpy(&value, data + offset, sizeof(T));
  offset += sizeof(T);
  return value;
}

std::string endpoint(const ClientOptions& options, const int tcp_port,
                     const std::string& ipc_path) {
  if (options.protocol == "ipc") return "ipc://" + ipc_path;
  if (options.protocol == "tcp")
    return "tcp://" + options.remote_ip + ":" + std::to_string(tcp_port);
  throw std::invalid_argument("protocol must be 'ipc' or 'tcp'.");
}

std::uint64_t monotonic_ns() {
  timespec now{};
  clock_gettime(CLOCK_MONOTONIC, &now);
  return static_cast<std::uint64_t>(now.tv_sec) * 1000000000ULL +
         static_cast<std::uint64_t>(now.tv_nsec);
}

double monotonic_s() { return static_cast<double>(monotonic_ns()) * 1.0e-9; }

int to_ms(const double timeout_s) {
  if (timeout_s < 0.0) return -1;
  return static_cast<int>(std::ceil(timeout_s * 1.0e3));
}

}  // namespace

std::vector<std::uint8_t> encode_command(const std::uint32_t seq,
                                         const std::uint64_t stamp_ns,
                                         const std::uint64_t client_session_id,
                                         const JointsCommand& command) {
  const std::size_t joints = command.joint_ids.size();
  if (command.pvesd.size() != 5 * joints || command.ctrl_mode.size() != joints)
    throw std::invalid_argument("command arrays disagree with joint_ids.");
  std::vector<std::uint8_t> out(kCommandHeaderSize +
                                joints * sizeof(std::int32_t) +
                                5 * joints * sizeof(double) +
                                joints * sizeof(std::int32_t));
  const std::uint32_t joints_num = static_cast<std::uint32_t>(joints);
  std::size_t offset = 0;
  append(out, offset, &seq, 1);
  append(out, offset, &stamp_ns, 1);
  append(out, offset, &joints_num, 1);
  append(out, offset, &client_session_id, 1);
  append(out, offset, command.joint_ids.data(), joints);
  append(out, offset, command.pvesd.data(), 5 * joints);
  append(out, offset, command.ctrl_mode.data(), joints);
  return out;
}

void decode_state(const void* const data, const std::size_t size,
                  const std::size_t joints_num, const std::size_t imus_num,
                  RobotState& out) {
  if (size < kStateHeaderSize)
    throw std::invalid_argument("state message is shorter than its header.");
  const auto* bytes = static_cast<const std::uint8_t*>(data);
  std::size_t offset = 0;
  const auto seq = read<std::int32_t>(bytes, offset);
  const auto stamp = read<double>(bytes, offset);
  const auto imus = read<std::int32_t>(bytes, offset);
  const auto joints = read<std::int32_t>(bytes, offset);
  if (joints < 0 || static_cast<std::size_t>(joints) != joints_num)
    throw std::invalid_argument("state message has an unexpected joint count.");
  if (imus < 0 || static_cast<std::size_t>(imus) != imus_num)
    throw std::invalid_argument("state message has an unexpected IMU count.");
  const std::size_t joint_values = joints_num * kJointFieldCount;
  const std::size_t imu_values = imus_num * kImuFieldCount;
  if (size != kStateHeaderSize + (joint_values + imu_values) * sizeof(double))
    throw std::invalid_argument("state message has an unexpected size.");
  out.seq = seq;
  out.stamp = stamp;
  out.joints.resize(joint_values);
  out.imus.resize(imu_values);
  if (joint_values != 0) {
    std::memcpy(out.joints.data(), bytes + offset,
                joint_values * sizeof(double));
    offset += joint_values * sizeof(double);
  }
  if (imu_values != 0)
    std::memcpy(out.imus.data(), bytes + offset, imu_values * sizeof(double));
}

ClockDeltaEstimator::ClockDeltaEstimator(const std::size_t window,
                                         const std::size_t refit_every)
    : window_(window),
      refit_every_(refit_every),
      recv_(window, 0.0),
      stamp_(window, 0.0) {
  if (window < 2 || refit_every < 1)
    throw std::invalid_argument(
        "ClockDeltaEstimator needs a window of at least 2 and refit_every >= 1.");
  reset();
}

void ClockDeltaEstimator::reset() {
  count_ = 0;
  head_ = 0;
  recv_origin_ = 0.0;
  stamp_origin_ = 0.0;
  last_stamp_ = -std::numeric_limits<double>::infinity();
  rate_ = 1.0;
  offset_ = 0.0;
}

void ClockDeltaEstimator::update(const double server_stamp,
                                 const double recv_time) {
  if (server_stamp < last_stamp_) reset();
  last_stamp_ = server_stamp;
  if (count_ == 0) {
    recv_origin_ = recv_time;
    stamp_origin_ = server_stamp;
  }
  const double t = recv_time - recv_origin_;
  const double s = server_stamp - stamp_origin_;
  recv_[head_] = t;
  stamp_[head_] = s;
  head_ = (head_ + 1) % window_;
  ++count_;
  if (count_ == 1) {
    offset_ = s - rate_ * t;
  } else if (count_ % refit_every_ == 0) {
    refit();
  } else {
    offset_ = std::max(offset_, s - rate_ * t);
  }
}

void ClockDeltaEstimator::refit() {
  const std::size_t n = std::min(count_, window_);
  double t_mean = 0.0, s_mean = 0.0;
  for (std::size_t i = 0; i < n; ++i) {
    t_mean += recv_[i];
    s_mean += stamp_[i];
  }
  t_mean /= static_cast<double>(n);
  s_mean /= static_cast<double>(n);
  double var = 0.0, cov = 0.0;
  for (std::size_t i = 0; i < n; ++i) {
    const double dt = recv_[i] - t_mean;
    var += dt * dt;
    cov += dt * (stamp_[i] - s_mean);
  }
  if (var > 0.0) rate_ = std::max(0.0, cov / var);
  offset_ = -std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < n; ++i)
    offset_ = std::max(offset_, stamp_[i] - rate_ * recv_[i]);
}

double ClockDeltaEstimator::server_time(const double now) const {
  if (count_ == 0) return std::numeric_limits<double>::quiet_NaN();
  return stamp_origin_ + rate_ * (now - recv_origin_) + offset_;
}

double ClockDeltaEstimator::age(const double server_stamp,
                                const double now) const {
  return server_time(now) - server_stamp;
}

struct Client::Impl {
  explicit Impl(ClientOptions client_options)
      : options(std::move(client_options)),
        context(1),
        service_url(endpoint(options, options.tcp_service_port,
                             options.ipc_service_path)),
        state_url(
            endpoint(options, options.tcp_state_port, options.ipc_state_path)),
        command_url(endpoint(options, options.tcp_command_port,
                             options.ipc_command_path)) {
    std::random_device device;
    std::uniform_int_distribution<std::uint64_t> session;
    session_id = session(device);
  }

  ClientOptions options;
  mutable zmq::context_t context;
  std::string service_url;
  std::string state_url;
  std::string command_url;
  std::unique_ptr<zmq::socket_t> state_socket;
  std::unique_ptr<zmq::socket_t> command_socket;
  std::vector<std::string> joint_names;
  std::vector<std::string> imu_names;
  std::unordered_map<std::string, std::int32_t> joint_ids;
  RobotState state;
  bool has_state = false;
  ClockDeltaEstimator clock_delta;
  std::uint32_t command_seq = 0;
  std::uint64_t session_id = 0;
};

Client::Client(ClientOptions options)
    : impl_(std::make_unique<Impl>(std::move(options))) {}

Client::~Client() = default;

std::string Client::request(const std::string& type,
                            const double timeout_s) const {
  zmq::socket_t socket(impl_->context, ZMQ_REQ);
  const int timeout_ms = std::max(1, to_ms(timeout_s));
  socket.set(zmq::sockopt::linger, 0);
  socket.set(zmq::sockopt::rcvtimeo, timeout_ms);
  socket.set(zmq::sockopt::sndtimeo, timeout_ms);
  socket.connect(impl_->service_url);

  YAML::Emitter request_yaml;
  request_yaml << YAML::BeginMap << YAML::Key << "type" << YAML::Value << type
               << YAML::EndMap;
  const std::string request_text = request_yaml.c_str();
  if (!socket.send(zmq::buffer(request_text), zmq::send_flags::none))
    throw std::runtime_error("timed out sending the '" + type + "' request.");
  zmq::message_t reply;
  if (!socket.recv(reply, zmq::recv_flags::none))
    throw std::runtime_error("timed out waiting for the '" + type + "' reply.");

  const YAML::Node response = YAML::Load(reply.to_string());
  if (!response["success"] || !response["success"].as<bool>()) {
    const std::string message = response["message"]
                                    ? response["message"].as<std::string>()
                                    : "no message from the server";
    throw std::runtime_error("request '" + type + "' failed: " + message);
  }
  std::ostringstream data;
  data << response["data"];
  return data.str();
}

void Client::start(const double timeout_s) {
  const auto fetch_names = [&](const std::string& type) {
    return YAML::Load(request(type, timeout_s)).as<std::vector<std::string>>();
  };
  impl_->joint_names = fetch_names("joint_names");
  impl_->imu_names = fetch_names("imu_names");
  impl_->joint_ids.clear();
  for (std::size_t k = 0; k < impl_->joint_names.size(); ++k)
    impl_->joint_ids[impl_->joint_names[k]] = static_cast<std::int32_t>(k);

  impl_->state_socket =
      std::make_unique<zmq::socket_t>(impl_->context, ZMQ_SUB);
  impl_->state_socket->set(zmq::sockopt::linger, 0);
  impl_->state_socket->set(zmq::sockopt::conflate, 1);
  impl_->state_socket->set(zmq::sockopt::subscribe, "");
  impl_->state_socket->connect(impl_->state_url);

  impl_->command_socket =
      std::make_unique<zmq::socket_t>(impl_->context, ZMQ_PUB);
  impl_->command_socket->set(zmq::sockopt::linger, 0);
  impl_->command_socket->connect(impl_->command_url);
}

const std::vector<std::string>& Client::joint_names() const {
  return impl_->joint_names;
}

const std::vector<std::string>& Client::imu_names() const {
  return impl_->imu_names;
}

std::int32_t Client::joint_id(const std::string& name) const {
  const auto found = impl_->joint_ids.find(name);
  if (found == impl_->joint_ids.end())
    throw std::out_of_range("unknown joint '" + name + "'.");
  return found->second;
}

bool Client::sense(const double timeout_s) {
  if (!impl_->state_socket)
    throw std::logic_error("Client::sense requires start().");
  zmq::pollitem_t item{impl_->state_socket->handle(), 0, ZMQ_POLLIN, 0};
  if (zmq::poll(&item, 1, std::chrono::milliseconds(to_ms(timeout_s))) == 0)
    return false;
  zmq::message_t message;
  if (!impl_->state_socket->recv(message, zmq::recv_flags::dontwait))
    return false;
  const double recv_time = monotonic_s();
  decode_state(message.data(), message.size(), impl_->joint_names.size(),
               impl_->imu_names.size(), impl_->state);
  impl_->clock_delta.update(impl_->state.stamp, recv_time);
  impl_->has_state = true;
  return true;
}

const RobotState& Client::state() const { return impl_->state; }

double Client::server_time() const {
  return impl_->clock_delta.server_time(monotonic_s());
}

double Client::state_age() const {
  if (!impl_->has_state) return std::numeric_limits<double>::quiet_NaN();
  return impl_->clock_delta.age(impl_->state.stamp, monotonic_s());
}

void Client::send(const JointsCommand& command) {
  if (!impl_->command_socket)
    throw std::logic_error("Client::send requires start().");
  const std::vector<std::uint8_t> raw = encode_command(
      impl_->command_seq, monotonic_ns(), impl_->session_id, command);
  impl_->command_socket->send(zmq::buffer(raw), zmq::send_flags::dontwait);
  ++impl_->command_seq;
}

}  // namespace xbot2_zmq
