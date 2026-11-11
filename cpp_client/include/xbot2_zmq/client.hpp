#ifndef XBOT2_ZMQ_CLIENT_HPP_
#define XBOT2_ZMQ_CLIENT_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace xbot2_zmq {

/** Where the zmq_io plugin listens. */
struct ClientOptions {
  std::string protocol = "ipc";  ///< "ipc" (same machine) or "tcp"
  std::string remote_ip = "localhost";
  int tcp_service_port = 5557;
  int tcp_state_port = 5559;
  int tcp_command_port = 5558;
  std::string ipc_service_path = "/tmp/xbot2_zmq_rep.ipc";
  std::string ipc_state_path = "/tmp/xbot2_zmq_pub.ipc";
  std::string ipc_command_path = "/tmp/xbot2_zmq_cmd.ipc";
};

/** Columns of RobotState::joints. */
enum JointField : std::size_t {
  kJointPosition = 0,
  kMotorPosition,
  kJointVelocity,
  kMotorVelocity,
  kJointEffort,
  kMotorTemperature,
  kDriverTemperature,
  kPositionReference,
  kVelocityReference,
  kEffortReference,
  kStiffness,
  kDamping,
  kJointFieldCount
};

/** Columns of RobotState::imus. The quaternion is stored x, y, z, w. */
enum ImuField : std::size_t {
  kLinearAcceleration = 0,
  kAngularVelocity = 3,
  kOrientation = 6,
  kImuFieldCount = 10
};

/** One state message. Matrices are flat and row-major. */
struct RobotState {
  std::int32_t seq = 0;
  double stamp = 0.0;  ///< seconds, on the XBot2 clock
  std::vector<double> joints;
  std::vector<double> imus;
};

/** One command. `pvesd` is flat and row-major, N x [pos, vel, effort, K, D]. */
struct JointsCommand {
  std::vector<std::int32_t> joint_ids;
  std::vector<double> pvesd;
  std::vector<std::int32_t> ctrl_mode;
};

/** Serialize a command in the zmq_io wire format. */
std::vector<std::uint8_t> encode_command(std::uint32_t seq,
                                         std::uint64_t stamp_ns,
                                         std::uint64_t client_session_id,
                                         const JointsCommand& command);

/** Parse a zmq_io state message. Throws std::invalid_argument when the size or
 * the joint and IMU counts disagree with the expected ones. */
void decode_state(const void* data, std::size_t size, std::size_t joints_num,
                  std::size_t imus_num, RobotState& out);

/** Maps the client's monotonic clock onto the server's clock, to estimate state
 * ages. pyxbot's ClockDeltaEstimator, step for step.
 *
 * The server stamps run at rate() server seconds per client second: 1 on a robot,
 * the real-time factor when the server runs on a simulated clock. Over a window of
 * states, rate() is the least-squares slope of the stamps against their receive
 * times and offset() places the line through the states received fastest. The
 * minimum one-way latency is indistinguishable from the offset, so ages are lower
 * bounds: the time a state has spent in transit beyond the fastest state in the
 * window, plus the time since it arrived. The window and the fit restart when the
 * stamps go backwards (a restarted server or simulation). */
class ClockDeltaEstimator {
 public:
  explicit ClockDeltaEstimator(std::size_t window = 2000,
                               std::size_t refit_every = 50);

  /** Forget all states. */
  void reset();
  /** Add a state stamped `server_stamp`, received at `recv_time` (CLOCK_MONOTONIC,
   * seconds). */
  void update(double server_stamp, double recv_time);
  /** Lower bound on the age of a state stamped `server_stamp` at client time `now`
   * (CLOCK_MONOTONIC, seconds), in server seconds. */
  double age(double server_stamp, double now) const;
  /** Server seconds per client second. */
  double rate() const { return rate_; }
  /** Server time, relative to the first stamp, at the first receive time. */
  double offset() const { return offset_; }

 private:
  void refit();

  std::size_t window_;
  std::size_t refit_every_;
  std::vector<double> recv_;
  std::vector<double> stamp_;
  std::size_t count_ = 0;
  std::size_t head_ = 0;
  double recv_origin_ = 0.0;
  double stamp_origin_ = 0.0;
  double last_stamp_ = 0.0;
  double rate_ = 1.0;
  double offset_ = 0.0;
};

/** Client of the XBot2 zmq_io plugin.
 *
 * sense() and send() use separate sockets, so one thread may read the state
 * while another sends commands. Each of the two must stay on a single thread
 * once start() has returned. */
class Client {
 public:
  explicit Client(ClientOptions options = ClientOptions());
  ~Client();
  Client(const Client&) = delete;
  Client& operator=(const Client&) = delete;

  /** Fetch the joint and IMU names and connect the streams. */
  void start(double timeout_s = 5.0);

  const std::vector<std::string>& joint_names() const;
  const std::vector<std::string>& imu_names() const;
  /** Index of a joint in the state matrix and in JointsCommand::joint_ids.
   * Throws std::out_of_range for an unknown name. */
  std::int32_t joint_id(const std::string& name) const;

  /** Receive the newest state, waiting up to `timeout_s` (zero polls, negative
   * waits forever). Returns false when no new state arrived. */
  bool sense(double timeout_s = 0.0);
  /** Newest state received by sense(). */
  const RobotState& state() const;
  /** Lower bound on the age of state() now, in server seconds (simulation
   * seconds when the server runs on a simulated clock); NaN before the first
   * state. See ClockDeltaEstimator. */
  double state_age() const;

  /** Publish one command, stamped with the monotonic clock. */
  void send(const JointsCommand& command);

  /** YAML string of a service reply's `data` field. Throws std::runtime_error
   * when the request times out or the server reports failure. */
  std::string request(const std::string& type, double timeout_s = 5.0) const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace xbot2_zmq

#endif  // XBOT2_ZMQ_CLIENT_HPP_
