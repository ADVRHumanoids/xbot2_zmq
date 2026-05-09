#include <xbot2/rt_plugin/control_plugin.h>
#include <zmq.hpp>

typedef double DoubleType;
typedef int IntType;

static_assert(sizeof(DoubleType) == 8, "DoubleType is not 64 bits");
static_assert(sizeof(IntType) == 4, "IntType is not 32 bits");

namespace XBot {


class ZmqIO : public ControlPlugin {

public:

    using ControlPlugin::ControlPlugin;

    bool on_initialize() override;
    void run() override;

private:

    void publish_state();
    void recv_cmd_v3();
    void handle_request_response();

    void getJointPosition(Eigen::Ref<Eigen::VectorXd> out) const;
    void getMotorPosition(Eigen::Ref<Eigen::VectorXd> out) const;
    void getJointPositionReference(Eigen::Ref<Eigen::VectorXd> out) const;
    void readToMat(const std::string& data_str, Eigen::Ref<Eigen::MatrixXd> out,
                      int rows, int cols);
    void readToMat(const std::string& data_str, Eigen::Ref<Eigen::MatrixXi> out,
                      int rows, int cols);
    void readToMat(const DoubleType* data, size_t byte_size, Eigen::Ref<Eigen::MatrixXd> out,
                      int rows, int cols);
    void readToMat(const IntType* data, size_t byte_size, Eigen::Ref<Eigen::MatrixXi> out,
                      int rows, int cols);
    std::vector<uint8_t> build_state_msg_raw(std::vector<std::string> imu_names,
                                    Eigen::Ref<Eigen::MatrixXd> joints_state,
                                    int joints_num,
                                    Eigen::Ref<Eigen::MatrixXd> imus_state);
    std::unique_ptr<zmq::context_t> context;
    std::unique_ptr<zmq::socket_t> cmd_subscriber, req_resp_socket, raw_publisher;

    uint32_t seq = 0;
    JointNameMap jmap;

    chrono::steady_clock::time_point cmd_timeout;

};

}