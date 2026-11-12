#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include "xbot2_zmq/client.hpp"

namespace nb = nanobind;
using namespace xbot2_zmq;

NB_MODULE(xbot2_zmq_client, m) {
  nb::class_<ClientOptions>(m, "ClientOptions")
      .def(nb::init<>())
      .def_rw("protocol", &ClientOptions::protocol)
      .def_rw("remote_ip", &ClientOptions::remote_ip)
      .def_rw("tcp_service_port", &ClientOptions::tcp_service_port)
      .def_rw("tcp_state_port", &ClientOptions::tcp_state_port)
      .def_rw("tcp_command_port", &ClientOptions::tcp_command_port)
      .def_rw("ipc_service_path", &ClientOptions::ipc_service_path)
      .def_rw("ipc_state_path", &ClientOptions::ipc_state_path)
      .def_rw("ipc_command_path", &ClientOptions::ipc_command_path);

  nb::class_<RobotState>(m, "RobotState")
      .def_ro("seq", &RobotState::seq)
      .def_ro("stamp", &RobotState::stamp)
      .def_ro("joints", &RobotState::joints)
      .def_ro("imus", &RobotState::imus);

  nb::class_<JointsCommand>(m, "JointsCommand")
      .def(nb::init<>())
      .def_rw("joint_ids", &JointsCommand::joint_ids)
      .def_rw("pvesd", &JointsCommand::pvesd)
      .def_rw("ctrl_mode", &JointsCommand::ctrl_mode);

  m.def("encode_command",
        [](std::uint32_t seq, std::uint64_t stamp_ns, std::uint64_t session,
           const JointsCommand& command) {
          const auto raw = encode_command(seq, stamp_ns, session, command);
          return nb::bytes(reinterpret_cast<const char*>(raw.data()),
                           raw.size());
        },
        nb::arg("seq"), nb::arg("stamp_ns"), nb::arg("client_session_id"),
        nb::arg("command"));
  m.def("decode_state",
        [](nb::bytes message, std::size_t joints_num, std::size_t imus_num) {
          RobotState state;
          decode_state(message.c_str(), message.size(), joints_num, imus_num,
                       state);
          return state;
        },
        nb::arg("message"), nb::arg("joints_num"), nb::arg("imus_num"));

  nb::class_<ClockDeltaEstimator>(m, "ClockDeltaEstimator")
      .def(nb::init<std::size_t, std::size_t>(), nb::arg("window") = 2000,
           nb::arg("refit_every") = 50)
      .def("reset", &ClockDeltaEstimator::reset)
      .def("update", &ClockDeltaEstimator::update, nb::arg("server_stamp"),
           nb::arg("recv_time"))
      .def("server_time", &ClockDeltaEstimator::server_time, nb::arg("now"))
      .def("age", &ClockDeltaEstimator::age, nb::arg("server_stamp"),
           nb::arg("now"))
      .def_prop_ro("rate", &ClockDeltaEstimator::rate)
      .def_prop_ro("offset", &ClockDeltaEstimator::offset);

  nb::class_<Client>(m, "Client")
      .def(nb::init<ClientOptions>(), nb::arg("options") = ClientOptions())
      .def("start", &Client::start, nb::arg("timeout_s") = 5.0,
           nb::call_guard<nb::gil_scoped_release>())
      .def_prop_ro("joint_names", &Client::joint_names)
      .def_prop_ro("imu_names", &Client::imu_names)
      .def("joint_id", &Client::joint_id, nb::arg("name"))
      .def("sense", &Client::sense, nb::arg("timeout_s") = 0.0,
           nb::call_guard<nb::gil_scoped_release>())
      .def_prop_ro("state", &Client::state)
      .def("state_age", &Client::state_age)
      .def("server_time", &Client::server_time)
      .def("send", &Client::send, nb::arg("command"))
      .def("request", &Client::request, nb::arg("type"),
           nb::arg("timeout_s") = 5.0,
           nb::call_guard<nb::gil_scoped_release>());
}
