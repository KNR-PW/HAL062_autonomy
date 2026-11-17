#include "hal_communication/comm_server.h"

#include <sys/socket.h>

#include <iomanip>
#include <sstream>
#include <string>
// #include <rclcpp/rclcpp.hpp>

namespace HAL {

TCPClient::TCPClient(const TCPParams &serverParams)
    : serverParams_(serverParams) {}

TCPClient::~TCPClient() { close(sockFd_); }

auto TCPClient::initConnection_() -> std::optional<std::string> {
  if (sockFd_ > 0) {
      close(sockFd_);  // Close previous connection if any
      sockFd_ = -1;
  }

  // Initialize server address
  servAddr_.sin_family = AF_INET;
  servAddr_.sin_port = htons(serverParams_.port);
  if (inet_pton(AF_INET, serverParams_.IP.c_str(), &servAddr_.sin_addr) <= 0) {
      return "Invalid IP address";
  }

  // Create a new socket
  sockFd_ = socket(AF_INET, SOCK_STREAM, 0);
  if (sockFd_ < 0) {
      return "Cannot create the socket";
  }

  int status = connect(sockFd_, (struct sockaddr *)&servAddr_, sizeof(servAddr_));
  if (status < 0) {
      close(sockFd_);
      sockFd_ = -1;
      return "Cannot connect to the rover";
  }

  return {};
}

auto TCPClient::restartConnection() -> std::optional<std::string> {
  close(sockFd_);
  // RCLCPP_INFO(this->get_logger(), "RESTARTING CONNECTION");
  auto res = initConnection_();
  if (res.has_value()) {
    return res.value();
  }
  return {};
}

auto TCPClient::sendMessage(const std::string &data) -> std::optional<std::string> {
  if (sockFd_ <= 0) {
      // Try reconnecting if socket is closed
      auto res = initConnection_();
      if (res.has_value()) {
          return res.value();
      }
  }
  
  ssize_t res = send(sockFd_, data.c_str(), data.length(), 0);
  if (res < 0) {
      // If send fails, assume the socket is broken and restart connection
      close(sockFd_);
      sockFd_ = -1;  // Mark as closed
      return "Cannot send data to the rover, connection lost. Reconnecting...";
  }

  return {};
}

CommunicationServer::CommunicationServer()
  : Node("communication_server") {
  declare_parameter<std::string>(comm::CONST::PARAM_IP, "192.168.1.98");
  declare_parameter<int>(comm::CONST::PARAM_PORT, 8888);
  server_ = create_service<hal_interfaces::srv::Communication>(
      comm::CONST::SRV_NAME,
      std::bind(&CommunicationServer::sendData, this, std::placeholders::_1, std::placeholders::_2));

  TCPParams params;
  auto result = readConnectionParams_(params);
  if (result.has_value()) {
    throw std::runtime_error((*result).c_str());
  }
  tcpHandler_ = TCPClient(params);
  RCLCPP_INFO(this->get_logger(), "Initialize communication server");
}

auto CommunicationServer::convert2Hex_(int dec_num) -> std::string {
  std::stringstream stream;
  stream << std::uppercase << std::hex << dec_num;
  std::string result(stream.str());
  return result;
}

auto CommunicationServer::convert2Standard_(uint8_t id,
                                            std::vector<uint8_t> &data)
    -> std::string {
  std::string result;
  result = "#";  // # starts the frame
  data.insert(data.begin(), id);
  for (auto element : data) {
    auto hex_str = convert2Hex_(element);
    if (hex_str.size() == 1) {
      // for one-positional hex numbers
      hex_str.insert(0, 1, '0');
    }
    if (hex_str.size() != 2) {
      // when single hex is longer/shorter than 2 signs
      throw std::runtime_error("Message conversion error");
    }
    result.append(hex_str);
  }
  // for fills remaining fields with 'x' according to standard
  for (int i = result.size(); i < comm::CONST ::COMMUNICATION_FRAME_SIZE; i++) {
    result.append("x");
  }
  RCLCPP_INFO(this->get_logger(), "Sendin': '%s'", result.c_str());
  return result;
}


auto CommunicationServer::readConnectionParams_(TCPParams &params) -> std::optional<std::string> {
  try {
    RCLCPP_INFO(this->get_logger(), "READING CONNECTION PARAMS");
    params.IP = get_parameter(comm::CONST::PARAM_IP).as_string();
    params.port = get_parameter(comm::CONST::PARAM_PORT).as_int();
    RCLCPP_INFO(this->get_logger(), "Params, IP: %s, %d", params.IP.c_str(), params.port);
  } catch (const rclcpp::ParameterTypeException &e) {
    return "Error reading parameters: " + std::string(e.what());
  }
  return {};
}

bool CommunicationServer::sendData(
    const std::shared_ptr<hal_interfaces::srv::Communication::Request> req,
    std::shared_ptr<hal_interfaces::srv::Communication::Response> resp) {
      
  RCLCPP_INFO(this->get_logger(), "BEGIN SENDING DATA");
  
  if (req->comm_mode == req->TCPIP) {
    auto res = tcpHandler_.sendMessage(convert2Standard_(req->id, req->data));
    if (res.has_value()) {
      resp->success = false;
      resp->message = res.value();
      return true;
    }
  } else if (req->comm_mode == req->BLUETOOTH) {
    RCLCPP_INFO(this->get_logger(), "TRAP");
  } else {
    resp->success = false;
    resp->message = "No appropriate 'comm_mode' field in request";
    return false;
  }
  resp->success = true;
  return true;
}

}  // namespace HAL
