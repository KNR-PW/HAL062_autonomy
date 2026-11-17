#pragma once
#include <arpa/inet.h>


#include "hal_interfaces/srv/communication.hpp"  // probably correct path
#include "rclcpp/rclcpp.hpp"

namespace HAL {
namespace comm::CONST {

/** Number of bytes in the frame */
const int COMMUNICATION_FRAME_SIZE = 19;
/** Maximal number of connection attempts before returning failure */
const int MAX_ATTEMPT_COUNT = 4;
/** Name of communication service */
const std::string SRV_NAME = "comm_server";
/** Name of communication params on parameter server */
const std::string PARAM_PREFIX = "communication";
/** Name of server IP on parameter server  */
const std::string PARAM_IP = "rover_IP";
/** Name of server port on parameter server  */
const std::string PARAM_PORT = "rover_port";

}  // namespace comm::CONST

/** Parameters of TCP connection  */
struct TCPParams {
  /** IP of TCP server  */
  std::string IP;
  /** port of TCP server  */
  int port;
};

class IHandler {
 public:
  /**
   * @brief Restarts connection by closing and reopening the socket
   * @returns if OK empty optional, if error optional with error string
   */
  virtual std::optional<std::string> restartConnection() = 0;

  /**
   * @brief Sends message to TCP server
   * @param data meassage to server
   * @returns if OK empty optional, if error optional with error string
   */
  virtual std::optional<std::string> sendMessage(const std::string &data) = 0;
};

/** Handling of TCP connection  */
class TCPClient : public IHandler {
 private:
  /** Parameters of TCP connection  */
  TCPParams serverParams_;
  /** IP address of TCP server  */
  struct sockaddr_in servAddr_;
  /** File descriptor of socket  */
  int sockFd_ = 0;

  /**
   * @brief Setup of connection params
   * @returns if OK empty optional, if error optional with error string
   */
  std::optional<std::string> initConnection_();

 public:
  TCPClient() {}
  TCPClient(const TCPParams &serverParams);
  ~TCPClient();

  /**
   * @brief Restarts connection by closing and reopening the socket
   * @returns if OK empty optional, if error optional with error string
   */
  std::optional<std::string> restartConnection() override;

  /**
   * @brief Sends message to TCP server
   * @param data meassage to server
   * @returns if OK empty optional, if error optional with error string
   */
  std::optional<std::string> sendMessage(const std::string &data) override;
};

/** Handling of bluetooth connection  */
class BTClient : public IHandler {
 public:
  ~BTClient() {}

  /**
   * @brief Restarts connection by closing and reopening the socket
   * @returns if OK empty optional, if error optional with error string
   */
  std::optional<std::string> restartConnection() override { return {}; }

  /**
   * @brief Sends message to TCP server
   * @param data meassage to server
   * @returns if OK empty optional, if error optional with error string
   */
  std::optional<std::string> sendMessage(const std::string &data) override {
    return {};
  }
};

/** ROS server class that handles [ROS <-> rover] communication */
class CommunicationServer : public rclcpp::Node {
 private:
  rclcpp::Service<hal_interfaces::srv::Communication>::SharedPtr server_;
  /** Handler of TCP connection  */
  TCPClient tcpHandler_;
  /** Handler of bluetooth connection  */
  BTClient btHandler_;

  /**
   * @brief Converts decimal numbers to hex
   * @param dec_num decimal number
   * @returns number in hex
   */
  std::string convert2Hex_(int dec_num);

  /**
   * @brief Converts decimal numbers to values to hex according to communication
   * standard described in docs
   * @param id ID of message
   * @param data vector with message data
   * @returns string of data formatted according to communication standard
   */
  std::string convert2Standard_(uint8_t id, std::vector<uint8_t> &data);

  /**
   * @brief Reads connection params from parameter server
   * @param[out] params returns connection parameters read from parameter server
   * @returns if OK empty optional, if error optional with error string
   */
  std::optional<std::string> readConnectionParams_(TCPParams &params);

 public:
  CommunicationServer();

  /**
   * @brief Handles the service request, sends messages over requested protocol
   * (TCP or Bluetooth)
   * @param req request handler
   * @param resp response handler
   * @returns true if succeeded, false otherwise
   */
  bool sendData(const std::shared_ptr<hal_interfaces::srv::Communication::Request> req,
                std::shared_ptr<hal_interfaces::srv::Communication::Response> resp);
};
}  // namespace HAL
