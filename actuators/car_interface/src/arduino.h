#pragma once

#include <iostream>
#include <boost/asio.hpp>
#include <boost/thread.hpp>

#define STX ((char)2)

class Arduino {

public:
  Arduino();

  void open(std::string serial_path, unsigned int baudrate);

  ~Arduino();

  void setSteering(double angle);

  void setSpeed(double speed);

  void setEncoderCallback(void (*callback)(int));

private:
  boost::asio::io_service ioservice;
  boost::asio::serial_port port;

  boost::thread write_thread;
  boost::thread read_thread;

  boost::mutex port_mutex;

  std::string device_path;

  void write_run();
  void read_run();

  void (*encoder_callback)(int);

  std::string padded_itoa(int i, int width);

  float speed; // m/s
  float steer; // rad

  bool threads_running;
};