#include <rosconsole/macros_generated.h>
#include <ros/console.h>
#include <ros/init.h>
#include "car/arduino.h"

using namespace std;

Arduino::Arduino()
    : port(ioservice),
      device_path(""),
      speed(0),
      steer(0)
{
}

void Arduino::open(string path, unsigned int baudrate)
{
  ROS_INFO("Opening Arduino serial port...");
  try {
    port.open(device_path);
  } catch (...) {
    ROS_ERROR("Failed to open serial port %s", device_path.c_str());
  }

  if(port.is_open()) {
    try {
      port.set_option(boost::asio::serial_port_base::baud_rate(baudrate));
      port.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
      port.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    } catch (...) {
      ROS_WARN("Failed to set all options to port: %s", device_path.c_str());
    }

    ROS_INFO("Starting Arduino read/write thread...");
    threads_running = true;
    write_thread = boost::thread(boost::bind(&Arduino::write_run, this));
    read_thread = boost::thread(boost::bind(&Arduino::read_run, this));
  }
}

Arduino::~Arduino() {
  threads_running = false;
  write_thread.join();
  read_thread.join();
  port.close();
}

void Arduino::write_run() {
  while(threads_running) {
    stringstream command;
    command << STX << speed << ',' << steer << '\n';
    try {
      boost::asio::write(port, boost::asio::buffer(command.str().c_str(), command.str().length()));
    } catch(...) {
      ROS_ERROR("An error occurred while writing to %s.", device_path.c_str());
    }
    usleep(10000);
  }
}

void Arduino::read_run() {
  while(threads_running) {
    char in = 0;
    boost::asio::read(port, boost::asio::buffer(&in, 1));
    if(in == STX) {
      char * buffer = new char[5];
      boost::asio::read(port, boost::asio::buffer(buffer, 5));
      int tickCount = atoi(buffer);
      if(encoder_callback != NULL) {
        encoder_callback(tickCount);
      }
    }
    usleep(10000);
  }
}

void Arduino::setSteering(double val) {
  steer = val;
}

void Arduino::setSpeed(double val) {
  speed = val;
}

void Arduino::setEncoderCallback(void (*callback)(int)) {
  encoder_callback = callback;
}

string Arduino::padded_itoa(int i, int width) {
  ostringstream ss;
  ss << setw(width) << setfill('0') << i;
  return ss.str();
}