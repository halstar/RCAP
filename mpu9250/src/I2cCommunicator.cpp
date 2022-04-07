#include "I2cCommunicator.h"

extern "C" {
#include <errno.h>
#include <fcntl.h>
#include <i2c/smbus.h>
#include <linux/i2c-dev.h>
#include <string.h>
#include <unistd.h>
}

#include <iostream>
#include <string>

I2cCommunicator::I2cCommunicator(int bus_number)
{
  const std::string filename_ = "/dev/i2c-" + std::to_string(bus_number);
  std::cout << filename_ << std::endl;
  file_ = open(filename_.c_str(), O_RDWR);
  if (file_ < 0) {
    std::cerr << "Failed to open file descriptor! Check your bus number! Errno: "
              << strerror(errno);
    exit(1);
  }
}

I2cCommunicator::~I2cCommunicator() { close(file_); }

int I2cCommunicator::read(unsigned char address)
{
  int result = i2c_smbus_read_byte_data(file_, address);
  if (result < 0) reportError(errno);
  return result;
}

int I2cCommunicator::write(unsigned char address, unsigned char value)
{
  int result = i2c_smbus_write_byte_data(file_, address, value);
  if (result < 0) reportError(errno);
  return result;
}

char I2cCommunicator::getFile() { return file_; }

void I2cCommunicator::reportError(int error)
{
  std::cerr << "Error! Errno: " << strerror(error);
}
