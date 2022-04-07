#ifndef __I2CCOMMUNICATOR_H__
#define __I2CCOMMUNICATOR_H__

class I2cCommunicator {
 public:
  I2cCommunicator(int bus_number = 1);
  ~I2cCommunicator();
  int  read   (unsigned char address);
  int  write  (unsigned char address, unsigned char value);
  char getFile();

 private:
  void reportError(int error);
  int file_;
};

#endif  // __I2CCOMMUNICATOR_H__
