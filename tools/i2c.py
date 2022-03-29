import smbus


class I2cDevice:

    def __init__(self, bus):
        self.bus = smbus.SMBus(bus)

    def read_byte(self, address, register):
        return self.bus.read_byte_data(address, register)

    def write_byte(self, address, register, value):
        self.bus.write_byte_data(address, register, value)
        return

    def read_block_data(self, address, register, size):
        return self.bus.read_i2c_block_data(address, register, size)
