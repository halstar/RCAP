import i2c
import math
import time

# MPU9250 hard coded addresses
I2C_BUS_NUMBER = 1
IMU_ADDRESS    = 0x68
MAG_ADDRESS    = 0x0C

# Accelerometers & gyroscopes registers
USER_CTRL    = 0x6A
PWR_MGMT_1   = 0x6B
GYRO_CONFIG  = 0x18
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
INT_PIN_CFG  = 0x37
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47
XA_OFFSET_H  = 0x06
YA_OFFSET_H  = 0x08
ZA_OFFSET_H  = 0x0A
XG_OFFSET_H  = 0x13
YG_OFFSET_H  = 0x15
ZG_OFFSET_H  = 0x17

# Magnetometers registers
AK8963_ST1   = 0x02
AK8963_HXL   = 0x03
AK8963_HYL   = 0x05
AK8963_HZL   = 0x07
AK8963_ST2   = 0x09
AK8963_CNTL1 = 0x0A
AK8963_ASAX  = 0x10
AK8963_ASAY  = 0x11
AK8963_ASAZ  = 0x12

# Accelerometers & gyroscopes registers
ACCELERATION_SCALE_FACTOR = 16384.0
GYROSCOPE_SCALE_FACTOR    =   131.0

class ImuDevice:

    def __init__(self):

        self.acceleration_x = 0
        self.acceleration_y = 0
        self.acceleration_z = 0

        self.gyroscope_x = 0
        self.gyroscope_y = 0
        self.gyroscope_z = 0

        self.magnetometer_x = 0
        self.magnetometer_y = 0
        self.magnetometer_z = 0

        self.acceleration_offset_x = 0
        self.acceleration_offset_y = 0
        self.acceleration_offset_z = 0

        self.gyroscope_offset_x = 0
        self.gyroscope_offset_y = 0
        self.gyroscope_offset_z = 0

        self.gyroscope_drift_correction_x = 0
        self.gyroscope_drift_correction_y = 0
        self.gyroscope_drift_correction_z = 0

        self.magnetometer_scale_x = 1.0
        self.magnetometer_scale_y = 1.0
        self.magnetometer_scale_z = 1.0   

        self.magnetometer_offset_x = 0
        self.magnetometer_offset_y = 0
        self.magnetometer_offset_z = 0   

        self.magnetometer_adjustment_x = 0
        self.magnetometer_adjustment_y = 0
        self.magnetometer_adjustment_z = 0 

        self.roll  = 0
        self.pitch = 0
        self.yaw   = 0

        self.roll_rate  = 0
        self.pitch_rate = 0
        self.yaw_rate   = 0

        self.last_gyroscope_read_time  = 0.0
        self.gyroscope_read_delta_time = 0.0

        self.i2c_device = i2c.I2cDevice(I2C_BUS_NUMBER)

        # Write power management register
        self.i2c_device.write_byte(IMU_ADDRESS, PWR_MGMT_1, 0x01)

        # Write configuration register
        self.i2c_device.write_byte(IMU_ADDRESS, CONFIG, 0x02)

        # Write gyroscope configuration register
        self.i2c_device.write_byte(IMU_ADDRESS, GYRO_CONFIG, 0x00)

        # Write sample rate divider register
        self.i2c_device.write_byte(IMU_ADDRESS, SMPLRT_DIV, 0x04)

    def __read_word__(self, address, register, isLittleEndian=True):

        # Acceleration and gyroscope data are 16-bit
        if (isLittleEndian == True):
            higher_byte = self.i2c_device.read_byte(address, register    )
            lower_byte  = self.i2c_device.read_byte(address, register + 1)
        else:
            lower_byte  = self.i2c_device.read_byte(address, register    )
            higher_byte = self.i2c_device.read_byte(address, register + 1)

        # Concatenate higher and lower bytes
        value = (higher_byte << 8) + lower_byte

        # Get signed value
        if value > 32768:
            value = value - 65536
        return value

    def __write_word__(self, address, register, value):

        # Acceleration and gyroscope data are 16-bit
        self.i2c_device.write_byte(address, register    , value >> 8    )
        self.i2c_device.write_byte(address, register + 1, value & 0x00FF)

    def __get_x_rotation__(self):
        radians = math.atan2(self.acceleration_y, math.sqrt(self.acceleration_x ** 2 +
                                                            self.acceleration_z ** 2))
        return math.degrees(radians)

    def __get_y_rotation__(self):
        radians = math.atan2(self.acceleration_x, math.sqrt(self.acceleration_y ** 2 +
                                                            self.acceleration_z ** 2))
        return -math.degrees(radians)

    def __get_z_rotation__(self):
        radians = math.atan2(-self.get_y_magnetometer_scaled(), self.get_x_magnetometer_scaled())
        return math.degrees(radians) + 180 

    def reset(self):
        self.i2c_device.write_byte(IMU_ADDRESS, PWR_MGMT_1, 0x81)
        time.sleep(1)
        self.i2c_device.write_byte(IMU_ADDRESS, PWR_MGMT_1, 0x01)
        time.sleep(1)

    def reset_offsets(self):

        self.__write_word__(IMU_ADDRESS, XA_OFFSET_H, 0)
        self.__write_word__(IMU_ADDRESS, YA_OFFSET_H, 0)
        self.__write_word__(IMU_ADDRESS, ZA_OFFSET_H, 0)

        self.__write_word__(IMU_ADDRESS, XG_OFFSET_H, 0)
        self.__write_word__(IMU_ADDRESS, YG_OFFSET_H, 0)
        self.__write_word__(IMU_ADDRESS, ZG_OFFSET_H, 0)

        self.acceleration_offset_x = 0
        self.acceleration_offset_y = 0
        self.acceleration_offset_z = 0

        self.gyroscope_offset_x = 0
        self.gyroscope_offset_y = 0
        self.gyroscope_offset_z = 0

        self.gyroscope_drift_correction_x = 0
        self.gyroscope_drift_correction_y = 0
        self.gyroscope_drift_correction_z = 0

        self.magnetometer_scale_x = 1.0
        self.magnetometer_scale_y = 1.0
        self.magnetometer_scale_z = 1.0

        self.magnetometer_offset_x = 0
        self.magnetometer_offset_y = 0
        self.magnetometer_offset_z = 0   

        self.magnetometer_adjustment_x = 0
        self.magnetometer_adjustment_y = 0
        self.magnetometer_adjustment_z = 0 

    def setup_magnetometer(self):

        # BYPASS_EN enable
        self.i2c_device.write_byte(IMU_ADDRESS, INT_PIN_CFG, 0x02) 

        # Disable master
        self.i2c_device.write_byte(IMU_ADDRESS, USER_CTRL, 0x00)

        # Set power down mode
        self.i2c_device.write_byte(MAG_ADDRESS, AK8963_CNTL1, 0x00)
        time.sleep(0.1)

        # Set Fuse ROM access mode
        self.i2c_device.write_byte(MAG_ADDRESS, AK8963_CNTL1, 0x0F)
        time.sleep(0.1)

        # Read Fuse ROM data
        self.magnetometer_adjustment_x = (self.i2c_device.read_byte(MAG_ADDRESS, AK8963_ASAX) - 128) * 0.5 / 128 + 1;
        self.magnetometer_adjustment_y = (self.i2c_device.read_byte(MAG_ADDRESS, AK8963_ASAY) - 128) * 0.5 / 128 + 1;
        self.magnetometer_adjustment_z = (self.i2c_device.read_byte(MAG_ADDRESS, AK8963_ASAZ) - 128) * 0.5 / 128 + 1;

        # Set power down mode
        self.i2c_device.write_byte(MAG_ADDRESS, AK8963_CNTL1, 0x00)
        time.sleep(0.1)
        
        # Set scale and continous mode
        self.i2c_device.write_byte(MAG_ADDRESS, AK8963_CNTL1, 0x12)
        time.sleep(0.1)

        # Set magnetometer resolution
        self.magnetometer_resolution = 4912.0 / 32760.0

    def read_acceleration_data(self):
        self.acceleration_x = self.__read_word__(IMU_ADDRESS, ACCEL_XOUT_H) - self.acceleration_offset_x
        self.acceleration_y = self.__read_word__(IMU_ADDRESS, ACCEL_YOUT_H) - self.acceleration_offset_y
        self.acceleration_z = self.__read_word__(IMU_ADDRESS, ACCEL_ZOUT_H) - self.acceleration_offset_z

    def read_gyroscope_data(self):
        self.gyroscope_x = self.__read_word__(IMU_ADDRESS, GYRO_XOUT_H) - self.gyroscope_offset_x
        self.gyroscope_y = self.__read_word__(IMU_ADDRESS, GYRO_YOUT_H) - self.gyroscope_offset_y
        self.gyroscope_z = self.__read_word__(IMU_ADDRESS, GYRO_ZOUT_H) - self.gyroscope_offset_z

        current_read_time = time.time()

        if self.last_gyroscope_read_time != 0:
            self.gyroscope_read_delta_time = current_read_time - self.last_gyroscope_read_time

        self.last_gyroscope_read_time = current_read_time

    def read_magnetometer_data(self):
        self.magnetometer_x = self.__read_word__(MAG_ADDRESS, AK8963_HXL, False) - self.magnetometer_offset_x
        self.magnetometer_y = self.__read_word__(MAG_ADDRESS, AK8963_HYL, False) - self.magnetometer_offset_y
        self.magnetometer_z = self.__read_word__(MAG_ADDRESS, AK8963_HZL, False) - self.magnetometer_offset_z
       
        # Read Status 2 register to trigger next reading
        self.i2c_device.read_byte(MAG_ADDRESS, AK8963_ST2)

    def correct_gyroscope_data(self):

        self.gyroscope_x -= self.gyroscope_drift_correction_x * self.gyroscope_read_delta_time
        self.gyroscope_y -= self.gyroscope_drift_correction_y * self.gyroscope_read_delta_time
        self.gyroscope_z -= self.gyroscope_drift_correction_z * self.gyroscope_read_delta_time

        self.gyroscope_x = int(self.gyroscope_x)
        self.gyroscope_y = int(self.gyroscope_y)
        self.gyroscope_z = int(self.gyroscope_z)

    def get_x_acceleration(self):
        return self.acceleration_x

    def get_y_acceleration(self):
        return self.acceleration_y

    def get_z_acceleration(self):
        return self.acceleration_z

    def get_x_gyroscope(self):
        return self.gyroscope_x

    def get_y_gyroscope(self):
        return self.gyroscope_y

    def get_z_gyroscope(self):
        return self.gyroscope_z

    def get_x_magnetometer(self):
        return self.magnetometer_x

    def get_y_magnetometer(self):
        return self.magnetometer_y

    def get_z_magnetometer(self):
        return self.magnetometer_z

    def get_x_acceleration_scaled(self):
        return self.acceleration_x / ACCELERATION_SCALE_FACTOR

    def get_y_acceleration_scaled(self):
        return self.acceleration_y / ACCELERATION_SCALE_FACTOR

    def get_z_acceleration_scaled(self):
        return self.acceleration_z / ACCELERATION_SCALE_FACTOR

    def get_x_gyroscope_scaled(self):
        return self.gyroscope_x / GYROSCOPE_SCALE_FACTOR

    def get_y_gyroscope_scaled(self):
        return self.gyroscope_y / GYROSCOPE_SCALE_FACTOR

    def get_z_gyroscope_scaled(self):
        return self.gyroscope_z / GYROSCOPE_SCALE_FACTOR

    def get_x_magnetometer_scaled(self):
        return self.magnetometer_x * self.magnetometer_adjustment_x * self.magnetometer_resolution * self.magnetometer_scale_x

    def get_y_magnetometer_scaled(self):
        return self.magnetometer_y * self.magnetometer_adjustment_y * self.magnetometer_resolution * self.magnetometer_scale_y

    def get_z_magnetometer_scaled(self):
        return self.magnetometer_z * self.magnetometer_adjustment_z * self.magnetometer_resolution * self.magnetometer_scale_z

    def compute_angles(self):
        self.roll  = self.__get_x_rotation__()
        self.pitch = self.__get_y_rotation__()
        self.yaw   = self.__get_z_rotation__()

    def compute_rates(self):
        self.roll_rate  = self.get_x_gyroscope_scaled()
        self.pitch_rate = self.get_y_gyroscope_scaled()
        self.yaw_rate   = self.get_z_gyroscope_scaled()

    def get_x_acceleration_offset(self):
        return self.acceleration_offset_x

    def set_x_acceleration_offset(self, offset):
        self.acceleration_offset_x = offset

    def get_y_acceleration_offset(self):
        return self.acceleration_offset_y

    def set_y_acceleration_offset(self, offset):
        self.acceleration_offset_y = offset

    def get_z_acceleration_offset(self):
        return self.acceleration_offset_z

    def set_z_acceleration_offset(self, offset):
        self.acceleration_offset_z = offset

    def get_x_gyroscope_offset(self):
        return self.gyroscope_offset_x

    def set_x_gyroscope_offset(self, offset):
        self.gyroscope_offset_x = offset

    def get_y_gyroscope_offset(self):
        return self.gyroscope_offset_y

    def set_y_gyroscope_offset(self, offset):
        self.gyroscope_offset_y = offset

    def get_z_gyroscope_offset(self):
        return self.gyroscope_offset_z

    def set_z_gyroscope_offset(self, offset):
        self.gyroscope_offset_z = offset

    def get_x_gyroscope_drift_correction(self):
        return self.gyroscope_drift_correction_x

    def set_x_gyroscope_drift_correction(self, correction):
        self.gyroscope_drift_correction_x = correction

    def get_y_gyroscope_drift_correction(self):
        return self.gyroscope_drift_correction_y

    def set_y_gyroscope_drift_correction(self, correction):
        self.gyroscope_drift_correction_y = correction

    def get_z_gyroscope_drift_correction(self):
        return self.gyroscope_drift_correction_z

    def set_z_gyroscope_drift_correction(self, correction):
        self.gyroscope_drift_correction_z = correction

    def get_x_magnetometer_scale(self):
        return self.magnetometer_scale_x

    def set_x_magnetometer_scale(self, scale):
        self.magnetometer_scale_x = scale

    def get_y_magnetometer_scale(self):
        return self.magnetometer_scale_y

    def set_y_magnetometer_scale(self, scale):
        self.magnetometer_scale_y = scale

    def get_z_magnetometer_scale(self):
        return self.magnetometer_scale_z

    def set_z_magnetometer_scale(self, scale):
        self.magnetometer_scale_z = scale

    def get_x_magnetometer_offset(self):
        return self.magnetometer_offset_x

    def set_x_magnetometer_offset(self, offset):
        self.magnetometer_offset_x = offset

    def get_y_magnetometer_offset(self):
        return self.magnetometer_offset_y

    def set_y_magnetometer_offset(self, offset):
        self.magnetometer_offset_y = offset

    def get_z_magnetometer_offset(self):
        return self.magnetometer_offset_z

    def set_z_magnetometer_offset(self, offset):
        self.magnetometer_offset_z = offset

    def get_x_magnetometer_adjustment(self):
        return self.magnetometer_adjustment_x

    def set_x_magnetometer_adjustment(self, offset):
        self.magnetometer_adjustment_x = offset

    def get_y_magnetometer_adjustment(self):
        return self.magnetometer_adjustment_y

    def set_y_magnetometer_adjustment(self, offset):
        self.magnetometer_adjustment_y = offset

    def get_z_magnetometer_adjustment(self):
        return self.magnetometer_adjustment_z

    def set_z_magnetometer_adjustment(self, offset):
        self.magnetometer_adjustment_z = offset

    def get_roll(self):
        return self.roll

    def get_pitch(self):
        return self.pitch

    def get_yaw(self):
        return self.yaw

    def get_roll_rate(self):
        return self.roll_rate

    def get_pitch_rate(self):
        return self.pitch_rate

    def get_yaw_rate(self):
        return self.yaw_rate

    def print_info(self):

        print("X / Y / Z acceleration offsets       : {} / {} / {}".format(self.acceleration_offset_x       , self.acceleration_offset_y       , self.acceleration_offset_z       ))
        print("X / Y / Z gyroscope    offsets       : {} / {} / {}".format(self.gyroscope_offset_x          , self.gyroscope_offset_y          , self.gyroscope_offset_z          ))
        print("X / Y / Z gyroscope drift corrections: {} / {} / {}".format(self.gyroscope_drift_correction_x, self.gyroscope_drift_correction_y, self.gyroscope_drift_correction_z))
        print("X / Y / Z magnetometer offsets       : {} / {} / {}".format(self.magnetometer_offset_x       , self.magnetometer_offset_y       , self.magnetometer_offset_z       ))
        print("")
        print("X / Y / Z magnetometer scales     : {:6.2f} / {:6.2f} / {:6.2f}".format(self.magnetometer_scale_x        , self.magnetometer_scale_y        , self.magnetometer_scale_z        ))
        print("X / Y / Z magnetometer adjustments: {:6.2f} / {:6.2f} / {:6.2f}".format(self.magnetometer_adjustment_x   , self.magnetometer_adjustment_y   , self.magnetometer_adjustment_z   ))

        self.read_acceleration_data()
        self.read_gyroscope_data   ()
        self.read_magnetometer_data()
        self.correct_gyroscope_data()
        self.compute_angles        ()

        print("")
        print("X / Y / Z acceleration raw data:  {} / {} / {}".format(self.acceleration_x, self.acceleration_y, self.acceleration_z))
        print("X / Y / Z gyroscope    raw data:  {} / {} / {}".format(self.gyroscope_x   , self.gyroscope_y   , self.gyroscope_z   ))
        print("X / Y / Z magnetometer raw data:  {} / {} / {}".format(self.magnetometer_x, self.magnetometer_y, self.magnetometer_z))
        print("")
        print("X / Y / Z acceleration scaled data: {:6.2f} / {:6.2f} / {:6.2f}".format(self.get_x_acceleration_scaled(), self.get_y_acceleration_scaled(), self.get_z_acceleration_scaled()))
        print("X / Y / Z gyroscope    scaled data: {:6.2f} / {:6.2f} / {:6.2f}".format(self.get_x_gyroscope_scaled   (), self.get_y_gyroscope_scaled   (), self.get_z_gyroscope_scaled   ()))
        print("X / Y / Z magnetometer scaled data: {:6.2f} / {:6.2f} / {:6.2f}".format(self.get_x_magnetometer_scaled(), self.get_y_magnetometer_scaled(), self.get_z_magnetometer_scaled()))
        print("")
        print("Roll : {:6.2f} - Pitch: {:6.2f} - Yaw: {:6.2f}".format(self.roll, self.pitch, self.yaw))
