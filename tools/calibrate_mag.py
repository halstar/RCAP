import os
import yaml
import imu
import time

CALIBRATION_DURATION          = 30
TRIAL_DURATION                = 30
PROGRESS_INDICATION_STEP_IN_S = 1

SETUP_FILE = "imu_calibration.yaml"


def main():

    os.system('clear')

    print('')
    print('/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/ ')
    print('|  Magnetometer calibration starting... | ')
    print('\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\ ')
    print('')

    with open(SETUP_FILE, 'r') as yaml_file:
        setup_data = yaml.safe_load(yaml_file)

    imu_device = imu.ImuDevice()

    imu_device.reset             ()
    imu_device.reset_offsets     ()
    imu_device.setup_magnetometer()

    imu_device.print_info()

    x_magnetometer_measure = 0
    y_magnetometer_measure = 0
    z_magnetometer_measure = 0

    x_magnetometer_min = 32667
    y_magnetometer_min = 32667
    z_magnetometer_min = 32667

    x_magnetometer_max = -32667
    y_magnetometer_max = -32667
    z_magnetometer_max = -32667

    print('')
    print('Please move magnetometer on all axis for {}s '.format(CALIBRATION_DURATION), end = '', flush = True)

    start_time    = time.time()
    progress_time = time.time()

    while time.time() - start_time < CALIBRATION_DURATION:

        if time.time() - progress_time > PROGRESS_INDICATION_STEP_IN_S:
            print('.', end = '', flush = True)
            progress_time = time.time()

        imu_device.read_magnetometer_data()

        x_magnetometer_measure = imu_device.get_x_magnetometer()
        y_magnetometer_measure = imu_device.get_y_magnetometer()
        z_magnetometer_measure = imu_device.get_z_magnetometer()

        if   x_magnetometer_measure < x_magnetometer_min:
             x_magnetometer_min     = x_magnetometer_measure
        if x_magnetometer_measure > x_magnetometer_max:
             x_magnetometer_max     = x_magnetometer_measure

        if   y_magnetometer_measure < y_magnetometer_min:
             y_magnetometer_min     = y_magnetometer_measure
        if y_magnetometer_measure > y_magnetometer_max:
             y_magnetometer_max     = y_magnetometer_measure

        if   z_magnetometer_measure < z_magnetometer_min:
             z_magnetometer_min     = z_magnetometer_measure
        if z_magnetometer_measure > z_magnetometer_max:
             z_magnetometer_max     = z_magnetometer_measure

        # Refresh rate is 100Hz, so let's wait for 10ms
        time.sleep(0.01)

    print('')

    x_magnetometer_range = (x_magnetometer_max - x_magnetometer_min) / 2.0
    y_magnetometer_range = (y_magnetometer_max - y_magnetometer_min) / 2.0
    z_magnetometer_range = (z_magnetometer_max - z_magnetometer_min) / 2.0

    average_range = (x_magnetometer_range + y_magnetometer_range + z_magnetometer_range) / 3.0

    x_magnetometer_scale = average_range / x_magnetometer_range
    y_magnetometer_scale = average_range / y_magnetometer_range
    z_magnetometer_scale = average_range / z_magnetometer_range

    imu_device.set_x_magnetometer_scale(x_magnetometer_scale)
    imu_device.set_y_magnetometer_scale(y_magnetometer_scale)
    imu_device.set_z_magnetometer_scale(z_magnetometer_scale)

    x_magnetometer_offset = int((x_magnetometer_min + x_magnetometer_max) / 2.0)
    y_magnetometer_offset = int((y_magnetometer_min + y_magnetometer_max) / 2.0)
    z_magnetometer_offset = int((z_magnetometer_min + z_magnetometer_max) / 2.0)

    imu_device.set_x_magnetometer_offset(x_magnetometer_offset)
    imu_device.set_y_magnetometer_offset(y_magnetometer_offset)
    imu_device.set_z_magnetometer_offset(z_magnetometer_offset)

    print('')
    imu_device.print_info()

    print('')
    print('Now you can check calibration result for {}s '.format(TRIAL_DURATION), end = '', flush = True)
    print('')

    i             = 0
    start_time    = time.time()
    progress_time = time.time()

    while time.time() - start_time < TRIAL_DURATION:

        imu_device.read_magnetometer_data()
        imu_device.compute_angles        ()
        print('Yaw = {:6.2f}'.format(imu_device.get_yaw()), end = '\r', flush = True)
        time.sleep(0.1)

    print('')

    setup_data['mpu9250driver']['ros__parameters']['magnetometer_x_offset'] = x_magnetometer_offset
    setup_data['mpu9250driver']['ros__parameters']['magnetometer_y_offset'] = y_magnetometer_offset
    setup_data['mpu9250driver']['ros__parameters']['magnetometer_z_offset'] = z_magnetometer_offset

    setup_data['mpu9250driver']['ros__parameters']['magnetometer_x_scale'] = x_magnetometer_scale
    setup_data['mpu9250driver']['ros__parameters']['magnetometer_y_scale'] = y_magnetometer_scale
    setup_data['mpu9250driver']['ros__parameters']['magnetometer_z_scale'] = z_magnetometer_scale

    with open(SETUP_FILE, 'w') as yaml_file:
        yaml.dump(setup_data, yaml_file)

    print('')
    print('/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/ ')
    print('| Magnetometer calibration done!  | ')
    print('\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\ ')
    print('')


if __name__ == '__main__':

    try:

        main()

    except KeyboardInterrupt:
        print('Keyboard interrupt...')

    except Exception as e:
        print('Error: ' + str(e))
