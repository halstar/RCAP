import os
import yaml 
import imu
import time

CALIBRATION_LOOP_COUNT        = 50000
DRIFT_CALIBRATION_DURATION    = 120
PROGRESS_INDICATION_STEP_IN_S = 1

SETUP_FILE = "imu_calibration.yaml"


def main():

    os.system('clear')

    print('')
    print('/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\ ')
    print('|  IMU calibration starting... | ')
    print('\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/ ')
    print('')

    with open(SETUP_FILE, 'r') as yaml_file:
        setup_data = yaml.safe_load(yaml_file)

    imu_device = imu.ImuDevice()

    imu_device.reset             ()
    imu_device.reset_offsets     ()
    imu_device.setup_magnetometer()

    imu_device.print_info()

    x_acceleration_measure = 0
    y_acceleration_measure = 0
    z_acceleration_measure = 0

    x_gyroscope_measure = 0
    y_gyroscope_measure = 0
    z_gyroscope_measure = 0

    print('')
    print('Computing accelerometers & gyroscopes offsets ', end = '', flush = True)

    progress_time = time.time()

    for i in range(0, CALIBRATION_LOOP_COUNT):

        if time.time() - progress_time > PROGRESS_INDICATION_STEP_IN_S:
            print('.', end = '', flush = True)
            progress_time = time.time()

        imu_device.read_acceleration_data()
        imu_device.read_gyroscope_data   ()

        x_acceleration_measure += imu_device.get_x_acceleration()
        y_acceleration_measure += imu_device.get_y_acceleration()
        z_acceleration_measure += imu_device.get_z_acceleration()

        x_gyroscope_measure += imu_device.get_x_gyroscope()
        y_gyroscope_measure += imu_device.get_y_gyroscope()
        z_gyroscope_measure += imu_device.get_z_gyroscope()

    print('')

    x_acceleration_measure /= CALIBRATION_LOOP_COUNT
    y_acceleration_measure /= CALIBRATION_LOOP_COUNT
    z_acceleration_measure /= CALIBRATION_LOOP_COUNT

    x_gyroscope_measure /= CALIBRATION_LOOP_COUNT
    y_gyroscope_measure /= CALIBRATION_LOOP_COUNT
    z_gyroscope_measure /= CALIBRATION_LOOP_COUNT

    x_acceleration_offset = int(x_acceleration_measure)
    y_acceleration_offset = int(y_acceleration_measure)
    z_acceleration_offset = int(z_acceleration_measure) - 16384

    x_gyroscope_offset = int(x_gyroscope_measure)
    y_gyroscope_offset = int(y_gyroscope_measure)
    z_gyroscope_offset = int(z_gyroscope_measure)

    imu_device.set_x_acceleration_offset(x_acceleration_offset)
    imu_device.set_y_acceleration_offset(y_acceleration_offset)
    imu_device.set_z_acceleration_offset(z_acceleration_offset)

    imu_device.set_x_gyroscope_offset(x_gyroscope_offset)
    imu_device.set_y_gyroscope_offset(y_gyroscope_offset)
    imu_device.set_z_gyroscope_offset(z_gyroscope_offset)

    print('')
    imu_device.print_info()

    print('')
    print('Computing gyroscopes drift corrections ', end = '', flush = True)

    i             = 0
    start_time    = time.time()
    progress_time = time.time()

    x_gyroscope_measure = 0
    y_gyroscope_measure = 0
    z_gyroscope_measure = 0

    while time.time() - start_time < DRIFT_CALIBRATION_DURATION:

        if time.time() - progress_time > PROGRESS_INDICATION_STEP_IN_S:
            print('.', end = '', flush = True)
            progress_time = time.time()

        imu_device.read_gyroscope_data()

        x_gyroscope_measure += imu_device.get_x_gyroscope()
        y_gyroscope_measure += imu_device.get_y_gyroscope()
        z_gyroscope_measure += imu_device.get_z_gyroscope()

        i += 1

    print('')

    x_gyroscope_measure /= DRIFT_CALIBRATION_DURATION
    y_gyroscope_measure /= DRIFT_CALIBRATION_DURATION
    z_gyroscope_measure /= DRIFT_CALIBRATION_DURATION

    x_gyroscope_drift_correction = int(x_gyroscope_measure)
    y_gyroscope_drift_correction = int(y_gyroscope_measure)
    z_gyroscope_drift_correction = int(z_gyroscope_measure)

    imu_device.set_x_gyroscope_drift_correction(x_gyroscope_drift_correction)
    imu_device.set_y_gyroscope_drift_correction(y_gyroscope_drift_correction)
    imu_device.set_z_gyroscope_drift_correction(z_gyroscope_drift_correction)

    print('')
    imu_device.print_info()

    setup_data['mpu9250driver']['ros__parameters']['acceleration_x_offset'] = x_acceleration_offset
    setup_data['mpu9250driver']['ros__parameters']['acceleration_y_offset'] = y_acceleration_offset
    setup_data['mpu9250driver']['ros__parameters']['acceleration_z_offset'] = z_acceleration_offset

    setup_data['mpu9250driver']['ros__parameters']['gyroscope_x_offset'] = x_gyroscope_offset
    setup_data['mpu9250driver']['ros__parameters']['gyroscope_y_offset'] = y_gyroscope_offset
    setup_data['mpu9250driver']['ros__parameters']['gyroscope_z_offset'] = z_gyroscope_offset

    setup_data['mpu9250driver']['ros__parameters']['gyroscope_x_drift_correction'] = x_gyroscope_drift_correction
    setup_data['mpu9250driver']['ros__parameters']['gyroscope_y_drift_correction'] = y_gyroscope_drift_correction
    setup_data['mpu9250driver']['ros__parameters']['gyroscope_z_drift_correction'] = z_gyroscope_drift_correction

    with open(SETUP_FILE, 'w') as yaml_file:
        yaml.dump(setup_data, yaml_file)

    print('')
    print('/\/\/\/\/\/\/\/\/\/\/\/\/\ ')
    print('| IMU calibration done!  | ')
    print('\/\/\/\/\/\/\/\/\/\/\/\/\/ ')
    print('')


if __name__ == '__main__':

    try:

        main()

    except KeyboardInterrupt:
        print('Keyboard interrupt...')

    except Exception as e:
        print('Error: ' + str(e))
