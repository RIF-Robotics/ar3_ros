import serial
import argparse

EOL = b'\r\n'

joint_info = {
    1: {'letter': b'A', 'cal_dir': 0, 'neg_ang_lim': -170.0, 'pos_ang_lim': 170.0,
        'step_lim': 15110, 'step_curr': 0, 'angle_curr': 0.0, 'rest_count': 7600},
    2: {'letter': b'B', 'cal_dir': 0, 'neg_ang_lim': -129.6, 'pos_ang_lim': 0.0,
        'step_lim': 7198, 'step_curr': 0, 'angle_curr': 0.0, 'rest_count': 2139},
    3: {'letter': b'C', 'cal_dir': 1, 'neg_ang_lim': +1.0, 'pos_ang_lim': 143.7,
        'step_lim': 7984, 'step_curr': 0, 'angle_curr': 0.0, 'rest_count': 7895},
    4: {'letter': b'D', 'cal_dir': 0, 'neg_ang_lim': -164.5, 'pos_ang_lim': 164.5,
        'step_lim': 14056, 'step_curr': 0, 'angle_curr': 0.0, 'rest_count': 7049},
    5: {'letter': b'E', 'cal_dir': 0, 'neg_ang_lim': -104.15, 'pos_ang_lim': 104.15,
        'step_lim': 4560, 'step_curr': 0, 'angle_curr': 0.0, 'rest_count': 887},
    6: {'letter': b'F', 'cal_dir': 1, 'neg_ang_lim': -148.1, 'pos_ang_lim': 148.1,
        'step_lim': 6320, 'step_curr': 0, 'angle_curr': 0.0, 'rest_count': 3062}
}


def parse_response(response):
    return response.strip().decode("utf-8")


def get_drive_to_limit_cmd(joints, speed):
    cmd = b'LL'

    for key, joint in joint_info.items():
        cmd += joint['letter'] + str(joint['cal_dir']).encode('utf-8')
        if key in joints:
            cmd += b'10000'
        else:
            cmd += b'0'

    # Append the speed
    cmd += b'S' + str(speed).encode('utf-8')

    cmd += EOL
    return cmd


def get_zero_calibrate_encoders_cmd():
    cmd = b'LM'

    for key, joint in joint_info.items():
        joint['step_curr'] = 0
        joint['angle_curr'] = joint['neg_ang_lim']
        cmd += joint['letter'] + str(joint['step_curr']).encode('utf-8')

    cmd += EOL
    return cmd


def get_move_away_from_limits_cmd(joints):
    cmd = b'MJ'

    for key, joint in joint_info.items():
        cmd += joint['letter'] + str(int(not joint['cal_dir'])).encode('utf-8')
        if key in joints:
            cmd += b'500'
        else:
            cmd += b'0'

    cmd += b'S15G15H10I20K5'

    cmd += EOL
    return cmd


def get_rest_position_cmd(joints):
    cmd = b'MJ'

    for key, joint in joint_info.items():
        cmd += joint['letter'] + str(int(not joint['cal_dir'])).encode('utf-8')
        if key in joints:
            cmd += str(joint['rest_count']).encode('utf-8')
        else:
            cmd += b'0'

    cmd += b'S50G10H10I10K10'

    cmd += EOL
    return cmd


def calibrate():
    parser = argparse.ArgumentParser(description='Calibrate the AR3 robot arm.')
    parser.add_argument('active_joints', metavar='N', type=int, nargs='+',
                        help='Joints to calibrate.')
    args = parser.parse_args()

    firmware_version = '0.0.1'

    ser = serial.Serial('/dev/ttyACM0', baudrate=115200,
                        bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                        stopbits=serial.STOPBITS_ONE, timeout=1)

    # Check the firmware version
    ser.write(b'FV'+EOL)
    if parse_response(ser.readline()) != firmware_version:
        print('Serial comms not working.')
        return

    print('Connected to AR3.')

    # Set a long timeout during the calibration movement
    ser.timeout = 60

    # 1. Command the motors to hit the limit switches
    cmd = get_drive_to_limit_cmd(args.active_joints, 20)
    print('1. Limit command: %s' % cmd)
    ser.write(cmd)

    # The LL command only uses \r for EOL
    if 'P' == parse_response(ser.read_until(b'\r')):
        print('Joints reached limit switches')
    else:
        print("Calibration fail")
        return

    # 2. Write calibration positions to encoders
    ser.timeout = 1
    cmd = get_zero_calibrate_encoders_cmd()
    print('2. Write encoder command: %s' % cmd)
    ser.write(cmd)
    if 'Done' == parse_response(ser.readline()):
        print('Successfully set encoder counts.')
    else:
        print('Failed to set encoder counts.')
        return

    # 3. Move away from limit switches
    cmd = get_move_away_from_limits_cmd(args.active_joints)
    print('3. Move away command: %s' % cmd)
    ser.timeout = 10
    ser.write(cmd)
    result = parse_response(ser.readline())
    print('MJ response: ', result)

    # 4. Command the motors to hit the limit switches at slower speed
    cmd = get_drive_to_limit_cmd(args.active_joints, 8)
    print('4. Drive to limit command: %s' % cmd)
    ser.write(cmd)

    # The LL command only uses \r for EOL
    if 'P' == parse_response(ser.read_until(b'\r')):
        print('Joints reached limit switches')
    else:
        print("Calibration fail")
        return

    # 5. Write calibration positions to encoders
    ser.timeout = 1
    cmd = get_zero_calibrate_encoders_cmd()
    print('5. Write encoder command: %s' % cmd)
    ser.write(cmd)
    if 'Done' == parse_response(ser.readline()):
        print('Successfully set encoder counts.')
    else:
        print('Failed to set encoder counts.')
        return

    # 6. Go to rest position
    print('Moving to rest position.')
    ser.timeout = 20
    cmd = get_rest_position_cmd(args.active_joints)
    print('6. Rest position command: %s' % cmd)
    ser.write(cmd)
    result = parse_response(ser.readline())
    print('MJ response: ', result)


if __name__ == '__main__':
    calibrate()
