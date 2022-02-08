import serial
import io
import argparse

EOL = b'\n'

joint_info = {
    1: {'letter': b'A', 'cal_dir': 0, 'neg_ang_lim': -170.0 , 'pos_ang_lim': 170.0 , 'step_lim': 15110, 'step_curr': 0, 'angle_curr': 0.0, 'rest_count': 7600},
    2: {'letter': b'B', 'cal_dir': 0, 'neg_ang_lim': -129.6 , 'pos_ang_lim': 0.0   , 'step_lim': 7198 , 'step_curr': 0, 'angle_curr': 0.0, 'rest_count': 2139},
    3: {'letter': b'C', 'cal_dir': 1, 'neg_ang_lim': +1.0   , 'pos_ang_lim': 143.7 , 'step_lim': 7984 , 'step_curr': 0, 'angle_curr': 0.0, 'rest_count': 7895},
    4: {'letter': b'D', 'cal_dir': 0, 'neg_ang_lim': -164.5 , 'pos_ang_lim': 164.5 , 'step_lim': 14056, 'step_curr': 0, 'angle_curr': 0.0, 'rest_count': 7049},
    5: {'letter': b'E', 'cal_dir': 0, 'neg_ang_lim': -104.15, 'pos_ang_lim': 104.15, 'step_lim': 4560 , 'step_curr': 0, 'angle_curr': 0.0, 'rest_count': 1500},
    6: {'letter': b'F', 'cal_dir': 1, 'neg_ang_lim': -148.1 , 'pos_ang_lim': 148.1 , 'step_lim': 6320 , 'step_curr': 0, 'angle_curr': 0.0, 'rest_count': 3062}
}

def parse_response(response):
    return response.strip().decode("utf-8")


def get_drive_to_limit_cmd(joints, speed):
    cmd = b'L,'

    for key, joint in joint_info.items():
        if key in joints:
            cmd += b'10000'
        else:
            cmd += b'0'
        cmd += b','

    # Append the speed
    cmd += str(speed).encode('utf-8')

    cmd += EOL
    return cmd

def get_zero_calibrate_encoders_cmd():
    cmd = b'C,'

    for key, joint in joint_info.items():
        if joint['cal_dir'] == 0:
            joint['step_curr'] = 0
        else:
            joint['step_curr'] = joint['step_lim']
        cmd += str(joint['step_curr']).encode('utf-8') + b','

    cmd += EOL
    return cmd

def get_move_away_from_limits_cmd(joints):
    cmd = b'M,'

    for key, joint in joint_info.items():
        if key in joints:
            cmd += b'500'
        else:
            cmd += b'0'
        cmd += b','

    cmd += b'15'
    cmd += EOL
    return cmd

def get_rest_position_cmd(joints):
    cmd = b'M,'

    for key, joint in joint_info.items():
        if key in joints:
            cmd += str(joint['rest_count']).encode('utf-8')
        else:
            cmd += b'0'
        cmd += b','

    cmd += b'50'
    cmd += EOL
    return cmd

def enable_control_loops(joints):
    cmd = b'X,'

    for key, joint in joint_info.items():
        if key in joints:
            cmd += b'1,'
        else:
            cmd += b'0,'

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
    ser.write(b'V'+EOL)
    if parse_response(ser.readline()) != firmware_version:
        print('Serial comms not working.')
        return

    print('Connected to AR3.')

    # Set a long timeout during the calibration movement
    ser.timeout = 60

    # 1. Command the motors to hit the limit switches
    cmd = get_drive_to_limit_cmd(args.active_joints, 30)
    print('1. Limit command: %s' % cmd)
    ser.write(cmd)
    response = parse_response(ser.readline())

    if 'P' == response:
        print('Joints reached limit switches')
    else:
        print("Calibration fail: %s", response)
        return

    # 2. Write calibration positions to encoders
    ser.timeout = 1
    cmd = get_zero_calibrate_encoders_cmd()
    print('2. Write encoder command: %s' % cmd)
    ser.write(cmd)
    response = parse_response(ser.readline())
    if 'c,OK' == response:
        print('Successfully set encoder counts.')
    else:
        print('Failed to set encoder counts: %s', response)
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
    response = parse_response(ser.readline())

    if 'P' == response:
        print('Joints reached limit switches')
    else:
        print("Calibration fail: %s", response)
        return

    # 5. Write calibration positions to encoders
    ser.timeout = 1
    cmd = get_zero_calibrate_encoders_cmd()
    print('5. Write encoder command: %s' % cmd)
    ser.write(cmd)
    response = parse_response(ser.readline())
    if 'c,OK' == response:
        print('Successfully set encoder counts.')
    else:
        print('Failed to set encoder counts: %s', response)
        return

    # 6. Enable all control loops:
    cmd = enable_control_loops(args.active_joints)
    ser.write(cmd)
    response = parse_response(ser.readline())
    if 'x,OK' == response:
        print('Enabled control loops')
    else:
        print('Failed to enable control loops')

    # 6. Go to rest position
    print('Moving to rest position.')
    #ser.timeout = 20
    cmd = b'D,0,-1.6,0.05,0,-0.6,0' + EOL
    print('6. Rest position command: %s' % cmd)
    ser.write(cmd)
    #result = parse_response(ser.readline())

if __name__ == '__main__':
    calibrate()
