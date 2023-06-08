import serial
import argparse
from math import isclose

EOL = b'\n'

joint_info = {
    1: {'cal_dir': 0, 'step_lim': 15110, 'rest': 0.0},
    2: {'cal_dir': 0, 'step_lim': 7198, 'rest': -1.6},
    3: {'cal_dir': 1, 'step_lim': 7984, 'rest': 0.05},
    4: {'cal_dir': 0, 'step_lim': 14056, 'rest': 0.0},
    5: {'cal_dir': 0, 'step_lim': 4685, 'rest': 0.0},
    6: {'cal_dir': 1, 'step_lim': 6320, 'rest': 0.0}
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


def get_zero_calibrate_encoders_cmd(joint_select):
    cmd = b'C,' + str(joint_select).encode('utf-8') + b','

    for key, joint in joint_info.items():
        if joint['cal_dir'] == 0:
            step_curr = 0
        else:
            step_curr = joint['step_lim']
        cmd += str(step_curr).encode('utf-8') + b','

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


def get_joint_positions(ser):
    cmd = b'P' + EOL
    ser.write(cmd)
    positions = parse_response(ser.readline()).split(',')
    if len(positions) != 8:
        print('Error: Invalid joint position size.')
    return list(map(float, positions[1:7]))


def wait_until_positions_reached(ser, active_joints, desired):
    active_select = [False if i + 1 in active_joints else True for i in range(len(desired))]

    not_reached = True
    while not_reached:
        current = get_joint_positions(ser)
        print('Desired: ', desired)
        print('Current: ', current)

        joints_reached = [isclose(n0, n1, abs_tol=0.01) for n0, n1 in zip(desired, current)]

        not_reached = not all(i or j for i, j in zip(active_select, joints_reached))


def enable_control_loops_cmd(joints):
    cmd = b'X,'

    for key, joint in joint_info.items():
        if key in joints:
            cmd += b'1,'
        else:
            cmd += b'0,'

    cmd += EOL
    return cmd


def enable_control_loops(ser, joints):
    cmd = enable_control_loops_cmd(joints)
    print('Enable control loops command: %s' % cmd)
    ser.write(cmd)
    response = parse_response(ser.readline())
    if 'x,OK' == response:
        print('Enabled control loops')
    else:
        print('Failed to enable control loops')


def partial_calibrate(ser, active_joints):
    # Create the bit select for the active joints
    joint_select = sum([1 << (x - 1) for x in active_joints])

    # Set a long timeout during the calibration movement
    ser.timeout = 60

    # 1. Command the motors to hit the limit switches
    cmd = get_drive_to_limit_cmd(active_joints, 30)
    print('1. Limit command: %s' % cmd)
    ser.write(cmd)
    response = parse_response(ser.readline())
    if 'P' == response:
        print('Joints reached limit switches')
    else:
        print("Move to limits failed: %s", response)
        return

    # 2. Write calibration positions to encoders
    ser.timeout = 1
    cmd = get_zero_calibrate_encoders_cmd(joint_select)
    print('2. Write encoder command: %s' % cmd)
    ser.write(cmd)
    response = parse_response(ser.readline())
    if 'c,OK' == response:
        print('Successfully set encoder counts.')
    else:
        print('Failed to set encoder counts: %s', response)
        return

    # 3. Move away from limit switches
    cmd = get_move_away_from_limits_cmd(active_joints)
    print('3. Move away command: %s' % cmd)
    ser.timeout = 10
    ser.write(cmd)
    response = parse_response(ser.readline())
    print('MJ response: ', response)

    # 4. Command the motors to hit the limit switches at slower speed
    cmd = get_drive_to_limit_cmd(active_joints, 8)
    print('4. Drive to limit command: %s' % cmd)
    ser.write(cmd)
    response = parse_response(ser.readline())
    if 'P' == response:
        print('Joints reached limit switches')
    else:
        print("Move to limits failed: %s", response)
        return

    # 5. Write calibration positions to encoders
    ser.timeout = 1
    cmd = get_zero_calibrate_encoders_cmd(joint_select)
    print('5. Write encoder command: %s' % cmd)
    ser.write(cmd)
    response = parse_response(ser.readline())
    if 'c,OK' == response:
        print('Successfully set encoder counts.')
    else:
        print('Failed to set encoder counts: %s', response)
        return

    # 6. Set the rest position as the desired position
    print('Moving to rest position.')

    rest_positions_str = ','.join([str(info['rest']) for _, info in joint_info.items()])
    # cmd = b'D,0,-1.6,0.05,0,0,0' + EOL
    cmd = b'D,' + rest_positions_str.encode('utf-8') + EOL
    print('6. Set desired position command: %s' % cmd)
    ser.write(cmd)
    response = parse_response(ser.readline())
    if 'd,OK' == response:
        print('Successfully set desired positions.')
    else:
        print('Failed to set desired positions: %s', response)
        return

    # 7. Enable all control loops:
    enable_control_loops(ser, active_joints)


def main():
    parser = argparse.ArgumentParser(description='Calibrate the AR3 robot arm.')
    parser.add_argument('--joints', metavar='N', type=int, nargs='+',
                        help='Joints to calibrate.')
    args = parser.parse_args()

    firmware_version = '0.0.1'

    ser = serial.Serial('/dev/ttyACM0', baudrate=115200,
                        bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                        stopbits=serial.STOPBITS_ONE, timeout=1)

    # Check the firmware version
    ser.write(b'V' + EOL)
    if parse_response(ser.readline()) != firmware_version:
        print('Serial comms not working.')
        return

    print('Connected to AR3.')

    if args.joints is not None:
        # If joints are specified, only calibrate those joints
        print('Performing partial calibration.')
        partial_calibrate(ser, args.joints)
    else:
        # If joints aren't specified, perform full calibration
        print('Performing full calibration.')
        active_joints = [1, 2, 3, 4, 6]
        partial_calibrate(ser, active_joints)

        # Wait for the desired positions to be achieved
        rest_positions = [info['rest'] for _, info in joint_info.items()]
        wait_until_positions_reached(ser, active_joints, rest_positions)

        active_joints = [5]
        partial_calibrate(ser, active_joints)
        wait_until_positions_reached(ser, active_joints, rest_positions)

        # Enable all control loops
        enable_control_loops(ser, [1, 2, 3, 4, 5, 6])


if __name__ == '__main__':
    main()
