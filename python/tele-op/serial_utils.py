import serial
import platform
import time
import re
from pid.position_feedback import Controller
from kinematics.wrappers import translate_dir_to_motor_angles
import serial.serialposix

tare_timeout = 1
user_timeout = 30

def init_serial(baudrate: int):
    """Helper function to initialize the required serial ports for tele-op
    
    Args:
        baudrate (int): Serial baudrate 
    Returns:
        motor_serial (Serial): Object representing the serial communication to the motor ATMEGA
        joystick_serial (Serial): Object representing the serial communication to the joystick arduino
    """
    computeOS = platform.system()
    if computeOS == 'Linux':
        input_com_port = '/dev/ttyACM0'
        output_com_port = '/dev/ttyUSB0'
    elif computeOS == 'Windows':
        input_com_port = 'COM8'
        output_com_port = 'COM5'

    try:
        motor_serial = serial.Serial(input_com_port, baudrate, timeout=1)
        joystick_serial = serial.Serial(output_com_port, baudrate, timeout=1)
    except serial.SerialException as e:
        print(f"Error: {e}")
        return None
    else:
        print(f"Serial Initialized")
        return motor_serial, joystick_serial

def send_encoded_motor_commands(motor_serial: serial.Serial, motor_command: str, motor_angles: tuple[float, float, float]):
    """Helper function that ASCII encodes a given motor command and set of motor angles in the expected format
    
    Args:
        motor_serial (Serial): Object representing the serial communication to the motor
        motor_command (character): Defines the mode of operation
        motor_angles (tuple of 3 floats, (motorA, motorB, motorC)): Angles of the motors to be operated on (rad)
    """
    motorA, motorB, motorC = motor_angles
    motor_output = f"<{motor_command}, {motorA:.2f}, {motorB:.2f}, {motorC:.2f}>".encode('ascii')

    motor_serial.write(motor_output)

def tare_motors(motor_serial: serial.Serial, zero_angles: tuple[float, float, float]):
    """Helper function to send the tare command and waits for confirmation
     
    Args: 
        motor_serial (Serial): Object representing the serial communication to the motor
        zero_angles (tuple of 3 floats, (motorA, motorB, motorC)): Angles of the motors' current position (rad)
    Returns:
        State of motor tare operation: True if Tare successful and false if unsuccessful
    """
    # Tare motors given a tuple of angles
    send_encoded_motor_commands(motor_serial, 't', (zero_angles))
    print("Sent 'TARE' Command")

    # Wait for Tare confirmation to be received
    timeout_start = time.time()
    received_message = ""
    while time.time() < timeout_start + tare_timeout:
        if motor_serial.in_waiting > 0:
            serial_data = motor_serial.read(motor_serial.in_waiting)
            try:
                # Decode the data as ASCII and strip extraneous characters
                received_message += serial_data.decode('ascii').strip()
                # Check if "TARE" is received
                if "TARE" in received_message:
                    print("'TARE' Confirmed")
                    return True
            except UnicodeDecodeError:
                print("Received Non-ASCII Data")
        time.sleep(0.01)
    
    print("Timeout Error: 'TARE' Not Confirmed")
    return False

def wait_for_user(joystick_serial: serial.Serial):
    """Blocking funciton to wait for user start"""
    timeout_start = time.time()
    while time.time() < timeout_start + user_timeout:
        if joystick_serial.in_waiting > 0:
            break
        time.sleep(0.01)
    print("Timeout Error: No User Input")

def parse_coord(input_data):
    """Helper function to extract coordinate values from Arduino serial"""
    # Expected ASCII data in the format <x,y>
    format = re.compile(r"<(-?\d+\.\d+),\s*(-?\d+\.\d+)>")
    match = format.match(input_data)
    if match:
        x = float(match.group(1))
        y = float(match.group(2))
        return x, y
    else:
        print("Data format error")
        return None
    
def joystick_decode(joystick_serial: serial.Serial):
    if joystick_serial.in_waiting > 0:
        data = joystick_serial.read(joystick_serial.in_waiting)
        try:
            decoded_data = data.decode('ascii').strip()
            actual_coord = parse_coord(decoded_data)
            if actual_coord:
                print(f"RX COORD: {decoded_data}")
                return actual_coord
        except UnicodeDecodeError:
            print("non-ASCII data")      

def close_serial(motor_serial, joystick_serial):
    """Helper function to close the serial communications"""
    motor_serial.close()
    joystick_serial.close()
    print("Ports Closed")
    