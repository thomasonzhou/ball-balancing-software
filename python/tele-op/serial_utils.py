import serial

def init_serial():
    """Helper function to initialize the required serial ports for tele-op
    
    Returns:
        motor_serial (Serial): Object representing the serial communication to the motor ATMEGA
        joystick_serial (Serial): Object representing the serial communication to the joystick arduino
    """
    motor_serial = serial.Serial()
    joystick_serial = serial.Serial()
    return motor_serial, joystick_serial

def tare_motors(motor_serial, angles: tuple[float, float, float]):
    """Helper function to send the tare command
    
    Args: 
        motor_serial (Serial): Object representing the serial communication to the motor
        angles (tuple of 3 floats, (motorA, motorB, motorC)): Angles of the motor's current position (rad)
    """
    # Tare motors given a tuple of angles
    # Send the tare command given angles and do whatever post processing
    pass

def wait_for_joystick_start(joystick_serial):
    """Blocking funcitno to wait for joystick start"""
    pass

def serial_forward(motor_serial, joystick_serial):
    """Main forwarding code"""
    # Everything in the while main loop (the try to unicode ascii error stuff), 
    # this function should go INSIDE OF the keyboard interrupt. the keyboard interrupt should be separate from this 
    # module
    pass

def teardown_serial(motor_serial, joystick_serial):
    """Helper function to teardown the serial communications"""
    # Teardown functions
    pass