import serial
import platform
import time

tare_timeout = 1

class MotorSerial:

    def __init__(self, baudrate: int):
        """Initialize the required serial ports
        
        Args:
            baudrate (int): Serial baudrate
        """
        self.baudrate = baurdate
        computeOS = platform.system()
        if computeOS == 'Linux':
            output_com_port = '/dev/ttyUSB0'
        elif computeOS == 'Windows':
            output_com_port = 'COM5'

        try:
            motor_serial = serial.Serial(output_com_port, baudrate, timeout=1)
        except serial.SerialException as e:
            print(f"Error: {e}")
            raise Exception("Failed to initalize serial!")
        else:
            print(f"Serial Initialized")
            self.motor_serial = motor_serial

    def send_encoded_motor_commands(motor_command: str, motor_angles: tuple[float, float, float]):
        """ASCII encodes a given motor command and set of motor angles in the expected format
        
        Args:
            motor_command (character): Defines the mode of operation
            motor_angles (tuple of 3 floats, (motorA, motorB, motorC)): Angles of the motors to be operated on (rad)
        """
        motorA, motorB, motorC = motor_angles
        motor_output = f"<{motor_command}, {motorA:.2f}, {motorB:.2f}, {motorC:.2f}>".encode('ascii')

        self.motor_serial.write(motor_output)

    def tare_motors(zero_angles: tuple[float, float, float]) -> bool:
        """Helper function to send the tare command and waits for confirmation
        
        Args: 
            zero_angles (tuple of 3 floats, (motorA, motorB, motorC)): Angles of the motors' current position (rad)
        Returns:
            State of motor tare operation: True if Tare successful and false if unsuccessful
        """
        # Tare motors given a tuple of angles
        self.send_encoded_motor_commands(self.motor_serial, 't', (zero_angles))
        print("Sent 'TARE' Command")

        # Wait for Tare confirmation to be received
        timeout_start = time.time()
        received_message = ""
        while time.time() < timeout_start + tare_timeout:
            if self.motor_serial.in_waiting > 0:
                serial_data = self.motor_serial.read(self.motor_serial.in_waiting)
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
    
    def close_serial(motor_serial):
        """Helper function to close the serial communications"""
        motor_serial.close()
        print("Ports Closed")
    