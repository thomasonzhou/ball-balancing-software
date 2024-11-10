import serial
import time

from parse_process import parse_coord, ascii_encode
from pid.position_feedback import Controller
from kinematics.wrappers import translate_dir_to_motor_angles

pid = Controller()
desired_coord = (0,0)
dir_x, dir_y, theta_mag = 0,0,0

input_com_port = 'COM8'
output_com_port = 'COM5'
baudrate = 115200

try:
    ser_in = serial.Serial(input_com_port, baudrate, timeout=1)
    ser_out = serial.Serial(output_com_port, baudrate, timeout=1)
except serial.SerialException as e:
    print(f"Error: {e}")
    exit(1)

try:
    while True:
        if ser_in.in_waiting > 0:
            data = ser_in.read(ser_in.in_waiting)
            try:
                decoded_data = data.decode('ascii').strip()
                actual_coord = parse_coord(decoded_data)
                if actual_coord:
                    actual_x, actual_y = actual_coord
                    print(f"RX COORD: {decoded_data}")
                    
                    # Calulate control signal
                    dir_x, dir_y, theta_mag = pid.calculate(desired_coord, actual_coord)
                # else:
                #     dir_x, dir_y, theta_mag = 0,0,0

                    # Convert unit direction vector and control signal magnitude to motor angles
                    motor_angles = translate_dir_to_motor_angles(dir_x, dir_y, theta_mag)

                    # Re-encode and send the ASCII-decoded data over the output port
                    motor_output = ascii_encode(motor_angles)
                    ser_out.write(motor_output)

                    # Debug: Confirm data was sent
                    print(f"PID VECTOR: {(dir_x, dir_y, theta_mag)}")
                    print(f"TX ANGLE: {motor_output.decode('ascii')}")
                
            except UnicodeDecodeError:
                print("non-ASCII data")

        time.sleep(0.01)

except KeyboardInterrupt:
    print("Keyboard Interrupt")

finally:
    ser_in.close()
    ser_out.close()
    print("Ports Closed")
