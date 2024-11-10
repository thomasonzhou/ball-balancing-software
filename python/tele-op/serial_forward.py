import serial
import time

from parse_process import parse_coord

# Configuration for COM ports
input_com_port = 'COM8'
output_com_port = 'COM5'
baudrate = 115200

# Open the input and output serial ports
try:
    ser_in = serial.Serial(input_com_port, baudrate, timeout=1)
    ser_out = serial.Serial(output_com_port, baudrate, timeout=1)
except serial.SerialException as e:
    print(f"Error: {e}")
    exit(1)

try:
    while True:
        # Read from the input port
        if ser_in.in_waiting > 0:
            data = ser_in.read(ser_in.in_waiting)  # Read incoming bytes
            try:
                # Decode the data as ASCII and strip any extraneous line endings
                decoded_data = data.decode('ascii').strip()
                # Parse the input x and y coordinates
                input_coord = parse_coord(decoded_data)
                if input_coord:
                    input_x, input_y = input_coord
                    print(f"RX: X={input_x}, Y={input_y}")

                
                # Re-encode and send the ASCII-decoded data over the output port
                # ser_out.write(decoded_data.encode('ascii'))  # Add line endings if needed

                # Debug: Confirm data was sent
                # print(f"TX ASCII Encoded: {decoded_data}")
            except UnicodeDecodeError:
                print("non-ASCII data")

        time.sleep(0.01)

except KeyboardInterrupt:
    print("Keyboard Interrupt")

finally:
    # Close the ports on exit
    ser_in.close()
    ser_out.close()
    print("Ports Closed")
