import serial
import time

#create serial port
ser = serial.Serial(
    port='/dev/tty.usbserial-A50285BI',  #use correct port ( '/dev/ttyUSB0' for Linux, macOS)
    baudrate=115200,  #baud rate set used for STM32
    timeout=1  #(seconds units)
)

#test ultrasonic format
# #see if the serial port is open
# if ser.is_open:
#     print("Serial port is open!")

# #get data from the serial port
# try:
#     while True:
#         #read line
#         data = ser.readline().decode('utf-8').strip()  
#         if data:
#             print(f"Received: {data}")
# except KeyboardInterrupt:
#     print("Program interrupted.")

# # Close the serial port when done
# ser.close()

#test IMU format
def read_accel_data():
    while True:
        try:
            time.sleep(0.1)
            if ser.in_waiting > 0:  #when we have data to read

                data = "X: 1, Y: 2, Z: 3"

                real_data = [ser.readline().decode('utf-8').strip()]

                print(f"Received data: {real_data}")

                #format: "X: value, Y: value, Z: value" (from stm32 cube ide packing)
                values = data.split(', ')

                real_values = real_data

                accel_data = {}

                print("VALUES:")
                print(values)
                print("REAL VALUES:")
                print(real_values)

                for value in values:
                    axis, val = value.split(': ')
                    accel_data[axis.strip()] = int(val.strip())

                print(f"Accel X: {accel_data.get('X')}, Y: {accel_data.get('Y')}, Z: {accel_data.get('Z')}")
            else:
                pass

        except KeyboardInterrupt:
            print("Exiting...")
            ser.close()
            break
        except Exception as e:
            print(f"Error: {e}")
            ser.close()
            break

if __name__ == "__main__":
    read_accel_data()
