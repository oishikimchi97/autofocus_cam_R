import serial
import time

def sendValue(ser, val):
    send_val = val.to_bytes(1, 'big')
    print(send_val)
    ser.write(send_val)
    # message = ser.readline()
    # print(message)


if __name__ == '__main__':
    ser = serial.Serial('/dev/tty.usbserial-1410') # adjust to your PC
    print(ser.name)

    lens_value = 100 # lens value shoud be int (1~254)
    sendValue(ser, lens_value)
    time.sleep(0.5) # shoud be more than 20ms

    ser.close()             # close port