import serial
import serial.tools.list_ports as lp
import time
import os

if __name__ == "__main__":

    port = [port.name for port in lp.comports() if "usbserial" in port.name][0]
    with serial.Serial() as ser:
        ser.baudrate = 9600
        ser.port = "/dev/" + port
        ser.open()

        # ser.write(bytes([48]))

        bw = 0
        with open("FIRMWARE.bin", "rb") as f:
            while (chunk := f.read(4)):
                ser.write(chunk)
                bw += 4
                time.sleep(0.05)
                print(f'{bw} / {os.path.getsize("FIRMWARE.bin")}')

        b = bytearray()
        b.extend(map(ord, "$END"))
        ser.write(b)
