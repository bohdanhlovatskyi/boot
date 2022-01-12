import serial
import serial.tools.list_ports as lp
import time

if __name__ == "__main__":

    port = [port.name for port in lp.comports() if "usbserial" in port.name][0]
    with serial.Serial() as ser:
        ser.baudrate = 9600
        ser.port = port
        ser.open()

        ser.write(b"0")

        with open("FIRMWARE.bin", "rb") as f:
            while (chunk := f.read(4)):
                ser.write(chunk)
                time.sleep(0.1)

        b = bytearray()
        b.extend(map(ord, "$END"))
        ser.write(b)
