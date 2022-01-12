import serial
import serial.tools.list_ports as lp
import time

if __name__ == "__main__":

    with open("FIRMWARE3.bin", "wb") as ser:
        ser.write(bytes([48]))
        with open("FIRMWARE.bin", "rb") as f:
            while (chunk := f.read(4)):
                ser.write(chunk)
                # time.sleep(0.05)
        b = bytearray()
        b.extend(map(ord, "$END"))
        ser.write(b)
