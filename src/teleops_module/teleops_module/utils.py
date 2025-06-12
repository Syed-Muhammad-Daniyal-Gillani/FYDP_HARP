import serial
import serial.tools.list_ports

def find_arduino_port():
    """Return the serial port of an Arduino using description/manufacturer matching."""
    ports = serial.tools.list_ports.comports()
    for port in ports:
        desc = (port.description or "").lower()
        manu = (port.manufacturer or "").lower()
        if "arduino" in desc or "arduino" in manu:
            return port.device
    raise Exception("Arduino not found!")
