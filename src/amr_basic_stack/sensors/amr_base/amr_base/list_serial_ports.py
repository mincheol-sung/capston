def main():
    try:
        from serial.tools import list_ports
    except ImportError:
        print("python3-serial is not installed. Run: sudo apt install python3-serial")
        return

    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports found.")
        return

    for port in ports:
        print(f"{port.device}\t{port.description}\tVID:PID={port.vid}:{port.pid}\tserial={port.serial_number}")


if __name__ == "__main__":
    main()
