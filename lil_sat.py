import Adafruit_BMP.BMP085 as BMP085
import time
from mpu6050 import mpu6050
from bluetooth import *
import datetime
import enum
import threading
import RPi.GPIO

# Global bluetooh socket
sock = None

# GPIO pin to read the payload input from
payload_in_pin = 4


class LilState(enum.IntEnum):
    """
    Satellite states
    """

    #Startup
    STARTUP = 0
    # First comms, before LOS
    FIRST_PASS = 1
    # LOS period
    COMM_LOSS = 2
    # Second comms pass, transmit telemetry out as quickly as possible
    SECOND_PASS = 3
    # End of life
    EOL = 4


def gpio_setup():
    """
    Setup GPIO pins for reading the payload in pin
    """
    GPIO.setmode(io.BCM)
    GPIO.setup(payload_in_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)


def set_pwm_sysfs(entry, value):
    """
    Set a pwm sysfs entry to a value
    :param entry: path to the sysfs entry
    :param value: 
    :return: 
    """
    try:
        f = open("/sys/class/rpi-pwm/pwm0/" + entry, 'w')
        f.write(value)
        f.close()
    except:
        print("Error writing to: " + entry + " value: " + value)


def pwm_setup():
    """
    Setup PWM to generated a 50% duty cycle square wave for the payload
    :return:
    """
    set_pwm_sysfs("delayed", "0")
    set_pwm_sysfs("mode", "pwm")
    set_pwm_sysfs("frequency", "100")
    set_pwm_sysfs("active", "1")
    set_pwm_sysfs("duty", "50")


def try_connect():
    """
    Try and connect to the ground station over blue tooth
    :return: bluetooth socket
    """
    service_matches = find_service(name="COM8", address="a8:6d:aa:41:3c:95")

    if len(service_matches):
        first_match = service_matches[0]
        port = first_match["port"]
        name = first_match["name"]
        host = first_match["host"]
    else:
        print("Not found")
        return None

    print("connecting to \"%s\" on %s" % (name, host))

    # Create the client socket
    sock = BluetoothSocket(RFCOMM)
    sock.connect((host, port))

    return sock


def connect_thread():
    """
    Thread to attempt to connect to over bluetooth without blocking the main thread
    """
    global sock
    while sock is None:
        sock = try_connect()


def gather_telemetry(bar: BMP085, accel: mpu6050):
    """
    Gather data from the satellite's systems
    :return: string of telemetry data
    """
    accel_data = accel.get_accel_data()
    gyro_data = accel.get_gyro_data()
    temp = bar.read_temperature()
    pressure = bar.read_pressure()
    alt = bar.read_altitude()
    accel_x = accel_data["x"]
    accel_y = accel_data["y"]
    accel_z = accel_data["z"]
    gyro_x = gyro_data["x"]
    gyro_y = gyro_data["y"]
    gyro_z = gyro_data["z"]
    payload_signal = GPIO.input(payload_in_pin)
    t = datetime.datetime.now().timestamp()
    data = "{},{},{},{},{},{},{},{},{},{},{}".format(t, temp, pressure, alt, accel_x, accel_y, accel_z, gyro_x, gyro_y,
                                                     gyro_z, payload_signal)
    data += "\n"

    return data


def main():
    """
    Main satellite logic
    """

    # Global bluetooth socket
    global sock

    # Set state to first pass
    state = LilState.STARTUP

    # Open output file
    f = open("output.csv", "w")

    # setup sensors
    bar = BMP085.BMP085()
    accel = mpu6050(0x68)

    # Setup payload
    pwm_setup()
    gpio_setup()

    # While the satellite is running
    data_queue = []
    while state != LilState.EOL:
        # Gather telemetry data
        data = gather_telemetry(bar, accel)

        # Write telemetry to file
        f.write(data)
        f.flush()

        # Connect to ground station over bluetooth
        if state == LilState.STARTUP:
            sock = try_connect()
            if sock is not None:
                start_msg = "Ready for Launch"
                print(start_msg)
                sock.send(start_msg.encode())
        # If we are in the first pass before LOS
        if state == LilState.FIRST_PASS:
            try:
                sock.send((data.encode()))
            except:
                # Handle comms loss
                print("Comms Loss")
                sock.close()
                sock = None

                # Start thread for trying to reacquire the signal
                thread = threading.Thread(target=connect_thread)
                thread.start()
                state = LilState.COMM_LOSS
        # While waiting for reacquire
        elif state == LilState.COMM_LOSS:
            # add data to the queue
            data_queue += data

            # check if we have gotten a connection
            if sock is not None:
                # transition to second pass state
                print("Comms found")
                state = LilState.SECOND_PASS
        # In the second comm pass
        elif state == LilState.SECOND_PASS:
            # send all data in the queue
            for data in data_queue:
                sock.send(data.encode())

            # Enter EOL
            state = LilState.EOL
            # Close file
            f.close()

        time.sleep(1)


if __name__ == '__main__':
    main()