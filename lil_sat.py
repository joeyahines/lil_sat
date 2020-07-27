import Adafruit_BMP.BMP085 as BMP085
import time
from mpu6050 import mpu6050
from bluetooth import *
import datetime
import enum
import threading
import RPi.GPIO

GPIO.setmode(io.BCM)

in_pin = 4
GPIO.setup(in_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)



bar = BMP085.BMP085()
accel = mpu6050(0x68)


class LilState(enum.IntEnum):
    FIRST_PASS = 1
    COMM_LOSS = 2
    SECOND_PASS = 3


def set_pwm(property, value):
    try:
        f = open("/sys/class/rpi-pwm/pwm0/" + property, 'w')
        f.write(value)
        f.close()
    except:
        print("Error writing to: " + property + " value: " + value)


def pwm_setup():
    set_pwm("delayed", "0")
    set_pwm("mode", "pwm")
    set_pwm("frequency", "100")
    set_pwm("active", "1")
    set_pwm("duty", "50")


def try_connect():
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


sock = None


def connect_thread():
    global sock
    while sock is None:
        sock = try_connect()


if __name__ == '__main__':
    state = LilState.FIRST_PASS
    data_queue = []
    pwm_setup()
    f = open("output.csv", "w")
    running = True

    while running:
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
        payload_signal = GPIO.input(in_pin)
        t = datetime.datetime.now().timestamp()
        data = "{},{},{},{},{},{},{},{},{},{},{}".format(t, temp, pressure, alt, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, payload_signal)
        data += "\n"
        f.write(data)
        f.flush()

        if state == LilState.FIRST_PASS:
            if sock is None:
                sock = try_connect()
            else:
                try:
                    sock.send((data.encode()))
                except:
                    print("Comms Loss")
                    sock.close()
                    sock = None
                    thread = threading.Thread(target=connect_thread)
                    thread.start()
                    state = LilState.COMM_LOSS
        elif state == LilState.COMM_LOSS:
            data_queue += data
            if sock is not None:
                print("Comms found")
                state = LilState.SECOND_PASS
        elif state == LilState.SECOND_PASS:
            for data in data_queue:
                sock.send(data.encode())

            running = False
            f.close()

        time.sleep(1)

