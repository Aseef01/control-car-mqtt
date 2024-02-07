import machine, time
import network
from machine import Pin, PWM, Timer
from utime import sleep
from umqtt.simple import MQTTClient

class HCSR04:
    """
    Driver to use the untrasonic sensor HC-SR04.
    The sensor range is between 2cm and 4m.
    The timeouts received listening to echo pin are converted to OSError('Out of range')
    """
    # echo_timeout_us is based in chip range limit (400cm)
    def __init__(self, trigger_pin, echo_pin, echo_timeout_us=500*2*30):
        """
        trigger_pin: Output pin to send pulses
        echo_pin: Readonly pin to measure the distance. The pin should be protected with 1k resistor
        echo_timeout_us: Timeout in microseconds to listen to echo pin.
        By default is based in sensor limit range (4m)
        """
        self.echo_timeout_us = echo_timeout_us
        # Init trigger pin (out)
        self.trigger = Pin(trigger_pin, mode=Pin.OUT, pull=None)
        self.trigger.value(0)

        # Init echo pin (in)
        self.echo = Pin(echo_pin, mode=Pin.IN, pull=None)

    def _send_pulse_and_wait(self):
        """
        Send the pulse to trigger and listen on echo pin.
        We use the method `machine.time_pulse_us()` to get the microseconds until the echo is received.
        """
        self.trigger.value(0) # Stabilize the sensor
        time.sleep_us(5)
        self.trigger.value(1)
        # Send a 10us pulse.
        time.sleep_us(10)
        self.trigger.value(0)
        try:
            pulse_time = machine.time_pulse_us(self.echo, 1, self.echo_timeout_us)
            return pulse_time
        except OSError as ex:
            if ex.args[0] == 110: # 110 = ETIMEDOUT
                raise OSError('Out of range')
            raise ex

    def distance_mm(self):
        """
        Get the distance in milimeters without floating point operations.
        """
        pulse_time = self._send_pulse_and_wait()

        # To calculate the distance we get the pulse_time and divide it by 2
        # (the pulse walk the distance twice) and by 29.1 becasue
        # the sound speed on air (343.2 m/s), that It's equivalent to
        # 0.34320 mm/us that is 1mm each 2.91us
        # pulse_time // 2 // 2.91 -> pulse_time // 5.82 -> pulse_time * 100 // 582
        mm = pulse_time * 100 // 582
        return mm

    def distance_cm(self):
        """
        Get the distance in centimeters with floating point operations.
        It returns a float
        """
        pulse_time = self._send_pulse_and_wait()

        # To calculate the distance we get the pulse_time and divide it by 2
        # (the pulse walk the distance twice) and by 29.1 becasue
        # the sound speed on air (343.2 m/s), that It's equivalent to
        # 0.034320 cm/us that is 1cm each 29.1us
        cms = (pulse_time / 2) / 29.1
        return cms
# ------------------------------------------------------------------------------------------------------------

# WiFi credentials
# WIFI_SSID = "Redmi Note 10S"
# WIFI_PASSWORD = "1234567890"

# WIFI_SSID = "13-6_2.4G"
# WIFI_PASSWORD = "01110292882ABC"

WIFI_SSID = "owoo"
WIFI_PASSWORD = "easypassword"

# WIFI_SSID = "UniKL MIIT 5GHz"
# WIFI_PASSWORD = ""

# Replace these with your MQTT broker details
MQTT_BROKER = "178.128.119.114"
MQTT_PORT = 1883
MQTT_USER = ""
MQTT_PASSWORD = ""
CLIENT_ID = ""

ena = PWM(Pin(25), freq=500)  # Direction
in1 = Pin(26, Pin.OUT)  # Direction
in2 = Pin(27, Pin.OUT)    # Speed

enb = PWM(Pin(13), freq=500)
in3 = Pin(14, Pin.OUT)
in4 = Pin(12, Pin.OUT)

speed = 1023

# sensor = HCSR04(trigger_pin=32, echo_pin=35,echo_timeout_us=1000000)
sensor = HCSR04(trigger_pin=32, echo_pin=35,echo_timeout_us=1000000)

def move_forward():
    print("Move Forward")
    ena.duty(speed)
    enb.duty(speed)
    in1.value(1)
    in2.value(0)
    in3.value(1)
    in4.value(0)
    
def reverse():
    print("Move Reverse")
    ena.duty(speed)
    enb.duty(speed)
    in1.value(0)
    in2.value(1)
    in3.value(0)
    in4.value(1)
    
def turn_right():
    print("Turn Right")
    ena.duty(speed)
    enb.duty(speed)
    in1.value(0)
    in2.value(1)
    in3.value(0)
    in4.value(0)
    
def turn_left():
    print("Turn Left")
    ena.duty(speed)
    enb.duty(speed)
    in1.value(0)
    in2.value(0)
    in3.value(0)
    in4.value(1)
    
def stop():
    print("Stop")
    in1.value(0)
    in2.value(0)
    in3.value(0)
    in4.value(0)
    ena.duty(0)
    enb.duty(0)

# Connect to WiFi
wifi = network.WLAN(network.STA_IF)
wifi.active(True)
wifi.connect(WIFI_SSID, WIFI_PASSWORD)

while not wifi.isconnected():
    print(".", end="")
    time.sleep(1)

print("Connected to WiFi")

# Callback function for incoming messages
def callback(topic, msg):
    print("Received message on topic {}: {}".format(topic, msg))
    if msg == b"1":
        print("Moving forward")
        move_forward()
    elif msg == b"2":
        print("Moving in left")
        turn_left()
    elif msg == b"3":
        print("Moving in right")
        turn_right()
    elif msg == b"4":
        print("Moving in reverse")
        reverse()
    elif msg == b"5":
        print("Moving in stop")
        stop()
    else:
        print("Unknown command")

# Connect to the MQTT broker
client = MQTTClient(CLIENT_ID, MQTT_BROKER, user=MQTT_USER, password=MQTT_PASSWORD, port=MQTT_PORT)
client.set_callback(callback)
client.connect()

# Subscribe to a topic
subscribe_topic = b"control_car"
client.subscribe(subscribe_topic)

# Main loop
try:
    while True:
        # Measure distance with the ultrasonic sensor
        distance = sensor.distance_cm()
        print(distance)
        
        # Publish ultrasonic sensor data
        publish_topic = b"control_car_ultrasonic_sensor"
#         publish_message = "Distance: {:.2f} cm".format(distance)
        publish_message = "{:.2f}".format(distance)
        client.publish(publish_topic, publish_message)

        # Wait for incoming messages for a short duration
        for i in range(10):
            client.check_msg()
            time.sleep(0.1)

        # Wait before the next iteration
        time.sleep(1)

except KeyboardInterrupt:
    print("Interrupted by user")
finally:
    # Clean up and disconnect
    client.disconnect()
    
    
    
    
    
# while True:
#     distance = sensor.distance_cm()
#     print(distance)
#     
#     if distance < 50:
#         stop()
#         reverse()
#         sleep(1)
#         turn_right()
#         sleep(1)
#     else:
#         move_forward()