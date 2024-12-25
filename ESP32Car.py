# นำเข้าไลบรารี 
from machine import Pin, PWM
import network
import time
from umqtt.simple import MQTTClient
import json

# WiFi settings
ssid = "wifi name"
password = "password"

# MQTT settings
MQTT_BROKER = "broker.hivemq.com"
MQTT_TOPIC = b"esp32/car/control" 

# Servo pin settings
PAN_PIN = 12
TILT_PIN = 13
pan_servo = PWM(Pin(PAN_PIN), freq=50)
tilt_servo = PWM(Pin(TILT_PIN), freq=50)
# Motor pin settings (if required for other control)
motor_pins = [
    {"pinIN1": Pin(16, Pin.OUT), "pinIN2": Pin(17, Pin.OUT), "pwm": PWM(Pin(22), freq=1000)},
    {"pinIN1": Pin(18, Pin.OUT), "pinIN2": Pin(19, Pin.OUT), "pwm": PWM(Pin(23), freq=1000)},
    {"pinIN1": Pin(26, Pin.OUT), "pinIN2": Pin(25, Pin.OUT), "pwm": PWM(Pin(32), freq=1000)},
    {"pinIN1": Pin(14, Pin.OUT), "pinIN2": Pin(27, Pin.OUT), "pwm": PWM(Pin(33), freq=1000)}
]
# Direction control
UP = 1
DOWN = 2
LEFT = 3
RIGHT = 4
STOP = 0
FORWARDLEFT = 6
FORWARDRIGHT = 7
BACKWARDLEFT = 8
BACKWARDRIGHT = 9
TURNLEFT = 10 
TURNRIGHT = 11	
# Global variables
use_mqtt = True  # Start with MQTT mode

# -------------------- WiFi connection function --------------------
def connect_wifi(ssid, password):
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        print('Connecting to WiFi...')
        wlan.connect(ssid, password)
        while not wlan.isconnected():
            pass
    print('WiFi connected:', wlan.ifconfig())

# -------------------- WiFi and MQTT Functions --------------------

def connect_wifi(ssid, password):
    """Connect to WiFi"""
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        print('Connecting to WiFi...')
        wlan.connect(ssid, password)
        while not wlan.isconnected():
            time.sleep(1)
    print('Connected to WiFi:', wlan.ifconfig())
    return wlan

def setup_mqtt():
    """Set up the MQTT Client."""
    client = MQTTClient("ESP32_client", MQTT_BROKER)
    
    # Set the callback function for handling messages
    client.set_callback(mqtt_callback)
    
    client.connect()
    client.subscribe(MQTT_TOPIC)  # Subscribe to the main topic
    client.subscribe(b'esp32/servo/control')  # For receiving servo control commands
    return client
        
def mqtt_callback(topic, msg):
    """Callback function to handle messages received from MQTT."""
    print(f"Received message from {topic}: {msg}")
    
    # Add control based on the received values from the topic
    if topic == b'esp32/servo/control':
        # Assume msg contains pan and tilt values separated by ','
        try:
            pan_value, tilt_value = map(int, msg.decode().split(','))
            control_servo(pan_value, tilt_value)
        except ValueError:
            print("Invalid servo values received")
    elif topic == b'esp32/car/control':
        try:
            command = int(msg.decode())
            control_movement(command)  # Control the movement of the car
        except ValueError:
            print("Invalid car control command received")


# -------------------- Servo control functions --------------------
def control_servo_pan(pan_value):
    print("Control Pan:", pan_value)
    pan_servo.duty(pan_value)

def control_servo_tilt(tilt_value):
    print("Control Tilt:", tilt_value)
    tilt_servo.duty(tilt_value)

# -------------------- Motor control functions --------------------
def rotate_motor(motor_number, motor_direction):
    """Rotate motor in the specified direction"""
    motor = motor_pins[motor_number]
    if motor_direction == 1:  # Forward
        motor['pinIN1'].on()
        motor['pinIN2'].off()
    elif motor_direction == -1:  # Reverse
        motor['pinIN1'].off()
        motor['pinIN2'].on()
    else:  # Stop
        motor['pinIN1'].off()
        motor['pinIN2'].off()

# Robot movement control (for other commands)
def control_movement(input_value):
    """Control the movement of the robot based on input value"""
if input_value == UP:
    rotate_motor(0, 1)  # Left Back Forward
    rotate_motor(1, 1)  # Left Front Forward
    rotate_motor(2, 1)  # Right Back Forward
    rotate_motor(3, 1)  # Right Front Forward
elif input_value == DOWN:
    rotate_motor(0, -1)  # Left Back Backward
    rotate_motor(1, -1)  # Left Front Backward
    rotate_motor(2, -1)  # Right Back Backward
    rotate_motor(3, -1)  # Right Front Backward

elif input_value == LEFT:
    rotate_motor(0, 1)  # Left Back Forward
    rotate_motor(1, -1)   # Left Front Backward
    rotate_motor(2, -1)  # Right Back Backward
    rotate_motor(3, 1)   # Right Front Forward
elif input_value == RIGHT:
    rotate_motor(0, -1)   # Left Back Backward
    rotate_motor(1, 1)  # Left Front Forward
    rotate_motor(2, 1)   # Right Back Forward
    rotate_motor(3, -1)  # Right Front Backward
elif input_value == FORWARDLEFT:
    rotate_motor(0, 1)   # Left Back Forward
    rotate_motor(1, 0)   # Left Front Stop
    rotate_motor(2, 0)   # Right Back Stop
    rotate_motor(3, 1)   # Right Front Forward
elif input_value == FORWARDRIGHT:
    rotate_motor(0, 0)   # Left Back Stop
    rotate_motor(1, 1)   # Left Front Forward
    rotate_motor(2, 1)   # Right Back Forward
    rotate_motor(3, 0)   # Right Front Stop
elif input_value == BACKWARDLEFT:
    rotate_motor(0, 0)   # Left Back Stop
    rotate_motor(1, -1)  # Left Front Backward
    rotate_motor(2, -1)  # Right Back Backward
    rotate_motor(3, 0)   # Right Front Stop
elif input_value == BACKWARDRIGHT:
    rotate_motor(0, -1)  # Left Back Backward
    rotate_motor(1, 0)   # Left Front Stop
    rotate_motor(2, 0)   # Right Back Stop
    rotate_motor(3, -1)  # Right Front Backward
elif input_value == TURNLEFT:
    rotate_motor(0, 0)  # Left Back Stop
    rotate_motor(1, 0)   # Left Front Stop
    rotate_motor(2, 1)  # Right Back Backward
    rotate_motor(3, 1)   # Right Front Forward

elif input_value == TURNRIGHT:
    rotate_motor(0, 1)   # Left Back Forward
    rotate_motor(1, 1)  # Left Front Backward
    rotate_motor(2, 0)   # Right Back Stop
    rotate_motor(3, 0)  # Right Front Stop
else:
    rotate_motor(0, 0)   # Left Back Stop
    rotate_motor(1, 0)   # Left Front Stop
    rotate_motor(2, 0)   # Right Back Stop
    rotate_motor(3, 0)   # Right Front Stop
# -------------------- Main function --------------------
def main():
    wlan = connect_wifi(ssid, password)  # Connect to WiFi
    mqtt_client = setup_mqtt()  # Set up MQTT client

    if mqtt_client is not None:
        mqtt_client.set_callback(mqtt_callback)
        while True:
            mqtt_client.wait_msg()  # Wait for incoming messages
            time.sleep(0.1)  # Small delay

# -------------------- Run the main program --------------------
if __name__ == "__main__":
    main()
