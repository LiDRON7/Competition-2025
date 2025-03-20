# Set up libraries and overall settings
import RPi.GPIO as GPIO  # Imports the standard Raspberry Pi GPIO library
from time import sleep   # Imports sleep (aka wait or pause) into the program
GPIO.setmode(GPIO.BOARD) # Sets the pin numbering system to use the physical layout

# Set up pin 11 for PWM
GPIO.setup(11,GPIO.OUT)  # Sets up pin 11 to an output (instead of an input)
p = GPIO.PWM(11, 50)     # Sets up pin 11 as a PWM pin
p.start(0)               # Starts running PWM on the pin and sets it to 0

# Move the servo back and forth
p.ChangeDutyCycle(3)     # Changes the pulse width to 3 (so moves the servo)
sleep(1)                 # Wait 1 second
p.ChangeDutyCycle(12)    # Changes the pulse width to 12 (so moves the servo)
sleep(1)

# Clean up everything
p.stop()                 # At the end of the program, stop the PWM
GPIO.cleanup()           # Resets the GPIO pins back to defaults


# import RPi.GPIO as GPIO
# import time

# # Set the GPIO mode
# GPIO.setmode(GPIO.BOARD)

# # Set the GPIO pin for the servo motor
# servo_pin = 11

# # Set the frequency of PWM
# frequency = 50

# # Set duty cycle for 0 degree position (2.5% of 20ms)
# duty_cycle_0_deg = 2.5

# # Set duty cycle for 180 degree position (12.5% of 20ms)
# duty_cycle_180_deg = 12.5

# # Set duty cycle for 90 degree position (7.5% of 20ms)
# duty_cycle_90_deg = 7.5

# # Set the duty cycle to the minimum to start at 0 degrees
# GPIO.setup(servo_pin, GPIO.OUT)
# servo_pwm = GPIO.PWM(servo_pin, frequency)
# servo_pwm.start(duty_cycle_0_deg)

# # Function to move the servo to a specific angle
# def set_angle(angle):
#     duty_cycle = (angle / 180) * (duty_cycle_180_deg - duty_cycle_0_deg) + duty_cycle_0_deg
#     servo_pwm.ChangeDutyCycle(duty_cycle)
#     time.sleep(0.5)  # Adjust as needed to allow servo to reach the position

# try:
#     while True:
#         # Move the servo to 0 degrees
#         set_angle(0)
#         time.sleep(1)
        
#         # Move the servo to 90 degrees
#         set_angle(90)
#         time.sleep(1)
        
#         # Move the servo to 180 degrees
#         set_angle(180)
#         time.sleep(1)

# except KeyboardInterrupt:
#     # Clean up GPIO
#     servo_pwm.stop()
#     GPIO.cleanup()
