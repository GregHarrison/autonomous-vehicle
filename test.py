import picar_4wd as fc
from picar_4wd.pwm import PWM
from picar_4wd.servo import Servo
from picar_4wd.filedb import FileDB 
import time

config = FileDB()
ultrasonic_servo_offset = int(config.get('ultrasonic_servo_offset', default_value = 0)) 
servo = Servo(PWM("P0"), offset=ultrasonic_servo_offset)

while True:
   angle = input("Angle: ")
   #servo.set_angle(angle)
   print('Distance: {}'.format(fc.get_distance_at(angle)))

