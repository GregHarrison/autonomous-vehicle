import picar_4wd as fc
import time

SPEED = 15

def reverse30():
   """
   #Car reverses ~30cm
   """
   speed4 = fc.Speed(25)
   speed4.start()
   fc.backward(SPEED)
   x = 0
   for i in range(5):
      time.sleep(0.1)
      speed = speed4()
      x += speed * 0.1
   speed4.deinit()
   fc.stop()

def main():
   """
   #Car moves forward until the ultrasonic sensor detects an object within a few centimeters 
   #of the front bumper. When an object is detected, the car stops, chooses another random
   #direction, backs up, turns, and then moves forward in the new direction.
   """

   while True:
      scan_list = fc.scan_step(35)
      if not scan_list:
         continue

      tmp = scan_list[3:7]
      if tmp != [2,2,2,2]:
         reverse30()
         fc.turn_right(SPEED)
      else:
         fc.forward(SPEED)


if __name__=='__main__':
   try:
      main()
   finally:
      fc.stop()