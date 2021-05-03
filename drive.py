from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import threading  

import numpy as np
import mapping
import detect_objects
import time
import queue
import threading
import math

SCALE = 5 # Defines the scale of the grid. Scale = nummer of cm per grid square



def main():
  
  # Create an array that will act as a grid map for routing. This should be large enough to cover the whole route
  grid = np.ones((int(300/SCALE), int(300/SCALE)))
  start = (int((grid.shape[0] / 2) - 50/SCALE), int(grid.shape[1] / 2)) # The start location of the car's ultrasonic sensor in grid coordinates
  goal = (int(210 / SCALE), int(150 / SCALE)) # The final location of the car in grid coordinates

  path = mapping.a_star(start, goal, grid)
  
  direction = {0: 'right', np.pi: 'left', np.pi/2: 'forward', -np.pi/2: 'back'}
  # Find the direction from the current location to the next location along path
  current_direction = direction[math.atan2(path[-1][0] - start[0], path[-1][1] - start[1])]

  if current_direction == 'right':
    print('turn left')
    mapping.left90()
  # Car will begin pointing forward, if next point is to the left (in grid coordinates), rotate car right
  elif current_direction == 'left':
    print('turn right')
    mapping.right90()
  elif current_direction == 'back':
    print ('turn around')
    mapping.left90()
    mapping.left90()
  print('move forward')
  mapping.forward5()

  print(path)
  for c in path:
    grid[c] = 3
  mapping.print_grid(grid, start)


  while start != goal:
    # start object detection. For each frame, add whether a stop sign is in the frame or not to a queue
    q = queue.Queue()
    t = threading.Thread(target=detect_objects.detect_stop_sign, name=detect_objects.detect_stop_sign,  args=(q,))
    t.start()
      
    # Queue is filled as stop signes are dtected in frames, so car should keep moving as long as
    # the queus is empty
    while q.empty() == True:
      start, grid, path = mapping.follow_path(path, start, grid, goal)

    # If stop sign detected, print to console, pause vehicle for 5 seconds, and empty queue
    print('STOP SIGN DETECTED!')
    time.sleep(5)
    q.queue.clear()
  print('END')


if __name__ == '__main__':
  main()