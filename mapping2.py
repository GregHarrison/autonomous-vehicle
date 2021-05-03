import picar_4wd as fc
import numpy as np
import math
import cv2
import time
from queue import PriorityQueue

RANGE = 50 # The distance in cm from the ultrasonic sensor to find obstacles\
SCALE = 1 # Defines the scale of the grid. Scale = nubmer of cm per grid square
ORIGIN = (0, int(RANGE / SCALE)) # The start location of the car's ultrasonic sensor in the grid

def forward10():
  """
  Car moves forward 10-cm.
  """
  fc.forward(25)
  time.sleep(0.3)
  fc.stop()




def left90():
  """
  Car rotates 90-degrees counterclockwise.
  """
  fc.turn_left(100)
  time.sleep(0.4)
  fc.stop()




def right90():
  """
  Car rotates 90-degrees clockwise.
  """
  fc.turn_right(100)
  time.sleep(0.4)
  fc.stop()




def obstacle_coords(angle, distance):
  """
    Inputs:
      angle: The angle of the ultrasonic sensor
      distance: The distance measurment from the ultrasonic sensor
    Outputs:
      The row and column index number where an obstacle is located on the grid
  """
  
  row = (distance * np.cos(math.radians(angle))) / SCALE
  col = (distance * np.sin(math.radians(angle))) / SCALE

  if angle < 0:
    col = ORIGIN[1] - abs(col)
  elif angle > 0:
    col = ORIGIN[1] + col

  return int(round(row)), int(round(col))



def interp(x1, y1, x2, y2, t):
  """
    Inputs:
      x1: The x-coordinate of the first point
      y1: The y-coordinate of the first point
      x2: The x-coordinate of the second point
      y2: The y-coordinate of the second point
      distance: a value in range [0,1] that defines the distance of the output coordiante from the first coordinate,
        0 is first point, and 1 is the second point
    Output:
      The coordinate of a point at a certain distance on line between points defined by (x1, y1) and (x2, y2)
  """
  x = x1 + t * (x2 - x1)
  y = y1 + t * (y2 - y1)

  return int(x), int(y)




def find_obstacles(max_servo_angle=60, step=5):
  """
    Inputs:
      max_servo_angle: An integer in range [-90, 90] that defines the radius that the servo (attached to
        the ultrasonic sensor) will rotate when taking distance measurements from the ultrasonic sensor. The
        servo will rotate from -max_servo_angle to +max_servo_angle
      step: The number of degrees between ultrasonic sensor reading
    Output:
      A numpy array that shows all of the obstacles within a 50-cm radius of the ultrasonic sensor. Grid contains 
      0s where no obstacle is present and 1s where an obstacle is present
  """
  
  grid = np.ones((int(RANGE / SCALE), int(RANGE / SCALE) * 2 + 1)) # An array that will act as a grid map for routing
  readings = [] # A list to store the coordinates of sensor readings

  step = 10

  # Rotate the servo, taking a distance measurement every 'step' degrees
  for angle in np.arange(-max_servo_angle, max_servo_angle + step, step):
    distance = fc.get_distance_at(angle)
    
    # We will only be concerned with what is within a 50-cm radius of the car
    if distance > 0 and distance <= RANGE:
      row, col = obstacle_coords(angle, distance)
      print('Angle: {}  Distance: {}  Coordinates: [{}, {}]'.format(angle, distance, row, col))
      
      # Add a 2 to the grid where an obstacle exists
      grid[row, col] = 2
      
      # Add additional 1s in a 9-cm radius around each obstacle point to compensate for the width 
      # of the vehicle
      grid = cv2.circle(grid, center=(col, row), radius=int(round(9/SCALE)), color=255, thickness=-1)

      if readings:
        # peek at the previous reading from the ultrasonic sensor
        previous_reading = readings[-1]
      
        # If the previous reading from the ultrasonic sensor showed an obsacle within 50-cm
        if previous_reading[0]:
          for t in range(step):
            x, y = interp(col, row, previous_reading[1][1], previous_reading[1][0], t/step)
            grid[y, x] = 2
            
            # Add additional 1s in a 9-cm radius around each interpolated obstacle point
            # to compensate for the width of the vehicle
            grid = cv2.circle(grid, center=(y, x), radius=int(round(9/SCALE)), color=255, thickness=-1)

      # Add the current obstacle to the list of sensor readings readings
      readings.append((1, (row, col)))
    
    # If the ultrasonic sensor measures a distance of greater than 50-cm
    else:
      readings.append((0, (np.nan, np.nan)))
    
  return grid




def print_grid(grid):
  grid[ORIGIN[0], ORIGIN[1]] = -1
  for row in range(grid.shape[0]):
    for col in range(grid.shape[1]):
      if grid[row, col] == 1:
        print('.', end='')
      elif grid[row, col] > 1:
        print('X', end='')
      else:
        print('O', end='')
    print('\n', end='')




def get_neighbors(coords):
  """
  Implementation taken from https://www.redblobgames.com/pathfinding/grids/graphs.html
  
  Inputs:
    coords: Coordinates in the form (x, y)
  Output:
    A list containing the coordinates of the 4 neighbors of the coordinate passed to the function.
    Each coordinate has four neighbors: up, down, left, and right.
  """
  directions = [[1,0], [0,1], [-1,0], [0,-1]]
  results = []
  for direction in directions:
    neighbor = [coords[0] + direction[0], coords[1] + direction[1]]
    
    # Check that the neighbors are within the range of the ultrasonic sensor defined by global variable, RANGE
    if 0 <= neighbor[0] < int(RANGE / SCALE) and 0 <= neighbor[1] < int(RANGE / SCALE) * 2:
      results.append((*neighbor, ))
  return results




def a_star(start_coords, goal_coords, grid):
  """
  
  Implementation taken from https://www.redblobgames.com/pathfinding/a-star/implementation.html section 1.4

  Output:
    A list of coordinates of grid that represented the shortest path from start_coords to goal_coords
  """
  print(start_coords, goal_coords)
  frontier = PriorityQueue()
  frontier.put(start_coords, 0)
  came_from = dict()
  cost_so_far = dict()
  came_from[start_coords] = None
  cost_so_far[start_coords] = 0

  while not frontier.empty():
    current_location = frontier.get()

    if current_location == goal_coords:
      break

    # for each neighbor
    for next in get_neighbors(current_location):
      # define the new cost as the cost to get from start location to the current location plus the cost
      # to get to the next location
      new_cost = cost_so_far[current_location] + grid[next[0], next[1]]

      # If we have not already visited this neighbors location or if the cost to move to this neighbor is less
      # than the cost to move to the previous neighbor that was assessed
      if next not in cost_so_far or new_cost < cost_so_far[next]:
        cost_so_far[next] = new_cost
        
        # measure the distance from the neighbor to the goal location, and add it to the priority queue
        frontier.put(next, abs(goal_coords[0] - next[0]) + abs(goal_coords[1] - next[1]))
        came_from[next] = current_location 

  print(cost_so_far)
  # Move backwards from the goal location, using the came_from dictionary to create
  # a list containing the coordinates of the shortest path from goal to start
  path = []
  while current_location != start_coords:
    path.append(current_location)
    current_location = came_from[current_location]

  return path


def follow_route(path):
  direction = {0: 'right', np.pi: 'left', np.pi/2: 'forward', -np.pi/2: 'back'}
  current_coord = ORIGIN # The location of the car's ultrasonic sensor in the grid
  
  #  Find the direction from the current location to the next location along path
  current_direction = direction[math.atan2(path[-1][0] - current_coord[0], path[-1][1] - current_coord[1])]

  # Car will begin pointing forward, if next point is to the right (in grid coordinates), rotate car left
  if current_direction == 'right':
    print('turn left')
    left90()
  
  # Car will begin pointing forward, if next point is to the left (in grid coordinates), rotate car right
  elif current_direction == 'left':
    print('turn right')
    right90()
  
  print('move forward')
  forward10()


  while path:
    next_coord = path.pop()
    
    if len(path) >= 1:
      next_direction = direction[math.atan2(path[-1][0] - next_coord[0], path[-1][1] - next_coord[1])]
    else:
      break
    
    if current_direction == 'forward' and next_direction == 'right':
      print('turn left')
      left90()

    elif current_direction == 'forward' and next_direction == 'left':
      print('turn right')
      right90()
    
    elif current_direction == 'right' and next_direction == 'forward':
      print('turn right')
      right90()
    
    elif current_direction == 'right' and next_direction == 'back':
      print('turn left')
      left90()

    elif current_direction == 'left' and next_direction == 'forward':
      print('turn left')
      left90()
    
    elif current_direction == 'left' and next_direction == 'back':
      print('turn right')
      right90()
    
    print('move forward')
    forward10()
    
    current_direction = next_direction
    current_coord = next_coord



if __name__=='__main__':
  try:
    grid = find_obstacles(90)
    print_grid(grid)
    path = a_star(ORIGIN, (int(50 / SCALE), int(0 / SCALE)), grid)
    print(path)
    for c in path:
      grid[c] = -1
    print_grid(grid)
    #follow_route(path)
    
    #forward10()
  finally:
    fc.stop()

