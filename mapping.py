import picar_4wd as fc
import numpy as np
import math
import cv2
import time
from queue import PriorityQueue

RANGE = 50 # The distance in cm from the ultrasonic sensor to find obstacles
SCALE = 5 # Defines the scale of the grid. Scale = number of cm per grid square
MAX_SERVO_ANGLE = 90

def forward5():
  """
  Car moves forward 5-cm.
  """
  fc.forward(30)
  time.sleep(0.2)
  fc.stop()




def left90():
  """
  Car rotates 90-degrees counterclockwise.
  """
  time.sleep(0.1) # The front left wheel is broken and prevents turns right after forward movement
  fc.turn_left(25)
  time.sleep(0.75)
  fc.stop()




def right90():
  """
  Car rotates 90-degrees clockwise.
  """
  fc.turn_right(25)
  time.sleep(0.75)
  fc.stop()




def obstacle_coords(angle, distance, origin, direction):
  """
    Inputs:
      angle: The angle of the ultrasonic sensor
      distance: The distance measurment from the ultrasonic sensor
    Outputs:
      The row and column index number where an obstacle is located on the grid
  """
  angle = np.deg2rad(angle)
  
  if direction == 'forward':
    if angle <= 0:
      row = origin[0] + abs((distance * np.cos(angle) / SCALE))
      col = origin[1] - abs((distance * np.sin(angle) / SCALE))
    else:
      row = origin[0] + abs((distance * np.cos(angle) / SCALE))
      col = origin[1] + abs((distance * np.sin(angle) / SCALE))
  elif direction == 'back':
    if angle <= 0:
      row = origin[0] - abs((distance * np.cos(angle) / SCALE))
      col = origin[1] + abs((distance * np.sin(angle) / SCALE))
    else:
      row = origin[0] - abs((distance * np.cos(angle) / SCALE))
      col = origin[1] - abs((distance * np.sin(angle) / SCALE))
  elif direction == 'left':
    if angle <= 0:
      row = origin[0] - abs((distance * np.sin(angle) / SCALE))
      col = origin[1] - abs((distance * np.cos(angle) / SCALE))
    else:
      row = origin[0] + abs((distance * np.sin(angle) / SCALE))
      col = origin[1] - abs((distance * np.cos(angle) / SCALE))
  elif direction == 'right':
    if angle <= 0:
      row = origin[0] + abs((distance * np.sin(angle) / SCALE))
      col = origin[1] + abs((distance * np.cos(angle) / SCALE))
    else:
      row = origin[0] - abs((distance * np.sin(angle) / SCALE))
      col = origin[1] + abs((distance * np.cos(angle) / SCALE))


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




def find_obstacles(grid, origin, direction, max_servo_angle=60, step=5):
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
  #grid = np.ones((int(300/SCALE), int(300/SCALE)))
  
  readings = [] # A list to store the coordinates of sensor readings

  step = int(MAX_SERVO_ANGLE / 10) # The number of degrees between readings

  # Rotate the servo, taking a distance measurement every 'step' degrees
  for angle in np.arange(-max_servo_angle, max_servo_angle + step, step):
    distance = fc.get_distance_at(angle)
    
    # We will only be concerned with what is within a RANGE-cm radius of the car
    if distance >= 0 and distance <= RANGE:
      
      row, col = obstacle_coords(angle, distance, origin, direction)
      print('Angle: {}  Distance: {}  Coordinates: [{}, {}]'.format(angle, distance, row, col))
      
      # Add a 100 to the grid where an obstacle exists
      grid[row, col] = 100
      
      # Add additional 100s in a radius around each obstacle point to compensate for the width 
      # of the vehicle. Only done on obstacles more than 2 spaces away because if obstacles are closer,
      # the added 100s will surround the vdehicles current possition
      if distance > SCALE * 2:
        radius=5
        start_point=(int(col - radius/SCALE), int(row - radius/SCALE))
        end_point=(int(col + radius/SCALE), int(row + radius/SCALE))
        grid = cv2.rectangle(grid, start_point, end_point, 100, -1)
        #grid = cv2.circle(grid, center=(col, row), radius=int(round(9/SCALE)), color=255, thickness=-1)

      if readings:
        # peek at the previous reading from the ultrasonic sensor
        previous_reading = readings[-1]
      
        # If the previous reading from the ultrasonic sensor showed an obsacle within RANGE-cm
        if previous_reading[0]:
          for t in range(step):
            row, col = interp(row, col, previous_reading[1][0], previous_reading[1][1], t/step)
            grid[row, col] = 100

            # Add additional 100s in a radius around each obstacle point to compensate for the width 
            # of the vehicle. Only done on obstacles more than 2 spaces away because if obstacles are closer,
            # the added 100s will surround the vdehicles current possition
            if distance > 2 * SCALE:
              start_point = (int(col - radius/SCALE), int(row - radius/SCALE))
              end_point=(int(col + radius/SCALE), int(row + radius/SCALE))
              grid = cv2.rectangle(grid, start_point, end_point, 100, -1)
              #grid = cv2.circle(grid, center=(x, y), radius=int(round(radius/SCALE)), color=255, thickness=-1)

      # Add the current obstacle to the list of sensor readings readings
      readings.append((1, (row, col)))
    
    # If the ultrasonic sensor measures a distance of greater than 50-cm
    else:
      readings.append((0, (np.nan, np.nan)))
    
  return grid




def print_grid(grid, start):
  grid[start[0], start[1]] = 4
  for row in range(grid.shape[0]):
    for col in range(grid.shape[1]):
      if grid[row, col] == 1:
        print('.', end=' ')
      # If obstacle is present (obstacle = 100) 
      elif grid[row, col] > 99:
        print('#', end=' ')
      # if measurement point (measurement point = 4)
      elif grid[row, col] > 3:
        print('X', end=' ')
      # If a pathline coordinate is present (path = 3)
      elif grid[row, col] > 2:
        print('O', end=' ')
        grid[row, col] = 1
    print('\n', end='')




def get_neighbors(coords, grid_shape):
  """
  Implementation taken from https://www.redblobgames.com/pathfinding/grids/graphs.html
  
  Inputs:
    coords: Coordinates in the form (x, y)
    grid_shape: The shape of the grid in [row, height] format
  Output:
    A list containing the coordinates of the 4 neighbors of the coordinate passed to the function.
    Each coordinate has four neighbors: up, down, left, and right.
  """
  directions = [[1,0], [0,1], [-1,0], [0,-1]]
  results = []
  for direction in directions:
    neighbor = [coords[0] + direction[0], coords[1] + direction[1]]
    
    # Check that the neighbors are within the range of the grid, where range defined by global variable, RANGE
    if neighbor[0] >= 0 and neighbor[0] < grid_shape[0] and neighbor[1] >= 0 and neighbor[1] < grid_shape[1]:
      results.append((*neighbor, ))
  return results




def a_star(start_coords, goal_coords, grid):
  """
  
  Implementation taken from https://www.redblobgames.com/pathfinding/a-star/implementation.html section 1.4

  Output:
    A list of coordinates of grid that represented the shortest path from start_coords to goal_coords
  """
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
    for next in get_neighbors(current_location, grid.shape):
      
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

  # Move backwards from the goal location, using the came_from dictionary to create
  # a list containing the coordinates of the shortest path from goal to start
  path = []
  while current_location != start_coords:
    path.append(current_location)
    current_location = came_from[current_location]

  return path


def scan(grid, current_coord, goal, current_direction):
  grid = find_obstacles(grid, current_coord, direction=current_direction, max_servo_angle=MAX_SERVO_ANGLE)
  path = a_star(current_coord, goal, grid)
  print(path)
  for c in path:
    grid[c] = 3
  print_grid(grid, current_coord)

  return grid, path



def follow_path(path, current_coord, grid, goal):  

  direction = {0: 'right', np.pi: 'left', np.pi/2: 'forward', -np.pi/2: 'back'}
  
  # Find the direction from the current location to the next location along path
  current_direction = direction[math.atan2(path[-1][0] - current_coord[0], path[-1][1] - current_coord[1])]

  grid, path = scan(grid, current_coord, goal, current_direction)
 
  for _ in range(3):
    next_coord = path.pop()
    
    if len(path) >= 1:
      next_direction = direction[math.atan2(path[-1][0] - next_coord[0], path[-1][1] - next_coord[1])]
    else:
      current_coord = next_coord
      break
    
    if current_direction == 'forward' and next_direction == 'right':
      print('turn left')
      print('move forward')
      left90()
      forward5()
    elif current_direction == 'forward' and next_direction == 'left':
      print('turn right')
      print('move forward')
      right90()
      forward5()
    elif current_direction == 'right' and next_direction == 'forward':
      print('turn right')
      print('move forward')
      right90()
      forward5()
    elif current_direction == 'right' and next_direction == 'back':
      print('turn left')
      print('move forward')
      left90()
      forward5()
    elif current_direction == 'left' and next_direction == 'forward':
      print('turn left')
      print('move forward')
      left90()
      forward5()
    elif current_direction == 'left' and next_direction == 'back':
      print('turn right')
      print('move forward')
      right90()
      forward5()
    elif current_direction == 'forward' and next_direction == 'back':
      print('turn around')
      left90()
      left90()
      print('move forward')
      forward5()
    elif current_direction == 'left' and next_direction == 'right':
      print('turn around')
      left90()
      left90()
      print('move forward')
      forward5()
    elif current_direction == 'right' and next_direction == 'left':
      print('turn around')
      left90()
      left90()
      print('move forward')
      forward5()
    elif current_direction == 'back' and next_direction == 'right':
      print('turn right')
      print('move forward')
      right90()
      forward5()
    elif current_direction == 'back' and next_direction == 'right':
      print('turn left')
      print('move forward')
      left90()
      forward5()
    else:
      print('move forward')
      forward5()

    current_direction = next_direction
    current_coord = next_coord

  return current_coord, grid, path



def follow_path_2(current_coord, goal, grid):  
  """
  Inputs:
    current_coord: The location of the car's ultrasonic sensor in the grid
  """
  direction = {0: 'right', np.pi: 'left', np.pi/2: 'forward', -np.pi/2: 'back'}
  
  grid = find_obstacles(grid, current_coord, direction='forward', max_servo_angle=MAX_SERVO_ANGLE)
  path = a_star(current_coord, goal, grid)
  print(path)
  for c in path:
    grid[c] = 3
  print_grid(grid, current_coord)

  # Find the direction from the current location to the next location along path
  current_direction = direction[math.atan2(path[-1][0] - current_coord[0], path[-1][1] - current_coord[1])]
  
  # Car will begin pointing forward, if next point is to the right (in grid coordinates), rotate car left
  if current_direction == 'right':
    print('turn left')
    left90()
  # Car will begin pointing forward, if next point is to the left (in grid coordinates), rotate car right
  elif current_direction == 'left':
    print('turn right')
    right90()
  elif current_direction == 'back':
    print ('turn around')
    left90()
    left90()
  print('move forward')
  forward5()

  count = 0 # Keeps track of the number of movements the car has made
  
  while path:
    next_coord = path.pop()

    count += 1 # Keep track of the number of loops
    # every third loop, scan the environment for obstacles
    if count % 4 == 0:
      grid = find_obstacles(grid, origin=next_coord, direction=current_direction, max_servo_angle=MAX_SERVO_ANGLE)
      path = a_star(next_coord, goal, grid)
      print(path)
      for c in path:
        grid[c] = 3
      print_grid(grid, next_coord)

    if len(path) >= 1:
      next_direction = direction[math.atan2(path[-1][0] - next_coord[0], path[-1][1] - next_coord[1])]
    else:
      break
    
    if current_direction == 'forward' and next_direction == 'right':
      print('turn left')
      print('move forward')
      left90()
      forward5()
    elif current_direction == 'forward' and next_direction == 'left':
      print('turn right')
      print('move forward')
      right90()
      forward5()
    elif current_direction == 'right' and next_direction == 'forward':
      print('turn right')
      print('move forward')
      right90()
      forward5()
    elif current_direction == 'right' and next_direction == 'back':
      print('turn left')
      print('move forward')
      left90()
      forward5()
    elif current_direction == 'left' and next_direction == 'forward':
      print('turn left')
      print('move forward')
      left90()
      forward5()
    elif current_direction == 'left' and next_direction == 'back':
      print('turn right')
      print('move forward')
      right90()
      forward5()
    elif current_direction == 'forward' and next_direction == 'back':
      print('turn around')
      left90()
      left90()
      print('move forward')
      forward5()
    elif current_direction == 'left' and next_direction == 'right':
      print('turn around')
      left90()
      left90()
      print('move forward')
      forward5()
    elif current_direction == 'right' and next_direction == 'left':
      print('turn around')
      left90()
      left90()
      print('move forward')
      forward5()
    elif current_direction == 'back' and next_direction == 'right':
      print('turn right')
      print('move forward')
      right90()
      forward5()
    elif current_direction == 'back' and next_direction == 'right':
      print('turn left')
      print('move forward')
      left90()
      forward5()
    else:
      print('move forward')
      forward5()

    current_direction = next_direction
    current_coord = next_coord

  return current_coord, current_direction

def main():
  # Create an array that will act as a grid map for routing. This should be large enough to cover the whole route
  grid = np.ones((int(300/SCALE), int(300/SCALE)))
  start = (int((grid.shape[0] / 2) - 100/SCALE), int(grid.shape[1] / 2)) # The start location of the car's ultrasonic sensor in grid coordinates
  goal = (int(120 / SCALE), int(150 / SCALE)) # The final location of the car in grid coordinates
  follow_path_2(start, goal, grid)
  
  """#direction = 'forward'
  #grid = find_obstacles(grid, start, direction=direction, max_servo_angle=MAX_SERVO_ANGLE)
  grid = find_obstacles(grid, start, direction='forward', max_servo_angle=MAX_SERVO_ANGLE)
  path = a_star(start, goal, grid)
  print(path)
  for c in path:
    grid[c] = 3
  print_grid(grid, start)
  #grid, path = scan(start, goal, grid, 'forward')

  direction = {0: 'right', np.pi: 'left', np.pi/2: 'forward', -np.pi/2: 'back'}
  # Find the direction from the current location to the next location along path
  current_direction = direction[math.atan2(path[-1][0] - start[0], path[-1][1] - start[1])]
  
  # Car will begin pointing forward, if next point is to the right (in grid coordinates), rotate car left
  if current_direction == 'right':
    print('turn left')
    left90()
  # Car will begin pointing forward, if next point is to the left (in grid coordinates), rotate car right
  elif current_direction == 'left':
    print('turn right')
    right90()
  elif current_direction == 'back':
    print ('turn around')
    left90()
    left90()
  print('move forward')
  #forward5()

  while start != goal:
    start, grid, current_direction, path = follow_path(path, start, grid, current_direction, goal)"""




if __name__=='__main__':
  try:
    main()

  finally:
    fc.stop()

