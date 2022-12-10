import pygame
from sys import exit
import numpy as np
import random
import math

WIDTH = 1600
HEIGHT = 900

pygame.init()
screen = pygame.display.set_mode((WIDTH , HEIGHT))
clock = pygame.time.Clock()

class Agent:
   def __init__(self, x, y):
      self.position = np.array([x, y])
      self.velocity = np.array([6, 2])
      self.acceleration = np.array([0, 0])
      self.__max_speed = 6 
      self.__max_force = 0.3 
      #In order to wander in random directions
      self.wandering_angle = math.pi/2 * random.uniform(-1, 1)

   
   def update(self):
      self.velocity = self.velocity + self.acceleration
      self.velocity = limit_vector_to_max_value(self.velocity, self.__max_speed)
      self.position = self.position + self.velocity 
      #accelartion is set each frame according to the force applied
      #if the is no force applied on that frame the acceleraton is 0
      self.acceleration = self.acceleration * 0

      # stay on screen
      if self.position[0] < 0:
         self.position[0] = self.position[0] + WIDTH
      if self.position[0] > WIDTH:
         self.position[0] = self.position[0] - WIDTH
      if self.position[1] > HEIGHT:
         self.position[1] = self.position[1] - HEIGHT
      if self.position[1] < 0:
         self.position[1] = self.position[1] + HEIGHT


   def apply_force(self, force_vector):
      #F = ma but we dont care about m
      self.acceleration = force_vector.copy()

   def draw(self):
      agent_direction_vector = self.get_agent_direction_vector_normalized()
  
      tangent_vector = agent_direction_vector.copy()
      tangent_vector = np.flip(tangent_vector)
      tangent_vector[0] *= -1
      
      triangle_side_length = 10
      triangle_height = 30
      p1 = self.position - tangent_vector * triangle_side_length - agent_direction_vector * triangle_height
      p2 = self.position + tangent_vector * triangle_side_length - agent_direction_vector * triangle_height
      p3 = self.position 
      
      pygame.draw.polygon(screen, 'Red', (p1, p2, p3))
      
   
   def seek(self, target):
      direction_vector = target - self.position
      distance = np.linalg.norm(direction_vector)
      direction_vector_normalized =  direction_vector/distance
      desired_velocity = self.__max_speed * direction_vector_normalized 
      
      steer_vector = desired_velocity - self.velocity
      steer_vector = limit_vector_to_max_value(steer_vector, self.__max_force)
      
      self.apply_force(steer_vector)

   def flee(self, target):
      direction_vector = -(target - self.position)
      distance = np.linalg.norm(direction_vector)
      direction_vector_normalized =  direction_vector/distance
      desired_velocity = self.__max_speed * direction_vector_normalized
      
      steer_vector = desired_velocity - self.velocity
      steer_vector = limit_vector_to_max_value(steer_vector, self.__max_force)
      
      self.apply_force(steer_vector)

   def pursue(self, target):
      time = 15
      target_future_position = target.position + target.velocity * time
      self.seek(target_future_position)

   def arrive(self , target):
      direction_vector = target - self.position
      distance = np.linalg.norm(direction_vector)
      direction_vector_normalized =  direction_vector/distance
      desired_velocity = self.__max_speed * direction_vector_normalized
      
      #Slow down when aproaching target
      if distance < 250:
         proximity_multiplier = (distance/100) * self.__max_speed / 10
         desired_velocity = desired_velocity * proximity_multiplier
      
      steer_vector = desired_velocity - self.velocity
      steer_vector = limit_vector_to_max_value(steer_vector, self.__max_force)

      self.apply_force(steer_vector)

   def wander(self):
      #The agent follows a point on a circle 
      #The point moves randomly around the circle
      circle_radius = 4
      future_point = self.get_future_location(100)
      
      self.wandering_angle += math.cos(random.uniform(0, 2*math.pi))
      angle = self.wandering_angle + self.get_angle_relative_to_x()
      x,y = circle_radius * math.cos(angle), circle_radius * math.sin(angle)
      wander_point = future_point + [x, y]
      
      self.seek(wander_point)

   def follow_path(self, path):                                      
      # Point that is closest to agents future location on given path
      future_point = self.get_future_location(150)
      segment_points = path.points
      closest_normal_point = math.inf
      for i in range(len(segment_points) - 1):
         point1 = segment_points[i]
         point2 = segment_points[i+1]

         normal_point = closest_point_on_line_to_point(future_point, point1, point2)
          #check if point is inside line start and end points
         if normal_point[0] < point1[0] or normal_point[0] > point2[0]:
            normal_point = point2
      
         distance_between_agent_and_normal_point = np.linalg.norm(future_point - normal_point)
         if distance_between_agent_and_normal_point < np.linalg.norm(closest_normal_point):
            closest_normal_point = normal_point

         if distance_between_agent_and_normal_point < path.radius:
            self.seek(future_point)
         else:
            self.seek(closest_normal_point) 
        

   def get_angle_relative_to_x(self):
      x, y = self.velocity
      return np.arctan2(y, x)
   
   def get_agent_direction_vector_normalized(self):
      if np.linalg.norm(self.velocity) == 0:
         direction =  np.array([1,0])
      else:
         direction = self.velocity/np.linalg.norm(self.velocity)
      return direction

   def get_future_location(self, time):
      agent_direction_vector = self.get_agent_direction_vector_normalized()
      return agent_direction_vector * time + self.position

   def seperate(self, other_agents):
      separation_distance = 30
      vector_sum_of_nearby_agents = np.array([0, 0]) 
 
      for agent in other_agents:
         distance_between_agents = np.linalg.norm(agent.position - self.position)
         if distance_between_agents <= 0 or distance_between_agents > separation_distance: continue
         
         relative_position_vector = agent.position - self.position
         relative_position_vector = relative_position_vector/distance_between_agents
         vector_sum_of_nearby_agents = relative_position_vector + vector_sum_of_nearby_agents
      

      if np.linalg.norm(vector_sum_of_nearby_agents) == 0: return
      vector_sum_normalized = vector_sum_of_nearby_agents/np.linalg.norm(vector_sum_of_nearby_agents)
      desired_velocity = vector_sum_normalized * self.__max_speed
      steer_vector = self.velocity - desired_velocity
      self.apply_force(steer_vector)
      
 


class Path:
   def __init__(self):
      self.radius = 50
      self.points = []

   def draw(self):
      for i in range(len(self.points) - 1):
         p1 = self.points[i]
         p2 = self.points[i+1]
         pygame.draw.line(screen, "Green", p1, p2, self.radius)
   
   def add_point(self, x, y):
      self.points.append(np.array([x, y]))

def limit_vector_to_max_value (vector, max_value):
   magnitude =  np.linalg.norm(vector)
   if magnitude > max_value:
      multiplier = max_value/magnitude
      vector = multiplier * vector
   return vector

def closest_point_on_line_to_point(point, line_start, line_end):
   p, a, b, = point, line_start, line_end
   ap = p - a
   ab = b - a
   ab_norm = ab/np.linalg.norm(ab)
   projection_vector = ap.dot(ab_norm) * ab_norm
   return a + projection_vector
   



#agent = Agent(1000, 600)
#target = Agent(300,200)
agents = [Agent(random.randint(0, WIDTH), random.randint(0, HEIGHT)) for i in range(30)]
path = Path()
path.add_point(0, HEIGHT/3)
path.add_point(WIDTH/2, HEIGHT/2)
path.add_point(WIDTH, HEIGHT/2)

while True:

   for event in pygame.event.get():
      if event.type == pygame.QUIT:
         pygame.quit()
         exit()

   screen.fill((0,0,0))

   path.draw()
   for agent in agents:
      agent.follow_path(path)
      agent.seperate(agents)
      agent.update()
      agent.draw()

  

   pygame.display.update()
   clock.tick(60)


#If it is seeking a path and tryingy to sepearte the velocity change creates twitching