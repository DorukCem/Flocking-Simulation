import pygame
from sys import exit
import numpy as np
import random
import math

WIDTH = 800 * 1.5
HEIGHT = 450  * 1.5

BLACK = "#000000"
BLUE = "#00bfb2"
PINK = "#e56db1"

pygame.init()
screen = pygame.display.set_mode((WIDTH , HEIGHT))
clock = pygame.time.Clock()

class Agent: #Random starting velocity
   def __init__(self, x, y):
      self.position = np.array([x, y])
      self.velocity = np.array([6, 2])
      self.acceleration = np.array([0, 0])
      self.max_speed = 3 
      self.max_force = 0.3
      self.forces_to_be_apllied = []
      
       #In order to wander in random directions
      self.wandering_angle = math.pi/2 * random.uniform(-1, 1)

      self.color = random.choice([BLUE, PINK])
      self.friends = []

      
   def update(self):
      self.apply_force()
      self.velocity = self.velocity + self.acceleration
      self.velocity = limit_vector_to_max_value(self.velocity, self.max_speed)
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

   def add_force(self, force, force_weight):
      self.forces_to_be_apllied.append((force, force_weight))

   def apply_force(self):
      #F = ma but we dont care about m
      sum_vector = np.array([0, 0])
      for force, weight in self.forces_to_be_apllied:
         sum_vector = sum_vector + force * weight
      force_vector = limit_vector_to_max_value(sum_vector, self.max_force)

      self.acceleration = self.acceleration + force_vector
      self.forces_to_be_apllied.clear()

   def draw(self): 
      agent_direction_vector = self.get_agent_direction_vector_normalized()
  
      tangent_vector = agent_direction_vector.copy()
      tangent_vector = np.flip(tangent_vector)
      tangent_vector[0] *= -1
      
      triangle_side_length = 7
      triangle_height = 21
      p1 = self.position - tangent_vector * triangle_side_length - agent_direction_vector * triangle_height
      p2 = self.position + tangent_vector * triangle_side_length - agent_direction_vector * triangle_height
      p3 = self.position 
      
      pygame.draw.polygon(screen, self.color, (p1, p2, p3))
       

   def get_nearby_agents(self, agents):
      self.friends.clear()
      friend_distance = 60
      for agent in agents:
         if agent is self: continue
         distance = np.linalg.norm(self.position - agent.position) 
         if  distance < friend_distance: 
            self.friends.append(agent)
         

   def seek(self, target):
      direction_vector = target - self.position
      distance = np.linalg.norm(direction_vector)
      direction_vector_normalized =  direction_vector/distance
      desired_velocity = self.max_speed * direction_vector_normalized 
      
      steer_vector = desired_velocity - self.velocity
      steer_vector = limit_vector_to_max_value(steer_vector, self.max_force)
      
      return steer_vector

   def wander(self):
      #The agent follows a point on a circle 
      #The point moves randomly around the circle
      circle_radius = 4
      future_point = self.get_future_location(100)
      self.wandering_angle += random.uniform(-0.2,0.2)
      #self.wandering_angle += math.cos(random.uniform(0, 2*math.pi))
      angle = self.wandering_angle + self.get_angle_relative_to_x()
      x,y = circle_radius * math.cos(angle), circle_radius * math.sin(angle)
      wander_point = future_point + [x, y]
      
      return self.seek(wander_point)

   def seperate(self):
         separation_distance = 30
         vector_sum_of_nearby_agents = np.array([0, 0]) 
   
         for agent in self.friends:
            distance_between_agents = np.linalg.norm(agent.position - self.position)
            if distance_between_agents == 0 or distance_between_agents > separation_distance: continue
            
            relative_position_vector = agent.position - self.position
            relative_position_vector = relative_position_vector/distance_between_agents
            vector_sum_of_nearby_agents = relative_position_vector + vector_sum_of_nearby_agents
         
         if np.linalg.norm(vector_sum_of_nearby_agents) == 0: return np.array([0, 0])
         vector_sum_normalized = vector_sum_of_nearby_agents/np.linalg.norm(vector_sum_of_nearby_agents)
         desired_velocity = vector_sum_normalized * self.max_speed
         steer_vector = self.velocity - desired_velocity
         steer_vector = limit_vector_to_max_value(steer_vector, self.max_force)
         
         return steer_vector


   def align(self):

      vector_sum = np.array([0, 0])
      
      for agent in self.friends:
         if self.color != agent.color: continue
         vector_sum = vector_sum + agent.velocity
      
      if np.linalg.norm(vector_sum) == 0: return vector_sum

      sum_normalized = vector_sum/np.linalg.norm(vector_sum)
      desired_velocity = sum_normalized * self.max_speed

      steer_vector = desired_velocity - self.velocity
      steer_vector = limit_vector_to_max_value(steer_vector, self.max_force)
      return steer_vector

   def cohesion(self):
      vector_sum = np.array([0, 0])
      count = 0
      
      for agent in self.friends:
         if self.color != agent.color: continue
         vector_sum = vector_sum + agent.position
         count += 1
      if count == 0: return vector_sum

      average_vector = vector_sum/count
      return self.seek(average_vector)


   def flock(self):
      seperation_vector = self.seperate()
      algnment_vector = self.align()
      cohesion_vector = self.cohesion()
      wandering_vector = self.wander()

      self.add_force(seperation_vector, 5)
      self.add_force(algnment_vector, 1)
      self.add_force(cohesion_vector, 0.25)
      self.add_force(wandering_vector, 1)

   def flee(self):
      vector_sum = np.array([0, 0])
      count = 0
      for agent in self.nearby_enemies:
         vector_sum = vector_sum + agent.position
         count += 1
    
      average_vector = self.max_speed * vector_sum/count
      seek_vector = self.seek(-average_vector)
      self.add_force(seek_vector, 1)
   

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


#########

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



agents = [Agent(random.randint(0, WIDTH), random.randint(0, HEIGHT)) for i in range(40)] 
#agents = [Agent(400+i,400+i) for i in range(30)]

while True:

   for event in pygame.event.get():
      if event.type == pygame.QUIT:
         pygame.quit()
         exit()

   screen.fill(BLACK)

   for agent in agents:
      agent.get_nearby_agents(agents)
      agent.flock()
      agent.update()
      agent.draw()


   pygame.display.update()
   clock.tick(60)

