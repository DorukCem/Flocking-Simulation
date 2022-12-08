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
      self.velocity = np.array([-6, 0])
      self.acceleration = np.array([0, 0])
      self.__max_speed = 6 
      self.__max_force = 0.3 
      #In order to wander in random directions
      self.wandering_angle = math.pi/2

   
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
      direction_vector = self.velocity/ np.linalg.norm(self.velocity)
  
      tangent_vector = direction_vector.copy()
      tangent_vector = np.flip(tangent_vector)
      tangent_vector[0] *= -1
      
      triangle_side_length = 10
      triangle_height = 30
      p1 = self.position - tangent_vector * triangle_side_length - direction_vector * triangle_height
      p2 = self.position + tangent_vector * triangle_side_length - direction_vector * triangle_height
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
      
      if distance < 250:
         proximity_multiplier = (distance/100) * self.__max_speed / 10
         desired_velocity = desired_velocity * proximity_multiplier
      
      steer_vector = desired_velocity - self.velocity
      steer_vector = limit_vector_to_max_value(steer_vector, self.__max_force)

      self.apply_force(steer_vector)

   def wander(self):
      radius = 4
      future_point = self.get_direction_angle() * 100 + self.position

      
      self.wandering_angle += math.cos(random.uniform(0, 2*math.pi))
      angle = self.wandering_angle + self.get_heading_angle()
      x,y = radius * math.cos(angle), radius * math.sin(angle)
      wander_point = future_point + [x, y]
      
      self.seek(wander_point)    

   def get_direction_angle(self):
      x, y = self.velocity
      return np.arctan2(y, x)   

    

def limit_vector_to_max_value (vector, max_value):
   magnitude =  np.linalg.norm(vector)
   if magnitude > max_value:
      multiplier = max_value/magnitude
      vector = multiplier * vector
   return vector

agent = Agent(1000, 600)
target = Agent(300,200)


while True:

   for event in pygame.event.get():
      if event.type == pygame.QUIT:
         pygame.quit()
         exit()

  
   screen.fill((0,0,0))
   agent.draw()

   target.position = pygame.mouse.get_pos()
   target.update()
   target.draw()

 
   agent.wander()
   agent.update()

   

   pygame.display.update()
   clock.tick(60)


#Add better draw method an refactor
#Do excerszies