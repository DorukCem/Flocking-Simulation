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

   def add_force(self, force, force_weight):
      self.forces_to_be_apllied.append((force, force_weight))

   def apply_force(self):
      sum_vector = np.array([0, 0])
      for force, weight in self.forces_to_be_apllied:
         sum_vector = sum_vector + force * weight
      force_vector = limit_vector_to_max_value(sum_vector, self.max_force)
      # F = ma but m is cosntant so F directly affects a
      self.acceleration = self.acceleration + force_vector
      self.forces_to_be_apllied.clear()

   def update(self):
      self.apply_force()
      self.velocity = self.velocity + self.acceleration
      self.velocity = limit_vector_to_max_value(self.velocity, self.max_speed)
      self.position = self.position + self.velocity 
      #acceleration is set each frame according to the force that is applied
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
         
   #Agents seeks a target
   def get_seeking_force(self, target):
      direction_vector = target - self.position
      direction_vector_normalized =  normalize_vector(direction_vector)
      desired_velocity = self.max_speed * direction_vector_normalized 
      
      steer_vector = desired_velocity - self.velocity
      steer_vector = limit_vector_to_max_value(steer_vector, self.max_force)
      
      return steer_vector

   #Agent wanders randomly 
   def get_wandering_force(self):
      #The agent follows a point on a circle 
      #The point moves randomly around the circle
      circle_radius = 4
      future_point = self.get_future_location(100)
      self.wandering_angle += random.uniform(-0.2,0.2)
      angle = self.wandering_angle + self.get_angle_relative_to_x()
      x,y = circle_radius * math.cos(angle), circle_radius * math.sin(angle)
      wander_point = future_point + [x, y]
      
      return self.get_seeking_force(wander_point)

   #Agents try not to bump into each other
   def get_seperation_force(self):
         separation_distance = 30
         vector_sum_of_nearby_agents = np.array([0, 0]) 

         #accumulate the relative positions of enarby agents as a vector
         for agent in self.friends:
            relative_position_vector = agent.position - self.position
            distance_between_agents = magnitude_of_vector(relative_position_vector)
            if distance_between_agents == 0 or distance_between_agents > separation_distance: 
               continue
            #The vector gets smaller as the target is farther away
            vector_sum_of_nearby_agents =  vector_sum_of_nearby_agents + relative_position_vector/distance_between_agents 
         
         #steer away from the accumulated position
         if magnitude_of_vector(vector_sum_of_nearby_agents) == 0:
            return np.array([0, 0])
         vector_sum_normalized = normalize_vector(vector_sum_of_nearby_agents)
         desired_velocity = vector_sum_normalized * self.max_speed
         steer_vector = self.velocity - desired_velocity
         steer_vector = limit_vector_to_max_value(steer_vector, self.max_force)
         
         return steer_vector

   #Agents try to allign
   def get_alignment_force(self):
      vector_sum = np.array([0, 0])
      for agent in self.friends:
         if self.color != agent.color: continue
         #The agent will try to steer in the average direction of nearby agents
         vector_sum = vector_sum + agent.velocity
      
      if magnitude_of_vector(vector_sum) == 0:
         return vector_sum
      sum_normalized = normalize_vector(vector_sum)
      desired_velocity = sum_normalized * self.max_speed

      steer_vector = desired_velocity - self.velocity
      steer_vector = limit_vector_to_max_value(steer_vector, self.max_force)
      return steer_vector

   #Agents try to stay together
   def get_cohesion_force(self):
      vector_sum = np.array([0, 0])
      count = 0
      
      for agent in self.friends:
         if self.color != agent.color: continue
         #agent will try to steer toward the average position of naerby agents
         vector_sum = vector_sum + agent.position
         count += 1
      if count == 0: return vector_sum

      average_vector = vector_sum/count
      return self.get_seeking_force(average_vector)


   def flock(self):
      seperation_force = self.get_seperation_force()
      algnment_force = self.get_alignment_force()
      cohesion_force = self.get_cohesion_force()
      wandering_force = self.get_wandering_force()

      self.add_force(seperation_force, 5)
      self.add_force(algnment_force, 1)
      self.add_force(cohesion_force, 0.25)
      self.add_force(wandering_force, 1)


   def get_angle_relative_to_x(self):
      x, y = self.velocity
      return np.arctan2(y, x)
   
   #The direction that the agent is heading
   def get_agent_direction_vector_normalized(self):
      if magnitude_of_vector(self.velocity) == 0:
         direction =  np.array([1,0])
      else:
         direction = normalize_vector(self.velocity)
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

def magnitude_of_vector(vector):
   return np.linalg.norm(vector)

def normalize_vector(vector):
   return vector/magnitude_of_vector(vector)

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

