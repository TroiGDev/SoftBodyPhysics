import pygame
import math

pygame.init()
screenWidth = 800
screenHeight = 600
screen = pygame.display.set_mode((screenWidth, screenHeight))
clock = pygame.time.Clock()
pygame.display.set_caption('SoftBodyPhysics')

#remove window icon
transparent_surface = pygame.Surface((32, 32), pygame.SRCALPHA)
transparent_surface.fill((0, 0, 0, 0))
pygame.display.set_icon(transparent_surface)

#---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

class Vertex:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.vX = 0
        self.vY = 0

    def applyVelocity(self):
        #apply gravity before damping
        self.vY += universalGravity

        #apply velocity
        self.x += self.vX
        self.y += self.vY

        #apply damping friction to velocities
        self.vX *= universalDamping
        self.vY *= universalDamping

        #collide with floor
        if self.y > floorY: 
            self.y = floorY

    def draw(self, color):
        pygame.draw.circle(screen, color, (self.x, self.y), 5)

class Spring:
    def __init__(self, vert1, vert2, restingLength, stiffnes):
        self.vert1 = vert1
        self.vert2 = vert2
        self.restingLength = restingLength
        self.stiffnes = stiffnes

    def updatePoints(self):
        #get vector from vert1 to vert 2
        d = (self.vert2.x - self.vert1.x, self.vert2.y - self.vert1.y)
        distance = math.sqrt(d[0] * d[0] + d[1] * d[1])
        
        #get force strength from stiffnes
        forceStrength = (distance - self.restingLength) * self.stiffnes

        #get new force vector from strength by angle
        angleRad = math.atan2(d[1], d[0])
        f = (forceStrength * math.cos(angleRad), forceStrength * math.sin(angleRad))

        #apply force to the points (ones force is opposite), extra division for less force
        self.vert1.vX += f[0] / 100
        self.vert1.vY += f[1] / 100
        self.vert2.vX += f[0] * -1 / 100
        self.vert2.vY += f[1] * -1 / 100

        #finaly update individual points position
        self.vert1.applyVelocity()
        self.vert2.applyVelocity()

    def draw(self, color):
        pygame.draw.line(screen, color, (self.vert1.x, self.vert1.y), (self.vert2.x, self.vert2.y), 2)


#---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

vert1 =  Vertex(325, 50)
vert2 = Vertex(475, 50)
vert3 = Vertex(325, 125)
vert4 = Vertex(475, 125)

spring1 = Spring(vert1, vert2, 150, 1.5)
spring2 = Spring(vert2, vert4, 150, 1.5)
spring3 = Spring(vert4, vert3, 150, 1.5)
spring4 = Spring(vert3, vert1, 150, 1.5)
bone1 = Spring(vert1, vert4, 210, 1.5)
bone2 = Spring(vert2, vert3, 210, 1.5)

floorY = 500
universalDamping = 0.99
universalGravity = 0.1

running = True
while running:
    # Handle events
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    
    # Fill screen
    screen.fill((0, 0, 0))

    #draw floor
    pygame.draw.line(screen, (255, 255, 255), (0, floorY), (screenWidth, floorY), 1)

    #physics update
    spring1.updatePoints()
    spring2.updatePoints()
    spring3.updatePoints()
    spring4.updatePoints()
    bone1.updatePoints()
    bone2.updatePoints()

    #draw update
    spring1.draw((0, 255, 0))
    spring2.draw((0, 255, 0))
    spring3.draw((0, 255, 0))
    spring4.draw((0, 255, 0))
    bone1.draw((155, 0, 0))
    bone2.draw((155, 0, 0))

    """vert1.draw((0, 0, 255))
    vert2.draw((0, 0, 255))
    vert3.draw((0, 0, 255))
    vert4.draw((0, 0, 255))"""

    # Update the display
    pygame.display.flip()
    clock.tick(60)

# Quit Pygame
pygame.quit()
