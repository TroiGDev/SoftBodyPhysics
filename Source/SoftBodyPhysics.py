import pygame
import math
import numpy as np

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

#initialize arrays
bodies = []
#---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
def collideSoftBodies(vert, body):
    #get other verts as tuple coords
    vertTuples = [(overt.x, overt.y) for overt in body.verts]

    #if vert is inside the body
    if isInside((vert.x, vert.y), vertTuples):
        #visual debug---------------------------------------------------------------------------------
        #pygame.draw.circle(screen, (255, 255, 255), (vert.x, vert.y), 5)
    
        #get closest spring to vert
        closestDist = 1000000
        closestSpringIndex = 0

        for i in range(len(body.springs)):
            #get tuple coords of spring
            ospring = body.springs[i]
            springCoords = ((ospring.vert1.x, ospring.vert1.y), (ospring.vert2.x, ospring.vert2.y))

            #compare to current closest
            dist = getDistanceToLineSegment((vert.x, vert.y), springCoords[0], springCoords[1])[0]
            if dist < closestDist:
                closestSpringIndex = i
                closestDist = dist

        #format closest spring
        closestSpring = body.springs[closestSpringIndex]

        #visual debug---------------------------------------------------------------------------------
        #pygame.draw.line(screen, (255, 255, 255), (closestSpring.vert1.x, closestSpring.vert1.y), (closestSpring.vert2.x, closestSpring.vert2.y), 4)

        #get normal from vert to spring
        springV = (closestSpring.vert1.x - closestSpring.vert2.x, closestSpring.vert1.y - closestSpring.vert2.y)
        
        #normalize spring vector, using numpy idk
        norm = np.linalg.norm(springV)
        springV =  springV / norm if norm > 0 else springV

        #get normal to a vector
        clockwise = True
        normalV = np.array([-springV[1], springV[0]] if clockwise else [springV[1], -springV[0]])
        
        #normalize nroaml vector using numpy again idk
        norm = np.linalg.norm(normalV)
        normalV =  normalV / norm if norm > 0 else normalV

        #visual debug---------------------------------------------------------------------------------
        #pygame.draw.line(screen, (255, 255, 255), (400, 300), (400 + normalV[0] * 100, 300 + normalV[1] * 100), 4)

        #apply force along normal outwards from body center to vert
        vert.vX += normalV[0]
        vert.vY += normalV[1]

        #apply force along normal inwards from vert to each springs vert proportionaly
        #get ratio from distances from vert to overts for proportion
        vToPoint1 = (ospring.vert1.x - vert.x, ospring.vert1.y - vert.y)
        vToPoint2 = (ospring.vert2.x - vert.x, ospring.vert2.y - vert.y)
        dist1 = math.sqrt(vToPoint1[0] * vToPoint1[0] + vToPoint1[1] * vToPoint1[1])
        dist2 = math.sqrt(vToPoint2[0] * vToPoint2[0] + vToPoint2[1] * vToPoint2[1])
        sum = dist1 + dist2
        ratio1 = dist1/sum
        ratio2 = dist2/sum

        closestSpring.vert1.vX += -normalV[0] * ratio1
        closestSpring.vert1.vY += -normalV[1] * ratio1

        closestSpring.vert2.vX += -normalV[0] * ratio2
        closestSpring.vert2.vY += -normalV[1] * ratio2

def isInside(point, polygon):
    x, y = point
    n = len(polygon)
    inside = False
    
    p1x, p1y = polygon[0]
    for i in range(n + 1):
        p2x, p2y = polygon[i % n]
        
        if y > min(p1y, p2y):
            if y <= max(p1y, p2y):
                if x <= max(p1x, p2x):
                    if p1y != p2y:
                        xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                    if p1x == p2x or x <= xinters:
                        inside = not inside
        
        p1x, p1y = p2x, p2y
    
    return inside

def getDistanceToLineSegment(point, line_start, line_end):
    px, py = point
    x1, y1 = line_start
    x2, y2 = line_end
    
    # Handle zero-length segments
    if x1 == x2 and y1 == y2:
        return ((px-x1)**2 + (py-y1)**2)**0.5, (x1, y1)
    
    # Length of line segment squared
    segment_length_squared = (x2-x1)**2 + (y2-y1)**2
    
    # If line has zero length, calculate distance to endpoint
    if segment_length_squared < 1e-10:
        return ((px-x1)**2 + (py-y1)**2)**0.5, (x1, y1)
    
    # Parameter in equation of line
    t = max(0, min(1, (
        (px-x1)*(x2-x1) + (py-y1)*(y2-y1)) / segment_length_squared))
    
    # Point on line segment closest to point
    closest_x = x1 + t*(x2-x1)
    closest_y = y1 + t*(y2-y1)
    
    # Distance from point to closest point on segment
    distance = ((px-closest_x)**2 + (py-closest_y)**2)**0.5
    
    return distance, (closest_x, closest_y)
#---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

#optional class for moving points around with your mouse
class MouseGrabber:
    def __init__(self):
        self.isHolding = False
        self.grabDistance = 25
        self.grabbedVert = None

    def grabPoint(self):
        #get mouse pos
        mPos = pygame.mouse.get_pos()
        mX = mPos[0]
        mY = mPos[1]

        #get closest vert
        closestDist = self.grabDistance
        closestVert = None
        for body in bodies:
            for vert in body.verts:
                #get distance to vert
                vX = mX - vert.x
                vY = mY - vert.y
                dist = math.sqrt(vX * vX + vY * vY)

                #compare distances
                if dist < closestDist and dist < self.grabDistance:
                    closestDist = dist
                    closestVert = vert
        
        #if found point
        if closestVert != None and self.isHolding == False:
            self.grabbedVert = closestVert
            self.isHolding = True

    def movePoint(self):
        #get mouse pos
        mPos = pygame.mouse.get_pos()
        mX = mPos[0]
        mY = mPos[1]

        #set grabbed points pos to mouse pos
        self.grabbedVert.x = mX
        self.grabbedVert.y = mY

    def dropPoint(self):
        #clear grabbed point, set isholding to false
        self.grabbedVert = None
        self.isHolding = False

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

class SoftBody():
    def __init__(self, verts, springs, color):
        bodies.append(self)
        self.verts = verts
        self.springs = springs
        self.color = color

    def DrawUpdate(self):
        for spring in self.springs:
            spring.draw(self.color)

    def PhysicsUpdate(self):
        for spring in self.springs:
            spring.updatePoints()

    def CollisionUpdate(self):
        #for every other object
        for otherBody in bodies:

            #if not self
            if self != otherBody:

                #for every vert in self                                 
                for vert in self.verts:

                    #get collision between velf vert and other spring
                    collideSoftBodies(vert, otherBody)
#---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

squareVerts = [
    Vertex(325, 50),
    Vertex(475, 50),
    Vertex(475, 125),
    Vertex(325, 125)
]
squareSprings = [
    Spring(squareVerts[0], squareVerts[1], 150, 1.5),
    Spring(squareVerts[1], squareVerts[2], 150, 1.5),
    Spring(squareVerts[2], squareVerts[3], 150, 1.5),
    Spring(squareVerts[3], squareVerts[0], 150, 1.5),
    Spring(squareVerts[0], squareVerts[2], 210, 1.5),
    Spring(squareVerts[1], squareVerts[3], 210, 1.5)
]
square = SoftBody(squareVerts, squareSprings, (0, 255, 0))

squareVerts2 = [
    Vertex(325 + 25, 50 - 175),
    Vertex(475 + 25, 50 - 175),
    Vertex(475 + 25, 125 - 175),
    Vertex(325 + 25, 125 - 175)
]
squareSprings2 = [
    Spring(squareVerts2[0], squareVerts2[1], 200, 1.5),
    Spring(squareVerts2[1], squareVerts2[2], 200, 1.5),
    Spring(squareVerts2[2], squareVerts2[3], 200, 1.5),
    Spring(squareVerts2[3], squareVerts2[0], 200, 1.5),
    Spring(squareVerts2[0], squareVerts2[2], 200, 1.5),
    Spring(squareVerts2[1], squareVerts2[3], 200, 1.5)
]
square2 = SoftBody(squareVerts2, squareSprings2, (255, 0, 0))

floorY = 500
universalDamping = 0.98
universalGravity = 0.05
universalCollisionForce = 0.1

#initialize mouse grabber for moving points
hand = MouseGrabber()

running = True
while running:
    # Handle events
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        if event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1:
                hand.grabPoint()
        elif event.type == pygame.MOUSEBUTTONUP:
            if event.button == 1:
                hand.dropPoint()
    
    # Fill screen
    screen.fill((0, 0, 0))

    #draw floor
    pygame.draw.line(screen, (255, 255, 255), (0, floorY), (screenWidth, floorY), 1)

    #move points 
    if hand.isHolding:
        hand.movePoint()

    #update all softbodies
    for body in bodies:
        body.CollisionUpdate()
        body.PhysicsUpdate()
        body.DrawUpdate()

    # Update the display
    pygame.display.flip()
    clock.tick(60)

# Quit Pygame
pygame.quit()
