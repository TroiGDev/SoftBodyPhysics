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
#function used for collision
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

        for i in range(len(body.edges)):
            #get tuple coords of spring
            ospring = body.edges[i]
            springCoords = ((ospring.vert1.x, ospring.vert1.y), (ospring.vert2.x, ospring.vert2.y))

            #compare to current closest
            dist = getDistanceToLineSegment((vert.x, vert.y), springCoords[0], springCoords[1])[0]
            if dist < closestDist:
                closestSpringIndex = i
                closestDist = dist

        #format closest spring
        closestSpring = body.edges[closestSpringIndex]

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

#function used for collision
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

#function used for collision
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

#function used for shape matching
def getAngleBetweenVectors(v1, v2):
    # Convert to numpy arrays
    v1 = np.array(v1)
    v2 = np.array(v2)
    
    # Calculate dot product and magnitudes
    dot_product = np.dot(v1, v2)
    magnitude_v1 = np.linalg.norm(v1)
    magnitude_v2 = np.linalg.norm(v2)
    
    # Calculate cosine of angle
    cos_theta = dot_product / (magnitude_v1 * magnitude_v2)
    cos_theta = np.clip(cos_theta, -1.0, 1.0)
    
    # Calculate angle in radians using arccos
    angle_rad = np.arccos(cos_theta)
    
    # Determine direction using cross product
    cross_product = np.cross(v1, v2)
    if cross_product > 0:
        angle_rad = -angle_rad  # Counterclockwise
    
    # Convert to degrees
    angle = np.degrees(angle_rad) * -1
    
    return angle

#function used for shape matching
def rotatePointAroundCenter(c, p, angle):
    angleRad = math.radians(angle)
    #translate center (get vector from center to point)
    t = (p[0] - c[0], p[1] - c[1])
    #do some math magic
    rotatedP = (t[0] * math.cos(angleRad) - t[1] * math.sin(angleRad), t[0] * math.sin(angleRad) + t[1] * math.cos(angleRad))
    #move center back
    finalP = (rotatedP[0] + c[0] , rotatedP[1] + c[1])
    
    return finalP
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
        
        #collide with walls
        if self.x > screenWidth:
            self.x = screenWidth
        if self.x < 0:
            self.x = 0

        #collide with roof
        if self.y < 0:
            self.y = 0

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
    def __init__(self, verts, springs, numOfEdges):

        #each soft body has springs
        #the springs are made up of edges and bones
            #edges = first numOfEdges elements of springs
            #bones = all other springs that are not edges
        #edges = outside springs for collision with other bodies and shape
        #bones = inside springs for structural stability without collision

        bodies.append(self)
        self.verts = verts
        self.springs = springs
        self.edges = springs[:numOfEdges]

        #------------------------------------------------------------------------------
        #get original shape for shape match
        #get avarage position of points
        avPos = (0, 0)
        for vert in self.verts:
            avPos = (avPos[0] + vert.x, avPos[1] + vert.y)
        avPos = (avPos[0] / len(self.verts), avPos[1] / len(self.verts))

        #get vector from center to each point
        self.originVectors = []
        for vert in self.verts:
            self.originVectors.append((vert.x - avPos[0], vert.y - avPos[1]))

        #use vectors as target position from center point in shape match
        #------------------------------------------------------------------------------

    def DrawUpdate(self):
        #draw edges and bones individualy
        for edge in self.edges:
            edge.draw((0, 255, 0))

        for bone in self.springs:
            if bone not in self.edges:
                bone.draw((255, 0, 0))

        #draw face polygon
        """polygonVerts = [(vert.x, vert.y) for vert in self.verts]
        pygame.draw.polygon(screen, (255, 255, 255), polygonVerts, 4)"""

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

    def ShapeMatch(self):
        #get avarage point center
        avPos = (0, 0)
        for vert in self.verts:
            avPos = (avPos[0] + vert.x, avPos[1] + vert.y)
        avPos = (avPos[0] / len(self.verts), avPos[1] / len(self.verts))

        #visual debug ----------------------------------------------------------
        """for i in range(len(self.verts)):
            pygame.draw.circle(screen, (0, 0, 255), (avPos[0] + self.originVectors[i][0], avPos[1] + self.originVectors[i][1]), 5)"""

        #get avarage angle
        #get vectors from center to vert again
        currentVectors = []
        for vert in self.verts:
            currentVectors.append((vert.x - avPos[0], vert.y - avPos[1]))

        #get avarage angle between 2 corresponding vectors
        avDiff = 0
        for i in range(len(self.originVectors)):
            avDiff += getAngleBetweenVectors(self.originVectors[i], currentVectors[i])
        avDiff = avDiff / len(self.originVectors)

        #rotate target shape by angle
        #visual debug ----------------------------------------------------------
        """for i in range(len(self.verts)):
            rotatedVector = rotatePointAroundCenter((0, 0), self.originVectors[i], avDiff)
            pygame.draw.circle(screen, (255, 255, 255), (avPos[0] + rotatedVector[0], avPos[1] + rotatedVector[1]), 5)"""

        #apply forces on verts to target vert in form of an additional spring
        for i in range(len(self.verts)):
            #rotate origin vector
            rotatedVector = rotatePointAroundCenter((0, 0), self.originVectors[i], avDiff)

            #now apply spring physics between vert and point at rotated vector of that vert

            #note: the next code is taken from Spring-updatePoints but modified to take in an input of 2 points instead of springs vert1 and vert2
            #get vector from vert1 to vert 2
            d = ((avPos[0] + rotatedVector[0]) - vert.x, (avPos[1] + rotatedVector[1]) - vert.y)
            distance = math.sqrt(d[0] * d[0] + d[1] * d[1])
            
            #get force strength from stiffnes
            forceStrength = (distance - 0) * universalShapeMatchingStifnes        #resting length of 0

            #get new force vector from strength by angle
            angleRad = math.atan2(d[1], d[0])
            f = (forceStrength * math.cos(angleRad), forceStrength * math.sin(angleRad))

            #apply force only to the current vertex
            vert.vX += f[0] / 100 
            vert.vY += f[1] / 100
#---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

squareVerts = [
    Vertex(325, 50),
    Vertex(475, 50),
    Vertex(475, 200),
    Vertex(325, 200)
]
squareSprings = [
    Spring(squareVerts[0], squareVerts[1], 150, 2),
    Spring(squareVerts[1], squareVerts[2], 150, 2),
    Spring(squareVerts[2], squareVerts[3], 150, 2),
    Spring(squareVerts[3], squareVerts[0], 150, 2),
    Spring(squareVerts[0], squareVerts[2], 210, 2),
    Spring(squareVerts[1], squareVerts[3], 210, 2)
]
square = SoftBody(squareVerts, squareSprings, 4)

squareVerts2 = [
    Vertex(325 + 25, 50 - 175),
    Vertex(475 + 25, 50 - 175),
    Vertex(475 + 25, 200 - 175),
    Vertex(325 + 25, 200 - 175)
]
squareSprings2 = [
    Spring(squareVerts2[0], squareVerts2[1], 200, 2),
    Spring(squareVerts2[1], squareVerts2[2], 200, 2),
    Spring(squareVerts2[2], squareVerts2[3], 200, 2),
    Spring(squareVerts2[3], squareVerts2[0], 200, 2),
    Spring(squareVerts2[0], squareVerts2[2], 200, 2),
    Spring(squareVerts2[1], squareVerts2[3], 200, 2)
]
square2 = SoftBody(squareVerts2, squareSprings2, 4)

floorY = 500
universalDamping = 0.98
universalGravity = 0.05

universalCollisionForce = 0.2
universalShapeMatchingStifnes = 0

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
        body.ShapeMatch()
        body.PhysicsUpdate()
        body.DrawUpdate()

    # Update the display
    pygame.display.flip()
    clock.tick(60)

# Quit Pygame
pygame.quit()
