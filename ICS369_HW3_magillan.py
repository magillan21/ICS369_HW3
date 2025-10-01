# ICS369_HW3_magillan.py
# do all import calls at beginning of file -------------------------------------
import maya.cmds as cmds
import math
import random as rand

# util functions ---------------------------------------------------------------
# add two vectors
def vecAdd(v1, v2):
    return (v1[0]+v2[0], v1[1]+v2[1], v1[2]+v2[2])

# scale a vector by a scalar (test)
def vecScale(v, s):
    return (v[0]*s, v[1]*s, v[2]*s)
                
# create a cube in Maya
def makeCube():
    nodeList = cmds.polyCube(width=1, height=1, depth=1)
    return nodeList[0]

# particle code ----------------------------------------------------------------
# define particle as list of attributes [node, position, velocity]
def makeParticle(pos, vel):
    return [makeCube(), pos, vel]

def getNode(ptcl):
    return ptcl[0]

def setNode(ptcl, node):
    ptcl[0] = node
    
def getPos(ptcl):
    return ptcl[1]

def setPos(ptcl, pos):
    ptcl[1] = pos
    
def getVel(ptcl):
    return ptcl[2]

def setVel(ptcl, vel):
    ptcl[2] = vel

# move Maya node to the particle position
def updateNode(ptcl):
    pos = getPos(ptcl)
    cmds.move(pos[0], pos[1], pos[2], getNode(ptcl))

# update particle velocity
def applyForces(ptcl, gravity, wind, groundY):
    vel = getVel(ptcl)
    pos = getPos(ptcl)
    
    # always apply gravity
    newVel = vecAdd(vel, gravity)
    
    # only apply wind if particle is above ground
    if pos[1] > groundY:
        newVel = vecAdd(newVel, wind)
    
    setVel(ptcl, newVel)

# update particle position by its velocity
def updatePos(ptcl):
    pos = getPos(ptcl)
    vel = getVel(ptcl)
    newPos = vecAdd(pos, vel)
    setPos(ptcl, newPos)

# handle ground collision with elasticity and friction
def handleGroundCollision(ptcl, groundY, elasticity, friction):
    pos = getPos(ptcl)
    vel = getVel(ptcl)
    
    # check if particle is at or below ground
    if pos[1] <= groundY:
        # move particle back to ground level
        setPos(ptcl, (pos[0], groundY, pos[2]))
        
        # apply elasticity (bounce) - reverse y velocity and scale by elasticity
        newVelY = -vel[1] * elasticity
        
        # apply friction - slow down x and z components
        newVelX = vel[0] * (1.0 - friction)
        newVelZ = vel[2] * (1.0 - friction)
        
        setVel(ptcl, (newVelX, newVelY, newVelZ))

# update the particle (position, forces, collision, node)
def updateParticle(ptcl, gravity, wind, groundY, elasticity, friction):
    applyForces(ptcl, gravity, wind, groundY)
    updatePos(ptcl)
    handleGroundCollision(ptcl, groundY, elasticity, friction)
    updateNode(ptcl)

# set keyframe for particle
def keyframeParticle(ptcl):
    cmds.setKeyframe(getNode(ptcl))

# create and return a list of particles with random velocities
def initParticles(num, minVel, maxVel):
    particleList = []
    for i in range(num):
        pos = (0, 10, 0)
        vel = (rand.uniform(minVel[0], maxVel[0]), 
               rand.uniform(minVel[1], maxVel[1]), 
               rand.uniform(minVel[2], maxVel[2]))
        particleList.append(makeParticle(pos, vel))
    return particleList

# run a list of particles for a number of frames
def runParticles(particles, frames, gravity, wind, groundY, elasticity, friction):
    for frame in range(0, frames):
        cmds.currentTime(frame+1)
        for ptcl in particles:
            updateParticle(ptcl, gravity, wind, groundY, elasticity, friction)
            keyframeParticle(ptcl)

# test: initialize and run particles -------------------------------------------
# set up parameters
minVel = (-1.0, -1.0, -1.0)
maxVel = (1.0, 1.0, 1.0)
gravity = (0, -0.098, 0)  # gravity force in -y direction
wind = (0.01, 0, 0)  # slight wind in +x direction
groundY = 1  # ground plane at y=1
elasticity = 0.7  # 70% bounce
friction = 0.5  # 50% slowdown per collision

# create and run particles
particles = initParticles(100, minVel, maxVel)
runParticles(particles, 100, gravity, wind, groundY, elasticity, friction)
