# ICS_369_HW3_Setup.py

# do all import calls at beginning of file -------------------------------------
import maya.cmds as cmds
import math
import random as rand

# util functions ---------------------------------------------------------------

# add two vectors
def vecAdd(v1, v2):
    return (v1[0]+v2[0], v1[1]+v2[1], v1[2]+v2[2])
                
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

# update particle position by its velocity
def updatePos(ptcl):
    pos = getPos(ptcl)
    vel = getVel(ptcl)
    newPos = vecAdd(pos, vel)
    setPos(ptcl, newPos)

# update the particle (position and node)
def updateParticle(ptcl):
    updatePos(ptcl)
    updateNode(ptcl)

# set keyframe for particle
def keyframeParticle(ptcl):
    cmds.setKeyframe(getNode(ptcl))

# create and return a list of particles with random velocities
def initParticles(num):
    particleList = []
    for i in range(num):
        pos = (0, 10, 0)
        vel = (rand.uniform(-1.0, 1.0), rand.uniform(-1.0, 1.0), rand.uniform(-1.0, 1.0))
        particleList.append(makeParticle(pos, vel))
    return particleList

# run a list of particles for a number of frames
def runParticles(particles, frames):
    for frame in range(0, frames):
        cmds.currentTime(frame+1)
        for ptcl in particles:
            updateParticle(ptcl)
            keyframeParticle(ptcl)

# test: initialize and run particles -------------------------------------------

particles = initParticles(100)
runParticles(particles, 240)







