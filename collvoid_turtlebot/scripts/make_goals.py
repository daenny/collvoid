#!/usr/bin/env python

import math
import sys
import getopt
import commands

def create_world_file(argv):
    offset = 0
    try:
        opts, args= getopt.getopt(argv, "hx:y:n:s:o:", ["help","x=","y=","num=","size=","offset="])
    except getopt.GetoptError:
        print 'make_goals.py -x <x> -y <y> -n <num>'
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print 'make_goals.py -n <numRobots> -s <circleSize> <-h> <-l> <-x> <-f> bagfile'
            sys.exit(2)
        elif opt in ("-n","--num"):
            num = int(arg)
        elif opt in ("-o","--offset"):
            offset = int(arg)
        elif opt in ("-x","--x"):
            x = float(arg)
        elif opt in ("-y","--y"):
            y = float(arg)
        elif opt in ("-s","--size"):
            size = float(arg)
    if not num or not x or not y or not x or not size:
        print 'make_goals.py -n <numRobots> -s <circleSize> <-h> <-l> <-x> <-f> bagfile'
        sys.exit(2)
       
    create_goal_file(size,num,x,y, offset)
    
def create_goal_file(circleSize, numRobots, centerX, centerY, offset=0):
    yamlWrite = open('goals_created.yaml','w')
    
    angle = 360.0 / numRobots
    for x in range(numRobots):
        angleX = offset + x * angle
        posX = circleSize*math.cos(angleX/360*2*math.pi)
        posY = circleSize*math.sin(angleX/360*2*math.pi)
  
        yamlWrite.write('tb_{0}:\n'.format(x))
        yamlWrite.write('    goals:\n')
        yamlWrite.write('        x: [{0:f}, {1:f}]\n'.format(centerX+posX, centerX-posX))
        yamlWrite.write('        y: [{0:f}, {1:f}]\n'.format(centerY+posY, centerY-posY))
        yamlWrite.write('        ang: [{0:f}, {1:f}]\n'.format((angleX-180) / 360.0 * 2 * math.pi,(angleX) / 360.0 * 2 * math.pi))
    
    yamlWrite.close()
    
    
if __name__ == "__main__":
    create_world_file(sys.argv[1:])

