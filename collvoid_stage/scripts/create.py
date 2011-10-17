#!/usr/bin/env python

import math
import sys
import getopt
import commands

def create_world_file(argv):
    numRobots = 0
    circleSize = 0
    centerX = -2.2
    centerY = 2
    holo = False
    truePos = False
    simulation = True
    runExperiments = False
    scaleRadius = False
    useNoise = False
    bagFileName = "collvoid.bag"
    try:
        opts, args= getopt.getopt(argv, "hn:s:Rolrxf:N", ["help","numRobots=","circleSize=","radiusScaling","holo","truePosition","real","experiments","bagFileName="])
    except getopt.GetoptError:
        print 'CreateLaunchAndWorld.py -n <numRobots> -s <circleSize> <-h> <-l> <-r> <-x> <-f> bagFile <-r>'
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print 'CreateLaunchAndWorld.py -n <numRobots> -s <circleSize> <-h> <-l> <-r> <-x> <-f> bagfile <-r>'
            sys.exit(2)
        elif opt in ("-n","--numRobots"):
            numRobots = int(arg)
        elif opt in ("-s","--circleSize"):
            circleSize = float(arg)
        elif opt in ("-o","--holo"):
            holo = True
        elif opt in ("-l","--truePosition"):
            truePos = True
        elif opt in ("-r","--real"):
            simulation = False
        elif opt in ("-x", "--experiments"):
            runExperiments = True
        elif opt in ("-f", "--bagFileName"):
            bagFileName = str(arg)
        elif opt in ("-R", "--radiusScaling"):
            scaleRadius = True
        elif opt in ("-N"):
            useNoise = True
    
    direct = commands.getoutput('rospack find collvoid_sim')
    if (holo):
        worldFileTemp = open(direct + '/world/simple_tempHolo.world','r')
    else:
        worldFileTemp = open(direct + '/world/simple_temp.world','r')
    worldFileNew = open(direct + '/world/simple_create.world','w')
    worldFileNew.write(worldFileTemp.read())
    direct = commands.getoutput('rospack find stage')

    colors = open(direct + '/share/stage/rgb.txt','r')
    line = colors.readline()
    line = colors.readline()
    cols = []
    while line:
        cols.append(line[line.rfind("\t")+1:line.rfind("\n")])
        line = colors.readline()
    colors.close()
    #print cols
    for x in range(numRobots):
        angle = 360.0 / numRobots
        anglePrint = x * angle-180-45
        angle = x * angle - 45
        posX = circleSize*math.cos(angle/360*2*math.pi)
        
        posY = circleSize*math.sin(angle/360*2*math.pi)
        #if (holo):
        #   worldFileNew.write('tele_robot( pose [ {0:f} {1:f} 0 90 ] name "robot_{2:d}" color "{3}")\n'.format(centerX+posX,centerY+posY,x,cols[40 +  1 * x]))
        #else:
        worldFileNew.write('tele_robot( pose [ {0:f} {1:f} 0 {2:f} ] name "robot_{3:d}" color "{4}")\n'.format(centerX+posX,centerY+posY,anglePrint,x,cols[40 +  10 * x]))

    
    #tele_robot( pose [ -1 0 0 180.000 ] name "bob" color "blue")
    worldFileTemp.close()
    worldFileNew.close()
    create_yaml_file(circleSize,numRobots,holo,simulation,truePos,centerX,centerY,scaleRadius,useNoise)
    create_launch_file(numRobots,holo,runExperiments,bagFileName,truePos)
    
    
def create_yaml_file(circleSize, numRobots,holo,simulation,truePos,centerX,centerY,scaleRadius,useNoise):
    direct = commands.getoutput('rospack find collvoid_sim')
    yamlWrite = open(direct + '/params.yaml','w')
    if (simulation):
        yamlWrite.write("/use_sim_time: true\n")
        yamlWrite.write("/simulation_mode: true\n")
    else:
        yamlWrite.write("/simulation_mode: false\n")
    if (truePos):
        yamlWrite.write('/useTruePos: true\n')
    else:
        yamlWrite.write('/useTruePos: false\n')
    if (scaleRadius):
       yamlWrite.write('/scaleRadius: true\n')
    else:
       yamlWrite.write('/scaleRadius: false\n')
    
    yamlWrite.write('/numRobots: ' + str(numRobots) + '\n')
    
    yamlWrite.write("/maxNeighbors: 10\n")
    yamlWrite.write("/neighborDist: 15.0\n")
    yamlWrite.write("/timeHorizon: 10.0\n")
    yamlWrite.write("/timeHorizonObst: 10.0\n")
    yamlWrite.write("/thresholdGoal: 0.1\n")
    yamlWrite.write("/thresholdLastSeen: 1.0\n")
    yamlWrite.write("/nrIntialGuess: 20.0\n")
    if not useNoise:
        yamlWrite.write("/noiseStd: 0.0\n")
    else:
        yamlWrite.write("/noiseStd: 0.20\n")

    angle = 360.0 / numRobots
    for x in range(numRobots):
        angleX = 90 + x * angle - 45
        posX = circleSize*math.cos(angleX/360*2*math.pi)
        posY = circleSize*math.sin(angleX/360*2*math.pi)
  
        yamlWrite.write('robot_{0}:\n'.format(x))
        yamlWrite.write('    wheel_base: 0.32\n')
        yamlWrite.write("    maxSpeed_linear: 0.5\n")
        yamlWrite.write("    maxSpeed_angular: 1.5\n")
        yamlWrite.write("    minSpeed_linear: 0.01\n")
        yamlWrite.write("    minSpeed_angular: 0.01\n")
        yamlWrite.write("    radius: 0.25\n") # sqrt(0.17^2*2)
        yamlWrite.write('    goal:\n')
        yamlWrite.write('        x: {0:f}\n'.format(centerX-posY))
        yamlWrite.write('        y: {0:f}\n'.format(centerY+posX))
        yamlWrite.write('        ang: {0:f}\n'.format(angleX))
       
    
    yamlWrite.close()
    
def create_launch_file(numRobots,holo,runExperiments, bagFilename, truePos):
    direct = commands.getoutput('rospack find collvoid_sim')

    launchWrite = open(direct + '/launch/control.launch','w')
    launchWrite.write("<launch>\n")
    launchWrite.write('<node name="map_server" pkg="map_server" type="map_server" args="$(find collvoid_sim)/world/map.yaml"/>\n')
    launchWrite.write('<rosparam command="load" file="$(find collvoid_sim)/params.yaml"/>\n')
    if (runExperiments):
        launchWrite.write('<node pkg="controller" type="watchdog.py" name="watchdog" output="screen"/>\n')
    else:
        launchWrite.write('<node pkg="controller" type="controller.py" name="controller" output="screen"/>\n')
    launchWrite.write('<node pkg="stage" type="stageros" name="stageros" args="$(find collvoid_sim)/world/simple_create.world" respawn="false" output="screen" />\n')
#    launchWrite.write('<node pkg="path_listener" type="global_listener.py" name="glob_listener" output="screen"/>\n')

    for x in range(numRobots):
#        launchWrite.write('<node pkg="path_listener" type="listener.py" name="listener" ns="robot_{0}" output="screen"/>\n'.format(x))
        if True: # TODO: use "not truePos" amcl is still used in orca_planner
            launchWrite.write('<include file="$(find collvoid_sim)/launch/amcl_diff.launch">\n')
            launchWrite.write('<arg name="robot" value="robot_{0}"/>\n'.format(x))
            launchWrite.write('</include>\n')
        if (holo):
            launchWrite.write('<node pkg="random_nav_pub" type="orcaCPPdecentralized" name="orcaCPPdecentralized" ns="robot_{0}"/>\n'.format(x))
        else:
            launchWrite.write('<node pkg="orca_planner" type="ROSAgent" name="ROSAgent" ns="robot_{0}" output="screen"/>\n'.format(x))
            launchWrite.write('<rosparam command="load" file="$(find collvoid_sim)/orca_lines.yaml" ns="/robot_{0}"/>\n'.format(x))

            #launchWrite.write('<node pkg="random_nav_pub" type="nhOrcaCPPdecentralized" name="ROSAgent" ns="robot_{0}" output="screen"/>\n'.format(x))
        
    
    s = ""
    for x in range(numRobots):
        s += "/robot_%d/debug "%(x)

    launchWrite.write('<node pkg="rosbag" type="record" name="rosbag" args="record {0} /stall /stall_resolved /exceeded -O $(find collvoid_sim)/{1}" output="screen"/>\n'.format(s,bagFilename))
    

    launchWrite.write("</launch>\n")

    
    launchWrite.close()

    
if __name__ == "__main__":
    create_world_file(sys.argv[1:])

