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
    omni = False
    groudTruth = True
    simulation = True
    runExperiments = False
    scaleRadius = False
    useNoise = False
    useBagFile = False
    bagFileName = "collvoid.bag"
    try:
        opts, args= getopt.getopt(argv, "hn:s:Rolxf:N", ["help","numRobots=","circleSize=","radiusScaling","omni","localization","experiments","bagFileName=","noise"])
    except getopt.GetoptError:
        print 'create.py -n <numRobots> -s <circleSize> <-h> <-l> <-x> <-f> bagFile <-R>'
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print 'create.py -n <numRobots> -s <circleSize> <-h> <-l> <-x><-f> bagfile <-R>'
            sys.exit(2)
        elif opt in ("-n","--numRobots"):
            numRobots = int(arg)
        elif opt in ("-s","--circleSize"):
            circleSize = float(arg)
        elif opt in ("-o","--omni"):
            omni = True
        elif opt in ("-l","--localization"):
            groundTruth = False
        elif opt in ("-x", "--experiments"):
            runExperiments = True
        elif opt in ("-f", "--bagFileName"):
            useBagFile = True
            bagFileName = str(arg)
        elif opt in ("-R", "--radiusScaling"):
            scaleRadius = True
        elif opt in ("-N"):
            useNoise = True
    
    direct = commands.getoutput('rospack find collvoid_stage')
    worldFileTemp = open(direct + '/world/swarmlab_template.world','r')
    worldFileNew = open(direct + '/world/swarmlab_created.world','w')
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
        if (omni):
           worldFileNew.write('omni_robot( pose [ {0:f} {1:f} 0 {2:f} ] name "robot_{3:d}" color "{4}")\n'.format(centerX+posX,centerY+posY,anglePrint,x,cols[40 +  10 * x]))
        else:
            worldFileNew.write('diff_robot( pose [ {0:f} {1:f} 0 {2:f} ] name "robot_{3:d}" color "{4}")\n'.format(centerX+posX,centerY+posY,anglePrint,x,cols[40 +  10 * x]))

    
    #tele_robot( pose [ -1 0 0 180.000 ] name "bob" color "blue")
    worldFileTemp.close()
    worldFileNew.close()
    create_yaml_file(circleSize,numRobots,omni,simulation,localization,centerX,centerY,scaleRadius,useNoise)
    create_launch_file(numRobots,omni,runExperiments,bagFileName,localization,useBagFile)
    
    
def create_yaml_file(circleSize, numRobots,omni,simulation,localization,centerX,centerY,scaleRadius,useNoise):
    direct = commands.getoutput('rospack find collvoid_stage')
    yamlWrite = open(direct + '/params_created.yaml','w')
    yamlWrite.write("/use_sim_time: true\n")
    yamlWrite.write("/simulation_mode: true\n")
    if not localization:
        yamlWrite.write('/use_ground_truth: true\n')
    else:
        yamlWrite.write('/use_ground_truth: false\n')
    if (scaleRadius):
       yamlWrite.write('/scale_radius: true\n')
    else:
       yamlWrite.write('/scale_radius: false\n')
    
    yamlWrite.write('/num_robots: ' + str(numRobots) + '\n')
    
    yamlWrite.write("/max_neighbors: 10\n")
    yamlWrite.write("/neighbor_dist: 15.0\n")
    yamlWrite.write("/time_horizon: 10.0\n")
    yamlWrite.write("/time_horizon_obst: 10.0\n")
    yamlWrite.write("/threshold_goal: 0.1\n")
    yamlWrite.write("/threshold_last_seen: 1.0\n")
    yamlWrite.write("/nr_initial_guess: 20.0\n")
    if not useNoise:
        yamlWrite.write("/init_guess_noise_std: 0.0\n")
    else:
        yamlWrite.write("/init_guess_noise_std: 0.20\n")

    angle = 360.0 / numRobots
    for x in range(numRobots):
        angleX = 90 + x * angle - 45
        posX = circleSize*math.cos(angleX/360*2*math.pi)
        posY = circleSize*math.sin(angleX/360*2*math.pi)
  
        yamlWrite.write('robot_{0}:\n'.format(x))
        yamlWrite.write('    wheel_base: 0.25\n') 
        if (omni):
            yamlWrite.write('    diff: false\n')
        else:
            yamlWrite.write('    diff: true\n')
        yamlWrite.write("    maxSpeed_linear: 0.5\n")
        yamlWrite.write("    maxSpeed_angular: 1.5\n")
        yamlWrite.write("    minSpeed_linear: 0.01\n")
        yamlWrite.write("    minSpeed_angular: 0.01\n")
        yamlWrite.write("    radius: 0.24\n") # sqrt(0.17^2*2)
        yamlWrite.write('    goal:\n')
        yamlWrite.write('        x: {0:f}\n'.format(centerX-posY))
        yamlWrite.write('        y: {0:f}\n'.format(centerY+posX))
        yamlWrite.write('        ang: {0:f}\n'.format(angleX))
       
    
    yamlWrite.close()
    
def create_launch_file(numRobots,omni,runExperiments, bagFilename, localization,useBagFile):
    direct = commands.getoutput('rospack find collvoid_stage')

    launchWrite = open(direct + '/launch/sim_created.launch','w')
    launchWrite.write("<launch>\n")
    launchWrite.write('<node name="map_server" pkg="map_server" type="map_server" args="$(find collvoid_stage)/world/swarmlab.yaml"/>\n')
    launchWrite.write('<rosparam command="load" file="$(find collvoid_stage)/params_created.yaml"/>\n')
    if (runExperiments):
        launchWrite.write('<node pkg="collvoid_controller" type="watchdog.py" name="watchdog" output="screen"/>\n')
    else:
        launchWrite.write('<node pkg="collvoid_controller" type="controller.py" name="controller" output="screen"/>\n')
    launchWrite.write('<node pkg="stage" type="stageros" name="stageros" args="$(find collvoid_stage)/world/swarmlab_created.world" respawn="false" output="screen" />\n')
    for x in range(numRobots):
        if localization: # TODO: use "not localizationx" amcl is still used in orca_planner
            launchWrite.write('<include file="$(find collvoid_stage)/launch/amcl_diff.launch">\n')
            launchWrite.write('<arg name="robot" value="robot_{0}"/>\n'.format(x))
            launchWrite.write('</include>\n')
            launchWrite.write('<node pkg="orca_planner" type="ROSAgent" name="ROSAgent" ns="robot_{0}" output="screen"/>\n'.format(x))
            launchWrite.write('<rosparam command="load" file="$(find collvoid_stage)/world/swarmlab_orca_lines.yaml" ns="/robot_{0}"/>\n'.format(x))

                
    s = ""
    for x in range(numRobots):
        s += "/robot_%d/debug "%(x)
    if useBagFile:
        launchWrite.write('<node pkg="rosbag" type="record" name="rosbag" args="record {0} /stall /stall_resolved /exceeded -O $(find collvoid_stage)/{1}" output="screen"/>\n'.format(s,bagFilename))
    

    launchWrite.write("</launch>\n")

    
    launchWrite.close()

    
if __name__ == "__main__":
    create_world_file(sys.argv[1:])

