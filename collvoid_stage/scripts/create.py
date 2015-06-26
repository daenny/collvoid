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
    localization = True
    simulation = True
    runExperiments = False
    scaleRadius = True
    useNoise = False
    useBagFile = False
    bagFileName = "collvoid.bag"
    useSticks = False
    dwa = False
    extraSampling = False
    try:
        opts, args= getopt.getopt(argv, "hn:s:oldtxf:S", ["help","numRobots=","circleSize=","omni","localization","dwa","extraSampling","experiments","bagFileName=","Sticks"])
    except getopt.GetoptError:
        print 'create.py -n <numRobots> -s <circleSize> <-h> <-l> <-d> <-x> <-f> bagFile'
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print 'create.py -n <numRobots> -s <circleSize> <-h> <-l> <-d> <-x> <-f> bagfile'
            sys.exit(2)
        elif opt in ("-n","--numRobots"):
            numRobots = int(arg)
        elif opt in ("-s","--circleSize"):
            circleSize = float(arg)
        elif opt in ("-o","--omni"):
            omni = True
        elif opt in ("-d","--dwa"):
            dwa = True
        elif opt in ("-t", "--extraSampling"):
            extraSampling = True
        elif opt in ("-l","--localization"):
            localization = False
        elif opt in ("-x", "--experiments"):
            runExperiments = True
        elif opt in ("-f", "--bagFileName"):
            useBagFile = True
            bagFileName = str(arg)
#        elif opt in ("-S", "--Sticks"):
#            useSticks = True
#            omni = True
    
    direct = commands.getoutput('rospack find collvoid_stage')
    worldFileTemp = open(direct + '/world/swarmlab_template.world','r')
    worldFileNew = open(direct + '/world/swarmlab_created.world','w')
    worldFileNew.write(worldFileTemp.read())
    direct = commands.getoutput('rospack find stage')

    colors = open(direct + '/rgb.txt','r')
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
        if (omni or useSticks):
            if(not useSticks):
                worldFileNew.write('pr2( pose [ {0:f} {1:f} 0 {2:f} ] name "robot_{3:d}" color "{4}")\n'.format(centerX+posX,centerY+posY,anglePrint,x,cols[40 +  10 * x]))
            else:
                worldFileNew.write('stick( pose [ {0:f} {1:f} 0 {2:f} ] name "robot_{3:d}" color "{4}")\n'.format(centerX+posX,centerY+posY,anglePrint,x,cols[40 +  10 * x]))
        else:
            worldFileNew.write('roomba( pose [ {0:f} {1:f} 0 {2:f} ] name "robot_{3:d}" color "{4}")\n'.format(centerX+posX,centerY+posY,anglePrint,x,cols[40 +  10 * x]))

    
    #tele_robot( pose [ -1 0 0 180.000 ] name "bob" color "blue")
    worldFileTemp.close()
    worldFileNew.close()
    create_yaml_file(circleSize,numRobots,omni,simulation,localization,centerX,centerY,scaleRadius,useNoise, useSticks)
    create_launch_file(numRobots,omni,runExperiments,bagFileName,localization,useBagFile, dwa, extraSampling, useSticks)
    
    
def create_yaml_file(circleSize, numRobots,omni,simulation,localization,centerX,centerY,scaleRadius,useNoise, useSticks):
    direct = commands.getoutput('rospack find collvoid_stage')
    yamlWrite = open(direct + '/params_created.yaml','w')
    yamlWrite.write("/use_sim_time: true\n")
    
    angle = 360.0 / numRobots
    for x in range(numRobots):
        angleX = x * angle - 45
        posX = circleSize*math.cos(angleX/360*2*math.pi)
        posY = circleSize*math.sin(angleX/360*2*math.pi)
  
        yamlWrite.write('robot_{0}:\n'.format(x))
        yamlWrite.write('    goals:\n')
        yamlWrite.write('        x: [{0:f}]\n'.format(centerX-posX))
        yamlWrite.write('        y: [{0:f}]\n'.format(centerY-posY))
        yamlWrite.write('        ang: [{0:f}]\n'.format((angleX) / 360.0 * 2 * math.pi))
    
    yamlWrite.close()
    
def create_launch_file(numRobots,omni,runExperiments, bagFilename, localization,useBagFile, dwa, extraSampling, useSticks):
    direct = commands.getoutput('rospack find collvoid_stage')

    launchWrite = open(direct + '/launch/sim_created.launch','w')
    launchWrite.write("<launch>\n")
    launchWrite.write('  <node name="map_server" pkg="map_server" type="map_server" args="$(find collvoid_stage)/world/swarmlab_map.yaml"/>\n')
    launchWrite.write('  <rosparam command="load" file="$(find collvoid_stage)/params_created.yaml"/>\n')
    if runExperiments:
        launchWrite.write('  <node pkg="stage_ros" type="stageros" name="stageros" args="-g $(find collvoid_stage)/world/swarmlab_created.world" respawn="false" output="screen" />\n')
    else:
        launchWrite.write('  <node pkg="stage_ros" type="stageros" name="stageros" args="$(find collvoid_stage)/world/swarmlab_created.world" respawn="false" output="screen" />\n')
        
    type_name = "turtle"
    if omni:
        if useSticks:
            type_name = "stick"
        else:
            type_name = "pr2"

    launchWrite.write('\n\n')
    for x in range(numRobots):
        if (localization):
            if (omni):
                launchWrite.write('  <include file="$(find collvoid_stage)/launch/amcl_omni_multi.launch">\n')
            else:
                launchWrite.write('  <include file="$(find collvoid_stage)/launch/amcl_diff_multi.launch">\n')
            launchWrite.write('    <arg name="robot" value="robot_{0}"/>\n'.format(x))
            launchWrite.write('  </include>\n')
        else:
            launchWrite.write('  <node name="fake_localization" pkg="fake_localization" ns="robot_{0}" type="fake_localization" respawn="false">\n'.format(x))
            launchWrite.write('    <param name="~tf_prefix" value="robot_{0}" />\n'.format(x))
            launchWrite.write('    <param name="~odom_frame_id" value="/robot_{0}/odom" />\n'.format(x))
            launchWrite.write('    <param name="~base_frame_id" value="/robot_{0}/base_link" />\n'.format(x))
            launchWrite.write('  </node>')

        if dwa:
            launchWrite.write('  <include file="$(find collvoid_stage)/launch/move_base_dwa.launch">\n')
        else:
            launchWrite.write('  <include file="$(find collvoid_stage)/launch/move_base_collvoid.launch">\n')
        launchWrite.write('    <arg name="robot" value="robot_{0}"/>\n'.format(x))
        launchWrite.write('    <arg name="type" value="{0}"/>\n'.format(type_name))
        launchWrite.write('    <arg name="controlled" value="true"/>\n')

        launchWrite.write('  </include>\n')
        launchWrite.write('  <node pkg="collvoid_controller" type="controllerRobots.py" name="controllerRobots" ns="robot_{0}" output="screen" />\n'.format(x))
        launchWrite.write('\n\n')


        
    if (runExperiments):
        launchWrite.write('  <node pkg="collvoid_controller" type="watchdog.py" name="watchdog" output="screen"/>\n')
    else:
        launchWrite.write('  <node pkg="collvoid_controller" type="controller.py" name="controller" output="screen"/>\n')
        launchWrite.write('  <node pkg="rviz" type="rviz" name="rviz" args="-d $(find collvoid_stage)/multi_view.rviz" output="screen" />\n')
   
    s = ""

    for x in range(numRobots):
        s += "/robot_%d/base_pose_ground_truth "%(x)
    if useBagFile:
        launchWrite.write('  <node pkg="rosbag" type="record" name="rosbag" args="record {0} /position_share /stall /stall_resolved /num_run /exceeded -O $(find collvoid_stage)/bags/{1}" output="screen"/>\n'.format(s,bagFilename))
    
    launchWrite.write("</launch>\n")
    launchWrite.close()

    
if __name__ == "__main__":
    create_world_file(sys.argv[1:])

