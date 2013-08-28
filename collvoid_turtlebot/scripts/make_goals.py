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
    
def create_launch_file(numRobots,omni,runExperiments, bagFilename, localization,useBagFile, useSticks):
    direct = commands.getoutput('rospack find collvoid_stage')

    launchWrite = open(direct + '/launch/sim_created.launch','w')
    launchWrite.write("<launch>\n")
    launchWrite.write('  <node name="map_server" pkg="map_server" type="map_server" args="$(find collvoid_stage)/world/swarmlab_map.yaml"/>\n')
    launchWrite.write('  <rosparam command="load" file="$(find collvoid_stage)/params_created.yaml"/>\n')
    if runExperiments:
        launchWrite.write('  <node pkg="stage" type="stageros" name="stageros" args="-g $(find collvoid_stage)/world/swarmlab_created.world" respawn="false" output="screen" />\n')
    else:
        launchWrite.write('  <node pkg="stage" type="stageros" name="stageros" args="$(find collvoid_stage)/world/swarmlab_created.world" respawn="false" output="screen" />\n')

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


            
        launchWrite.write('  <node pkg="move_base" type="move_base" respawn="true" name="move_base" ns="robot_{0}" output="screen">\n'.format(x))
        if (omni) and not useSticks:
            launchWrite.write('    <rosparam command="load" file="$(find collvoid_stage)/params/params_pr2.yaml" />\n')
        elif useSticks:
            launchWrite.write('    <rosparam command="load" file="$(find collvoid_stage)/params/params_stick.yaml" />\n')
        else:
            launchWrite.write('    <rosparam command="load" file="$(find collvoid_stage)/params/params_turtle.yaml" />\n')

        if(runExperiments):
            launchWrite.write('    <rosparam command="load" file="$(find collvoid_stage)/params/collvoid_exp_created.yaml" />\n')
        else:
            launchWrite.write('    <rosparam command="load" file="$(find collvoid_stage)/params/collvoid_config.yaml" />\n')

        launchWrite.write('    <remap from="map" to="/map" />\n')
        launchWrite.write('    <param name="~tf_prefix" value="robot_{0}" />\n'.format(x))
        launchWrite.write('    <param name="~/global_costmap/robot_base_frame" value="robot_{0}/base_link" /> \n    <param name="~/local_costmap/robot_base_frame" value="robot_{1}/base_link" /> \n    <param name="~/local_costmap/global_frame" value="robot_{0}/odom" /> \n'.format(x,x,x))
        launchWrite.write('    <param name="base_local_planner" value="collvoid_local_planner/CollvoidLocalPlanner" />\n')
        launchWrite.write('    <param name="base_global_planner" value="collvoid_simple_global_planner/CollvoidSimpleGlobalPlanner" />\n')
        launchWrite.write('    <remap from="/position_share_in" to="/position_share" />\n')
        launchWrite.write('    <remap from="/position_share_out" to="/position_share" />\n')

       # launchWrite.write('    <rosparam file="$(find pr2_navigation_config)/move_base/dwa_local_planner.yaml" command="load" ns="DWAPlannerROS" />\n')
       
        launchWrite.write('  </node> \n')
        launchWrite.write('  <node pkg="collvoid_controller" type="controllerRobots.py" name="controllerRobots" ns="robot_{0}" output="screen" />\n'.format(x))
    if (runExperiments):
        launchWrite.write('  <node pkg="collvoid_controller" type="watchdog.py" name="watchdog" output="screen"/>\n')
    else:
        launchWrite.write('  <node pkg="collvoid_controller" type="controller.py" name="controller" output="screen"/>\n')
        launchWrite.write('  <node pkg="rviz" type="rviz" name="rviz" args="-d $(find collvoid_stage)/double_view.vcg" output="screen" />\n')
   
    s = ""

    for x in range(numRobots):
        s += "/robot_%d/base_pose_ground_truth "%(x)
    if useBagFile:
        launchWrite.write('  <node pkg="rosbag" type="record" name="rosbag" args="record {0} /position_share /stall /stall_resolved /num_run /exceeded -O $(find collvoid_stage)/bags/{1}" output="screen"/>\n'.format(s,bagFilename))
    
    launchWrite.write("</launch>\n")
    launchWrite.close()

    
if __name__ == "__main__":
    create_world_file(sys.argv[1:])

