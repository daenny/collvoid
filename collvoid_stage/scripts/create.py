#!/usr/bin/env python

import math
import os
import sys
import getopt
import commands
import yaml


class CreateRunFiles(object):
    num_robots = 0
    circle_size = 0
    center_x = -2.2
    center_y = 2
    omni = False
    localization = True
    simulation = True
    run_experiments = False
    scale_radius = True
    use_noise = False
    use_bag_file = False
    bag_file_name = "collvoid.bag"
    use_sticks = False
    dwa = False
    real_dwa = False
    extra_sampling = True
    world_name = "swarmlab"
    settings = None
    visualize = False
    num_repetitions = 50

    def __init__(self, argv):
        try:
            opts, args = getopt.getopt(argv, "hn:s:oldDtxf:Sw:y:vr:",
                                       ["help", "numRobots=", "circleSize=", "omni", "localization", "dwa","real_dwa",
                                        "old_cocalu", "experiments", "bagFileName=", "Sticks", "world",
                                        "yaml_file=", "visualize", "num_repetitions"])
        except getopt.GetoptError:
            print 'create.py -n <numRobots> -s <circleSize> <-h> <-l> <-d> <-x> <-f> bagFile'
            sys.exit(2)
        for opt, arg in opts:
            if opt == '-h':
                print 'create.py -n <numRobots> -s <circleSize> <-h> <-l> <-d> <-x> <-f> bagfile'
                sys.exit(2)
            elif opt in ("-n", "--numRobots"):
                self.num_robots = int(arg)
            elif opt in ("-s", "--circleSize"):
                self.circle_size = float(arg)
            elif opt in ("-o", "--omni"):
                self.omni = True
            elif opt in ("-d", "--dwa"):
                self.dwa = True
            elif opt in ("-D", "--real_dwa"):
                self.real_dwa = True

            elif opt in ("-t", "--old_cocalu"):
                self.extra_sampling = False
            elif opt in ("-l", "--localization"):
                self.localization = False
            elif opt in ("-r", "--num_repetitions"):
                self.num_repetitions = int(arg)
            elif opt in ("-x", "--experiments"):
                self.run_experiments = True
            elif opt in ("-v", "--visualize"):
                self.visualize = True
            elif opt in ("-f", "--bagFileName"):
                self.use_bag_file = True
                self.bag_file_name = str(arg)
            elif opt in ("-y", "--yaml_file"):
                self.yamlfile = os.path.basename(arg)
                self.output_dir = commands.getoutput('rospack find collvoid_stage')
                try:
                    with open(os.path.join(self.output_dir, self.yamlfile)) as f:
                        self.settings = yaml.load(f)
                except Exception as e:
                    print("coudl not load yaml file from %s", os.path.join(self.output_dir, self.yamlfile))
                    sys.exit(2)
                self.num_robots = self.settings['num_robots']
                self.world_name = "5x5"
                self.center_x = 0
                self.center_y = 0

            elif opt in ("-w", "--world"):

                self.world_name = arg
                self.center_x = 0
                self.center_y = 0
                #        elif opt in ("-S", "--Sticks"):
                #            useSticks = True
                #            omni = True
        self.output_dir = commands.getoutput('rospack find collvoid_stage')

        self.create_world_file()
        self.create_launch_file()
        if self.settings is None:
            self.create_yaml_file()

    def create_world_file(self):
        with open(self.output_dir + '/world/' + self.world_name + '_created.world', 'w') as self.worldFileNew:
            with open(self.output_dir + '/world/' + self.world_name + '_template.world', 'r') as temp_world:
                self.worldFileNew.write(temp_world.read())

            direct = commands.getoutput('rospack find stage')

            colors = open(direct + '/rgb.txt', 'r')
            line = colors.readline()
            line = colors.readline()
            cols = []
            while line:
                cols.append(line[line.rfind("\t") + 1:line.rfind("\n")])
                line = colors.readline()
            colors.close()
            # print cols
            if self.settings is not None:
                for o in range(self.settings['num_obstacles']):
                    self.worldFileNew.write(
                        'obst( pose [ {0:f} {1:f} 0 {2:f} ] name "obst_{3:d}" color "red")\n'.format(
                            self.settings["obst_%d" % o]['x'], self.settings["obst_%d" % o]['y'],
                            math.degrees(self.settings["obst_%d" % o]['ang']), o))

            for x in range(self.num_robots):

                if self.settings is None:
                    angle = 360.0 / self.num_robots
                    anglePrint = x * angle - 180 - 45
                    angle = x * angle - 45
                    posX = self.circle_size * math.cos(angle / 360 * 2 * math.pi)

                    posY = self.circle_size * math.sin(angle / 360 * 2 * math.pi)
                else:
                    posX = self.settings["robot_%d" % x]['init_pose']['x']
                    posY = self.settings["robot_%d" % x]['init_pose']['y']
                    anglePrint = math.degrees(self.settings["robot_%d" % x]['init_pose']['ang'])

                if self.omni or self.use_sticks:
                    if not self.use_sticks:
                        self.worldFileNew.write(
                            'pr2( pose [ {0:f} {1:f} 0 {2:f} ] name "robot_{3:d}" color "{4}")\n'.format(
                                self.center_x + posX,
                                self.center_y + posY,
                                anglePrint, x,
                                cols[
                                    40 + 10 * x]))
                    else:
                        self.worldFileNew.write(
                            'stick( pose [ {0:f} {1:f} 0 {2:f} ] name "robot_{3:d}" color "{4}")\n'.format(
                                self.center_x + posX, self.center_y + posY, anglePrint, x, cols[40 + 10 * x]))
                else:
                    self.worldFileNew.write(
                        'roomba( pose [ {0:f} {1:f} 0 {2:f} ] name "robot_{3:d}" color "{4}")\n'.format(
                            self.center_x + posX,
                            self.center_y + posY,
                            anglePrint, x,
                            cols[
                                40 + 10 * x]))

    def create_yaml_file(self):
        with open(self.output_dir + '/params_created.yaml', 'w') as yaml_file:
            angle = 360.0 / self.num_robots
            for x in range(self.num_robots):
                angle_x = x * angle - 45
                pos_x = self.circle_size * math.cos(angle_x / 360 * 2 * math.pi)
                pos_y = self.circle_size * math.sin(angle_x / 360 * 2 * math.pi)

                yaml_file.write('robot_{0}:\n'.format(x))
                yaml_file.write('    goals:\n')
                yaml_file.write("        - x: {0:f}\n".format(self.center_x - pos_x))
                yaml_file.write('          y: {0:f}\n'.format(self.center_y - pos_y))
                yaml_file.write("          ang: {0:f}\n".format(angle_x / 360.0 * 2 * math.pi))

    def create_launch_file(self):
        with open(self.output_dir + '/launch/sim_created.launch', 'w') as f_launch:
            f_launch.write("<launch>\n")
            f_launch.write(
                '  <node name="map_server" pkg="map_server" type="map_server" args="$(find collvoid_stage)/world/' + self.world_name + '_map.yaml"/>\n')
            f_launch.write('  <rosparam command="load" file="$(find collvoid_stage)/params/stage_params.yaml"/>\n')

            if self.settings is not None:
                f_launch.write('  <rosparam command="load" file="$(find collvoid_stage)/' +self.yamlfile + '"/>\n')
            else:
                f_launch.write('  <rosparam command="load" file="$(find collvoid_stage)/params_created.yaml"/>\n')

            if self.run_experiments and not self.visualize:
                f_launch.write(
                    '  <node pkg="stage_ros" type="stageros" name="stageros" args="-g $(find collvoid_stage)/world/' + self.world_name + '_created.world" respawn="false" output="screen" />\n')
            else:
                f_launch.write(
                    '  <node pkg="stage_ros" type="stageros" name="stageros" args="$(find collvoid_stage)/world/' + self.world_name + '_created.world" respawn="false" output="screen" />\n')

            type_name = "turtle"
            if self.omni:
                if self.use_sticks:
                    type_name = "stick"
                else:
                    type_name = "pr2"

            f_launch.write('\n\n')
            for x in range(self.num_robots):
                if self.localization:
                    if self.omni:
                        f_launch.write('  <include file="$(find collvoid_stage)/launch/amcl_omni_multi.launch">\n')
                    else:
                        f_launch.write('  <include file="$(find collvoid_stage)/launch/amcl_diff_multi.launch">\n')
                    f_launch.write('    <arg name="robot" value="robot_{0}"/>\n'.format(x))
                    f_launch.write('  </include>\n')
                else:
                    f_launch.write(
                        '  <node name="fake_localization" pkg="fake_localization" ns="robot_{0}" type="fake_localization" respawn="false">\n'.format(
                            x))
                    f_launch.write('    <param name="~tf_prefix" value="robot_{0}" />\n'.format(x))
                    f_launch.write('    <param name="~odom_frame_id" value="/robot_{0}/odom" />\n'.format(x))
                    f_launch.write('    <param name="~base_frame_id" value="/robot_{0}/base_link" />\n'.format(x))
                    f_launch.write('  </node>\n')

                if self.dwa:
                    f_launch.write('  <include file="$(find collvoid_stage)/launch/move_base_dwa.launch">\n')
                elif self.real_dwa:
                    f_launch.write('  <include file="$(find collvoid_stage)/launch/move_base_dwa_simple.launch">\n')
                elif self.extra_sampling:
                    f_launch.write('  <include file="$(find collvoid_stage)/launch/move_base_collvoid.launch">\n')
                else:
                    f_launch.write('  <include file="$(find collvoid_stage)/launch/move_base_collvoid_legacy.launch">\n')
                f_launch.write('    <arg name="robot" value="robot_{0}"/>\n'.format(x))
                f_launch.write('    <arg name="type" value="{0}"/>\n'.format(type_name))
                f_launch.write('    <arg name="controlled" value="true"/>\n')

                f_launch.write('  </include>\n')
                f_launch.write(
                    '  <node pkg="collvoid_controller" type="controller_robots.py" name="controller_robots" ns="robot_{0}"/>\n'.format(
                        x))
                f_launch.write('\n\n')

            if self.run_experiments:
                f_launch.write(
                    '  <node pkg="collvoid_controller" type="watchdog.py" name="Watchdog" output="screen">\n')
                f_launch.write('  <param name="num_repetitions" value="' + str(self.num_repetitions) + '"/>\n')
                f_launch.write('  </node>\n')
                if self.visualize:
                    f_launch.write(
                        '  <node pkg="collvoid_controller" type="collvoid_visualizer.py" name="controller_viz" output="screen"/>\n')
                    f_launch.write(
                        '  <node pkg="rviz" type="rviz" name="rviz" args="-d $(find collvoid_stage)/multi_view.rviz" output="screen" />\n')
            else:
                f_launch.write(
                    '  <node pkg="collvoid_controller" type="controller.py" name="controller" output="screen"/>\n')
                f_launch.write(
                    '  <node pkg="collvoid_controller" type="collvoid_visualizer.py" name="controller_viz" output="screen"/>\n')
                f_launch.write(
                    '  <node pkg="rviz" type="rviz" name="rviz" args="-d $(find collvoid_stage)/multi_view.rviz" output="screen" />\n')

            s = ""

            for x in range(self.num_robots):
                s += "/robot_%d/base_pose_ground_truth " % (x)
            if self.use_bag_file:
                f_launch.write(
                    '  <node pkg="rosbag" type="record" name="rosbag" args="record {0} /position_share /stall /stall_resolved /obstacles /goals /num_run /exceeded -O $(find collvoid_stage)/bags/{1}" output="screen"/>\n'.format(
                        s, self.bag_file_name))

            f_launch.write("</launch>\n")


if __name__ == "__main__":
    CreateRunFiles(sys.argv[1:])
