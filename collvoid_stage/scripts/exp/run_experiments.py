#!/usr/bin/env python
import commands
import os
import random
import sys
from psutil import TimeoutExpired
from subprocess import *
import time

import datetime

try:
    from subprocess import DEVNULL  # py3k
except ImportError:
    DEVNULL = open(os.devnull, 'wb')
import psutil


settings = {"cocalu": "--old_cocalu",
            "cocalu_sampling": "",
            "cocalu_dwa": "--dwa",
            "dwa": "--real_dwa"
            }

rand = random.Random()
rand.seed(0)

circle = False
verbose = True
visualize = False

NUM_OBSTACLES = [10]
NUM_ROBOTS = [6, 5, 4, 3, 2]

#NUM_ROBOTS = [3, 2]

if not circle:
    NUM_RUNS = 10  # for random
    NUM_REPETITIONS = 5
else:
    NUM_RUNS = 1  # all via reset
    NUM_REPETITIONS = 50

SETTINGS = ["dwa"]

YAML_TEMPLATE = 'goals_robots_R_obstacles_O_S_created.yaml'


def kill(proc_pid):
    process = psutil.Process(proc_pid)
    for proc in process.get_children(recursive=True):
        proc.kill()
    process.kill()


def start_environment(output_screen=True):
    print("deleting all params")
    cmd = ["rosparam", "delete", "/"]
    proc = Popen(cmd)
    proc.wait()

    cmd = ["roslaunch", "collvoid_stage", "sim_created.launch"]
    print('calling "%s" ...' % cmd)
    if output_screen:
        ret_process = Popen(cmd, stdout=None, stderr=PIPE)
    else:
        ret_process = Popen(cmd, stdout=DEVNULL, stderr=PIPE)
    time.sleep(2.)
    return ret_process


if __name__ == '__main__':
    SHUTDOWN_TIME = 15
    circ_size = 1.7  # constant
    work_dir = commands.getoutput('rospack find collvoid_stage')
    work_dir = os.path.join(work_dir, 'scripts')
    for num_run in range(NUM_RUNS):
        for num_robots in NUM_ROBOTS:
            for num_obstacles in NUM_OBSTACLES:
                seed = rand.randint(0, sys.maxint)
                for setting in SETTINGS:
                    create_cmd = os.path.join(work_dir, "create.py")
                    args = ""
                    if circle:
                        bag_fname = "_".join(["circle", "robots", str(num_robots), setting, ".bag"])
                        args = " -n " + str(num_robots) + " -s " + str(circ_size)
                    else:
                        bag_fname = "_".join(["random", "robots", str(num_robots), "obstacles", str(num_obstacles),
                                              "seed", str(seed), setting, ".bag"])
                        yaml_file_name = YAML_TEMPLATE.replace("R", str(num_robots)).replace("O", str(num_obstacles)).replace("S", str(seed))
                        goal_args = ["-n", str(num_robots), "-o", str(num_obstacles), "-s", str(seed)]
                        goal_cmd = [os.path.join(work_dir, "create_random_goals.py")]
                        goal_cmd.extend(goal_args)
                        print 'calling "%s" ...' % goal_cmd
                        retcode = call(goal_cmd)
                        if not retcode == 0:
                            print("child process was terminated by signal", retcode)
                            sys.exit(-1)
                        args = " -y " + yaml_file_name

                    args = " ".join([args, "-x", "-f", bag_fname, "-r", str(NUM_REPETITIONS)])  # add -x for experiments
                    if visualize:
                        args = " ".join([args, "-v"])  # add visualization flag
                    args = " ".join([args, settings[setting]])

                    print 'calling "%s" ...' % ([create_cmd, args])
                    retcode = call(create_cmd + args, shell=True)
                    if not retcode == 0:
                        print "child process was terminated by signal", -retcode
                        sys.exit(-1)

                    print "child process returned", retcode
                    popen = start_environment(verbose)

                    done = False
                    while not done:
                        output = popen.stderr.readline()[:-1]
                        print output

                        if "I AM DONE" in output:
                            done = True

                    popen.terminate()
                    print "-" * 80

                    start = datetime.datetime.now()
                    while popen.poll() is None:
                        print "still running"
                        time.sleep(0.1)
                        now = datetime.datetime.now()
                        if (now - start).seconds > SHUTDOWN_TIME:
                            try:
                                print "hard killing process"
                                kill(popen.pid)
                                os.waitpid(-1, os.WNOHANG)
                                time.sleep(2)
                                break
                            except:
                                break
                    print "killed process"
                    time.sleep(1)
