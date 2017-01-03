#!/usr/bin/env python

import sys
from subprocess import *
import time

settings = {"cocalu": "",
            "cocalu_sampling": "-t",
            "cocalu_dwa": "-d"}

if __name__ == '__main__':
    SHUTDOWN_TIME = 10
    circ_size = 1.7  # constant

    for num_robots in [5, 4, 3, 2]:
        for setting in ["cocalu", "cocalu_sampling", "cocalu_dwa"]:
            bag_fname = "_".join(["collvoid", str(num_robots), setting, ".bag"])

            cmd = "../create.py"
            args = " -n " + str(num_robots) + " -s " + str(circ_size) + " -f " + str(bag_fname)
            args += " -x"  # add -x for experiments

            args += settings[setting]

            print 'calling "%s" ...' % (cmd)
            retcode = call(cmd + args, shell=True)
            if retcode < 0:
                print "child process was terminated by signal", -retcode
                sys.exit(-1)
            else:
                print "child process returned", retcode

                cmd = "roslaunch"  # collvoid_sim control.launch"
                args = "../../launch/sim_created.launch"
                print 'calling "%s" ...' % (cmd)
                popen = Popen([cmd, args], universal_newlines=True, stdout=None, stderr=PIPE)
                done = False
                while not done:
                    output = popen.stderr.readline()[:-1]
                    print output

                    if "I AM DONE" in output:
                        done = True

                popen.terminate()
                print "-" * 80

                time.sleep(SHUTDOWN_TIME)
