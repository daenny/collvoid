#!/usr/bin/env python

import os
import sys
from subprocess import *
import time

if __name__ == '__main__':
    SHUTDOWN_TIME = 10
    circ_size = 1.7  # constant

    for num_robots in [5, 4, 3, 2]:
        for cocalu in [True]:
            for extra_sampling in [True]:
                bag_fname = "collvoid_" + str(num_robots) + "_cocalu_" + str(cocalu) + "_extrasampling_" + str(
                    extra_sampling) + ".bag"

                #               print "#robots: %d\t radiuslocalization: %s\t bag-file-name: %s"%(num_robots, str(localization), bag_fname)

                cmd = "../create.py"
                args = " -n " + str(num_robots) + " -s " + str(circ_size) + " -f " + str(bag_fname)
                args += " -x"  # add -x for experiments

                if extra_sampling:
                    args += " -t"

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
