#!/usr/bin/env python

import os
import sys
from subprocess import *
import time

if __name__ == '__main__':

#    WAIT_FOR_INIT = 10
#    MAX_TIME = 40
#    RUNS = 2 //set Runs in Watchdog.py in controller
    SHUTDOWN_TIME = 10
#    MAX_EXPERIMENT_TIME = (WAIT_FOR_INIT + MAX_TIME + 5) * RUNS

 #   print "sleep time is: %d"%MAX_EXPERIMENT_TIME

    circ_size = 1.8 # constant

    for num_robots in [8, 7, 6, 5, 4, 3, 2]:
       for radius_scaling in [True, False]:
           for add_noise in [False]:
            
               bag_fname = "collvoid_" + str(num_robots) + "_True_" + "rad_scale_" + str(radius_scaling) + "noise_" + str(add_noise) +".bag"
               
#               print "#robots: %d\t radiuslocalization: %s\t bag-file-name: %s"%(num_robots, str(localization), bag_fname)
               
               cmd = "./CreateLaunchAndWorld.py"
               args = " -n " + str(num_robots) + " -s " + str(circ_size) + " -f " + str(bag_fname)
               
               if radius_scaling:
                   args += " -R"

               args += " -x" #add -R for radiusscaling

               if add_noise:
                   args += " -N"


               print 'calling "%s" ...'%(cmd)
               retcode = call(cmd + args, shell=True)
               if retcode < 0:
                   print "child process was terminated by signal", -retcode
                   sys.exit(-1)
               else:
                   print "child process returned", retcode

                   cmd = "roslaunch" # collvoid_sim control.launch"
                   args = "launch/control.launch"
                   print 'calling "%s" ...'%(cmd)
                   popen = Popen([cmd, args],universal_newlines=True,stdout=None,stderr=PIPE)
                   done = False
                   while not done: 
                       output = popen.stderr.readline()[:-1]
                       print output
                       
                       if "I AM DONE" in output:
                           done = True;
             
                   popen.terminate()
                   print "-" * 80
            
                   time.sleep(SHUTDOWN_TIME)


    
    
    


