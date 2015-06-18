#!/usr/bin/env python
import roslib
roslib.load_manifest('collvoid_sim')
import rosbag
import sys
import math
import numpy
import tf

def dist(a, b):
    return math.sqrt(math.pow(a[0] - b[0], 2) + math.pow(a[1] - b[1], 2))

def dist_to_center(a):
    return dist(a, (-2.2, 2.0))

def bounding_box(pos):
    if pos[0] < -4.5 or pos[0] > -0.0 or pos[1] < -0.5 or pos[1] > 5.5 :
        return True # in the box
    else:
        return False # out of the box

def twist_to_uv((x,pose)):
    q = []
    q.append(pose.orientation.x)
    q.append(pose.orientation.y)
    q.append(pose.orientation.z)
    q.append(pose.orientation.w)
    alpha = tf.transformations.euler_from_quaternion(q)[2]
#    alpha += math.pi / 2
    return (math.cos(alpha) * abs(x), math.sin(alpha) * abs(x))

if __name__== '__main__':
    if not (len(sys.argv) == 2):
        print "usage: evaluate.py <filename>"
        sys.exit(-1)


    fname = argv = sys.argv[1]

    print "reading %s .."%(fname)
    bag = rosbag.Bag(fname)
    run = {}
    for topic, msg,t in bag.read_messages():
        robot_name = msg.id
        if not robot_name in run:
            print "first time i have seen %s in"%(robot_name)
            run[robot_name] = {}
            robot = run[robot_name]
            robot['start_time'] = msg.header.stamp
            robot['last_time'] = robot['start_time']
            robot['distance'] = 0
            robot['last_pos_ground_truth'] = (msg.pose.pose.position.x, 
                                          msg.pose.pose.position.y)

            robot['pos_ground_truth'] = [robot['last_pos_ground_truth']]
            robot['twist_ground_truth'] = [twist_to_uv((msg.twist.twist.linear.x, msg.pose.pose))]
            robot['time'] = [0]
            robot['vel_lin'] = [0]
            robot['vel_ang'] = [0]
            robot['dt'] = [0]
            robot['temp_distance'] = 0
            robot['temp_speed_lin'] = 0
            robot['temp_last_time'] = robot['start_time']


        else:
            robot = run[robot_name]
        
            # update last position and distance
            pos =  (msg.pose.pose.position.x, 
                msg.pose.pose.position.y)
            robot['pos_ground_truth'].append(pos)
            robot['twist_ground_truth'].append(twist_to_uv((msg.twist.twist.linear.x, msg.pose.pose)))
            robot['temp_distance'] += dist(robot['last_pos_ground_truth'], pos)
            robot['distance'] += dist(robot['last_pos_ground_truth'], pos)
            robot['last_pos_ground_truth'] = pos
        
        
            dt = (msg.header.stamp - robot['temp_last_time']).to_sec()
            if dt > 0.0:
                robot['dt'].append(dt)
                robot['vel_lin'].append(dist((msg.twist.twist.linear.x,msg.twist.twist.linear.y),(0.0,0.0)))
                #robot['vel_lin'].append(robot['temp_distance'] / dt)
                robot['temp_distance'] = 0
                robot['temp_last_time'] = msg.header.stamp
                robot['vel_ang'].append(msg.twist.twist.angular.z)
           





    # update timer
    robot['last_time'] = msg.header.stamp


    
    print " done!"
#    print "parsed %d msgs"%count
    #print runs
#    print "SKIPPED" ,skipped
    print "-" * 30

    num_robots = max(map(lambda x: len(x), run))
    print "found %d robots"%num_robots


    run_count = 0
    collision_count = 0
    deadlock_count = 0
    out_of_box_count = 0
    not_started_count = 0
    small_jerk_count = 0
    coll_runs = 0
    coll_resolved_runs = 0
    exceeded_count = 0
    jerk_lin_array = []
    jerk_ang_array = []
    time_max_array = []
    distance_array = []
    loc_err_array = []

    COLLISION_TIME = 60

    # run loop
    # print "run %d (%d robots):"%(run, num_robots)

        # generating trajectories for matlab
        
    POS_X = "X = ["
    POS_Y = "Y = ["
    POS_U = "U = ["
    POS_V = "V = ["
    
    max_length = 0
    for robot_name in run:
        robot = run[robot_name]
        max_length = max(max_length, len(robot['pos_ground_truth']))

    for robot_name in run:
        robot = run[robot_name]
            # unpack x and y
        pos_x = map(lambda x: x[0], robot['pos_ground_truth'])
        while len(pos_x) < max_length:
            pos_x.append(pos_x[-1])

        pos_y = map(lambda x: x[1], robot['pos_ground_truth'])
        while len(pos_y) < max_length:
            pos_y.append(pos_y[-1])
            
        pos_u = map(lambda x: x[0], robot['twist_ground_truth'])
        while len(pos_u) < max_length:
            pos_u.append(0)

        pos_v = map(lambda x: x[1], robot['twist_ground_truth'])
        while len(pos_v) < max_length:
            pos_v.append(0)

            
        for x in pos_x:
            POS_X += str(x) + ", "
        POS_X = POS_X[0:-2] # delete last ,
        POS_X += ";\n"

        for y in pos_y:
            POS_Y += str(y) + ", "
        POS_Y = POS_Y[0:-2] # delete last ,
        POS_Y += ";\n"

        for u in pos_u:
            POS_U += str(u) + ", "
        POS_U = POS_U[0:-2] # delete last ,
        POS_U += ";\n"

        for v in pos_v:
            POS_V += str(v) + ", "
        POS_V = POS_V[0:-2] # delete last ,
        POS_V += ";\n"


    POS_X = POS_X[0:-2] # delete last ; and \n
    POS_X += "];\n"
    
    POS_Y = POS_Y[0:-2] # delete last ; and \n
    POS_Y += "];\n"
    
    POS_U = POS_U[0:-2] # delete last ; and \n
    POS_U += "];\n"
    
    POS_V = POS_V[0:-2] # delete last ; and \n
    POS_V += "];\n"

    # saving trajectories to file
    matlab_fname = "./runs/%s_run%d.m"%(fname[0:-4],1)
    print "saving trajectories to %s ..."%matlab_fname
    f = open(matlab_fname, 'w')
    f.write(POS_X)
    f.write(POS_Y)
    f.write(POS_U)
    f.write(POS_V)
    f.write("plot(-X', Y')\n")
    f.write("%quiver(-X(1,:), Y(1,:), -U(1,:), V(1,:))\n")
    f.close()

