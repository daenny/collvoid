#!/usr/bin/env python
import glob
import numpy as np
import os
import rosbag
import sys
import math
try:
    import cPickle as pickle
except ImportError:
    import pickle
import rospy
import tf.transformations


def dist(a, b):
    return math.sqrt(math.pow(a[0] - b[0], 2) + math.pow(a[1] - b[1], 2))


def get_yaw(pose):
    q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    alpha = tf.transformations.euler_from_quaternion(q)[2]
    return alpha


def twist_to_uv((x, pose)):
    q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    alpha = tf.transformations.euler_from_quaternion(q)[2]
    #    alpha += math.pi / 2
    return math.cos(alpha) * abs(x), math.sin(alpha) * abs(x)


def evaluate_dir(dirname):
    bag_files = sorted(glob.glob(os.path.join(dirname, "*.bag")))
    res_time = []
    res_dist = []
    res_jerk_ang = []
    res_jerk_lin = []
    for bag in bag_files:
        t, d, lin, ang = evalutate_bagfile(bag)
        res_time.append(t)
        res_dist.append(d)
        res_jerk_lin.append(lin)
        res_jerk_ang.append(ang)

    fname = "res_" + "_".join(dirname.strip('.').strip('/').split('/')[-2:])
    print fname + "_time = ...\n[" + "\n".join([str(x) for x in res_time]) + "];"
    print fname + "_dist = ...\n[" + "\n".join([str(x) for x in res_dist]) + "];"
    print fname + "_jerk_lin = ...\n[" + "\n".join([str(x) for x in res_jerk_lin]) + "];"
    print fname + "_jerk_ang = ...\n[" + "\n".join([str(x) for x in res_jerk_ang]) + "];"


def evalutate_bagfile(bagfile):
    bag = rosbag.Bag(bagfile)

    bag_name = os.path.basename(bagfile)
    work_dir = os.path.dirname(bagfile)
    suffix = ''
    if stop:
        suffix = str(stop_time)

    if os.path.exists(bagfile[:-4] + suffix + '.p') and not force_new:
        print("Pickle file exists, reloading from saved path")
        with open(bagfile[:-4] + '.p', 'r') as f:
            data = pickle.load(f)
            run = data['run']
            goals = data['goals']
            obst = data['obst']
    else:
        count = 0
        obst = []
        goals = []
        sys.stdout.write("evaluating run ...")
        start_time = rospy.Time(0)
        skipped = 0
        run = {}
        started = False
        # read all messages
        for topic, msg, t in bag.read_messages(topics=['/position_share']):
            if topic == "/obstacles":
                for p in msg.poses:
                    obstacle = [p.position.x, p.position.y, math.degrees(get_yaw(p))]
                    obst.append(obstacle)
            elif topic == "/goals":
                for p in msg.poses:
                    obstacle = [p.position.x, p.position.y, math.degrees(get_yaw(p))]
                    goals.append(obstacle)

            elif "position_share" in topic:
                robot_name = msg.robot_id
                #    continue
                # which run


                # run = msg.run
                # if topic == "/position_share":
                #   robot_name = msg.robot_id
                count += 1

                # create new robot
                if robot_name not in run:
                    # print "first time i have seen %s in run %d"%(robot_name, run)
                    run[robot_name] = {}
                    robot = run[robot_name]
                    robot['start_time'] = msg.header.stamp
                    robot['last_time'] = robot['start_time']
                    robot['distance'] = 0
                    robot['last_pos_ground_truth'] = (msg.pose.pose.position.x,
                                                      msg.pose.pose.position.y)

                    alpha = get_yaw(msg.pose.pose)

                    robot['ang'] = [math.degrees(alpha)]
                    robot['pos_ground_truth'] = [robot['last_pos_ground_truth']]
                    robot['twist_ground_truth'] = [twist_to_uv((msg.twist.twist.linear.x, msg.pose.pose))]

                    # robot['loc_error'] = [msg.loc_error]
                    # robot['last_pos_ground_truth'] = (msg.ground_truth.pose.pose.position.x,
                    # msg.ground_truth.pose.pose.position.y)

                    # robot['pos_ground_truth'] = [robot['last_pos_ground_truth']]
                    # robot['twist_ground_truth'] = [twist_to_uv((msg.ground_truth.twist.twist.linear.x, msg.ground_truth.pose.pose))]
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
                    pos = (msg.pose.pose.position.x,
                           msg.pose.pose.position.y)

                    if not started:
                        if dist(robot['last_pos_ground_truth'], pos) < 0.02:
                            skipped += 1
                            start_time = msg.header.stamp
                            continue
                        else:
                            started = True
                            print "moved for more than 2 cm, assuming start"
                            print "skipped initial", skipped
                            for r in run:
                                run[r]['start_time'] = start_time
                                run[r]['temp_last_time'] = start_time

                    #if dist(robot['last_pos_ground_truth'], pos) < 0.005:
                    #    skipped += 1
                    #    continue

                    if stop:
                        #print (msg.header.stamp - start_time).to_sec(), int(stop_time)
                        if (msg.header.stamp - start_time).to_sec() > int(stop_time):
                            skipped += 1
                            #print "STOP"
                            continue

                    alpha = get_yaw(msg.pose.pose)

                    robot['ang'].append(math.degrees(alpha))

                    robot['pos_ground_truth'].append(pos)
                    robot['twist_ground_truth'].append(twist_to_uv((msg.twist.twist.linear.x, msg.pose.pose)))
                    robot['temp_distance'] += dist(robot['last_pos_ground_truth'], pos)
                    # pos =  (msg.ground_truth.pose.pose.position.x,
                    # msg.ground_truth.pose.pose.position.y)
                    # robot['pos_ground_truth'].append(pos)
                    # robot['twist_ground_truth'].append(twist_to_uv((msg.ground_truth.twist.twist.linear.x, msg.ground_truth.pose.pose)))
                    # robot['temp_distance'] += dist(robot['last_pos_ground_truth'], pos)
                    robot['distance'] += dist(robot['last_pos_ground_truth'], pos)
                    robot['last_pos_ground_truth'] = pos

                    # robot['loc_error'].append(msg.loc_error)

                    # dt = (msg.ground_truth.header.stamp - robot['temp_last_time']).to_sec()
                    dt = max((msg.header.stamp - robot['temp_last_time']).to_sec(), 0.11)
                    if dt > 0.0:
                        robot['dt'].append(dt)
                        # robot['vel_lin'].append(dist((msg.ground_truth.twist.twist.linear.x,msg.ground_truth.twist.twist.linear.y),(0.0,0.0)))
                        # robot['vel_lin'].append(robot['temp_distance'] / dt)
                        robot['vel_lin'].append(dist((msg.twist.twist.linear.x, msg.twist.twist.linear.y), (0.0, 0.0)))
                        # robot['vel_lin'].append(robot['temp_distance'] / dt)
                        robot['temp_distance'] = 0
                        robot['temp_last_time'] = msg.header.stamp
                        robot['vel_ang'].append(msg.twist.twist.angular.z)
                        # robot['vel_ang'].append(msg.ground_truth.twist.twist.angular.z)
                    else:
                        skipped += 1

                    # update timer
                    robot['last_time'] = msg.header.stamp
        print " done!"
        print "parsed %d msgs" % count
        # print runs
        print "SKIPPED", skipped
        print "-" * 30

        print("saving pickle")
        suffix = ''
        if stop:
            suffix = str(stop_time)

        with open(bagfile[:-4] + suffix + '.p', 'w') as f:
            data = {'run': run,
                    'obst': obst,
                    'goals': goals}
            pickle.dump(data, f)
        print "-" * 30

    num_robots = len(run)
    print "found %d robots" % num_robots
    print "finding end"

    max_length = 0
    for robot_name in run:
        robot = run[robot_name]
        max_length = max(max_length, len(robot['pos_ground_truth']))

    finish_time = max_length

    while finish_time > 0:
        found_nonzero_speed = False
        for robot_name in run:
            robot = run[robot_name]
            if len(robot['pos_ground_truth']) >= finish_time:
                if robot['vel_ang'][finish_time-1] > 0.01 or robot['vel_lin'][finish_time-1] > 0.01:
                    found_nonzero_speed = True
        if found_nonzero_speed:
            break
        else:
            finish_time -= 1

    print max_length, finish_time

    # generating trajectories for matlab
    if save_traj:
        POS_X = "X = ["
        POS_Y = "Y = ["
        POS_U = "U = ["
        POS_V = "V = ["
        ANG = "ang = ["

        GOALS = 'goals = ['

        for robot_name in sorted(run.keys()):
            robot = run[robot_name]
            # unpack x and y
            ang = robot['ang']
            while len(ang) < finish_time:
                ang.append(ang[-1])

            # unpack x and y
            pos_x = map(lambda x: x[0], robot['pos_ground_truth'])
            while len(pos_x) < finish_time:
                pos_x.append(pos_x[-1])

            pos_y = map(lambda x: x[1], robot['pos_ground_truth'])
            while len(pos_y) < finish_time:
                pos_y.append(pos_y[-1])

            pos_u = map(lambda x: x[0], robot['twist_ground_truth'])
            while len(pos_u) < finish_time:
                pos_u.append(0)

            pos_v = map(lambda x: x[1], robot['twist_ground_truth'])
            while len(pos_v) < finish_time:
                pos_v.append(0)

            for x in ang[:finish_time]:
                ANG += str(x) + ", "
            ANG = ANG[0:-2]  # delete last ,
            ANG += ";\n"

            for x in pos_x[:finish_time]:
                POS_X += str(x) + ", "
            POS_X = POS_X[0:-2]  # delete last ,
            POS_X += ";\n"

            for y in pos_y[:finish_time]:
                POS_Y += str(y) + ", "
            POS_Y = POS_Y[0:-2]  # delete last ,
            POS_Y += ";\n"

            for u in pos_u[:finish_time]:
                POS_U += str(u) + ", "
            POS_U = POS_U[0:-2]  # delete last ,
            POS_U += ";\n"

            for v in pos_v[:finish_time]:
                POS_V += str(v) + ", "
            POS_V = POS_V[0:-2]  # delete last ,
            POS_V += ";\n"

            ## find goal
            min_dist = sys.maxint
            goal = []
            for g in goals:
                temp_dist = dist(g, robot['last_pos_ground_truth'])
                if temp_dist < min_dist:
                    goal = g
                    min_dist = temp_dist

            GOALS += str(goal)[1:-1]
            GOALS += "; \n"

        ANG = ANG[0:-2]  # delete last ; and \n
        ANG += "];\n"

        POS_X = POS_X[0:-2]  # delete last ; and \n
        POS_X += "];\n"

        POS_Y = POS_Y[0:-2]  # delete last ; and \n
        POS_Y += "];\n"

        POS_U = POS_U[0:-2]  # delete last ; and \n
        POS_U += "];\n"

        POS_V = POS_V[0:-2]  # delete last ; and \n
        POS_V += "];\n"

        GOALS += '];\n'

        OBST = ''
        if len(obst) > 0:
            OBST = 'obstacles = [\n'
            for o in obst:
                OBST += str(o)[1:-1]
                OBST += "; \n"
            OBST += '];\n'

        matlab_path = os.path.join(work_dir, 'runs')
        if not os.path.exists(matlab_path):
            os.makedirs(matlab_path)

        # saving trajectories to file
        mfname = "_".join(bag_name[:-4].split("_")[0:2])
        if stop:
            mfname += str(stop_time)
        matlab_fname = os.path.join(matlab_path, "%s.m" % mfname)

        print "saving trajectories to %s ..." % matlab_fname

        with open(matlab_fname, 'w') as f:
            f.write(POS_X)
            f.write(POS_Y)
            f.write(POS_U)
            f.write(POS_V)
            f.write(ANG)
            f.write("%plot(-X', Y')\n")
            f.write("%quiver(-X(1,:), Y(1,:), -U(1,:), V(1,:))\n")
            f.write(OBST)
            f.write(GOALS)

    run_time = []
    run_distances = []
    run_jerk_lin_cost = []
    run_jerk_ang_cost = []

    for robot_name in sorted(run.keys()):
        robot = run[robot_name]
        #time = (robot['last_time'] - robot['start_time']).to_sec()
        time = np.sum(np.array(robot['dt'][:finish_time]))
        distance = robot['distance']

        # compute linear jerk cost
        dts = robot['dt'][:finish_time]
        vel_lin = robot['vel_lin'][:finish_time]

        acc_lin = [0]
        for i in range(len(vel_lin) - 1):
            acc_lin.append((vel_lin[i + 1] - vel_lin[i]) / dts[i + 1])

        jerk_lin = [0]
        for i in range(len(acc_lin) - 1):
            jerk_lin.append((acc_lin[i + 1] - acc_lin[i]) / dts[i + 1])
        # print jerk_lin,acc_lin, dts,"robot ", robot_name
        cost_jerk_lin = 0.5 * sum(map(lambda (x, dt): math.pow(x, 2) * dt, zip(jerk_lin, dts)))

        if distance < 0.1:
            print "!" * 10 + " 0 DIST " + "!" * 10
            break
        # compute angular jerk cost

        vel_ang = robot['vel_ang'][:finish_time]

        acc_ang = [0]
        for i in range(len(vel_ang) - 1):
            acc_ang.append((vel_ang[i + 1] - vel_ang[i]) / dts[i + 1])

        jerk_ang = [0]
        for i in range(len(acc_lin) - 1):
            jerk_ang.append((acc_ang[i + 1] - acc_ang[i]) / dts[i + 1])

        cost_jerk_ang = 0.5 * sum(map(lambda (x, dt): math.pow(x, 2) * dt, zip(jerk_ang, dts)))
        run_time.append(time)
        run_distances.append(distance)
        run_jerk_ang_cost.append(cost_jerk_ang)
        run_jerk_lin_cost.append(cost_jerk_lin)
    print sorted(run.keys())
    return run_time, run_distances, run_jerk_lin_cost, run_jerk_ang_cost


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print "usage: evaluate.py <dirname> <stop_time>"
        sys.exit(-1)

    fname = sys.argv[1]
    stop = False
    force_new = False
    if len(sys.argv) > 2:
        stop = True
        stop_time = sys.argv[2]

    save_traj = True
    path = os.path.join(os.getcwd(), fname)
    print "reading %s .." % (path)
    evaluate_dir(path)
