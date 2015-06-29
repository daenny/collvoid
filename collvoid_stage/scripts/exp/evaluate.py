#!/usr/bin/env python
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
    if pos[0] < -4.5 or pos[0] > -0.0 or pos[1] < -0.5 or pos[1] > 5.5:
        return True  # in the box
    else:
        return False  # out of the box


def twist_to_uv((x, pose)):
    q = []
    q.append(pose.orientation.x)
    q.append(pose.orientation.y)
    q.append(pose.orientation.z)
    q.append(pose.orientation.w)
    alpha = tf.transformations.euler_from_quaternion(q)[2]
    #    alpha += math.pi / 2
    return (math.cos(alpha) * abs(x), math.sin(alpha) * abs(x))


if __name__ == '__main__':
    if not (len(sys.argv) == 2):
       print "usage: evaluate.py <filename>"
       sys.exit(-1)

    fname = argv = sys.argv[1]
    # fname = "../../bags/collvoid_5_cocalu_True_extrasampling_True.bag"
    print "reading %s .." % (fname)
    bag = rosbag.Bag(fname)

    count = 0

    runs = []
    robots = {}
    stall = []
    stall_resolved = []
    num_robots = -1
    exceeded = []
    sys.stdout.write("evaluating run ...")

    distance = 0
    skipped = 0
    # read all messages
    run = -1
    stopped = True
    for topic, msg, t in bag.read_messages():

        if topic == "/stall":
            stall.append(int(msg.data))
            stopped = True
            continue
        if topic == "/stall_resolved":
            stall_resolved.append(int(msg.data))
            stopped = True
            continue
        if topic == "/exceeded":
            exceeded.append(int(msg.data))
            stopped = True
            continue

        if topic == "/num_run":
            run += 1
            stopped = False
            continue
        if stopped:
            continue
        if "ground_truth" in topic:
            robot_name = topic[1:8]
        #    continue
        # which run
        # run = msg.run
        #if topic == "/position_share":
        #    robot_name = msg.robot_id
            count += 1
            # create new run
            if len(runs) < run + 1:
                sys.stdout.write(" %d" % run)
                sys.stdout.flush()
                runs.append({})

            # create new robot
            if not robot_name in runs[run]:
                # print "first time i have seen %s in run %d"%(robot_name, run)
                runs[run][robot_name] = {}
                robot = runs[run][robot_name]
                robot['start_time'] = msg.header.stamp
                robot['last_time'] = robot['start_time']
                robot['distance'] = 0
                robot['last_pos_ground_truth'] = (msg.pose.pose.position.x,
                                                  msg.pose.pose.position.y)

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
                robot = runs[run][robot_name]

                # update last position and distance
                pos = (msg.pose.pose.position.x,
                       msg.pose.pose.position.y)
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
                dt = (msg.header.stamp - robot['temp_last_time']).to_sec()
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

    num_robots = max(map(lambda x: len(x), runs))
    print "found %d robots" % num_robots

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
    for run in range(len(runs)):
        print "run %d (%d robots):" % (run, num_robots)

        # generating trajectories for matlab

        POS_X = "X = ["
        POS_Y = "Y = ["
        POS_U = "U = ["
        POS_V = "V = ["

        max_length = 0
        for robot_name in runs[run]:
            robot = runs[run][robot_name]
            max_length = max(max_length, len(robot['pos_ground_truth']))

        for robot_name in runs[run]:
            robot = runs[run][robot_name]
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
            POS_X = POS_X[0:-2]  # delete last ,
            POS_X += ";\n"

            for y in pos_y:
                POS_Y += str(y) + ", "
            POS_Y = POS_Y[0:-2]  # delete last ,
            POS_Y += ";\n"

            for u in pos_u:
                POS_U += str(u) + ", "
            POS_U = POS_U[0:-2]  # delete last ,
            POS_U += ";\n"

            for v in pos_v:
                POS_V += str(v) + ", "
            POS_V = POS_V[0:-2]  # delete last ,
            POS_V += ";\n"

        POS_X = POS_X[0:-2]  # delete last ; and \n
        POS_X += "];\n"

        POS_Y = POS_Y[0:-2]  # delete last ; and \n
        POS_Y += "];\n"

        POS_U = POS_U[0:-2]  # delete last ; and \n
        POS_U += "];\n"

        POS_V = POS_V[0:-2]  # delete last ; and \n
        POS_V += "];\n"

        # saving trajectories to file
        matlab_fname = "./runs/%s_run%d.m" % (fname[0:-4], run)
        #matlab_fname = "../../bags/run%d.m" % run
        print "saving trajectories to %s ..." % matlab_fname
        f = open(matlab_fname, 'w')
        f.write(POS_X)
        f.write(POS_Y)
        f.write(POS_U)
        f.write(POS_V)
        f.write("plot(-X', Y')\n")
        f.write("%quiver(-X(1,:), Y(1,:), -U(1,:), V(1,:))\n")
        f.close()

        if len(runs[run]) != num_robots:
            print "!" * 10 + " NOT ALL ROBOTS STARTED " + "!" * 10
            not_started_count += 1
            continue

        run_max_time = 0
        run_avg_time = 0
        run_avg_distance = 0
        run_avg_loc_error = 0
        skip = False

        run_min_jerk_lin_cost = 10000000000000
        run_max_jerk_lin_cost = 0
        run_avg_jerk_lin_cost = 0

        run_min_jerk_ang_cost = 10000000000000
        run_max_jerk_ang_cost = 0
        run_avg_jerk_ang_cost = 0
        run_avg_loc_err = 0

        if not stall[run] == 0:
            coll_runs += 1
            continue
        if not stall_resolved[run] == 0:
            coll_resolved_runs += 1
            continue
        if exceeded[run]:
            exceeded_count += 1
            continue
        # robot loop
        # if run in [6,7,8,9,15,25]:
        #     continue
        for robot_name in runs[run]:
            robot = runs[run][robot_name]
            time = (robot['last_time'] - robot['start_time']).to_sec()
            distance = robot['distance']
            # avg_loc_error = sum(robot['loc_error']) / len(robot['loc_error'])

            in_box = map(lambda x: bounding_box(x), robot['pos_ground_truth'])
            # if not min(in_box):
            #    print "!" * 10 + " ROBOT OUT OF BOX " + "!" * 10
            #    skip = True
            #    out_of_box_count += 1
            #    break




            # if (time > COLLISION_TIME):

            #    if dist_to_center(robot['last_pos_ground_truth']) > 2.0:
            #        print "!" * 10 + " DEADLOCK " + "!" * 10
            #        deadlock_count += 1
            #    else:
            #        print "!" * 10 + " COLLISION " + "!" * 10
            #        collision_count += 1
            #    
            #    skip = True
            #    break
            #
            if run_max_time < time:
                run_max_time = time


            # compute linear jerk cost
            dts = robot['dt']
            vel_lin = robot['vel_lin']

            acc_lin = [0]
            for i in range(len(vel_lin) - 1):
                acc_lin.append((vel_lin[i + 1] - vel_lin[i]) / dts[i + 1])

            jerk_lin = [0]
            for i in range(len(acc_lin) - 1):
                jerk_lin.append((acc_lin[i + 1] - acc_lin[i]) / dts[i + 1])
            # print jerk_lin,acc_lin, dts,"robot ", robot_name
            cost_jerk_lin = 0.5 * sum(map(lambda (x, dt): math.pow(x, 2) * dt, zip(jerk_lin, dts)))

            if cost_jerk_lin < 250:
                print "!" * 10 + " SMALL JERK " + "!" * 10
                # skip = True
                small_jerk_count += 1
                # break

            # compute angular jerk cost

            vel_ang = robot['vel_ang']

            acc_ang = [0]
            for i in range(len(vel_ang) - 1):
                acc_ang.append((vel_ang[i + 1] - vel_ang[i]) / dts[i + 1])

            jerk_ang = [0]
            for i in range(len(acc_lin) - 1):
                jerk_ang.append((acc_ang[i + 1] - acc_ang[i]) / dts[i + 1])

            cost_jerk_ang = 0.5 * sum(map(lambda (x, dt): math.pow(x, 2) * dt, zip(jerk_ang, dts)))

            run_min_jerk_lin_cost = min(cost_jerk_lin, run_min_jerk_lin_cost)
            run_max_jerk_lin_cost = max(cost_jerk_lin, run_max_jerk_lin_cost)

            run_min_jerk_ang_cost = min(cost_jerk_ang, run_min_jerk_ang_cost)
            run_max_jerk_ang_cost = max(cost_jerk_ang, run_max_jerk_ang_cost)

            run_avg_time += time / num_robots
            run_avg_distance += distance / num_robots
            # run_avg_loc_error += avg_loc_error / num_robots
            run_avg_jerk_lin_cost += cost_jerk_lin / num_robots
            run_avg_jerk_ang_cost += cost_jerk_ang / num_robots
            # run_avg_loc_err += avg_loc_error / num_robots
            avg_loc_error = 0
            print "%s\ttime:\t%f\tdist:\t%f\tavg_loc_error:\t %f\tcost_jerk_ang: %f" % (
                robot_name, time, distance, avg_loc_error, cost_jerk_ang)

        if skip:
            continue

        # print "run avg-time: %f"%run_avg_time
        # print "run max-time: %f"%run_max_time
        # print "run avg-distance: %f"%run_avg_distance
        # print "run avg-jerk-lin-cost: %f"%run_avg_jerk_lin_cost
        # print "run min-jerk-lin-cost: %f"%run_min_jerk_lin_cost
        # print "run max-jerk-lin-cost: %f"%run_max_jerk_lin_cost
        # print "run avg-jerk-ang-cost: %f"%run_avg_jerk_ang_cost
        # print "run min-jerk-ang-cost: %f"%run_min_jerk_ang_cost
        # print "run max-jerk-ang-cost: %f"%run_max_jerk_ang_cost

        loc_err_array.append(run_avg_loc_err)
        jerk_lin_array.append(run_avg_jerk_lin_cost)
        jerk_ang_array.append(run_avg_jerk_ang_cost)
        time_max_array.append(run_max_time)
        distance_array.append(run_avg_distance)
        run_count += 1

    loc_err_avg = numpy.mean(loc_err_array)
    jerk_lin_avg = numpy.mean(jerk_lin_array)
    jerk_ang_avg = numpy.mean(jerk_ang_array)
    time_max_avg = numpy.mean(time_max_array)
    distance_avg = numpy.mean(distance_array)

    loc_err_std = numpy.std(loc_err_array)
    jerk_lin_std = numpy.std(jerk_lin_array)
    jerk_ang_std = numpy.std(jerk_ang_array)
    time_max_std = numpy.std(time_max_array)
    distance_std = numpy.std(distance_array)

    # exceeded_count = numpy.sum(exceeded)

    coll_total = numpy.sum(stall)
    # coll_runs = numpy.sum(not(stall==0))
    coll_res_total = numpy.sum(stall_resolved)
    # coll_res_std = numpy.std(stall_resolved)

    localization = fname.find("True")
    if localization < 0:
        localization = False
    else:
        localization = True

    print "-" * 80
    print "#robots: %d\nlocalization: %s\n#runs: %d\ncollisions: %d in\t%d runs\ncollisions_res: %d in \t%d\n#runs exceeded time=: %d\n#runs out of box: %d\n#runs not started: %d\n#runs small jerk: %d\navg max time: %f\t%f\navg distance: %f\t%f\navg-jerk-lin-cost: %f\t%f\navg-jerk-ang-cost: %f\t%f\nloc-err: %f\t%f" % (
        num_robots, str(localization), run_count, coll_total, coll_runs, coll_res_total, coll_resolved_runs, exceeded_count,
        out_of_box_count, not_started_count, small_jerk_count, time_max_avg, time_max_std, distance_avg, distance_std,
        jerk_lin_avg, jerk_lin_std, jerk_ang_avg, jerk_ang_std, loc_err_avg, loc_err_std)
    #    print stall
    #   print stall_resolved
    #
    #print exceeded
    print "collisions=[collisions %f];"%coll_total
    print "avg_time=[avg_time %f];"%time_max_avg
    print "avg_time_var=[avg_time_var %f];"%time_max_std
    print "avg_distance=[avg_distance %f];"%distance_avg
    print "avg_distance_std=[avg_distance_std %f];"%distance_std
    print "avg_jerk_lin=[avg_jerk_lin %f];"%jerk_lin_avg
    print "avg_jerk_lin_std=[avg_jerk_lin_std %f];"%jerk_lin_std
    print "avg_jerk_ang=[avg_jerk_ang %f];"%jerk_ang_avg
    print "avg_jerk_ang_std=[avg_jerk_ang_std %f];"%jerk_ang_std

