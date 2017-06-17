#!/usr/bin/env python
import commands
import getopt
import os
import yaml
import random
import sys
import math

NUM_GOALS = 10
MIN_DIST = 0.9

MIN_DIST_INIT_OTHER = 0.5
real = False
if real:
    MIN_DIST_INIT_OWN = 1.5  # was 2 for exps
    X_RANGE = [-1.2, 1.2]
    Y_RANGE = [-1.6, 1.6]

else:
    MIN_DIST_INIT_OWN = 2
    X_RANGE = [-2.0, 2.0]
    Y_RANGE = [-2.0, 2.0]

MAX_TRIES_SINGLE = 1000

MAX_TRIES_GLOBAL = 50


def create_position():
    res = dict()
    res['x'] = random.uniform(X_RANGE[0], X_RANGE[1])
    res['y'] = random.uniform(Y_RANGE[0], Y_RANGE[1])
    res['ang'] = random.uniform(0, 2 * math.pi)
    return res


def dist(pos_a, pos_b):
    return math.hypot(pos_a['x'] - pos_b['x'], pos_a['y'] - pos_b['y'])


class CreateRandomGoals(object):
    num_robots = None
    num_obstacles = None

    def __init__(self, argv):

        self.seed = 0
        try:
            opts, args = getopt.getopt(argv, "n:o:s:", ["num_robots=", "num_obstacles=", "seed="])
        except getopt.GetoptError:
            print("wrong usage")
            sys.exit(2)
        for opt, arg in opts:
            if opt in ("-n", "--num_robots"):
                self.num_robots = int(arg)
            if opt in ("-o", "--num_obstacles"):
                self.num_obstacles = int(arg)
            if opt in ("-s", "--seed"):
                self.seed = int(arg)

        if not self.num_robots or not self.num_obstacles:
            print("wrong usage")
            sys.exit(2)
        random.seed(self.seed)
        yaml_file = dict()
        for n in range(self.num_robots):
            yaml_file["_".join(['robot', str(n)])] = dict()

        yaml_file['num_robots'] = self.num_robots
        yaml_file['num_obstacles'] = self.num_obstacles
        found = False
        for g in range(MAX_TRIES_GLOBAL):
            print("generating obstacles")

            self.created_obstacles = []
            for idx in range(self.num_robots):
                yaml_file["_".join(['robot', str(idx)])]['goals'] = list()

            for obstacle in range(self.num_obstacles):
                pos = None
                while not self.check_if_valid(pos, self.created_obstacles):
                    pos = create_position()
                self.created_obstacles.append(pos)

            for idx, obst in enumerate(self.created_obstacles):
                yaml_file["_".join(['obst', str(idx)])] = obst

            init_conf = None
            for g in range(NUM_GOALS + 1):
                current_conf = []
                for _ in range(MAX_TRIES_GLOBAL):
                    found = True
                    for n in range(self.num_robots):
                        found_single = False
                        for _ in range(MAX_TRIES_SINGLE):
                            pos = create_position()
                            if self.check_if_valid(pos, current_conf, init_conf, n):
                                current_conf.append(pos)
                                found_single = True
                                break
                        if not found_single:
                            found = False
                            break
                    if found:
                        break
                    else:
                        current_conf = []
                if not found:
                    break
                for idx, p in enumerate(current_conf):
                    if g == 0:
                        print("found init conf")
                        init_conf = current_conf
                        yaml_file["_".join(['robot', str(idx)])]['init_pose'] = p
                    else:
                        if real:
                            init_conf = current_conf
                        yaml_file["_".join(['robot', str(idx)])]['goals'].append(p)
            if found:
                break
        if not found:
            print("could not find configuration")
            sys.exit(2)

        self.output_dir = commands.getoutput('rospack find collvoid_stage')
        if self.seed == 0:
            file_name = os.path.join(self.output_dir,
                                     "_".join( ['goals', 'robots', str(self.num_robots), 'obstacles', str(self.num_obstacles), 'created.yaml']))
        else:
            file_name = os.path.join(self.output_dir,
                                     "_".join( ['goals', 'robots', str(self.num_robots), 'obstacles', str(self.num_obstacles), str(self.seed), 'created.yaml']))

        with open(file_name, 'w') as f:
            yaml.dump(yaml_file, f)

    def check_if_valid(self, pos, current_conf, init_conf=None, n=0):
        if pos is None:
            return False
        for pos_b in self.created_obstacles:
            if dist(pos, pos_b) < MIN_DIST:
                return False
        if init_conf is not None:
            if dist(pos, init_conf[n]) < MIN_DIST_INIT_OWN:
                return False
            for p in init_conf:
                if dist(pos, p) < MIN_DIST_INIT_OTHER:
                    return False

        for pos_b in current_conf:
            if dist(pos, pos_b) < MIN_DIST:
                return False
        return True


if __name__ == '__main__':
    CreateRandomGoals(sys.argv[1:])
