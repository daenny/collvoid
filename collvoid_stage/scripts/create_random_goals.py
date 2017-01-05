#!/usr/bin/env python
import commands
import getopt
import os
import yaml
import random
import sys
import math

NUM_GOALS = 10
MIN_DIST = 0.75

X_RANGE = [-2., 2.]
Y_RANGE = [-2., 2.]

MAX_TRIES_SINGLE = 100

MAX_TRIES_GLOBAL = 10


def create_position():
    res = dict()
    res['x'] = random.uniform(X_RANGE[0], X_RANGE[1])
    res['y'] = random.uniform(Y_RANGE[0], Y_RANGE[1])
    res['ang'] = random.uniform(0, 2 * math.pi)
    return res


def dist(pos_a, pos_b):
    return math.hypot(pos_a['x'] - pos_b['x'], pos_a['y'] - pos_b['y'])


class CreateRandomGoals(object):
    def __init__(self, argv):
        random.seed(0)

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
                random.seed(int(arg))
        if not self.num_robots or not self.num_obstacles:
            print("wrong usage")
            sys.exit(2)

        yaml_file = dict()
        for n in range(self.num_robots):
            yaml_file["_".join(['robot', str(n)])] = dict()
            yaml_file["_".join(['robot', str(n)])]['goals'] = list()

        self.created_obstacles = []
        yaml_file['num_robots'] = self.num_robots
        yaml_file['num_obstacles'] = self.num_obstacles

        for obstacle in range(self.num_obstacles):
            pos = None
            while not self.check_if_valid(pos, self.created_obstacles):
                pos = create_position()
            self.created_obstacles.append(pos)

        for idx, obst in enumerate(self.created_obstacles):
            yaml_file["_".join(['obst', str(idx)])] = obst

        for g in range(NUM_GOALS + 1):
            found = False
            current_conf = []
            for _ in range(MAX_TRIES_GLOBAL):
                found = True
                for n in range(self.num_robots):
                    found_single = False
                    for _ in range(MAX_TRIES_SINGLE):
                        pos = create_position()
                        if self.check_if_valid(pos, current_conf):
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
                print("could not find configuration")
                sys.exit(2)

            for idx, p in enumerate(current_conf):
                if g == 0:
                    yaml_file["_".join(['robot', str(idx)])]['init_pose'] = p
                else:
                    yaml_file["_".join(['robot', str(idx)])]['goals'].append(p)

        self.output_dir = commands.getoutput('rospack find collvoid_stage')
        with open(os.path.join(self.output_dir,
                               "_".join(['goals', 'robots', str(self.num_robots), 'obstacles', str(self.num_obstacles), 'created.yaml'])),
                  'w') as f:
            yaml.dump(yaml_file, f)

    def check_if_valid(self, pos, current_conf):
        if pos is None:
            return False
        for pos_b in self.created_obstacles:
            if dist(pos, pos_b) < MIN_DIST:
                return False

        for pos_b in current_conf:
            if dist(pos, pos_b) < MIN_DIST:
                return False
        return True


if __name__ == '__main__':
    CreateRandomGoals(sys.argv[1:])
