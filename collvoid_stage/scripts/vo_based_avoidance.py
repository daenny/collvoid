from __future__ import division
import numpy as np
from collections import namedtuple

RVO_TYPE = 1
HRVO_TYPE = 2

Line = namedtuple("Line", "origin direction")


def line_line_intersection(line1, line2):
    xdiff = (line1.direction[0], line2.direction[0])
    ydiff = (line1.direction[1], line2.direction[1])

    div = det(xdiff, ydiff)
    if div == 0:
        return None
    s = det(line1.direction, line1.origin - line2.origin)
    r = det(line2.direction, line2.origin - line1.origin)
    return line1.origin + r * line1.direction


def normalize(vector):
    return vector / np.linalg.norm(vector)


def normal(vector):
    return normalize(np.array([vector[1], -vector[0]]))


def rotate_vector_by_angle(vector, ang):
    cos_a = np.cos(ang)
    sin_a = np.sin(ang)
    return np.array(cos_a * vector[0] - sin_a * vector[1], cos_a * vector[1] + sin_a * vector[0])


def det(vector1, vector2):
    return vector1[0] * vector2[1] - vector1[1] * vector2[0]


def right_of(line, point, left_pref=0):
    if det(line.origin - point, line.direction) <= -left_pref:
        return True
    else:
        return False


def left_of(line, point, left_pref=0):
    if det(line.origin - point, line.direction) >= -left_pref:
        return True
    else:
        return False


class VO(object):
    apex = None
    left_leg_dir = None
    right_leg_dir = None
    # combined_radius = None
    rel_position = None
    truncated = False
    truncation_radius = None
    truncation_center_ = None
    truncation_left_ = None
    truncation_right_ = None

    @property
    def truncation_center(self):
        return self.truncation_center_ + self.apex

    @truncation_center.setter
    def truncation_center(self, truncation_center):
        self.truncation_center_ = truncation_center

    @property
    def truncation_left(self):
        if self.truncation_left_ is not None:
            return self.truncation_left_ + self.apex
        else:
            return self.apex

    @truncation_left.setter
    def truncation_left(self, truncation_left):
        self.truncation_left_ = truncation_left

    @property
    def truncation_right(self):
        if self.truncation_right_ is not None:
            return self.truncation_right_ + self.apex
        else:
            return self.apex

    @truncation_right.setter
    def truncation_right(self, truncation_right):
        self.truncation_right_ = truncation_right


def create_vo(combined_radius, my_pose, my_vel, other_pose, other_vel, trunc_time=0, left_pref=0, vo_type=None):
    res_vo = VO()
    # res_vo.combined_radius = combined_radius
    res_vo.apex = other_vel
    rel_position = other_pose - my_pose
    res_vo.rel_position = rel_position
    rel_velocity = my_vel - other_vel

    dist = np.linalg.norm(rel_position)
    if dist < combined_radius:
        print("Virtual collision")
        res_vo.right_leg_dir = normal(rel_position)
        res_vo.left_leg_dir = -normal(rel_position)
        res_vo.apex = (dist - combined_radius) * normalize(rel_position)
        return res_vo
    else:
        angle_to_other = np.arctan2(rel_position[1], rel_position[0])
        angle_of_opening = np.arcsin(combined_radius / dist)
        res_vo.right_leg_dir = np.array([np.cos(angle_to_other - angle_of_opening),
                                         np.sin(angle_to_other - angle_of_opening)])

        res_vo.left_leg_dir = np.array([np.cos(angle_to_other + angle_of_opening),
                                        np.sin(angle_to_other + angle_of_opening)])

    if vo_type == RVO_TYPE:
        res_vo.apex = (my_vel + other_vel) / 2.

    if vo_type == HRVO_TYPE:
        if left_of(Line(rel_position, rel_position), rel_velocity, left_pref):
            res_vo.apex = line_line_intersection(Line((my_vel + other_vel) / 2., res_vo.left_leg_dir),
                                                 Line(other_vel, res_vo.right_leg_dir))
        else:
            res_vo.apex = line_line_intersection(Line((my_vel + other_vel) / 2., res_vo.right_leg_dir),
                                                 Line(other_vel, res_vo.left_leg_dir))

    if trunc_time > 0:
        res_vo.truncated = True
        res_vo.truncation_radius = combined_radius / trunc_time
        truncation_dist = res_vo.truncation_radius / np.sin(angle_of_opening)
        res_vo.truncation_center = normalize(rel_position) * truncation_dist
        leg_trunc_dist = np.cos(angle_of_opening) * truncation_dist
        res_vo.truncation_left = res_vo.left_leg_dir * leg_trunc_dist
        res_vo.truncation_right = res_vo.right_leg_dir * leg_trunc_dist

    return res_vo


def get_best_velocity(pref_vel, vo, left_pref=0):
    assert isinstance(vo, VO)
    # outside cone
    if left_of(Line(vo.apex, vo.left_leg_dir), pref_vel) or right_of(Line(vo.apex, vo.right_leg_dir), pref_vel):
        return pref_vel
    if vo.truncated:
        # outside circle and tangents
        if np.linalg.norm(vo.truncation_center - pref_vel) >= vo.truncation_radius and \
                left_of(Line(vo.truncation_left, vo.truncation_left - vo.truncation_right), pref_vel):
            return pref_vel
    # pref_vel is inside VO:
    # if left of center line:
    if left_of(Line(vo.apex, vo.rel_position), pref_vel, left_pref):
        res = line_line_intersection(Line(vo.apex, vo.left_leg_dir), Line(pref_vel, normal(vo.left_leg_dir)))
    else:
        res = line_line_intersection(Line(vo.apex, vo.right_leg_dir), Line(pref_vel, normal(vo.right_leg_dir)))

    if vo.truncated:
        if np.linalg.norm(res-vo.apex) < np.linalg.norm(vo.truncation_left-vo.apex) \
                and np.linalg.norm(pref_vel-vo.truncation_center) > 0:
            res = vo.truncation_center + vo.truncation_radius * normalize(pref_vel - vo.truncation_center)

    return res


if __name__ == '__main__':
    import matplotlib.pyplot as plt


    def plot_line(line, length, *args, **kwargs):
        plt.plot([line.origin[0], (line.origin + length * line.direction)[0]],
                 [line.origin[1], (line.origin + length * line.direction)[1]], *args, **kwargs)


    def plot_vo(vo):
        plot_line(Line(vo.truncation_left, vo.left_leg_dir), length, 'k-')
        plot_line(Line(vo.truncation_right, vo.right_leg_dir), length, 'k-')
        if vo.truncated:
            trunc = plt.Circle(vo.truncation_center, vo.truncation_radius, color='k', fill=False)
            ax.add_artist(trunc)


    zero_vec = np.array([0, 0])
    my_pos = np.array([1, 0])
    other_pos = np.array([0, 0.18])
    my_vel = np.array([-1.47, 1.49])
    other_vel = np.array([0, 1])
    combined_radius = 0.2
    length = 4
    trunc_time = 2

    fig, ax = plt.subplots()  # note we must use plt.subplots, not plt.subplot
    plt.axis('equal')
    # plot vo
    vo = create_vo(combined_radius, my_pos, my_vel, other_pos, other_vel, trunc_time=trunc_time, vo_type=HRVO_TYPE)
    res = get_best_velocity(my_vel, vo)

    plot_vo(vo)

    footprints = plt.Circle(vo.rel_position, combined_radius, color='r', fill=False)
    ax.add_artist(footprints)
    plot_line(Line(vo.rel_position, other_vel), 1)

    plot_line(Line(zero_vec, my_vel), 1, 'r-', linewidth=3)

    res = get_best_velocity(my_vel, vo)
    print res
    plot_line(Line(zero_vec, res), 1, 'g-', linewidth=4)
    plt.show()


