#!/usr/bin/env python

import sys
import mprim

import math
import numpy 
from primitives import *

from pylab import *

# mirror about the X axis
def mirror_x(p):
    return (p[0], -p[1], -p[2])

# mirror about the Y axis
def mirror_y(p):
    return (-p[0], p[1], -p[2])

# mirror about x=y
def mirror_xy(p):
    return (p[1], p[0], -p[2])

# mirror about x=-y
def mirror_x_y(p):
    return (-p[1], -p[0], p[2])

def expand_primitives(prim):
    # mirror angle 0 primitives about X
    prim_0 = list(prim[0])
    for p in prim[0]:
        if p[2] != 0:
            prim_0.append(mirror_x(p))
    prim[0] = prim_0
    # mirror angle 2 primitives about x=y
    prim_2 = list(prim[2])
    for p in prim[2]:
        if p[2] != 0:
            prim_2.append(mirror_xy(p))
    prim[2] = prim_2
    # mirror angle 1 primitives about x=y
    prim[3] = []
    for p in prim[1]:
        prim[3].append(mirror_xy(p))

    # rotate and mirror primitives about the origin
    prim[4] = map(mirror_xy, prim[0])
    for i in [ 5, 6, 7, 8 ]:
        prim[i] = map(mirror_y, prim[8-i])
    for i in range(9,16):
        prim[i] = map(mirror_x, prim[16 - i])

def generate_mprim(prim):
    res = {}
    for start_a in prim:
        start = (0, 0, start_a)
        res[start_a] = []
        for end_a in prim[start_a]:
            end_b = (end_a[0], end_a[1], start_a + end_a[2])
            poses = [ start, end_b ]
            res[start_a].append(mprim.MPrim(start, end_b, poses))
    return res

def generate_trajectories(min_radius, num_angles):
    reachable = {}
    print reachable
    precision = 1 # number of digits of precision to use

    # is a point on our planning lattice?
    def is_lattice(p):
        # x and y error to nearest point
        e1 = abs(p[0] - round(p[0]))
        e2 = abs(p[1] - round(p[1]))
        # theta error to nearest angle
        angle = p[2] * num_angles / (math.pi * 2)
        e3 = abs(angle - round(angle))
        if e1 < 0.01 and e2 < 0.01 and e3 < 0.01:
            return (round(p[0]), round(p[1]),
                    math.pi * 2 * round(angle) / num_angles, p[3])
        else:
            return None

    def try_segment(segment):
        p = is_lattice(segment.get_end())
        if p:
            if p in reachable:
                score = segment.get_score(p)
                old_score = reachable[p].get_score(p)
                if score < old_score:
                    return p
            else:
                return p
        return None

    max_dist = 10
    start = (0, 0, 0, 0)
    # Straight lines
    for d in range(max_dist):
        segment = Linear(start, d)
        p = try_segment(segment)
        if p:
            reachable[p] = segment

    # Spiral, ...
    for l1 in numpy.arange(0.01, max_dist, 0.01):
        w1_max = 1 / (l1 * min_radius)
        w1_step = w1_max / 100
        for w1 in numpy.arange(w1_step, w1_max, w1_step):
            s1 = Spiral(start, l1, w1)
            print l1, w1

            # Spiral, Spiral
            for l2 in numpy.arange(0.1, max_dist - l1, 0.1):
                w2 = -1 * w1 * l1 / ( l2 )
                s2 = Spiral(s1.get_end(), l2, w2)
                p = try_segment(s2)
                if p:
                    print p
                    reachable[p] = Compound(s1, s2)

            # Spiral, Arc, Spiral
            for l2 in numpy.arange(0.01, max_dist - 2*l1, 0.01):
                s2 = Arc(s1.get_end(), l2)

                l3 = l1
                w3 = -w1
                s3 = Spiral(s2.get_end(), l3, w3)
                p = try_segment(s3)
                if p:
                    print p
                    reachable[p] = Compound(s1, s2, s3)
                #for l3 in numpy.arange(0.1, max_dist - l1 - l2, 0.1):
                #    w3 = -1 * w1 * l1 / ( l3 )
                #    s3 = Spiral(s2.get_end(), l3, -w3)
                #    p = try_segment(s3)
                #    if p:
                #        print p
                #        reachable[p] = Compound(s1, s2, s3)

    # TODO: rewrite this as a string of Segment classes
    #  which can be either linear, easment or circular segments
    #for l1 in numpy.arange(0.01, 10.0, 0.01):
    #    for l2 in numpy.arange(0.01, 10.0, 0.01):
    #        w1_max = 1 / (l1 * min_radius)
    #        w2_max = 1 / (l2 * min_radius)
    #        # don't consider 0 angle change yet, when we aren't computing
    #        # s-curves
    #        for angle in range(1, num_angles):
    #            angle_rad = angle * (math.pi * 2.0) / num_angles
    #            w1 = angle_rad / ( l1*l1 + 0.5*l1*l2 )
    #            w2 = -1 * w1 * l1 / ( l2 )

    #            if abs(w1) > w1_max:
    #                continue
    #            if abs(w2) > w2_max:
    #                continue

    #            s1 = Spiral((0, 0, 0, 0), l1, w1)
    #            s2 = Spiral(s1.get_end(), l2, -w2)
    #            p = try_segment(s2)
    #            if p:
    #                print p
    #                reachable[p] = Compound(s1, s2)

    print reachable.keys()
    return reachable

def main():
    import argparse
    parser = argparse.ArgumentParser('Motion primitive generation')
    parser.add_argument('-o', '--output', 
                        help="Output file")
    parser.add_argument('-r', '--resolution', default=0.1,
                        help="Primitive resolution (in meters)")
    parser.add_argument('-m', '--min-radius', default=0.6,
                        help="Minimum radius (in meters)")

    args = parser.parse_args()

    # TODO: parse/handle these properly
    args.num_angles = 16

    # primitives should explicitly allow differing numbers of primitives
    # per start angle
    # we should also have reflection/rotation functions to generate the
    # remainder of the primitives from the initial primitives
    primitives = {
        0: [ (1, 0, 0), (4, 1, 1), (5, 2, 2) ],
        1: [ (2, 1, 0), (3, 2, 1), (4, 1, -1), (4, 4, 2), (6, 0, -2) ],
        2: [ (1, 1, 0), (2, 3, 1), (2, 5, 2) ]
        }

    trajectories = generate_trajectories(args.min_radius / args.resolution,
                                args.num_angles)

    for p in trajectories:
        print p
        segment = trajectories[p]
        print segment

        cla() # clear axes
        segment.plot(resolution=0.02)
        axis('equal')
        show()

    for p in trajectories:
        print p
        segment = trajectories[p]
        print segment

        segment.plot(resolution=0.02)
    axis('equal')
    show()

    expand_primitives(primitives)

    prim = generate_mprim(primitives)

    if not args.output:
        import yaml
        print yaml.dump(mprim)
    else:
        mprim.write_mprim(args.output, prim, args.resolution)

if __name__ == '__main__':
    main()
