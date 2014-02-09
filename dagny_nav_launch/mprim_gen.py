#!/usr/bin/env python

import sys
import mprim

import math
import numpy 
import scipy.optimize
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
    tolerance = 0.1 # tolerance for matching to the grid

    def LSASL(start, l1, l2, w, l3, l4):
        l2 = max(l2, 0.00000001)
        l2 = min(l2, 20)
        #w = 1 / (2 * l2 * radius )
        w_max = 1 / (2 * l2 * min_radius)
        w = min(w, w_max)
        w = max(w, -w_max)
        l1 = max(l1, 0)
        l1 = min(l1, 1)
        l3 = max(l3, 0)
        l4 = max(l4, 0)
        l4 = min(l4, 1)
        if w == 0:
            return Linear(start, l1 + l2 * 2 + l3 + l4)
        s1 = Linear(start, l1)
        s2 = Spiral(s1.get_end(), l2, w)
        s3 = Arc(s2.get_end(), l3)
        s4 = Spiral(s3.get_end(), l2, -w)
        s5 = Linear(s4.get_end(), l4)
        return Compound(s1, s2, s3, s4, s5)
    
    def SAS(start, l1, w, l2):
        #w = 1 / (2 * l1 * radius )
        l1 = max(l1, 0.00000001)
        w_max = 1 / (2 * l1 * min_radius)
        w = min(w, w_max)
        w = max(w, -w_max)
        l2 = max(l2, 0)
        if w == 0:
            return Linear(start, l1 * 2 + l2)
        s1 = Spiral(start, l1, w)
        s2 = Arc(s1.get_end(), l2)
        s3 = Spiral(s2.get_end(), l1, -w)
        return Compound(s1, s2, s3)
    
    def LS_Curve(start, l1, l2, w, l3, l4):
        #w = 1 / (2 * l2 * radius )
        l2 = max(l2, 0.00000001)
        l2 = min(l2, 20)
        w_max = 1 / (2 * l2 * min_radius)
        w = min(w, w_max)
        w = max(w, -w_max)
        l1 = max(l1, 0)
        l1 = min(l1, 1)
        l3 = max(l3, 0)
        l4 = max(l4, 0)
        l4 = min(l4, 1)
        if w == 0:
            return Linear(start, l1 + l2 * 4 + l3 * 2 + l4)
        s1 = Linear(start, l1)
        s2 = Spiral(s1.get_end(), l2, w)
        s3 = Arc(s2.get_end(), l3)
        s4 = Spiral(s3.get_end(), l2*2.0, -w)
        s5 = Arc(s4.get_end(), l3)
        s6 = Spiral(s5.get_end(), l2, w)
        s7 = Linear(s6.get_end(), l4)
        return Compound(s1, s2, s3, s4, s5, s6, s7)
    
    def S_Curve(start, l1, w, l2):
        #w = 1 / (2 * l1 * radius )
        l1 = max(l1, 0.00000001)
        w_max = 1 / (2 * l1 * min_radius)
        w = min(w, w_max)
        w = max(w, -w_max)
        l2 = max(l2, 0)
        if w == 0:
            return Linear(start, l1 * 4 + l2 * 2)
        s1 = Spiral(start, l1, w)
        s2 = Arc(s1.get_end(), l2)
        s3 = Spiral(s2.get_end(), l1*2.0, -w)
        s4 = Arc(s3.get_end(), l2)
        s5 = Spiral(s4.get_end(), l1, w)
        return Compound(s1, s2, s3, s4, s5)

    def score(p, target):
        e1 = (p[0] - target[0])*(p[0] - target[0])
        e2 = (p[1] - target[1])*(p[1] - target[1])
        # theta error to nearest angle
        angle = p[2] * num_angles / (math.pi * 2)
        target_angle = target[2] * num_angles / (math.pi * 2)
        e3 = (angle - target_angle)*(angle - target_angle)
        return e1, e2, e3

    # is a point on our planning lattice?
    def is_lattice(p):
        # x and y error to nearest point
        e1 = abs(p[0] - round(p[0]))
        e2 = abs(p[1] - round(p[1]))
        # theta error to nearest angle
        angle = p[2] * num_angles / (math.pi * 2)
        e3 = abs(angle - round(angle))
        if e1 < tolerance and e2 < tolerance and e3 < tolerance:
            return (round(p[0]), round(p[1]),
                    math.pi * 2 * round(angle) / num_angles, p[3])
        else:
            return None

    def index(p):
        """ Get the index numers for a given point """
        x = int(round(p[0]))
        y = int(round(p[1]))
        t = int(round(p[2] * num_angles / ( math.pi * 2 )))
        w = int(round(p[3]))
        return (x, y, t, w)

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
    res = 0.2

    # generate seed points for the optimizer
    seeds = {}
    for angle in range(3):
        start_angle = angle * math.pi * 2 / num_angles
        start = (0, 0, start_angle, 0)

        for l1 in numpy.arange(res, max_dist, res):
            w1_max = 1 / (2 * l1 * min_radius)
            w1_step = w1_max / 100
            for w1 in numpy.arange(w1_step, w1_max, w1_step):
                s1 = Spiral(start, l1, w1)

                # Spiral, Arc, Spiral
                for l2 in numpy.arange(res, max_dist - 2*l1, res):
                    s2 = Arc(s1.get_end(), l2)

                    # optimize by using matching lead-in and lead-our spirals
                    s3 = Spiral(s2.get_end(), l1, -w1)
                    p = is_lattice(s3.get_end())
                    if p:
                        s = sum(score(s3.get_end(), p))
                        i = (index(start), index(p))
                        s4 = Compound(s1, s2, s3)
                        if i in seeds:
                            if s < seeds[i][0]:
                                seeds[i] = (s, SAS, l1, w1, l2, s4)
                        else:
                            print i
                            seeds[i] = (s, SAS, l1, w1, l2, s4)

                # Spiral, Arc, Spiral(x2), Arc, Spiral (S-curve)
                # length = 4*l1 + 2*l2 <= max_dist
                #          l2 <= (max_dist / 2) - 2 * l1
                for l2 in numpy.arange(res, max_dist/2.0 - 2*l1, res):
                    s2 = Arc(s1.get_end(), l2)
                    s3 = Spiral(s2.get_end(), 2*l1, -w1)
                    s4 = Arc(s3.get_end(), l2)
                    s5 = Spiral(s4.get_end(), l1, w1)
                    p = is_lattice(s5.get_end())
                    if p:
                        s = sum(score(s5.get_end(), p))
                        i = (index(start), index(p))
                        s6 = Compound(s1, s2, s3, s4, s5)
                        if i in seeds:
                            if s < seeds[i][0]:
                                seeds[i] = (s, S_Curve, l1, w1, l2, s6)
                        else:
                            print i
                            seeds[i] = (s, S_Curve, l1, w1, l2, s6)

    print len(seeds)
    tolerance = 0.01

    # Straight lines
    #for d in range(max_dist):
    #    segment = Linear(start, d)
    #    p = try_segment(segment)
    #    if p:
    #        reachable[p] = segment

    def sas(start, end):
        def err(args):
            return score(SAS(start, *args).get_end(), end)
        return err

    def scurve(start, end):
        def err(args):
            return score(S_Curve(start, *args).get_end(), end)
        return err

    #primitives = {
    #    0: [ (4, 1, 1), (5, 1, 1), (6, 1, 1), (6, 2, 2),
    #         (5, 1, 0), (8, 2, 0) ],
    #    1: [ (3, 2, 1), (4, 1, -1), (4, 4, 2), (6, 0, -2),
    #         (3, 2, 0), (4, 1, 0),  (4, 4, 0), (6, 0, 0) ],
    #    2: [ (2, 3, 1), (2, 5, 2),
    #         (2, 3, 0), (2, 5, 0) ]
    #    }
    primitives = {}
    for start_angle in range(3):
        primitives[start_angle] = []
        for x in range(11):
            for y in range(11):
                for angle in [ -2, -1, 1, 2 ]:
                    primitives[start_angle].append((x, y, angle,))

    max_iter = 100000
    xtol = 0.001 * 0.001 * 3
    print "xtol", xtol

    found_seeds = 0
    missed_seeds = 0

    for start_angle in primitives:
        start = (0, 0, 2 * math.pi * start_angle / num_angles , 0)
        for end_pose in primitives[start_angle]:
            end_angle = start_angle + end_pose[2]
            end = (end_pose[0], end_pose[1], 2.0 * math.pi * end_angle / \
                    num_angles, 0)

            # starting guess:
            #           l1,  w1, l2
            #l_est = math.sqrt(end[0]*end[0] + end[1]*end[1]) + 0.2
            #if end_pose[2] == 0:
            #    radius_est = 0
            #    w_est = 0
            #else:
            #    radius_est = l_est / (2 * math.pi * end_pose[2] / num_angles )
            #    w_est = 1 / ( 2 * radius_est * l_est )

            i = ((0, 0, start_angle, 0),
                    (end_pose[0], end_pose[1], start_angle + end_pose[2], 0))
            i = (index(start), index(end))
            if i in seeds:
                seed = seeds[i]
                found_seeds += 1
            else:
                #print i
                missed_seeds += 1
                continue

            print "Solving for", start, end
            print "Starting score", seed[0]
            estimate = [seed[2], seed[3], seed[4]]
            segment = seed[1](start, *estimate)
            s = sum(score(segment.get_end(), end))
            if s != seed[0]:
                print "Starting score", s
                print segment.get_end()
                print "Seed", seed
                print segment
                segment.plot(resolution=0.02)
                seed[5].plot(resolution=0.02)
                show()

            if end[2] == start[2]:
                if seed[1] != S_Curve:
                    print "WARNING: wrong seed type for", start, "->", end
                # estimate with s-curve
                args, info, ier, mesg = scipy.optimize.fsolve(
                        scurve(start, end), estimate, maxfev=max_iter,
                        full_output=True, xtol=xtol)
                segment = S_Curve(start, *args)
                print "Ending score", sum(score(segment.get_end(), end))
                if ier == 1:
                    p = is_lattice(segment.get_end())
                    assert(p)
                    reachable[(start, end)] = segment
                    print "Found", start, end

                #estimate = [l_est / 8.0, w_est, 3.0 * l_est / 8.0]
                ##estimate = [0, l_est / 8.0, w_est, 3.0 * l_est / 8.0, 0]
                ## estimate with s-curve
                #args, info, ier, mesg = scipy.optimize.fsolve(
                #        scurve(start, end), estimate, maxfev=max_iter,
                #        full_output=True, xtol=xtol)
                #segment = S_Curve(*args)
                ##segment = LS_Curve(*args)
                #if ier == 1:
                #    p = is_lattice(segment.get_end())
                #    #assert(p)
                #    reachable[(start, end)] = segment
                #    print "Found", start, end
                #estimate = [l_est / 8.0, w_est, 3.0 * l_est / 8.0]
                ##estimate = [0, l_est / 8.0, w_est, 3.0 * l_est / 8.0, 0]
                ## estimate with s-curve
                #args, info, ier, mesg = scipy.optimize.fsolve(
                #        scurve(start, end), estimate, maxfev=max_iter,
                #        full_output=True, xtol=xtol)
                #segment = S_Curve(*args)
                ##segment = LS_Curve(*args)
                #if ier == 1:
                #    p = is_lattice(segment.get_end())
                #    #assert(p)
                #    reachable[(start, end)] = segment
                #    print "Found", start, end
            else:
                if seed[1] != SAS:
                    print "WARNING: wrong seed type for", start, "->", end
                # estimate with arc
                args, info, ier, mesg = scipy.optimize.fsolve(
                        sas(start, end), estimate, maxfev=max_iter,
                        full_output=True, xtol=xtol)
                segment = SAS(start, *args)
                print "Ending score", sum(score(segment.get_end(), end))
                if ier == 1:
                    p = is_lattice(segment.get_end())
                    assert(p)
                    reachable[(start, end)] = segment
                    print "Found", start, end
                #estimate = [l_est / 4.0, w_est / 2.0, 3.0 * l_est / 4.0]
                ##estimate = [0, l_est / 4.0, w_est / 2.0, 3.0 * l_est / 4.0, 0]
                ## estimate with arc
                #args, info, ier, mesg = scipy.optimize.fsolve(
                #        sas(start, end), estimate, maxfev=max_iter,
                #        full_output=True, xtol=xtol)
                #segment = SAS(*args)
                ##segment = LSASL(*args)
                #if ier == 1:
                #    p = is_lattice(segment.get_end())
                #    #assert(p)
                #    reachable[(start, end)] = segment
                #    print "Found", start, end
                #estimate = [l_est / 4.0, -w_est / 2.0, 3.0 * l_est / 4.0]
                ##estimate = [0, l_est / 4.0, -w_est / 2.0, 3.0 * l_est / 4.0, 0]
                ## estimate with arc
                #args, info, ier, mesg = scipy.optimize.fsolve(
                #        sas(start, end), estimate, maxfev=max_iter,
                #        full_output=True, xtol=xtol)
                #segment = SAS(*args)
                ##segment = LSASL(*args)
                #if ier == 1:
                #    p = is_lattice(segment.get_end())
                #    #assert(p)
                #    reachable[(start, end)] = segment
                #    print "Found", start, end

    total_seeds = found_seeds + missed_seeds
    print "Found %d seeds out of %d primitives" % ( found_seeds, total_seeds)

    print reachable.keys()
    print min_radius
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
    #print repr(trajectories)
    print len(trajectories)

    #for p in trajectories:
    #    #print p
    #    segment = trajectories[p]
    #    #print segment

    #    cla() # clear axes
    #    segment.plot(resolution=0.02)
    #    axis('equal')
    #    show()

    if len(trajectories) > 5:
        for i in range(20):
            sample = {}
            for p in trajectories:
                end = p[1]
                if end[0] == i and end[1] <= i:
                    sample[p] = trajectories[p]
                elif end[0] < i and end[1] == i:
                    sample[p] = trajectories[p]
            if len(sample) > 0:
                for p in sample:
                    sample[p].plot(resolution=0.02)
                axis('equal')
                print i, len(sample)
                show()

    if len(trajectories) > 0:
        for p in trajectories:
            #print p
            segment = trajectories[p]
            #print segment

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
