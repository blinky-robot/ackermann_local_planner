#!/usr/bin/env python

import sys
import mprim

import math
import numpy 
from scipy.special import fresnel

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

def generate_trajectories(min_radius):
    reachable = {
            (0, 0 ,0): ( (0, 0, 0), ),
            (1, 0, 0): ( (1, 0, 0), ),
            (2, 0, 0): ( (2, 0, 0), ),
            (3, 0, 0): ( (3, 0, 0), ),
            (4, 0, 0): ( (4, 0, 0), ),
            (5, 0, 0): ( (5, 0, 0), ),
            (6, 0, 0): ( (6, 0, 0), ),
            (7, 0, 0): ( (7, 0, 0), ),
            (8, 0, 0): ( (8, 0, 0), ),
            (9, 0, 0): ( (9, 0, 0), ),
            (10,0, 0): ( (10,0, 0), ),
            }
    print reachable
    angles = 4.0
    for t1 in numpy.arange(0.01, 10.0, 0.01):
        for t2 in numpy.arange(t1 + 0.01, 10.0, 0.01):
            #print "(%f, %f)" % ( t1, t2 )
            w1_max = 1 / (t1 * min_radius)
            # TODO: should be able to compute w1 and w2 for desired d-theta
            #  or a range of d-theta values
            for w1 in numpy.arange(0, w1_max, w1_max / 100):
                w2 = w1 * t1 / (t2 - t1)
                a1 = math.sqrt(w1 / math.pi)
                s1, c1 = fresnel( t1 * a1 )
                x1, y1 = c1 / a1, s1 / a1
                o1 = w1 * t1 * t1 / 2

                dt = t2 - t1
                a2 = math.sqrt(w2 / dt)
                s2, c2 = fresnel( (t2 - t1) * a2 )
                x2, y2 = - c2 / a2, -s2 / a2
                o2 = - w2 * dt * dt / 2

                x2, y2 = x2 + x1, y2 + y1
                o2 = o1 + o2
                o2_pi = o2 * angles / math.pi
                if round(x2, 2) == round(x2, 0) and \
                   round(y2, 2) == round(y2, 0) and \
                   round(o2_pi, 2) == round(o2_pi, 0):
                    # index into our reachability array
                    i = ( round(x2),round(y2),round(o2_pi)/angles )
                    # final position and control values
                    d = ( (x2, y2, o2_pi), (t1, t2, w1) )
                    if not i in reachable:
                        reachable[i] = d
                    else:
                        def err(p1, p2):
                            # TODO: this doesn't weight distance and angle equally 
                            return (p1[0] - p2[0]) * (p1[0] - p2[0]) + \
                                   (p1[1] - p2[1]) * (p1[1] - p2[1]) + \
                                   (p1[2] - p2[2]) * (p1[2] - p2[2]) 

                        i1 = d[0]
                        i2 = reachable[i][0]
                        if err(i, i1) < err(i, i2):
                            reachable[i] = d
                            print "(%d, %d, %f)" % i
    print reachable.keys()
    print reachable

def main():
    import argparse
    parser = argparse.ArgumentParser('Motion primitive generation')
    parser.add_argument('-o', '--output', help="Output file")

    args = parser.parse_args()

    # primitives should explicitly allow differing numbers of primitives
    # per start angle
    # we should also have reflection/rotation functions to generate the
    # remainder of the primitives from the initial primitives
    primitives = {
        0: [ (1, 0, 0), (4, 1, 1), (5, 2, 2) ],
        1: [ (2, 1, 0), (3, 2, 1), (4, 1, -1), (4, 4, 2), (6, 0, -2) ],
        2: [ (1, 1, 0), (2, 3, 1), (2, 5, 2) ]
        }

    generate_trajectories(3)

    expand_primitives(primitives)

    prim = generate_mprim(primitives)

    if not args.output:
        import yaml
        print yaml.dump(mprim)
    else:
        mprim.write_mprim(args.output, prim, 0.1)

if __name__ == '__main__':
    main()
