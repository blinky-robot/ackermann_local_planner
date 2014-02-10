#!/usr/bin/env python

import sys
import mprim

import math
import numpy 
import scipy.optimize
from primitives import *

from pylab import *

def normalize(angle):
    while angle > 16:
        angle = angle - 16
    while angle < 0:
        angle = angle + 16
    return angle

# mirror about the X axis
def mirror_x(p):
    return (p[0], -p[1], normalize(-p[2]))

# mirror about the Y axis
def mirror_y(p):
    return (-p[0], p[1], normalize(8-p[2]))

# mirror about x=y
def mirror_xy(p):
    return (p[1], p[0], normalize(4 - p[2]))

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

def expand_trajectories(traj):
    # mirror angle 0 primitives about X
    traj_0 = list(traj[0])
    for t in traj[0]:
        # transform will return None if the primitive was unmodified
        m = t.transform(mirror_x)
        if m:
            traj_0.append(m)
    traj[0] = traj_0

    # mirror angle 2 trajectories about x=y
    traj_2 = list(traj[2])
    for t in traj[2]:
        m = t.transform(mirror_xy)
        if m:
            traj_2.append(m)
    traj[2] = traj_2

    # mirror angle 1 primitives about x=y
    traj[3] = []
    for t in traj[1]:
        m = t.transform(mirror_xy)
        assert(m)
        if m:
            traj[3].append(m)

    # rotate and mirror primitives about the origin
    traj[4] = [ m.transform(mirror_xy) for m in traj[0] ]
    for i in [ 5, 6, 7, 8 ]:
        traj[i] = [ m.transform(mirror_y) for m in traj[8 - i] ]
    for i in range(9,16):
        traj[i] = [ m.transform(mirror_x) for m in traj[16 - i] ]

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

def index(p, num_angles):
    """ Get the index numers for a given point """
    x = int(round(p[0]))
    y = int(round(p[1]))
    t = int(round(p[2] * num_angles / ( math.pi * 2 )))
    return (x, y, t)

def trajectory_to_mprim(start, end, trajectory, num_poses, num_angles):
    st = index(start, num_angles)
    en = index(end, num_angles)
    poses = list(trajectory.get_poses(n=num_poses-1))
    poses.append(end)
    return mprim.MPrim(st, en, poses)

def generate_trajectories(min_radius, num_angles):
    reachable = {}
    tolerance = 0.01 # tolerance for matching to the grid
    print "Minimum radius", min_radius

    def LSASL(start, l1, l2, w, l3, l4):
        l2 = max(l2, 0.00000001)
        l2 = min(l2, 20)
        #w = 1 / (2 * l2 * radius )
        w_max = 1 / (l2 * min_radius)
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
        w_max = 1 / (l1 * min_radius)
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
        w_max = 1 / (l2 * min_radius)
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
        w_max = 1 / (l1 * min_radius)
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

    def yt_score(p, target):
        if p[0] > target[0]:
            # penalize overshooting the goal
            e1 = (p[0] - target[0])*(p[0] - target[0])*10
        else:
            # don't penalize undershooting the goal
            e1 = 0
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

    def sas(start, end):
        def err(args):
            return yt_score(SAS(start, *args).get_end(), end)
        return err

    def scurve(start, end):
        def err(args):
            return yt_score(S_Curve(start, *args).get_end(), end)
        return err

    #primitives = {
    #    0: [ (4, 1, 1), (5, 1, 1), (6, 1, 1), (6, 2, 2),
    #         (5, 1, 0), (8, 2, 0) ],
    #    1: [ (3, 2, 1), (4, 1, -1), (4, 4, 2), (6, 0, -2),
    #         (3, 2, 0), (4, 1, 0),  (4, 4, 0), (6, 0, 0) ],
    #    2: [ (2, 3, 1), (2, 5, 2),
    #         (2, 3, 0), (2, 5, 0) ]
    #    }
    #primitives = {
    #    0: [ (4, 1, 1), (5, 1, 1), (6, 1, 1), (7, 1, 1), (8, 1, 1), (9, 1, 1),
    #         (4, -1, -1), (8, -1, -1), (9, -1, -1),
    #         (5, 2, 2), (6, 2, 2), (7, 2, 2), (8, 2, 2), (9, 2, 2),
    #         (10, 1, 1), (11, 1, 1)],
    #    }
    primitives = {}
    for start_angle in range(3):
        primitives[start_angle] = []
        for x in range(8):
            for y in range(x+1):
                if x ==0 and y == 0:
                    continue
                for angle in [ -2, -1, 0, 1, 2 ]:
                    primitives[start_angle].append((x, y, angle,))

    max_iter = 10000000
    xtol = 0.001 * 0.001 * 3 * 0.01
    print "xtol", xtol

    for start_angle in primitives:
        start = (0, 0, 2 * math.pi * start_angle / num_angles , 0)
        for end_pose in primitives[start_angle]:
            end_angle = start_angle + end_pose[2]
            end = (end_pose[0], end_pose[1], 2.0 * math.pi * end_angle / \
                    num_angles, 0)

            # Normalize to starting angle 0,
            #  then optimize for delta-y and delta-theta
            #  then add a linear section to match the desired delta-x
            d_theta = end_pose[2] * 2.0 * math.pi / num_angles
            hypotenuse = math.sqrt( end_pose[0]*end_pose[0] +
                                    end_pose[1]*end_pose[1] )
            angle = math.atan2( end_pose[1], end_pose[0] ) - start[2]
            d_x = math.cos(angle) * hypotenuse
            d_y = math.sin(angle) * hypotenuse

            normal_start = (0, 0, 0, 0)
            normal_end = (d_x, d_y, d_theta, 0)

            if d_theta > 0:
                estimate = [ 0.25, 0.5, 2.5 ]
            else:
                estimate = [ 0.25, -0.5, 2.5 ]

            if abs(d_theta) < 0.0001 and abs(d_y) < 0.0001:
                t = Linear
                ier = 1
                mesg = "Avoid optimizer and use linear solution"
                args = [ d_x ]
            else:
                if d_theta == 0:
                    # estimate with s-curve
                    f = scurve(normal_start, normal_end)
                    t = S_Curve
                else:
                    # estimate with arc
                    f = sas(normal_start, normal_end)
                    t = SAS

                args, info, ier, mesg = scipy.optimize.fsolve( f, estimate,
                        maxfev=max_iter, full_output=True, xtol=xtol)

            if ier == 1:
                segment = t(normal_start, *args)
                remaining_x = d_x - segment.get_end()[0]
                if remaining_x < 0:
                    # bad solution. just toss it
                    #print "REJECT: solution overshoots x"
                    continue
                if segment.get_length() > 2 * hypotenuse:
                    #print "REJECT: solution too long"
                    continue
                s1 = Linear(start, remaining_x)
                s2 = t(s1.get_end(), *args)
                segment = Compound(s1, s2)
                #print "Ending score", score(segment.get_end(), end)
                reachable[(start, end)] = segment
                #print "Found", start, end
                #print args
                if len(args) == 3:
                    l1 = args[0] / hypotenuse
                    w = args[1]
                    l2 = args[2] / hypotenuse
                    print l1, w, l2

    #print reachable.keys()
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
    parser.add_argument('-p', '--plot', action="store_true",
                        help="Plot optimized trajectories")

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
    print len(trajectories), "base trajectories"

    # convert trajectories into a starting-angle-indexed map, similar to 
    #  how the primitives are laid out
    traj = {}
    for t in trajectories:
        i = index(t[0], args.num_angles)[2]
        if not i in traj:
            traj[i] = []
        traj[i].append(trajectory_to_mprim(t[0], t[1], trajectories[t], 10,
            args.num_angles))

    expand_trajectories(traj)

    if args.plot:
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
    
    print sum(len(traj[t]) for t in traj) , "total trajectories"

    if not args.output:
        import yaml
        print yaml.dump(mprim)
    else:
        mprim.write_mprim(args.output, traj, args.resolution)

if __name__ == '__main__':
    main()
