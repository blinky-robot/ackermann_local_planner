#!/usr/bin/env python

import sys
import mprim

import yaml
import math
import numpy 
import scipy.optimize
from primitives import *
from angles import *

from pylab import plot,axes,show


# mirror about the X axis
def mirror_x(p, max_angle):
    return (p[0], -p[1], normalize(-p[2], max_angle))

# mirror about the Y axis
def mirror_y(p, max_angle):
    return (-p[0], p[1], normalize(max_angle/2 - p[2], max_angle))

# mirror about x=y
def mirror_xy(p, max_angle):
    return (p[1], p[0], normalize(max_angle/4 - p[2], max_angle))

# mirror about x=-y
def mirror_x_y(p, max_angle):
    return (-p[1], -p[0], p[2])

# get redundant primitives
def find_redundancies(trajectories, primitives):
    # primitives is a map of (start_angle -> [MPrim, ... ])
    # primitives are the primitives input by the user
    #  a map of (start_angle -> [(x, y, d_theta), ...])

    # find the limits where we expect to look for redundancies.
    #  any paths that land outside this space will be pruned
    start_space = set()
    limits = [0, 0, primitives[0][0][0], primitives[0][0][1]]
    for start in primitives:
        start_space.add((0,0,start,))
        for p in primitives[start]:
            limits[2] = max(p[0], limits[2])
            limits[3] = max(p[1], limits[3])

    # build up our reachability space
    redundant = {}
    for start in start_space:
        space = {start: 0}
        redundant[start[2]] = set()

        open_set = set(space.keys())
        while len(open_set) > 0:
            pose = open_set.pop()
            for t in trajectories[pose[2]]:
                end = ( pose[0] + t.end[0], pose[1] + t.end[1], t.end[2] )
                if end[0] >= limits[0] and end[1] >= limits[1] and \
                        end[0] <= limits[2] and end[1] <= limits[3]:
                    if not end in space:
                        space[end] = space[pose] + 1
                        open_set.add(end)
                    else:
                        if 1 == space[end] and not end in redundant[start[2]]:
                            prim = [end[0], end[1], end[2] - start[2]]
                            if prim in primitives[start[2]]:
                                print "%d: %s is redundant" % ( start[2], prim )
                                redundant[start[2]].add(end)
    return redundant

def eliminate_redundancies(traj, redundant):
    print "Removing redundancies"
    for start in redundant:
        print "Start angle", start
        space = redundant[start]
        new_traj = []
        for prim in traj[start]:
            if isinstance(prim, mprim.MPrim):
                if prim.end not in space:
                    new_traj.append(prim)
                else:
                    print "Removed", prim.end
            else:
                prim_t = list(prim)
                prim_t[2] += start
                prim_t = tuple(prim_t)
                if prim_t not in space:
                    new_traj.append(prim)
                else:
                    print "Removed", prim_t
        print "%d old trajectories" % ( len(traj[start]) )
        print "%d new trajectories" % ( len(new_traj) )
        traj[start] = new_traj

def expand_trajectories(traj, num_angles):
    # mirror angle 0 primitives about X
    traj_0 = list(traj[0])
    for t in traj[0]:
        # transform will return None if the primitive was unmodified
        m = t.transform(mirror_x, num_angles)
        if m:
            traj_0.append(m)
    traj[0] = traj_0

    # TODO: don't hardcode angles here
    # mirror angle 2 trajectories about x=y
    traj_2 = list(traj[2])
    for t in traj[2]:
        m = t.transform(mirror_xy, num_angles)
        if m:
            traj_2.append(m)
    traj[2] = traj_2

    # TODO: don't hardcode angles here
    # mirror angle 1 primitives about x=y
    traj[3] = []
    for t in traj[1]:
        m = t.transform(mirror_xy, num_angles)
        assert(m)
        if m:
            traj[3].append(m)

    # TODO: don't hardcode angles here
    # rotate and mirror primitives about the origin
    traj[4] = [ m.transform(mirror_xy, num_angles) for m in traj[0] ]
    for i in range(num_angles/4 + 1, num_angles/2 + 1):
        traj[i] = [ m.transform(mirror_y, num_angles) for m in
                    traj[num_angles/2 - i] ]
    for i in range(num_angles/2 + 1, num_angles):
        traj[i] = [ m.transform(mirror_x, num_angles) for m in
                    traj[num_angles - i] ]

def index(p, num_angles):
    """ Get the index numers for a given point """
    x = int(round(p[0]))
    y = int(round(p[1]))
    t = index_angle(p[2], num_angles)
    return (x, y, t)

def trajectory_to_mprim(start, end, trajectory, num_poses, num_angles):
    st = index(start, num_angles)
    en = index(end, num_angles)
    poses = list(trajectory.get_poses(n=num_poses-1))
    assert(len(poses) == num_poses-1)
    poses.append(end)
    return mprim.MPrim(st, en, poses)

def generate_trajectories(min_radius, num_angles, primitives, seed):
    tolerance = 0.01 # tolerance for matching to the grid
    print "Minimum radius", min_radius
    print "Optimizing primitives..."

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
        angle = norm_angle(p[2], num_angles)
        target_angle = norm_angle(target[2], num_angles)
        angle_err = angle - target_angle
        if abs(angle_err) > 4:
            print "WARNING: high angle error", angle_err
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
        angle = norm_angle(p[2], num_angles)
        target_angle = norm_angle(target[2], num_angles)
        angle_err = norm_0(angle - target_angle, num_angles)
        if abs(angle_err) > 8:
            print "WARNING: high angle error", angle_err
        e3 = angle_err * angle_err
        return e1, e2, e3

    # is a point on our planning lattice?
    def is_lattice(p):
        # x and y error to nearest point
        e1 = abs(p[0] - round(p[0]))
        e2 = abs(p[1] - round(p[1]))
        # theta error to nearest angle
        angle = norm_angle(p[2], num_angles)
        e3 = abs(angle - round(angle))
        if e1 < tolerance and e2 < tolerance and e3 < tolerance:
            return (round(p[0]), round(p[1]), round_angle(p[2], num_angles), p[3])
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

    # if we weren't given any primitives, make some up
    if primitives is None:
        primitives = {}
        for start_angle in range(3):
            primitives[start_angle] = []
            for x in range(int(min_radius * 1.5)):
                for y in range(x+1):
                    if x ==0 and y == 0:
                        continue
                    for angle in [ -2, -1, 0, 1, 2 ]:
                        primitives[start_angle].append((x, y, angle,))

    max_iter = 10000000
    xtol = 0.001 * 0.001 * 3 * 0.01
    print "xtol", xtol

    reachable = {}

    l1 = []
    w = []
    l2 = []
    for start_angle in primitives:
        start = (0, 0, angle_from_index(start_angle, num_angles), 0)
        for end_pose in primitives[start_angle]:
            end_angle = start_angle + end_pose[2]
            end = (end_pose[0], end_pose[1], angle_from_index(end_angle,
                    num_angles), 0)

            # Normalize to starting angle 0,
            #  then optimize for delta-y and delta-theta
            #  then add a linear section to match the desired delta-x
            d_theta = angle_from_index(end_pose[2], num_angles)
            hypotenuse = math.sqrt( end_pose[0]*end_pose[0] +
                                    end_pose[1]*end_pose[1] )
            angle = math.atan2( end_pose[1], end_pose[0] ) - start[2]
            d_x = math.cos(angle) * hypotenuse
            d_y = math.sin(angle) * hypotenuse

            normal_start = (0, 0, 0, 0)
            normal_end = (d_x, d_y, d_theta, 0)

            estimate = list(seed)
            if d_theta <= 0:
                estimate[1] = -estimate[1]

            if abs(d_theta) < 0.0001 and abs(d_y) < 0.0001:
                t = Linear
                ier = 1
                mesg = "Avoid optimizer and use linear solution"
                args = [ d_x ]
            else:
                # better estimate for w?
                #estimate[1] = 0.1 * (d_y/d_x)

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
                if len(args) == 3 and d_theta == 0:
                    l1.append(args[0] / hypotenuse)
                    w.append( args[1] / (d_x/d_y))
                    l2.append(args[2] / hypotenuse)

    print "Average seed values:"
    print min(l1),         min(w),        min(l2)
    print sum(l1)/len(l1), sum(w)/len(w), sum(l2)/len(l2)
    print max(l1),         max(w),        max(l2)
    print max(l1)/min(l1), max(w)/min(w), max(l2)/min(l2)

    return reachable

def main():
    import argparse
    parser = argparse.ArgumentParser('Motion primitive generation')
    parser.add_argument('-o', '--output', 
                        help="Output file")
    parser.add_argument('-r', '--resolution', default=0.1, type=float,
                        help="Primitive resolution (in meters)")
    parser.add_argument('-m', '--min-radius', default=0.6, type=float,
                        help="Minimum radius (in meters)")
    parser.add_argument('-p', '--plot', action="store_true",
                        help="Plot optimized trajectories")
    parser.add_argument('-y', '--yaml',
                        help="YAML file to load configuration from")
    parser.add_argument('-d', '--dump-yaml',
                        help="File to dump generated YAML configuration to")
    parser.add_argument('--prune', action='store_true', default=False,
                        help="Prune redundant paths")

    args = parser.parse_args()

    num_angles = 16
    primitives = None
    seed = [ 0.25, 0.5, 2.5 ]
    if args.yaml:
        config = yaml.load(open(args.yaml))
        if 'primitives' in config:
            print "Loaded primitives from %s" % ( args.yaml )
            primitives = config['primitives']
        if 'seed' in config:
            print "Loaded seed from %s" % ( args.yaml )
            seed = config['seed']
        if 'num_angles' in config:
            print "Loaded num_angles from %s" % ( args.yaml )
            num_angles = config['num_angles']

    trajectories = generate_trajectories(args.min_radius / args.resolution,
                                num_angles, primitives, seed)
    print len(trajectories), "base trajectories"

    # convert trajectories into a starting-angle-indexed map, similar to 
    #  how the primitives are laid out
    traj = {}
    for t in trajectories:
        i = index(t[0], num_angles)[2]
        if not i in traj:
            traj[i] = []
        traj[i].append(trajectory_to_mprim(t[0], t[1], trajectories[t], 10,
            num_angles))

    # if we were asked to find specific primitives, report which ones
    #  couldn't be found
    if primitives:
        found = {}
        for i in traj:
            found[i] = [ [t.end[0], t.end[1], 
                norm_0(t.end[2] - i, num_angles)] for t in traj[i] ]
        for i in primitives:
            for p in primitives[i]:
                if not p in found[i]:
                    print "Failed to find solution for primitive %d -> %s" % \
                            ( i, str(p) )

    primitives = {}
    for i in traj:
        primitives[i] = [ [p.end[0], p.end[1], 
            norm_0(p.end[2] - i, num_angles)] for p in traj[i] ]

    expand_trajectories(traj, num_angles)

    if args.prune:
        redundant = find_redundancies(traj, primitives)

        eliminate_redundancies(traj, redundant)
        eliminate_redundancies(primitives, redundant)
    
    if args.dump_yaml:
        print primitives
        config = { 'primitives': primitives,
                   'seed': seed,
                   'num_angles': num_angles }
        with open(args.dump_yaml, 'w') as out:
            out.write(yaml.dump(config))
        print "Wrote config to %s" % ( args.dump_yaml )
        return

    total = sum(len(traj[t]) for t in traj)
    max_branch = max(len(traj[t]) for t in traj)
    min_branch = min(len(traj[t]) for t in traj)
    print total, "total trajectories"
    if max_branch != min_branch:
        print "==================================="
        print " >>  Branching factor analysis  << "
        print "Average branching factor:", float(total)/num_angles
        print "Maximum branching factor:", max_branch
        print "Minimum branching factor:", min_branch
        for i in range(1 + num_angles / 8):
            print "Angle %d, branching factor %d" % ( i, len(traj[i]) )
        if args.plot:
            for i in range(1 + num_angles / 8):
                print "Plotting angle %d" % ( i )
                for p in trajectories:
                    if index(p[0], num_angles)[2] == i:
                        trajectories[p].plot(resolution=0.02)
                axis('equal')
                show()
    else:
        print "Branching factor:", float(total)/num_angles
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



    if args.output:
        mprim.write_mprim(args.output, traj, args.resolution)

if __name__ == '__main__':
    # simple test for index_angle
    #n = 16
    #ix = []
    #norm = []
    #index = []
    #angles = []
    #rounded = []
    #out = []

    #for i in range(n*4):
    #    angle = (math.pi * 2) * i / (n*4)
    #    ix.append(i)
    #    angles.append(angle)
    #    norm.append(norm_angle(angle, n))
    #    index.append(index_angle(angle, n))
    #    rounded.append(round_angle(angle, n))
    #    out.append(angle_from_index(i/4, n))
    #plot(ix, norm)
    #plot(ix, index)
    #show()
    #plot(ix, angles)
    #plot(ix, rounded)
    #plot(ix, out)
    #show()
    main()
