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

class Segment(object):
    """ A partial path segment, with the relevant interfaces """
    # Create with a given start pose and parameters
    def __init__(self):
        self._length = 0
        self._end = (0, 0, 0, 0) # degenerate case: a point

    # Get the end pose (x, y, theta and angular velocity)
    def get_end(self):
        return self._end

    # Get a pose length from this segment's starting point
    #  this is used as the generator for final poses and
    #  inside of get_poses()
    def get_pose(self, length):
        raise NotImplementedError()

    # Get N intermediate poses
    def get_poses(self, n=None, resolution=None):
        # determine n to achive the desired resolution
        #  minimum resolution
        assert(n is not None or resolution is not None)
        if n is None:
            n = self._length / resolution
            n = math.ceil(n)
        r = self._length / n
        if resolution < r:
            assert( self._length / resolution > n )
            n = self._length / resolution
            n = math.ceil(n)
        for i in range(n):
            yield self.get_pose(i * resolution)
        # explicitly don't yield the end. It the user wants it, they
        #  can call get_end()

class Linear(Segment):
    def __init__(self, start, length):
        assert(start[3] == 0) # angular velocity of start must be 0
        self._length = length
        self._start = tuple(start)
        self._end = self.get_pose(length)


    def get_pose(self, length):
        pose = (0, 0, self._start[2], 0 )
        pose = self._start[0] + length * cos(self._start[2])
        pose = self._start[1] + length * sin(self._start[2])

class Arc(Segment):
    def __init__(self, start, length):
        assert(start[3] != 0)
        self._length = length
        self._start = tuple(start)
        self._end = self.get_pose(length)

    def get_pose(self, length):
        angular_velocity = self._start[3]
        velocity = 1
        time = length
        pose = ( 0, 0, 0, self._start[3] )
        pose[2] = self._start[2] + angular_velocity * time
        pose[0] = self._start[0] + sin( self._start[2] + \
                                        time * angular_velocity ) \
                                   / posengular_velocity
        pose[1] = self._start[1] - cos( self._start[2] + \
                                        time * angular_velocity ) \
                                   / angular_velocity
        return pose


class Spiral(Segment):
    def __init__(self, start, length, w):
        self._start = tuple(start)
        self._length = length
        self._w = w
        self._end = self.get_pose(length)

    def get_pose(self, length):
        time = length
        pose = ( 0, 0, 0, 0 )
        pose[3] = self._start[3] + self._w * time
        pose[2] = self._start[2] + self._w * time * time / 2

        S, C = fresnel(math.sqrt( self._w / math.pi ) * time)

        pi_w = math.sqrt( math.pi / self._w )

        t0 = self._start[2]

        dx = pi_w * ( sin( t0 ) * C + cos( t0 ) * S )
        dy = pi_w * ( cos( t0 ) * C + sin( t0 ) * S )
        pose[0] = self._start[0] + dx
        pose[1] = self._start[1] + dy

        return pose



def generate_trajectories(min_radius, num_angles):
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
    # TODO: rewrite this as a string of Segment classes
    #  which can be either linear, easment or circular segments
    for t1 in numpy.arange(0.01, 10.0, 0.01):
        for t2 in numpy.arange(t1 + 0.01, 10.0, 0.01):
            #print "(%f, %f)" % ( t1, t2 )
            dt = t2 - t1
            w1_max = 1 / (t1 * min_radius)
            w2_max = 1 / (dt * min_radius)
            #print "t1, t2: %f, %f" % ( t1, t2 )
            #print "w1, w2 max: %f, %f" % ( w1_max, w2_max )
            # don't consider 0 angle change yet, when we aren't computing
            # s-curves
            for angle in range(1, num_angles):
                angle_rad = angle * (math.pi * 2.0) / num_angles
                w1 = angle_rad / ( t1*t1 + 0.5*t1*t2 )
                w2 = -1 * w1 * t1 / ( dt )

                if abs(w1) > w1_max:
                    #print "w1 = %f too big for (t1, t2, theta) = (%f, %f, %f)" \
                    #        % ( w1, t1, t2, angle_rad )
                    continue
                if abs(w2) > w2_max:
                    #print "w2 = %f too big for (t1, t2, theta) = (%f, %f, %f)" \
                    #        % ( w2, t1, t2, angle_rad )
                    continue

                #print "t1: %f" % t1
                #print "t2: %f" % t2
                #print "w1: %f" % w1
                #print "w2: %f" % w2
                #print "theta: %f" % angle_rad

            # TODO: should be able to compute w1 and w2 for desired d-theta
            #  or a range of d-theta values
            #for w1 in numpy.arange(0, w1_max, w1_max / 100):
            #    w2 = w1 * t1 / (t2 - t1)
                a1 = math.sqrt(w1 / math.pi)
                s1, c1 = fresnel( t1 * a1 )
                x1, y1 = c1 / a1, s1 / a1
                o1 = w1 * t1 * t1 / 2

                # this doesn't take into account the starting angle;
                #  pretty sure that it's wrong
                a2 = math.sqrt(abs(w2 / dt)) # FIXME: shouldn't need abs here
                s2, c2 = fresnel( (t2 - t1) * a2 )
                x2, y2 = - c2 / a2, -s2 / a2
                o2 = - w2 * dt * dt / 2

                x2, y2 = x2 + x1, y2 + y1
                o2 = o1 + o2
                o2_pi = o2 * num_angles / ( math.pi * 2.0 ) 
                if round(x2, 2) == round(x2, 0) and \
                   round(y2, 2) == round(y2, 0) and \
                   round(o2_pi, 2) == round(o2_pi, 0):
                    # index into our reachability array
                    i = ( round(x2),round(y2),round(o2_pi)/num_angles )
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

    generate_trajectories(args.min_radius / args.resolution, args.num_angles)

    expand_primitives(primitives)

    prim = generate_mprim(primitives)

    if not args.output:
        import yaml
        print yaml.dump(mprim)
    else:
        mprim.write_mprim(args.output, prim, args.resolution)

if __name__ == '__main__':
    main()
