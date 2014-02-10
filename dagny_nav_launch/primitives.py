#!/usr/bin/env python

import math
import cmath
import numpy 
from scipy.special import fresnel

from pylab import *

class Segment(object):
    """ A partial path segment, with the relevant interfaces """
    # Create with a given start pose and parameters
    def __init__(self):
        self._length = 0
        self._end = (0, 0, 0, 0) # degenerate case: a point
        self._reference = None

    # Get the end pose (x, y, theta and angular velocity)
    def get_end(self):
        assert(self._end)
        return self._end

    def get_length(self):
        return self._length

    def get_score(self, reference):
        if self._reference and reference == self._reference:
            return self._score
        else:
            diff = ( reference[0] - self._end[0],
                     reference[1] - self._end[1],
                     reference[2] - self._end[2],
                     reference[3] - self._end[3] )
            self._score = diff[0] * diff[0] + diff[1] * diff[1] + \
                          diff[2] * diff[2] + diff[3] * diff[3]
            return self._score 

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
        if n == 0:
            return
        r = self._length / n
        if resolution and resolution < r:
            if self._length / resolution > n:
                print self._length, resolution, n
                assert(False)
            n = self._length / resolution
        n = int(round(n + 0.5))
        if n > 1:
            n -= 1
        resolution = self._length / n
        for i in range(n):
            yield self.get_pose(i * resolution)
        # explicitly don't yield the end. It the user wants it, they
        #  can call get_end()

    def plot(self, resolution):
        X = []
        Y = []
        for pose in self.get_poses(resolution=resolution):
            X.append(pose[0])
            Y.append(pose[1])
        X.append(self._end[0])
        Y.append(self._end[1])
        plot(X, Y)

class Compound(Segment):
    def __init__(self, *segments):
        Segment.__init__(self)
        self._segments = segments
        self._start = (0, 0, 0, 0)
        self._end   = segments[-1].get_end()
        self._length = sum( s.get_length() for s in segments )

    def __repr__(self):
        return "Compound(%s)" %( ", ".join(map(repr, self._segments)) )
    
    def get_pose(self, length):
        i = 0
        base = 0
        l = self._segments[i].get_length()
        while length > l:
            i += 1
            if i >= len(self._segments):
                return self._end
            base = l
            l += self._segments[i].get_length()
        pose = self._segments[i].get_pose(length - base)
        return pose

    def plot(self, resolution):
        for s in self._segments:
            s.plot(resolution)
        

class Linear(Segment):
    def __init__(self, start, length):
        Segment.__init__(self)
        assert(round(start[3], 4) == 0) # angular velocity of start must be 0
        self._length = length
        self._start = tuple(start)
        self._end = self.get_pose(length)
        assert(self._end)

    def get_pose(self, length):
        x = self._start[0] + length * math.cos(self._start[2])
        y = self._start[1] + length * math.sin(self._start[2])
        return (x, y, self._start[2], 0)

    def __repr__(self):
        return "Linear(%s, %f)" % (repr(self._start), self._length)

    def __str__(self):
        return "Linear(%s, %f)->(%s)" % (repr(self._start), self._length,
                                         repr(self._end))

class Arc(Segment):
    def __init__(self, start, length):
        Segment.__init__(self)
        assert(start[3] != 0)
        self._length = length
        self._start = tuple(start)
        self._end = self.get_pose(length)
        assert(self._end)

    def get_pose(self, length):
        angular_velocity = self._start[3]
        velocity = 1
        time = length
        pose = ( 0, 0, 0, self._start[3] )
        t = self._start[2] + angular_velocity * time

        # calculations for x and y here are incorrect
        x = self._start[0] + ( math.sin( self._start[2] + \
                                         (time * angular_velocity) ) - \
                               math.sin( self._start[2] ) )\
                             / angular_velocity

        y = self._start[1] - ( math.cos( self._start[2] + \
                                         (time * angular_velocity) ) - \
                               math.cos( self._start[2]) )\
                             / angular_velocity
        return (x, y, t, self._start[3])

    def __repr__(self):
        return "Arc(%s, %f)" % (repr(self._start), self._length)

    def __str__(self):
        return "Arc(%s, %f)->(%s)" % (repr(self._start), self._length,
                                         repr(self._end))


class Spiral(Segment):
    def __init__(self, start, length, w):
        Segment.__init__(self)
        assert(w != 0) 
        self._start = tuple(start)
        self._length = length
        self._w = w
        self._end = self.get_pose(length)
        assert(self._end)

    def get_pose(self, length):
        # This implements:
        # dx = http://www.wolframalpha.com/input/?i=integral+of+cos%28+w+*+0.5+*+t%5E2+%2B+q*t+%2B+theta+%29+dt+from+0+to+x
        # dy = 
        time = length

        omega0 = self._start[3]
        theta0 = self._start[2]

        omega = omega0 + self._w * time
        theta = theta0 + self._w * time * time / 2 + omega0 * time

        w = self._w

        f1 = omega0 / cmath.sqrt( math.pi * w )
        S1, C1 = fresnel( f1 )

        f2 = ( omega0 + w * time ) / cmath.sqrt( math.pi * w )
        S2, C2 = fresnel( f2 )

        pi_w = cmath.sqrt( math.pi / self._w )

        t0 = ( omega0 * omega0 / (2 * self._w ) ) - theta0

        dx = pi_w * ( math.cos( t0 ) * (C2 - C1) + math.sin( t0 ) * (S2 - S1) )
        dy = pi_w * ( math.sin( t0 ) * (C1 - C2) + math.cos( t0 ) * (S2 - S1) )
        imag = False
        if dx.imag != 0:
            print "WARNING: imaginary result for dx:", dx
            imag = True
        if dy.imag != 0:
            print "WARNING: imaginary result for dy:", dy
            imag = True
        if imag:
            print "  With w", w, ", length", self._length, "at", length
        dx = dx.real
        dy = dy.real

        # not quite sure why... but this works
        if w < 0:
            dx = -dx
            dy = -dy

        x = self._start[0] + dx
        y = self._start[1] + dy

        return (x, y, theta, omega)

    def __repr__(self):
        return "Spiral(%s, %f, %f)" % (repr(self._start), self._length, self._w)

    def __str__(self):
        return "Spiral(%s, %f, %f)->(%s)" % (repr(self._start), self._length,
                                         self._w, repr(self._end))

if __name__ == '__main__':
    # test plots
    start = (0, 0, 0, 0)
    
    def test_plot(segment):
        print segment
        segment.plot(resolution=0.02)
        axis('equal')

    def test_end(segment, end):
        assert(len(end) == 4)
        e = segment.get_end()
        err = [0]*4 
        assert(len(e) == 4)
        for i in range(4):
            err[i] = abs(end[i] - e[i])
        if max(err) > 0.000001:
            print "ERROR: endpoint for", segment, "(", segment.get_end(), ")", \
                "does not match", end, "(", err, ")"
        else:
            print "Endpoint test passed ", err

    def test_show():
        show()
        cla()
    # lines

    test_end(Linear(start, 5),
             (5, 0, 0, 0))
    test_end(Spiral(start, 5,  0.1),
             (4.2732691420089255, 1.8620681128161767, 1.25, 0.5))
    test_end(Spiral(start, 5, -0.1),
             (4.2732691420089255, -1.8620681128161767, -1.25, -0.5))

    test_plot(Linear(start, 5))
    test_plot(Spiral(start, 5,  0.1))
    test_plot(Spiral(start, 5, -0.1))
    test_show()

    arc_start = (0, 0, 0, 0.5)

    test_end(Arc(arc_start, 5),
             (1.1969442882079129, 3.602287231093867, 2.5, 0.5))
    test_end(Spiral(arc_start, 5, 0.1),
             (-0.055657624721361917, 2.5445328635419475, 3.75, 1.0))
    test_end(Spiral(arc_start, 5, -0.1),
             (3.1145313202640916, 3.4681149738592167, 1.25, 0))

    test_plot(Arc(arc_start, 5))
    test_plot(Spiral(arc_start, 5, 0.1))
    test_plot(Spiral(arc_start, 5, -0.1))
    test_show()


    angle_start = (0, 0, math.pi / 4.0, 0)

    test_plot(Linear(angle_start, 5))
    test_plot(Spiral(angle_start, 5,  0.1))
    test_plot(Spiral(angle_start, 5, -0.1))
    test_show()

    angle_arc_start = (0, 0, math.pi / 4.0, 0.5)

    test_plot(Arc(angle_arc_start, 5))
    test_plot(Spiral(angle_arc_start, 5,  0.1))
    test_plot(Spiral(angle_arc_start, 5, -0.1))
    test_show()

    angle_arc_start2 = (0, 0, math.pi / 4.0, -0.5)

    test_plot(Arc(angle_arc_start2, 5))
    test_plot(Spiral(angle_arc_start2, 5,  0.1))
    test_plot(Spiral(angle_arc_start2, 5, -0.1))
    test_show()

