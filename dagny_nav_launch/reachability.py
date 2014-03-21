#!/usr/bin/python

import sys
import math
import mprim
import argparse
from PIL import Image, ImageDraw
from dubins import sample_dubins_path as dubin
from angles import *

def sum_pose(a, b):
    endangle = b[2]
    if endangle < 0:
        endangle += 16
    if endangle > 15:
        endangle -= 16
    return (a[0]+b[0], a[1]+b[1], endangle)


def green_red(v):
    green = (0, 255, 0)
    red = (255, 0, 0)
    yellow = (255, 255, 0)
    if v < 0.5:
        g = v * 2
        r = 1.0
    else:
        r = (1 - v) * 2
        g = 1.0
    return (int(255*r), int(255*g), 0)


def main():
    parser = argparse.ArgumentParser("Reachability analysis for SBPL motion primitives")
    parser.add_argument("file", help="Motion primitive file")
    parser.add_argument("-i", "--iterations", help="Number of iterations to do",
        default=0, type=int)
    parser.add_argument("-o", "--outfile", help="Output File",
        default="out")
    parser.add_argument('-a', '--all', help="All angles", action="store_true",
        default=False)
    parser.add_argument('-s', '--start', help="Start Angle", default=0,
        type=int)
    parser.add_argument('-r', '--range', help="Maximum range", default=0,
            type=int)
    parser.add_argument('-g', '--grids', action='store_true',
                        help="Render reachability grids")
    parser.add_argument('-p', '--paths', action='store_true',
                        help="Render paths")
    parser.add_argument('--score', action='store_true',
                        help="Score final trajectories")

    args = parser.parse_args()

    # TODO: read num_angles from file
    num_angles = 16
    primitives = mprim.read_mprim(args.file)

    # The space is currently the number of steps and total path length to a
    # point
    space = {(0,0,args.start): (0, 0)}
    if args.all:
        for i in range(16):
            space[(0, 0, i)] = (0, 0)

    min_x = 0
    max_x = 0
    min_y = 0
    max_y = 0

    paths = []

    if args.iterations == 0:
        if args.range == 0:
            print "ERROR: must specify range or iterations"
            sys.exit(1)
        #args.iterations = args.range * 2

    old_len = len(space)
    for i in range(args.iterations):
        new_space = {}
        for start in space:
            if space[start][0] == i:
                for p in primitives[start[2]]:
                    end = sum_pose(start, p.end)

                    if args.range > 0:
                        if abs(end[0]) > args.range:
                            continue
                        if abs(end[1]) > args.range:
                            continue

                    min_x = min(min_x, end[0])
                    max_x = max(max_x, end[0])

                    min_y = min(min_y, end[1])
                    max_y = max(max_y, end[1])

                    path = [(start[0] + q[0], start[1] + q[1]) 
                        for q in p.poses]
                    paths.append(path)
                    if not end in space:
                        new_space[end] = (i+1, space[start][1] + p.length())

        for s in new_space:
            space[s] = new_space[s]
        if len(space) == old_len:
            print "Didn't find any new points after %d iterations. Done!" % i
            break
        old_len = len(space)


    if args.range > 0:
        min_x = max(min_x, -args.range)
        max_x = min(max_x,  args.range)
        min_y = max(min_y, -args.range)
        max_y = min(max_y,  args.range)

    print "Found %d final poses"%(len(space))
    print
    print "Limits"
    print " X:", (min_x, max_x)
    print " Y:", (min_y, max_y)
    width = max_x - min_x
    height = max_y - min_y
    print

    # do some numeric analysis on the results:
    #  maximum number of angles we can reach at a point
    #  minimum number of angles we can reach at a point

    # endpoints will be ( number of primitives that end at xy, minimum
    #  iterations to reach xy ) indexed by ending position
    endpoints = {}
    # coverage_count will collect the number of points for each count of
    #  reachable positions. ie coverage_count[1] = 12 means that there are
    #  12 points where only 1 angle is reachable
    coverage_count = {}
    max_count = 0
    min_count = 1000 # fixme

    for p in space:
        value = space[p][0]
        xy = (p[0], p[1])
        count = 1
        if xy in endpoints:
            count = endpoints[xy][0] + 1
            value = min(value, endpoints[xy][1])
        endpoints[xy] = (count, value)

    for x in range(min_x, max_x+1):
        for y in range(min_y, max_y+1):
            xy = (x, y)
            if xy in endpoints:
                count = endpoints[xy][0]
            else:
                count = 0
                endpoints[xy] = (0, -1)
            max_count = max(count, max_count)
            min_count = min(count, min_count)
            if not count in coverage_count:
                coverage_count[count] = 0
            coverage_count[count] += 1
            
    print "Max count", max_count
    print "Min count", min_count

    spaces = (width+1)*(height+1)
    if 0 in coverage_count and coverage_count[0] > 0:
        print "%d points are totally unreachable" % ( coverage_count[0] )

    print "Coverage data:"
    for count in coverage_count:
        print "%d angles reachable at %d points" % ( count,
                                                     coverage_count[count] )

    if args.score:
        # Idea for metric: distance to travel to reach a point vs linear
        #  distance vs dubin's path?
        # Evaluate the efficiency of the lattice primitives by comparing them
        # to Dubin's path
        d_ratios = []
        #for p in space:
        for p in [ (x, y, t) for x in range(min_x, max_x+1) 
                  for y in range(min_y, max_y+1)
                  for t in range(num_angles) ]:
            # Compute dubin's path to P with minimum radius 6
            end = (p[0], p[1], angle_from_index(p[2], 16))
            d = dubin((0,0,0), end, 6.0, 0.1)
            # sum length of Dubin's path
            l = 0
            a = (0,0,0)
            for b in d[0]:
                dx = b[0] - a[0]
                dy = b[1] - a[1]
                l += math.sqrt(dx*dx + dy*dy)
                a = b
            dx = p[0] - a[0]
            dy = p[1] - a[1]
            l += math.sqrt(dx*dx + dy*dy)
            #print "Length to", p, "is", space[p][1]
            #print "Dubins length to", p, "is", l
            if p in space:
                if space[p][1] != 0:
                    d_ratios.append(l / space[p][1])
            else:
                # we get a score of 0 for missing a goal state
                d_ratios.append(0.0)
        # this is flawed because our motion primitives allow us to move
        # backwards, which can result in a SHORTER path than Dubins. This tends
        # to throw off the metric a little
        #  - for now, simply don't include reverse paths when analyzing
        #    primitives
        # A perfect score would be 1.0; lesser scores indicate how suboptimal
        #  our set of motion primitives is
        # the score is: optimality * coverage
        print "Dubins path score", (sum(d_ratios) / len(d_ratios))

    if args.grids:
        # reachability grids represent the number of iterations required to
        #  reach a given target angle (img_A.png) for every point
        print "Rendering reachability grids"
        print (width, height)
        print

        im = [ Image.new("RGB", (width+1, height+1)) for i in range(16) ]

        for p in space:
            xy = (p[0]-min_x, p[1]-min_y)
            color = im[p[2]].getpixel(xy)
            v = space[p][0] * 255 / args.iterations
            color = (v, v, v)
            im[p[2]].putpixel(xy, color)

        # put a green pixel in the middle for reference, and write to disk
        for i in range(16):
            im[i].putpixel((-min_x, -min_y), (0, 255, 0))
            im[i].save("%s_%d.png"%(args.outfile, i), "PNG")

    path_scale = 64
    if args.paths:
        print "Rendering paths"
        print (width*path_scale, height*path_scale)
        print
        path_im = Image.new("RGB", (width*path_scale + 1,
                                    height*path_scale + 1),
                            (255, 255, 255))
        draw = ImageDraw.Draw(path_im)

        for path in paths:
            path = [ (p[0]-min_x, p[1]-min_y) for p in path ]
            path = [ (int(p[0]*path_scale), int(p[1]*path_scale)) for p in
                    path ]
            for start,end in zip(path[0:-1], path[1:]):
                draw.line(start + end, fill=(0, 128, 128))

        for p in endpoints:
            xy = (p[0]-min_x, p[1]-min_y)
            xy = (xy[0]*path_scale, xy[1]*path_scale)
            box = (xy[0] - 4, xy[1] - 4, xy[0] + 4, xy[1] + 4)
            #v = (args.iterations - endpoints[p][1]) * 255 / args.iterations
            #v = endpoints[p][0] * 255 / max_count
            #color = (v, v, v)
            if max_count - min_count == 0:
                color = green_red(1.0)
            else:
                color = green_red(float(endpoints[p][0] - min_count) / 
                                  (max_count - min_count) )
            draw.ellipse(box, outline=(0, 128, 128), fill=color)

        origin = ( (0-min_x)*path_scale, (0-min_y)*path_scale )
        draw.ellipse((origin[0] - 4, origin[1] - 4,
                      origin[0] + 4,
                      origin[1] + 4),
            outline=(0, 128, 128), fill=(0, 255, 0))

        path_im.save("%s_path.png"%(args.outfile), "PNG")

if __name__ == '__main__':
    main()
