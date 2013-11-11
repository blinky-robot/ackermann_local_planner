#!/usr/bin/python

import mprim
import argparse
from PIL import Image, ImageDraw

def sum(a, b):
    endangle = b[2]
    if endangle < 0:
        endangle += 16
    if endangle > 15:
        endangle -= 16
    return (a[0]+b[0], a[1]+b[1], endangle)

def main():
    parser = argparse.ArgumentParser("Reachability analysis for SBPL motion primitives")
    parser.add_argument("file", help="Motion primitive file")
    parser.add_argument("-i", "--iterations", help="Number of iterations to do",
        default=4, type=int)
    parser.add_argument("-o", "--outfile", help="Output File",
        default="out")

    args = parser.parse_args()

    primitives = mprim.read_mprim(args.file)

    space = {(0,0,0): 0}

    min_x = 0
    max_x = 0
    min_y = 0
    max_y = 0

    paths = []

    for i in range(args.iterations):
        new_space = {}
        for start in space:
            if space[start] == i:
                for p in primitives[start[2]]:
                    end = sum(start, p.end)

                    min_x = min(min_x, end[0])
                    max_x = max(max_x, end[0])

                    min_y = min(min_y, end[1])
                    max_y = max(max_y, end[1])

                    path = [(start[0] + q[0], start[1] + q[1]) 
                        for q in p.poses]
                    paths.append(path)
                    if not end in space:
                        new_space[end] = i+1

        for s in new_space:
            space[s] = new_space[s]

    print "Found %d final poses"%(len(space))
    print
    print "Limits"
    print " X:", (min_x, max_x)
    print " Y:", (min_y, max_y)
    width = max_x - min_x
    height = max_y - min_y
    print

    print "Rendering reachability grids"
    print (width, height)
    print

    im = [ Image.new("RGB", (width+1, height+1)) for i in range(16) ]

    for p in space:
        xy = (p[0]-min_x, p[1]-min_y)
        color = im[p[2]].getpixel(xy)
        v = space[p] * 255 / args.iterations
        color = (v, v, v)
        im[p[2]].putpixel(xy, color)

    for i in range(16):
        im[i].putpixel((-min_x, -min_y), (0, 255, 0))
        im[i].save("%s_%d.png"%(args.outfile, i), "PNG")

    path_scale = 64
    print "Rendering paths"
    print (width*path_scale, height*path_scale)
    print
    path_im = Image.new("RGB", (width*path_scale + 1, height*path_scale + 1),
        (255, 255, 255))
    draw = ImageDraw.Draw(path_im)

    for path in paths:
        path = [ (p[0]-min_x, p[1]-min_y) for p in path ]
        path = [ (int(p[0]*path_scale), int(p[1]*path_scale)) for p in path ]
        for start,end in zip(path[0:-1], path[1:]):
            draw.line(start + end, fill=(0, 128, 128))

    endpoints = {}
    max_count = 0

    for p in space:
        value = space[p]
        xy = (p[0], p[1])
        count = 1
        if xy in endpoints:
            count = endpoints[xy][0] + 1
            value = min(value, endpoints[xy][1])
        endpoints[xy] = (count, value)
        max_count = max(count, max_count)
            
    print "Max count", max_count

    for p in endpoints:
        xy = (p[0]-min_x, p[1]-min_y)
        xy = (xy[0]*path_scale, xy[1]*path_scale)
        box = (xy[0] - 4, xy[1] - 4, xy[0] + 4, xy[1] + 4)
        #v = (args.iterations - endpoints[p][1]) * 255 / args.iterations
        v = endpoints[p][0] * 255 / max_count
        color = (v, v, v)
        draw.ellipse(box, outline=(0, 128, 128), fill=color)

    origin = ( (0-min_x)*path_scale, (0-min_y)*path_scale )
    draw.ellipse((origin[0] - 4, origin[1] - 4, origin[0] + 4, origin[1] + 4),
        outline=(0, 128, 128), fill=(0, 255, 0))

    path_im.save("%s_path.png"%(args.outfile), "PNG")

if __name__ == '__main__':
    main()
