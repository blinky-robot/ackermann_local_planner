#!/usr/bin/python

import mprim
import argparse
from PIL import Image

def sum(a, b):
    endangle = a[2] + b[2]
    if endangle < 0:
        endangle += 16
    if endangle > 15:
        endangle -= 16
    return (a[0]+b[0], a[1]+b[1], endangle)

def main():
    parser = argparse.ArgumentParser("Reachability analysis for SBPL motion primitives")
    parser.add_argument("file", help="Motion primitive file")
    parser.add_argument("-i", "--iterations", help="Number of iterations to do",
        default=5, type=int)
    parser.add_argument("-o", "--outfile", help="Output File",
        default="out")

    args = parser.parse_args()

    primitives = mprim.read_mprim(args.file)

    space = {(0,0,0): 0}

    min_x = 0
    max_x = 0
    min_y = 0
    max_y = 0

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

                    if not end in space:
                        new_space[end] = i+1

        for s in new_space:
            space[s] = new_space[s]

    print "X:", (min_x, max_x)
    print "Y:", (min_y, max_y)
    width = max_x - min_x + 1
    height = max_y - min_y + 1
    print (width, height)
    print

    im = [ Image.new("RGB", (width, height)) for i in range(16) ]

    for p in space:
        xy = (p[0]-min_x, p[1]-min_y)
        color = im[p[2]].getpixel(xy)
        v = space[p] * 255 / args.iterations
        color = (v, v, v)
        im[p[2]].putpixel(xy, color)

    for i in range(16):
        im[i].putpixel((-min_x, -min_y), (0, 255, 0))
        im[i].save("%s_%d.png"%(args.outfile, i), "PNG")


if __name__ == '__main__':
    main()
