#!/usr/bin/python

import mprim
import argparse

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
        default=5)

    args = parser.parse_args()

    primitives = mprim.read_mprim(args.file)

    print primitives

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
                    print "Start pose", start
                    print "End pose", end
                    print

                    min_x = min(min_x, end[0])
                    max_x = max(max_x, end[0])

                    min_y = min(min_y, end[1])
                    max_y = max(max_y, end[1])

                    if not end in space:
                        new_space[end] = i+1

        print new_space

        for s in new_space:
            space[s] = new_space[s]

    print space

if __name__ == '__main__':
    main()
