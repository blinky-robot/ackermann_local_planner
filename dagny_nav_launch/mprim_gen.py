#!/usr/bin/env python

import mprim

def expand_primitives(prim):
    # mirror angle 0 primitives about X
    prim_0 = list(prim[0])
    for p in prim[0]:
        if p[2] != 0:
            prim_0.append((p[0], -p[1], -p[2]))
    prim[0] = prim_0
    # mirror angle 2 primitives about x=y
    prim_2 = list(prim[2])
    for p in prim[2]:
        if p[2] != 0:
            prim_2.append((p[1], p[0], -p[2]))
    prim[2] = prim_2
    # mirror angle 1 primitives about x=y
    prim[3] = []
    for p in prim[1]:
        prim[3].append((p[1], p[0], -p[2]))


def main():
    # primitives should explicitly allow differing numbers of primitives
    # per start angle
    # we should also have reflection/rotation functions to generate the
    # remainder of the primitives from the initial primitives
    primitives = {
        0: [ (1, 0, 0), (4, 1, 1), (5, 2, 2) ],
        1: [ (2, 1, 0), (3, 2, 1), (4, 1, -1), (4, 4, 2), (6, 0, -2) ],
        2: [ (1, 1, 0), (2, 3, 1), (2, 5, 2) ]
        }

    expand_primitives(primitives)

    print primitives

if __name__ == '__main__':
    main()
