#!/usr/bin/env python

class MPrim():
    def __init__(self, start, end, poses):
        self.start = start
        self.end = end
        self.poses = poses

def read_int(f):
    return int(f.readline().split()[1])

def read_mprim(file):
    primitives = {}
    with open(file) as f:
        resolution = float(f.readline().split()[1])
        angles = read_int(f)
        total = read_int(f)
        for i in range(total):
            id = read_int(f)
            startangle = read_int(f)
            if not startangle in primitives:
                primitives[startangle] = []
            endpose = tuple(int(a) for a in f.readline().split()[1:])
            cost = read_int(f)
            pose_cnt = read_int(f)
            poses = []
            for j in range(pose_cnt):
                pose = tuple(float(a)/resolution for a in f.readline().split())
                poses.append(pose)
            primitives[startangle].append(MPrim((0, 0, startangle), endpose, poses))
    return primitives

def write_mprim(file):
    print "ERROR: write mprim not implemented"

def main():
    import yaml
    import argparse
    parser = argparse.ArgumentParser("python mprim library test")
    parser.add_argument("file", help="Filename to read primitives from")

    args = parser.parse_args()

    print yaml.dump(read_mprim(args.file))
    pass

if __name__ == '__main__':
    main()
