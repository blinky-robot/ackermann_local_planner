#!/usr/bin/env python

import math

class MPrim():
    def __init__(self, start, end, poses, cost=1, resolution=1.0):
        self.start = start
        self.end = end
        self.poses = poses
        self.cost = cost
        self.resolution = resolution

    def __str__(self):
        return """MPrim:
  Start Angle: %s
  End Pose: %s
  Poses:
%s"""%(str(self.start), str(self.end), "\n".join(self.poses))

    def __repr__(self):
        return "MPrim(start=%s, end=%s)"%(str(self.start), str(self.end))

    def outformat(self, res=0.1):
        s = """startangle_c: %d
endpose_c: %d %d %d
additionalactioncostmult: %d
intermediateposes: %d
"""%(self.start[2], self.end[0], self.end[1], self.end[2],
     self.cost, len(self.poses))
        for pose in self.poses:
            s += "%0.4f %0.4f %0.4f\n"%(pose[0]*res, pose[1]*res, pose[2])
        return s

    def transform(self, transform, max_angle):
        """ Return a NEW copy of this MPrim as transformed by transform """
        start = transform(self.start, max_angle)
        end = transform(self.end, max_angle)
        if start == self.start and end == self.end:
            return None
        cost = self.cost
        poses = []
        for pose in self.poses:
            poses.append(transform(pose, math.pi * 2))
        return MPrim(start, end, poses, cost)

    def length(self):
        """ Compute the length of this motion primitive """
        a = self.start
        l = 0
        for b in self.poses:
            dx = b[0] - a[0]
            dy = b[1] - a[1]
            l += math.sqrt(dx*dx + dy*dy)
            a = b
        dx = self.end[0] - a[0]
        dy = self.end[0] - a[0]
        l += math.sqrt(dx*dx + dy*dy)
        return l

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
                line = f.readline().split()
                pose = (float(line[0])/resolution, float(line[1])/resolution,
                        float(line[2]))
                poses.append(pose)
            primitives[startangle].append(MPrim((0, 0, startangle), endpose,
                                                poses, cost, resolution))
    return primitives

def write_mprim(file, primitives, res):
    out = """resolution_m: %0.6f
numberofangles: %d
totalnumberofprimitives: %d\n"""%(res, 16, sum([len(primitives[p]) for p in primitives]))
    for start in primitives:
        for i,p in enumerate(primitives[start]):
            out += "primID: %d\n"%(i)
            out += p.outformat(res)
    with open(file, "w") as f:
        f.write(out)

def main():
    import yaml
    import argparse
    parser = argparse.ArgumentParser("python mprim library test")
    parser.add_argument("file", help="Filename to read primitives from")
    parser.add_argument("-o", "--out", help="Output file", default=None)

    args = parser.parse_args()

    primitives = read_mprim(args.file)
    if args.out:
        write_mprim(args.out, primitives, 0.1)
    else:
        print primitives

if __name__ == '__main__':
    main()
