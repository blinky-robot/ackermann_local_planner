#!/usr/bin/env python

class MPrim():
    def __init__(self, start, end, poses, cost=1):
        self.start = start
        self.end = end
        self.poses = poses
        self.cost = cost

    def __str__(self):
        return """MPrim:
  Start Angle: %s
  End Pose: %s
  Poses:
%s"""%(str(self.start), str(self.end), "\n".join(self.poses))

    def __repr__(self):
      return "[MPrim: start: %s, end: %s]"%(str(self.start), str(self.end))

    def outformat(self, res=0.1):
      s = """startangle_c: %d
endpose_c: %d %d %d
additionalactioncostmult: %d
intermediateposes: %d
"""%(self.start[2], self.end[0], self.end[1], self.end[2],
    self.cost, len(self.poses))
      for pose in self.poses:
          s += "%0.4f %0.4f %0.4f\n"%(pose[0]*res, pose[1]*res, pose[2]*res)
      return s

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
            primitives[startangle].append(MPrim((0, 0, startangle), endpose, poses, cost))
    return primitives

def write_mprim(file, primitives, res):
    out = """resolution_m: %0.6f
numberofangles: %d
totalnumberofprimitives: %d\n"""%(res, 16, sum([len(primitives[p]) for p in primitives]))
    for start in primitives:
        for i,p in enumerate(primitives[start]):
            out += "primID: %d\n"%(i)
            out += p.outformat(res)
    print out

def main():
    import yaml
    import argparse
    parser = argparse.ArgumentParser("python mprim library test")
    parser.add_argument("file", help="Filename to read primitives from")

    args = parser.parse_args()

    primitives = read_mprim(args.file)
    write_mprim("/tmp/foo.mprim", primitives, 0.1)
    pass

if __name__ == '__main__':
    main()
