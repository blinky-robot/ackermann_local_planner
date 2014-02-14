#!/usr/bin/env python

import math

def normalize(angle, max_angle):
    """ normalize an angle to the range 0 to max_angle """
    while angle >= max_angle:
        angle = angle - max_angle 
    while angle < 0:
        angle = angle + max_angle
    return angle

def norm_0(angle, max_angle):
    """ normalize an angle to the range -max_angle/2 to max_angle/2 """
    while angle >= max_angle/2:
        angle = angle - max_angle 
    while angle < -max_angle/2:
        angle = angle + max_angle
    return angle

EQUAL_ANGLES = 1
GRID_ANGLES = 2
#ANGLE_TYPE = EQUAL_ANGLES
ANGLE_TYPE = GRID_ANGLES

def norm_angle(angle, num_angles):
    """
    normalize the given angle to the floating-point range: 0.0 to num_angles
    """
    if ANGLE_TYPE == EQUAL_ANGLES:
        # Assuming angles are evenly distributed
        n1 = angle * num_angles / ( math.pi * 2 )
        #n1 = normalize(n1, num_angles)
        return n1
    elif ANGLE_TYPE == GRID_ANGLES:
        # Assuming angles snap to the nearest endpoint
        #  assume num_angles = 16
        s = math.sin(angle)
        c = math.cos(angle)
        if abs(s) > abs(c):
            # normalize sine(y) to 1
            if s == 0:
                # We should never hit this if abs(s) > abs(c)
                assert(False)
                norm = 1.0
            else:
                norm = abs(1.0 / s)
            plus = norm * c * (num_angles / 8)
            # upper or lower triangle
            #  base 1(upper) or 3(lower)
            if s > 0:
                base = 1
                plus = -plus
            elif s < 0:
                base = 3
            else:
                # ??
                assert(False)
        else:
            # normalize cos(x) to 1
            if c == 0:
                # ??. we should never get here, but we do?
                assert(False)
            else:
                norm = abs(1.0 / c)
            plus = norm * s * (num_angles / 8)
            # left or right triangle
            #  base 0(right) or 2(left)
            if c > 0:
                base = 0
            elif c < 0:
                base = 2
                plus = -plus
            else:
                # ?? 
                assert(False)
        base = base * (num_angles / 4)
        n2 = normalize(base + plus, num_angles)
        return n2
    else:
        # neither angle type. die
        assert(False)

def index_angle(angle, num_angles):
    """ get the nearest index for a given angle """
    n1 = round(norm_angle(angle, num_angles))
    n1 = normalize(n1, num_angles)
    return int(n1)

def round_angle(angle, num_angles):
    """ round an angle to the nearest lattice angle """
    if ANGLE_TYPE == EQUAL_ANGLES:
        # Assuming angles are evenly distributed
        n1 = round( angle * num_angles / ( math.pi * 2 ))
        r1 = normalize(n1 * math.pi * 2 / num_angles, math.pi * 2)
        return r1
    elif ANGLE_TYPE == GRID_ANGLES:
        # Assuming angles snap to the nearest endpoint
        #  assume num_angles = 16
        s = math.sin(angle)
        c = math.cos(angle)
        if abs(s) > abs(c):
            # normalize sine(y) to 1
            if s == 0:
                assert(False)
            else:
                norm = abs(1.0 / s)
            x = round(norm * c * (num_angles / 8)) / (num_angles / 8)
            # upper or lower triangle
            if s > 0:
                y = 1
            elif s < 0:
                y = -1
            else:
                # ??
                assert(False)
        else:
            # normalize cos(x) to 1
            if c == 0:
                assert(False)
            else:
                norm = abs(1.0 / c)
            y = round(norm * s * (num_angles / 8)) / (num_angles / 8)
            # left or right triangle
            if c > 0:
                x = 1
            elif c < 0:
                x = -1
            else:
                # ?? 
                assert(False)
        r2 = normalize(math.atan2(y, x), math.pi * 2)
        return r2
    else:
        # neither angle type. die
        assert(False)

def angle_from_index(i, num_angles):
    if ANGLE_TYPE == EQUAL_ANGLES:
        return i * math.pi * 2 / num_angles
    elif ANGLE_TYPE == GRID_ANGLES:
        ia = normalize(i, num_angles)
        # base:
        # \  |  /
        #  \ 1 /
        #   \|/
        # -2-+-0-
        #   /|\
        #  / 3 \
        # /  |  \
        base = (ia + (num_angles/8)) / (num_angles/4)

        if base == 0:
            step = ia
            x = 1.0
            y = float(step) / (num_angles/8)
        elif base == 1:
            step = (num_angles/4) - ia
            y = 1.0
            x = float(step) / (num_angles/8)
        elif base == 2:
            step = (num_angles/2) - ia
            x = -1.0
            y = float(step) / (num_angles/8)
        elif base == 3:
            step = ia - (3*num_angles/4)
            y = -1.0
            x = float(step) / (num_angles/8)
        elif base == 4:
            step = ia - num_angles
            x = 1.0
            y = float(step) / (num_angles/8)
        else:
            print i, ia, base
            assert(False)
        angle = normalize(math.atan2(y, x), 2 * math.pi)
        if i >= 0:
            return angle
        else:
            return angle - (2 * math.pi)
    else:
        # neither angle type. die
        assert(False)

