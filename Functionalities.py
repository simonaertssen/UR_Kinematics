import time
import sys
import os

pi = 3.14159265359


def sleep(time_to_sleep, stop_event):
    start_time = time.time()
    while not stop_event.isSet():
        now = time.time()
        if now - start_time >= time_to_sleep:
            break
        else:
            time.sleep(0.1)


def communicateError():
    exc_type, exc_obj, exc_tb = sys.exc_info()
    file_name = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
    print(exc_type, file_name, exc_tb.tb_lineno)


def d_angle(a, b):
    return pi - abs(abs(a - b) - pi)


def toolPositionDifference(current_position, target_position):
    x1, y1, z1, xr1, yr1, zr1 = current_position
    x2, y2, z2, xr2, yr2, zr2 = target_position
    return [abs(x1 - x2), abs(y1 - y2), abs(z1 - z2), d_angle(xr1, xr2), d_angle(yr1, yr2), d_angle(zr1, zr2)]


def jointAngleDifference(current_position, target_position):
    b1, s1, e1, w11, w21, w31 = current_position
    b2, s2, e2, w12, w22, w32 = target_position
    return [d_angle(b1, b2), d_angle(s1, s2), d_angle(e1, e2), d_angle(w11, w12), d_angle(w21, w22), d_angle(w31, w32)]


def spatialDifference(current_position, target_position):
    r"""
    Compute the L2 spatial distance between two tool positions.

    Parameters:
    ----------
    current_position : list
        The current position, given by a tool position.
    target_position : list
        The target position, given by a tool position.
    """
    x1, y1, z1, _, _, _ = current_position
    x2, y2, z2, _, _, _ = target_position
    return ((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2) ** 0.5
