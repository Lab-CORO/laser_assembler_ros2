#!/usr/bin/env python3
"""
Methods to numpify LaserScan message.
"""

import numpy as np
from sensor_msgs.msg import LaserScan



def laserscan_to_array(
    scan, remove_invalid_ranges=False, include_ranges_and_intensities=False
):
    """
    Takes a sensor_msgs/msg/LaserScan msg and returns a structered array with
    fields x, y and z that correspond to cartesian position data. Optionally,
    ranges and intensities fields that correspond to the range and intensity
    of a point are also included if include_ranges_and_intensities is True.

    Parameters
    ----------
    scan : ROS2 LaserScan message
        Input laser scan message to get numpyed
    remove_invalid_ranges : bool, optional
        whether to remove invalid ranges from the input scan, by default False
    include_ranges_and_intensities : bool, optional
        whether to also return the ranges & intensities along with the cartesian
        position.

    Returns
    -------
    pts : numpy k-array where each element will be a structured record that
        contains either 3 or 5 fields. If include_ranges_and_intensities is
        False, each element of  output array is a structured record that has
        3 fields ['x', 'y', 'z'] of type float 32. Else, it has 5 fields ['x',
        'y', 'z', 'ranges', 'intensities']. Since output is a structured array,
        all the x-coordinates of the points can be accessed as out_array['x'].
        Similarly, the y and z coodinates can be accessed as out_array['y']
        and out_array['z'] respectively.
    """
    n_points = len(scan.ranges)
    angles = np.linspace(
        scan.angle_min,
        scan.angle_max,
        n_points,
    )
    ranges = np.array(scan.ranges, dtype="f4")
    intensities = np.array(scan.intensities, dtype="f4")
    if remove_invalid_ranges:
        indices_invalid_range = (
            np.isinf(ranges)
            | np.isnan(ranges)
            | (ranges < scan.range_min)
            | (ranges > scan.range_max)
        )
        ranges = ranges[~indices_invalid_range]
        angles = angles[~indices_invalid_range]
        intensities = intensities[~indices_invalid_range]

    x = np.array(ranges * np.cos(angles), dtype="f4")
    y = np.array(ranges * np.sin(angles), dtype="f4")
    z = np.zeros(ranges.shape[0], dtype="f4")
    if include_ranges_and_intensities:
        dtype = np.dtype(
            [
                ("x", "f4"),
                ("y", "f4"),
                ("z", "f4"),
                ("ranges", "f4"),
                ("intensities", "f4"),
            ]
        )
        out_array = np.empty(len(x), dtype=dtype)
        out_array["x"] = x
        out_array["y"] = y
        out_array["z"] = z
        out_array["ranges"] = ranges
        out_array["intensities"] = intensities
        return out_array
    else:
        dtype = np.dtype([("x", "f4"), ("y", "f4"), ("z", "f4")])
        out_array = np.empty(len(x), dtype=dtype)
        out_array["x"] = x
        out_array["y"] = y
        out_array["z"] = z
        xyz_array = out_array.view(np.float32).reshape(-1, 3)
        return xyz_array