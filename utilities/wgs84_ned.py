#!/usr/bin/env python3

import pymap3d as pm


def wgs84ToNed( lat, lon, alt, lat_ref, lon_ref, alt_ref):
    """Convert WGS84 coordinates to NED frame"""
    print(type(lat_ref))
    x_enu , y_enu, z_enu = pm.geodetic2enu(lat, lon, alt, lat_ref, lon_ref, alt_ref)

    # ENU to NED: Flip the Z axis
    x_ned = y_enu
    y_ned = x_enu
    z_ned = -z_enu

    return x_ned, y_ned, z_ned