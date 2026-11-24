"""Vehicle-Dynamics-inspired true planar body + four steerable tire polygons.

Pose refers to rear axle midpoint. Dimensions and rotation share real SI units.
"""
import math
from .geometry import Pose,Vehicle,polygon

def planar_parts(pose:Pose,vehicle:Vehicle,front_steer:float=0.,rear_steer:float=0.,
                 track_m:float=1.60,tire_length_m:float=.60,tire_width_m:float=.225):
    shift=vehicle.length/2-vehicle.rear_axle_from_center
    cx=pose.x+shift*math.cos(pose.yaw)
    cy=pose.y+shift*math.sin(pose.yaw)
    body=polygon(cx,cy,pose.yaw,vehicle.length,vehicle.width)
    wheels=[]
    for axle,steer in ((vehicle.wheelbase,front_steer),(0.,rear_steer)):
        for lateral in (track_m/2,-track_m/2):
            x=pose.x+axle*math.cos(pose.yaw)-lateral*math.sin(pose.yaw)
            y=pose.y+axle*math.sin(pose.yaw)+lateral*math.cos(pose.yaw)
            wheels.append(polygon(x,y,pose.yaw+steer,tire_length_m,tire_width_m))
    return (body,*wheels)
