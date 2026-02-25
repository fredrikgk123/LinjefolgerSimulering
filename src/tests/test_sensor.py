import sys, os
sys.path.insert(0, os.path.dirname(__file__))

from track.track_generator import generate_default_track
from sensors.qtr_array import QTRArray
from physics.robot_model import Robot
from config import *
import numpy as np
from PIL import ImageFilter

track = generate_default_track()
blurred = track.filter(ImageFilter.GaussianBlur(radius=3))
blur_arr = np.array(blurred)

robot = Robot()
W, H = MAP_SIZE_M
x0 = 0.05 * W
dydx = 0.25 * H * np.cos(3 * x0 / W * 2 * np.pi) * (3 / W * 2 * np.pi)
robot.position = np.array([x0 + 0.05, 0.5 * H])
robot.theta = np.arctan2(dydx, 1.0)

sensors = QTRArray()
readings = sensors.read(robot, blur_arr)

print(f"Initial readings max: {readings.max():.3f}  mean: {readings.mean():.3f}")
print(f"Total weight: {(readings**2).sum():.4f}")
print(f"Sensor span Y: {sensors.sensor_pos_body[:,1].min()*1000:.1f} mm  to  {sensors.sensor_pos_body[:,1].max()*1000:.1f} mm")
print(f"Track width: {TRACK_WIDTH_M*1000:.0f} mm  ({TRACK_WIDTH_M * PX_PER_METER:.0f} px)")
print(f"Sensors over track approx: {int(TRACK_WIDTH_M / QTR_SPACING_M)}")
print(f"Robot pos: {robot.position}  theta: {np.degrees(robot.theta):.1f} deg")

