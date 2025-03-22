import sys
import csv
from controller import Robot, Motor
import time


time.sleep(10)

TIME_STEP = 32

robot = Robot()

motors = [
    robot.getDevice('panda_joint1'),
    robot.getDevice('panda_joint2'),
    robot.getDevice('panda_joint3'),
    robot.getDevice('panda_joint4'),
    robot.getDevice('panda_joint5'),
    robot.getDevice('panda_joint6'),
    robot.getDevice('panda_joint7'),
    robot.getDevice('panda_finger::right')
]


def load_joint_angles(csv_file):
  joint_angles = []
  with open(csv_file, newline='') as file:
    reader = csv.reader(file)
    for row in reader:
      try:
        joint_angles.append([float(angle) for angle in row])
      except ValueError:
        print(f'Skipped row: {row}')
        continue
  return joint_angles


joint_angles = load_joint_angles(sys.argv[1])

for angles in joint_angles:
  print(angles)
  for motor, angle in zip(motors, angles):
    motor.setPosition(angle)
  robot.step(4 * TIME_STEP)
