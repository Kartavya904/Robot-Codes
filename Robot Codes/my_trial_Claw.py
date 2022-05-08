from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, GyroSensor, UltrasonicSensor
from pybricks.parameters import Port, Direction, Button
from pybricks.tools import wait
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import ImageFile

ev3 = EV3Brick()
medium_motor = Motor(Port.C, Direction.CLOCKWISE)
medium_motor.run_time(-300, time*1000)