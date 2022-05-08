#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, GyroSensor, UltrasonicSensor
from pybricks.parameters import Port, Direction, Button
from pybricks.tools import wait
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import ImageFile

ev3 = EV3Brick()
left_motor = Motor(Port.A, Direction.CLOCKWISE)
right_motor = Motor(Port.B, Direction.CLOCKWISE)
#medium_motor = Motor(Port.C, Direction.CLOCKWISE)
ultraSonicSensor = UltrasonicSensor(Port.S4)
WHEEL_DIAMETER = 56
AXLE_TRACK = 200
robot = DriveBase(left_motor, right_motor, WHEEL_DIAMETER, AXLE_TRACK)
gyro_sensor = GyroSensor(Port.S2)
steering = 60
overshoot = 11
def avoid_obstacle():
    x = 0
    while x < 6 :
        while ultraSonicSensor.distance() <= 120 : 
            print(ultraSonicSensor.distance())                 
            wait(10)
            reverse(5)
            turn_left_angle(90,1)
            avoid_obstacle()
        x += 1
def turn_right_angle(degrees, cf):
    gyro_sensor.reset_angle(0)
    robot.drive(0, steering)
    while gyro_sensor.angle() < (degrees*cf) - overshoot:
        wait(1)
    robot.drive(0, 0)
    ev3.speaker.beep()
    wait(100)
def turn_left_angle(degree,cf):
    gyro_sensor.reset_angle(0)
    robot.drive(0, -steering)
    while gyro_sensor.angle() > overshoot - degree*cf:
        wait(1)
    robot.drive(0, 0)
    ev3.speaker.beep()
    wait(100)
def forward(n):
    distance = 25.4*n
    robot.reset()
    while robot.distance() <= distance:
        robot.drive(100,0)
        wait(10)
    ev3.speaker.beep()
    robot.stop()
def reverse(n):
    distance = 25.4*n
    robot.reset()
    while -robot.distance() <= distance:
        robot.drive(-100,0)
        wait(10)
    ev3.speaker.beep()
    robot.stop()
def hands_go_up(time):
    Motor(Port.C, Direction.CLOCKWISE).run_time(300, time*1000)
    ev3.speaker.beep()
    robot.stop()
def hands_go_down(time):
    Motor(Port.C, Direction.CLOCKWISE).run_time(-300, time*1000)
    ev3.speaker.beep()
    robot.stop()


# #avoid_obstacle()
forward(4)
turn_left_angle(90,0.97)
#turn_right_angle(90,1.03)
forward(4)
turn_right_angle(90,1.03)
forward(4)
reverse(1)
#hands_go_down(1.625)
forward(2)
hands_go_up(4.5)
reverse(1)
turn_left_angle(180,0.97)
forward(8)
turn_left_angle(90,0.97)
forward(4)
turn_left_angle(90,0.97)
hands_go_down(4.5)
robot.stop()

#turn_left_angle(90,0.97) #Works
#turn_right_angle(90,1.03) #Works
# hands_go_up(4.5)
# hands_go_down(4.5)

robot.stop()
