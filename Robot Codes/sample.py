#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, GyroSensor, UltrasonicSensor, ColorSensor
from pybricks.parameters import Port, Direction, Button
from pybricks.tools import wait
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import ImageFile

ev3 = EV3Brick()
left_motor = Motor(Port.A, Direction.CLOCKWISE)
right_motor = Motor(Port.B, Direction.CLOCKWISE)
#medium_motor = Motor(Port.C, Direction.CLOCKWISE)
ultraSonicSensor = UltrasonicSensor(Port.S4)
colorSensor1 = ColorSensor(Port.S3)

WHEEL_DIAMETER = 56
AXLE_TRACK = 200
robot = DriveBase(left_motor, right_motor, WHEEL_DIAMETER, AXLE_TRACK)
gyro_sensor = GyroSensor(Port.S2)
steering = 60
overshoot = 5
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
        avoid_obstacle()
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
    Motor(Port.C, Direction.CLOCKWISE).run_time(100, time*1000)
    ev3.speaker.beep()
    robot.stop()
def hands_go_down(time):
    Motor(Port.C, Direction.CLOCKWISE).run_time(-100, time*1000)
    ev3.speaker.beep()
    robot.stop()
def identify_color():
    while True :
        color = colorSensor1.color()
        reflection = colorSensor1.reflection()
        print(color, '\t', reflection, '\n')
        wait(100)
def read_barcode():
    my_Barcode = [[0,1,1,0],[1,1,0,0],[0,0,1,1],[1,0,0,1]]
    scanned_Barcode = [[1,4,4,4],[3,3,3,3],[2,2,2,2],[4,4,4,4]]
    print(scanned_Barcode)
    for x in range (0,3) :
        print(scanned_Barcode)
        for y in range (1,3) :
            print(scanned_Barcode)
            while True :
                color = colorSensor1.color()
                if color == None and (color!= Color.Black or color != Color.White) :
                    color = colorSensor1.color()
                else:
                    if color == Color.Black: 
                        myNum = 1
                    elif color == Color.White:
                        myNum = 0
                    scanned_Barcode[x][y] = myNum
                    print(scanned_Barcode)
                    forward(0.5)
                    break
        forward(0.5)    
    

read_barcode()

robot.stop()
