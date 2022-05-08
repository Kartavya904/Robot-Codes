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
WHEEL_DIAMETER = 56
AXLE_TRACK = 200
robot = DriveBase(left_motor, right_motor, WHEEL_DIAMETER, AXLE_TRACK)
gyro_sensor = GyroSensor(Port.S2)
colorSensor1 = ColorSensor(Port.S3)
steering = 60
overshoot = 3
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
def barcode_turn_right_angle(degrees, cf):
    gyro_sensor.reset_angle(0)
    robot.drive(0, 20)
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
def barcode_turn_left_angle(degree,cf):
    gyro_sensor.reset_angle(0)
    robot.drive(0, -20)
    while gyro_sensor.angle() > overshoot - degree*cf:
        wait(1)
    robot.drive(0, 0)
    ev3.speaker.beep()
    wait(100)
def forward(n):
    robot.stop()
    Motor(Port.B, Direction.CLOCKWISE).run_time(170,0.175)
    robot.stop()
    if n< 50: 
        distance = 25.4*n*1.015
    else :
        distance = 25.4*n*1.0279
    robot.reset()
    while robot.distance() <= distance:
        robot.drive(100,0)
        wait(10)
    ev3.speaker.beep()
    robot.stop()
def barcode_forward(n):
    robot.stop()
    Motor(Port.B, Direction.CLOCKWISE).run_time(170,0.175)
    robot.stop()
    if n< 50: 
        distance = 25.4*n*1.015
    else :
        distance = 25.4*n*1.0279
    robot.reset()
    while robot.distance() <= distance:
        robot.drive(57.5,0)
        wait(10)
    ev3.speaker.beep()
    robot.stop()
def barcode_forward2(n):
    robot.stop()
    Motor(Port.B, Direction.CLOCKWISE).run_time(170,0.175)
    robot.stop()
    if n< 50: 
        distance = 25.4*n*1.015
    else :
        distance = 25.4*n*1.0279
    robot.reset()
    while robot.distance() <= distance:
        robot.drive(40,0)
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
def identify_color():
    while True :
        color = colorSensor1.color()
        reflection = colorSensor1.reflection()
        ambient = colorSensor1.ambient()
        print(color, '\t', reflection, '\t', ambient, '\n')
        wait(100)
def read_barcode():
    global my_Barcode
    my_Barcode = [1,0,0,1]
    global scanned_Barcode
    scanned_Barcode = [2,2,2,2]
    equalYesNo = True
    print(scanned_Barcode)
    for x in range (0,4) :
        while True :
            color = colorSensor1.color()
            ambient = colorSensor1.ambient()
            reflection = colorSensor1.reflection()
            while( color == None) :
                color = colorSensor1.color()
                ambient = colorSensor1.ambient()                    
                reflection = colorSensor1.reflection()
            else:
                if ambient < 3 and reflection < 10: 
                    myNum = 1
                else : 
                    myNum = 0
                scanned_Barcode[x] = myNum
                print(scanned_Barcode)
                barcode_forward(0.1)
                wait(1000)
                break
    print(scanned_Barcode)
    for myX in range (0,4):
        if my_Barcode[myX] != scanned_Barcode[myX] : 
            equalYesNo = False
            break
    # equalYesNo = True
    if equalYesNo :
        myTrial = 0
        # Print Barcode On Screen Along with Right Barcode
    else :
        ev3.speaker.beep()
        wait(100)
        ev3.speaker.beep()
        wait(100)
        ev3.speaker.beep()
        wait(100)
        myTrial = 1 
        # Print The Barcode On Screeb Along with Wrong Barcode


B = 5
N = "C1"
D = "C"
forward(12)
if B < 7:
    if N == "B1" or N == "A1":
        y = 3.5
    elif N== "A2" or N == "B2":
        y = 27.5
    elif N== "C1" or N == "D1":
        y = 51.5 
    elif N== "C2" or N == "D2":
        y = 75.5
    forward(y)
    barcode_turn_right_angle(87,1.05)
    if B == 1:
        x2 = 6.9375
    elif B == 2:
        x2 = 12.9375
    elif B == 3:
        x2 = 18.9375
    elif B == 4:
        x2 = 24.9375
    elif B == 5:
        x2 = 30.9375
    elif B == 6:
        x2 = 36.9375
    if N == "B1" or N == "B2" or N == "D1" or N == "D2":
        x2 = x2+48
    forward(x2)
    while ((colorSensor1.color() == None) or not(colorSensor1.reflection() < 10 )):
        barcode_forward2(0.125)
        robot.stop()
        wait(750)
    read_barcode()
    barcode = "Box Type: 2 Identified!!" 
    left_motor.run_time(-300, 1.575*1000)
    robot.stop()
    forward(4)
    robot.stop()
    hands_go_up(5)
    robot.stop()
    reverse(6.5)
    robot.stop()
    if D == "B":
        barcode_turn_right_angle(90, 1.05)
        x3 = 96-x2
        forward(x3)
        barcode_turn_right_angle(90, 1.05)
        forward(y+12)
        hands_go_down(4.9575)
        robot.stop()
        reverse(4.25)
        barcode_turn_left_angle(180,0.979)
        robot.stop()
        forward(7.75)
        robot.stop()
        wait(1000)
        barcode_turn_left_angle(90,0.97)
        robot.stop()
        wait(1000)
        forward(96)
        robot.stop()
        wait(1000)
        barcode_turn_left_angle(90,0.97)
        robot.stop()
        wait(1000)
        forward(12)
        robot.stop()
        wait(1000)
        robot.stop()
        robot.stop()
    elif D == "C":
        barcode_turn_left_angle(90,0.979)
        forward(x2)
        barcode_turn_right_angle(90, 1.05)
        forward(96-y)
        forward(12)
        hands_go_down(4.9575)
        robot.stop()
        reverse(4.25)
        barcode_turn_left_angle(180,0.979)
        robot.stop()
        forward(115.75)
    elif D == "D" :
        barcode_turn_right_angle(90, 1.05)
        x3 = 96-x2
        forward(x3)
        barcode_turn_left_angle(90, 0.979)
        forward(96-y)
        forward(12)
        hands_go_down(4.9575)
        robot.stop()
        reverse(4.25)
        barcode_turn_left_angle(180,0.979)
        robot.stop()
        forward(103.75)
        robot.stop()
        wait(1000)
        barcode_turn_right_angle(90,1.05)
        robot.stop()
        wait(1000)
        forward(96)
        robot.stop()
        wait(1000)
        barcode_turn_left_angle(90,0.97)
        robot.stop()
        wait(1000)
        forward(12)
        robot.stop()
        wait(1000)
        robot.stop()
        robot.stop()        
else:
    barcode_turn_right_angle(90, 1.05)
    if N == "B1" or N == "B2" or N == "D1" or N == "D2":
        x = 96
    else:
        x = 48
    forward(x)
    barcode_turn_left_angle(90,0.979)
    if N == "B1" or N == "A1":
        forward(21.5)
    elif N== "A2" or N == "B2":
        forward(45.5)
    elif N== "C1" or N == "D1":
        forward(69.5)
    elif N== "C2" or N == "D2":
        forward(93.5)
    barcode_turn_left_angle(90,0.979)
    if B == 12:
        x2 = 6.9375
    elif B == 11:
        x2 = 12.9375
    elif B == 10:
        x2 = 18.9375
    elif B == 9:
        x2 = 24.9375
    elif B == 8:
        x2 = 30.9375
    elif B == 7:
        x2 = 36.9375
    forward(x2) 
    while ((colorSensor1.color() == None) or not(colorSensor1.ambient() < 3 and colorSensor1.reflection() < 10 )):
        barcode_forward2(0.125)
        robot.stop()
        wait(750)
    read_barcode()
    robot.stop()
    wait(5000)
    left_motor.run_time(-300, 1.575*1000)
    robot.stop()
    forward(4)
    robot.stop()
    hands_go_up(5)
    robot.stop()
    reverse(6.5)
    robot.stop()
    if N == "B1" or N == "B2" or N == "D1" or N == "D2":
        x2 = x2 + 48
    if D == "B":
        barcode_turn_left_angle(90, 0.979)
        forward(x2)
        barcode_turn_right_angle(90, 1.05)
        forward(y+12)
        hands_go_down(4.9575)
        robot.stop()
        reverse(4.25)
        barcode_turn_left_angle(180,0.979)
        robot.stop()
        forward(7.75)
        robot.stop()
        wait(1000)
        barcode_turn_left_angle(90,0.97)
        robot.stop()
        wait(1000)
        forward(96)
        robot.stop()
        wait(1000)
        barcode_turn_left_angle(90,0.97)
        robot.stop()
        wait(1000)
        forward(12)
        robot.stop()
        wait(1000)
        robot.stop()
        robot.stop()
    elif D == "C":
        barcode_turn_right_angle(90,1.05)
        forward(96-x2)
        barcode_turn_right_angle(90, 1.05)
        forward(96-y)
        forward(12)
        hands_go_down(4.9575)
        robot.stop()
        reverse(4.25)
        barcode_turn_left_angle(180,0.979)
        robot.stop()
        forward(115.75)
    elif D == "D" :
        barcode_turn_left_angle(90, 0.979)
        forward(x2)
        barcode_turn_left_angle(90, 0.979)
        forward(96-y)
        forward(12)
        hands_go_down(4.9575)
        robot.stop()
        reverse(4.25)
        barcode_turn_left_angle(180,0.979)
        robot.stop()
        forward(103.75)
        robot.stop()
        wait(1000)
        barcode_turn_right_angle(90,1.05)
        robot.stop()
        wait(1000)
        forward(96)
        robot.stop()
        wait(1000)
        barcode_turn_left_angle(90,0.97)
        robot.stop()
        wait(1000)
        forward(12)
        robot.stop()
        wait(1000)
        robot.stop()
        robot.stop()     


















