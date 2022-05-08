#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, GyroSensor, UltrasonicSensor, ColorSensor
from pybricks.parameters import Port, Direction, Button
from pybricks.tools import wait
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import ImageFile
from pybricks.media.ev3dev import SoundFile

ev3 = EV3Brick()
left_motor = Motor(Port.A, Direction.CLOCKWISE)
right_motor = Motor(Port.B, Direction.CLOCKWISE)
#medium_motor = Motor(Port.C, Direction.CLOCKWISE)
ultraSonicSensor = UltrasonicSensor(Port.S4)
WHEEL_DIAMETER = 56
AXLE_TRACK = 200
robot = DriveBase(left_motor, right_motor, WHEEL_DIAMETER, AXLE_TRACK)
robot.settings(straight_speed=100, straight_acceleration=30, turn_rate=0, turn_acceleration=0)
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
    while gyro_sensor.angle() < (degrees-2) - overshoot:
        wait(1)
    robot.drive(0, 0)
    ev3.speaker.beep()
    wait(100)
def barcode_turn_right_angle(degrees, cf):
    gyro_sensor.reset_angle(0)
    robot.drive(0, 20)
    while gyro_sensor.angle() < (degrees-3) - overshoot:
        wait(1)
    robot.drive(0, 0)
    ev3.speaker.beep()
    wait(100)
def turn_left_angle(degree,cf):
    gyro_sensor.reset_angle(0)
    robot.drive(0, -steering)
    while gyro_sensor.angle() > overshoot - (degree-2):
        wait(1)
    robot.drive(0, 0)
    ev3.speaker.beep()
    wait(100)
def barcode_turn_left_angle(degree,cf):
    gyro_sensor.reset_angle(0)
    robot.drive(0, -20)
    while gyro_sensor.angle() > overshoot - (degree-2):
        wait(1)
    robot.drive(0, 0)
    ev3.speaker.beep()
    wait(100)
def forward(n):
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
    if n< 50: 
        distance = 25.4*n*1.015
    else :
        distance = 25.4*n*1.0279
    robot.reset()
    while robot.distance() <= distance:
        robot.drive(60,0)
        wait(10)
    ev3.speaker.beep()
    robot.stop()
def barcode_forward2(n):
    # robot.stop()
    # Motor(Port.B, Direction.CLOCKWISE).run_time(170,0.175)
    # robot.stop()
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
    global Barcode1, Barcode2, Barcode3, Barcode4, Barcode5, Barcode6, Barcode7
    Barcode1 = [1,0,0,0]
    Barcode2 = [1,0,1,0]
    Barcode5 = [1,0,1,1]
    Barcode3 = [1,1,0,0]
    Barcode6 = [1,1,1,1]
    Barcode7 = [1,1,1,0]
    Barcode4 = [1,0,0,1]
    global scanned_Barcode
    scanned_Barcode = [2,2,2,2]
    equalYesNo = 0
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
        if Barcode1[myX] == scanned_Barcode[myX] : 
            equalYesNo = 1
            break
        elif ((Barcode2[myX] == scanned_Barcode[myX]) or (Barcode5[myX] == scanned_Barcode[myX]))  : 
            equalYesNo = 2
            break
        elif ((Barcode3[myX] == scanned_Barcode[myX]) or (Barcode6[myX] == scanned_Barcode[myX]) or (Barcode7[myX] == scanned_Barcode[myX])) : 
            equalYesNo = 3
            break
        elif Barcode4[myX] == scanned_Barcode[myX] : 
            equalYesNo = 1
            break
    return 1

def subtask3():
    # forward(36)
    # turn_left_angle(180, 0.97)
    forward(20.0750)
    while ((colorSensor1.color() == None) or not(colorSensor1.ambient() < 3 and colorSensor1.reflection() < 10 )):
        barcode_forward2(0.125)
        robot.stop()
        wait(1000)
    barcode = read_barcode()
    barcode = "Box Type: " + str(barcode) + "Identified!!"
    ev3.screen.draw_text(50, 50, str(barcode))
    wait(5000)
    ev3.screen.clear()
    ev3.screen.load_image(ImageFile.ACCEPT)
    ev3.speaker.play_file(SoundFile.CHEERING)
    robot.stop()
    wait(5000)
    # forward(0.2)
    # robot.stop()
    left_motor.run_time(-300, 1.575*1000)
    robot.stop()
    forward(4)
    robot.stop()
    hands_go_up(5)
    robot.stop()
    reverse(6.5)
    robot.stop()
    barcode_turn_left_angle(90,0.95)
    robot.stop()
    forward(21)
    robot.stop()
    hands_go_down(4.9575)
    robot.stop()
    reverse(4.25)
    robot.stop()
    robot.stop()
    ev3.screen.draw_text(50, 50, str(barcode))
    wait(5000)
    ev3.screen.load_image(ImageFile.ACCEPT)
    ev3.speaker.play_file(SoundFile.CHEERING)
    robot.stop()
subtask3()

robot.stop()
robot.stop()