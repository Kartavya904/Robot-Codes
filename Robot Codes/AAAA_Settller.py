#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, GyroSensor, UltrasonicSensor, ColorSensor
from pybricks.parameters import Port, Direction, Button
from pybricks.tools import wait
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import ImageFile

ev3 = EV3Brick()
left_motor = Motor(Port.A, Direction.CLOCKWISE)
left_motor.run_time(-300, 1)
right_motor = Motor(Port.B, Direction.CLOCKWISE)
right_motor.run_time(-300, 1)
#medium_motor = Motor(Port.C, Direction.CLOCKWISE)
ultraSonicSensor = UltrasonicSensor(Port.S4)
colorSensor1 = ColorSensor(Port.S3)
WHEEL_DIAMETER = 56
AXLE_TRACK = 200
robot = DriveBase(left_motor, right_motor, WHEEL_DIAMETER, AXLE_TRACK)
gyro_sensor = GyroSensor(Port.S2)
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
    my_Barcode = [1,0,1,0]
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
                forward(0.25)
                wait(1000)
                break
    print(scanned_Barcode)
    for myX in range (0,4):
        if my_Barcode[myX] != scanned_Barcode[myX] : 
            equalYesNo = False
            break
    # equalYesNo = True
    if equalYesNo :
        reverse(10)
        wait(1000)
        turn_left_angle(90,0.97)
        reverse(5)
        wait(1000)
        turn_right_angle(90,1.03)
        forward(8)
        turn_left_angle(90,0.97)
        forward(5)
        #hands_go_down(4,5)
        hands_go_up(4.5)
        reverse(6)
        turn_left_angle(90,0.97)
        forward(10)
        hands_go_down(2.4)
        reverse(4)
    else :
        ev3.speaker.beep()
        wait(100)
        ev3.speaker.beep()
        wait(100)
        ev3.speaker.beep()
        wait(100)




turn_right_angle(89, 1)
turn_left_angle(84,1)
# reverse(10)
# turn_right_angle(10,1.03)
# forward(5)
# turn_right_angle(10,1.03)

# forward(84)
# turn_left_angle(90,1)
# forward(12)

# hands_go_down(0.2)

# forward(3.25)
# hands_go_up(5.0)
# hands_go_down(4.875)

# forward(12)
#turn_left_angle(90,0.82)
# robot.stop()
# forward(24)

# barcode_turn_right_angle(90, 1) #Works
# barcode_turn_left_angle(180,0.979) #Works
# barcode_turn_left_angle(90,0.979) #Works
#turn_right_angle(90,1.03) #Works
# hands_go_up(0.3)
# hands_go_down(2.6)
# hands_go_down(2.4)

robot.stop()
