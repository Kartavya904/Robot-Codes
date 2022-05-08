# [3:53 pm, 29/03/2022] ~ Kartavya Singh: there ya go
# [8:41 pm, 29/03/2022] US Atharva UC: def barcode():
    
#     total = 0
#     item_num = 0

#     for i in range (0, 3):
#         a = 0
#         reflection = 0
#         ambient = 0
#         while color_sensor.color == None:
#             reflection = color_sensor.reflection()
#             ambient = color_sensor.ambient()
#         else:        
#             if (reflection < 10) and (ambient < 3):
#                 a = 1

#         total = (a * (2**(i))) + total
#         medium_motor.hold()
#         ev3.speaker.beep()
#         lift_up(0.8)


#     if total == 8:
#         item_num = 1
#     elif total == 10:
#         item_num = 2
#     elif total == 12:
#         item_num = 3
#     elif total == 9:
#         item_num = 4
#     else:
#         item_num = 0

#     return item_num