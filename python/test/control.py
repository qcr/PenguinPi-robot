
#!usr/bin/python3

import math as m
import penguinPi as ppi
import penguinPi as ppi
import time   
#from drive import motionMotor  # drive python file


run = True



### MOTOR POWER ###
Motor_Power = 30



Scale_Value = 5
###################


mA = ppi.Motor(ppi.AD_MOTOR_A)
mB = ppi.Motor(ppi.AD_MOTOR_B)
ppi.init()

#mA.set_power(Motor_Power_Right)  #Right
#mB.set_power(Motor_Power_Left)  #Left

## ENCODERS ##
rightPrev = mA.get_ticks()
leftPrev = mB.get_ticks()

#pygame.init()


def getch():
    import sys, tty, termios
    old_settings = termios.tcgetattr(0)
    new_settings = old_settings[:]
    new_settings[3] &= ~termios.ICANON
    try:
        termios.tcsetattr(0, termios.TCSANOW, new_settings)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(0, termios.TCSANOW, old_settings)
    return ch



check = True

# Infinite loop that will not end until the user presses the
# exit key
while check:
  
    char = getch()
     # char = msvcrt.getch()
    print(char)

    if(char =="q"):
        Motor_Power = Motor_Power + 10
    elif(char =="e"):
        Motor_Power = Motor_Power - 10

    Motor_Power_Right = Motor_Power
    Motor_Power_Left = Motor_Power

        
     # The car will drive forward when the "w" key is pressed
    if(char == "w"):
        mA.set_power(Motor_Power_Right)  #Right
        mB.set_power(Motor_Power_Left *-1)  #Left
            

        # The car will reverse when the "s" key is pressed
    elif(char == "s"):
        mA.set_power(Motor_Power_Right * -1)  #Right
        mB.set_power(Motor_Power_Left)  #Left

    # The "a" key will toggle the steering left
    elif(char == "a"):
        mA.set_power(Motor_Power_Right * -1)  #Right
        mB.set_power(Motor_Power_Left* -1)  #Left

    # The "d" key will toggle the steering right
    elif(char == "d"):
        mA.set_power(Motor_Power_Right)  #Right
        mB.set_power(Motor_Power_Left)  #Left
        
    elif(char == "z"):
        mA.set_power(0)  #Right
        mB.set_power(0)  #Left

        # The "x" key will break the loop and exit the program
    if(char == "x"):
        mA.set_power(0)  #Right
        mB.set_power(0)  #Left
        print("Program Ended")
        check = False
        break


