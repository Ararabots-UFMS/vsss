#!/usr/bin/python
import sys
import os
import numpy as np
sys.path[0] = root_path = os.environ['ROS_ARARA_ROOT'] + "src/"
from utils.model import Model
from robot_module.comunication.sender import Sender
import rospy
import pygame

vel = 140
p = 1.0 

# Define some colors
BLACK    = (   0,   0,   0)
WHITE    = ( 255, 255, 255)
decrementX = 50.0
incrementX = 8.0
maxXAcceleration = 25.0


class TextPrint:
    def __init__(self):
        self.reset()
        self.font = pygame.font.Font(None, 48)
        self.outlier = pygame.font.Font(None, 48)
    def print_to_screen(self, screen, textString):
        
        textBitmap = self.font.render(textString, True, WHITE)
        textBitmapOutlier = self.outlier.render(textString, True, BLACK)

        screen.blit(textBitmapOutlier, [self.x-1, self.y])
        screen.blit(textBitmapOutlier, [self.x+1, self.y])
        screen.blit(textBitmapOutlier, [self.x, self.y-1])
        screen.blit(textBitmapOutlier, [self.x, self.y+1])
        
        screen.blit(textBitmap, [self.x, self.y])
        
        self.y += self.line_height
        
    def reset(self):
        self.x = 10
        self.y = 10
        self.line_height = 45
        
    def indent(self):
        self.x += 25
        
    def unindent(self):
        self.x -= 25


if __name__ == '__main__':

    model = Model()

    constant = 0
    
    controller = int(input("Qual controle? (0-N)"))

    bluetooth_list = []

    for key in model.robot_bluetooth:
        value = model.robot_bluetooth[key]
        print(str(constant)+" - "+ key +" " + value)
        bluetooth_list.append(value)

        constant+=1

    player = int(input("Qual jogador? "))
    
    if player >= constant:
        print("Resposta errada")
        sys.exit(-1)

    pygame.init()
    arara = pygame.image.load('darkararabots.png')
    arararect = arara.get_rect()
    
    size = [arararect[2],arararect[3]-40]
    screen = pygame.display.set_mode(size)
    #screen.fill(black)
    screen.blit(arara, arararect)

    pygame.display.set_caption("JoyRide")

    pygame.joystick.init()
    # Get ready to print
    textPrint = TextPrint()
    textPrint.reset()
    textPrint.print_to_screen(screen, "Controles:")
    textPrint.indent()
    textPrint.print_to_screen(screen, "Left Stick: Aceleracao em Y")
    textPrint.print_to_screen(screen, "Right Stick: Roda usando o X")
    textPrint.print_to_screen(screen, "Botao X (3): Troca estado reverso")
    textPrint.print_to_screen(screen, "Botao [](4): Spin mt loko") 
    pygame.display.flip()


    #cv2.namedWindow('test',0)
    # ADICIONAR ROBOS
    #nrobot = int(sys.argv[1]) 
    
    # vrobot = Sender(1,"84:0D:8E:0C:92:8E") 
    vrobot = Sender(player,bluetooth_list[player]) # Tijolinho
    #vrobot = Sender(1,"84:0D:8E:0C:92:8E")
    vrobot.connect()
    paused = False
    vesq = 0
    vdir = 0
    vel = 0
    transformed = False
    
    # Used to manage how fast the screen updates
    clock = pygame.time.Clock()
    joystick = pygame.joystick.Joystick(controller)
    joystick.init()
    accelX = 0
    accelY = 0
    done = False

    while done==False:

        for event in pygame.event.get(): # User did something
            if event.type == pygame.QUIT: # If user clicked close
                done=True # Flag that we are done so we exit this loop

        axisX = joystick.get_axis(3)
        axisY = joystick.get_axis(1)

        if axisY != 0:
            accelY += -axisY*12.75
            if abs(accelY) > 120: 
                if accelY > 0:
                    accelY = 120
                else:
                    accelY = -120
        else:
            if (abs(accelY) -51) < 0:
                accelY = 0
            else:
                if accelY > 0:
                    accelY-=51.0
                else:
                    accelY+=51.0


        if abs(axisX) >= 0.2:
            #accelY =accelY - accelY*0.15
            if abs(accelX) > maxXAcceleration:
                if accelX > 0:
                    accelX = maxXAcceleration
                else:
                    accelX = -maxXAcceleration
            else:    
                accelX += axisX*incrementX
        else:
            if (abs(accelX) -decrementX) <= 0:
                accelX = 0
            elif accelX > 0:
                accelX-=decrementX
            else:
                accelX+=decrementX


        #buttons = joystick.get_numbuttons()
        #print("Number of buttons: "+str(buttons))
        

        #for i in range( buttons ):
        xbutton = joystick.get_button(2)
        left_bumper  = joystick.get_button(4)
        right_bumper = joystick.get_button(5)
        #print(str(i)+" "+str(button))
        #textPrint.unindent()

        if xbutton:
            vrobot.sendPacket(
                int(255),
                int(-255)
            )
        elif bool(left_bumper) != bool(right_bumper):
            if left_bumper:
                vrobot.sendPacket(
                    int(-255),
                    int(255)
                )
            elif right_bumper:
                vrobot.sendPacket(
                    int(255),
                    int(-255)
                )
        else:
            sumX = accelX + np.sign(accelX)*50*(1 - abs(axisY))
            vrobot.sendPacket(
                int(accelY+ sumX ),
                int(accelY+-sumX)
            )


        # if paused:
        #     vrobot[nrobot].stop()
        # else: 
        #     print vrobot[nrobot].__print__()
        #if key != 255:
        #    print vesq+vel,vdir+vel

        # Limit to 20 frames per second
        clock.tick(60)

    # Close the window and quit.
    # If you forget this line, the program will 'hang'
    # on exit if running from IDLE.
    pygame.quit ()