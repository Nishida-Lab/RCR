#!/usr/bin/env python3

import pygame
from pygame.locals import *

pygame.joystick.init()
try:
    j = pygame.joystick.Joystick(0)
    j.init()
except pygame.error:
    print('Joystick not found')

def main():
    pygame.init()
    x_old, y_old = 0, 0
    while 1:
        for e in pygame.event.get():
            if e.type == QUIT:
                return
            if (e.type == KEYDOWN and e.key == K_ESCAPE):
                return
            if e.type == pygame.locals.JOYAXISMOTION:
                x_new, y_new = j.get_axis(0), j.get_axis(1)
                if not (x_new == x_old and y_new == x_old):
                    x_old , y_old = x_new, y_new
                    print('x:{}, y:{}'.format(x_new, y_new))
            elif e.type == pygame.locals.JOYBALLMOTION:
                print('ball motion')
            elif e.type == pygame.locals.JOYHATMOTION:
                print('hat motion')
            elif e.type == pygame.locals.JOYBUTTONDOWN:
                print('{} button was pushed'.format(e.button))
            elif e.type == pygame.locals.JOYBUTTONUP:
                print('{} button was released'.format(e.button))

if __name__ == '__main__':
    main()
