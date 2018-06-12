#!/usr/bin/env python
import fltk as fl
import sys

if __name__ == '__main__':
    window = fl.Fl_Menu_Window()
    #window.fullscreen_off()
    window.color(fl.fl_rgb_color(33,33 ,33))

    box = fl.Fl_Box(20,40,260,100,"Hello, World!")
    window.resizable(box)

    #box.box(FL_UP_BOX)
    box.labelsize(36)
    box.labelcolor(fl.FL_WHITE)
    box.labelfont(fl.FL_BOLD)
    #box.labeltype(FL_SHADOW_LABEL)
    #print window.get_data()
    x=0
    y=0
    w=0
    h = 0
    #Fl_screen_xywh(x,y,w,h)
    #print x,y,w,h

    window.end()
    window.show(sys.argv)
    fl.Fl_run()
