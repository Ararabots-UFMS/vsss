#!/usr/bin/python
# -*- coding: latin-1 -*-

import fltk as fl

class CameraSelectView:

    def __init__(self):

        # Get the usable screen proportions
        self.width = fl.Fl.w()
        self.height = fl.Fl.h()

        self.root = fl.Fl_Single_Window(self.proportion_width(37), self.proportion_height(45),
                                        self.proportion_width(26), self.proportion_height(10))
        self.width = self.root.w()
        self.height = self.root.h()

        self.root.border(0)

    def end(self):
        self.root.end()
        self.root.show()

    def proportion_height(self, proportion):
        """Returns the Y value for the designed vertical screen proportion"""
        return int(self.height * proportion / 100)

    def proportion_width(self, proportion):
        """Returns the X value for the designed horizontal screen proportion"""
        return int(self.width * proportion / 100)

if __name__ == '__main__':
    c = CameraSelectView()
    c.end()
    fl.Fl.run()