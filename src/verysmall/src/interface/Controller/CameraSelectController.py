#!/usr/bin/python
# -*- coding: latin-1 -*-
from ..View.CameraSelectView import CameraSelectView
import fltk as fl


class CameraSelectController:

    def update_list(self, camera_list):
        self.view.device_input.clear()
        self.view.device_input.add("Nenhum")
        self.view.device_input.value(0)
        self.view.device_input.add("Leitura de arquivo")

        for camera in camera_list:
            self.view.device_input.add(
                camera + " - " + camera_list[camera]["name"] + " (" + camera_list[camera]["product"] + ")")
        self.view.device_input.redraw()

    def button_callback(self,ptr):
        self.callable()
        print(self.camera_list)
        self.update_list(self.camera_list)

    def browser_callback(self, ptr):
        fl.Fl.background(200, 200, 200)
        self.view.file_browser.show()

        while self.view.file_browser.visible():
            fl.Fl.wait()

        self.view.file_box.align(fl.FL_ALIGN_INSIDE|fl.FL_ALIGN_RIGHT)
        self.view.file_box.label(self.view.file_browser.value())

    def input_callback(self, ptr):

        item = ptr.value() - 2
        if item == -1:
            print("arquivo")
            self.view.file_browser_label.show()
            self.view.file_button.show()
            self.view.file_box.show()
        else:
            if item == -2:
                print("nenhum")
            else:
                pass

            self.view.file_browser_label.hide()
            self.view.file_button.hide()
            self.view.file_box.hide()

    def __init__(self, _camera_model, _camera_list, _callable):
        self.camera_model = _camera_model
        self.camera_list = _camera_list
        self.callable = _callable
        self.view = CameraSelectView()

        self.result = False

        self.view.warning.label("Dispositivo \""+self.camera_model+"\" não encontrado.")

        self.update_list(self.camera_list)

        self.view.file_browser_label.hide()
        self.view.file_button.hide()
        self.view.file_box.hide()

        self.view.device_input.callback(self.input_callback)
        self.view.file_button.callback(self.browser_callback)
        self.view.refresh_button.callback(self.button_callback)

        self.view.end()



