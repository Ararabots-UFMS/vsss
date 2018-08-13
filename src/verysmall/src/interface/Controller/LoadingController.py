from ..View.LoadingView import LoadingView
from threading import Thread
from time import sleep

class LoadingController:
    def __init__(self):
        self.load_thread = None

    def start(self):
        self.load_thread = LoadingView()
        self.load_thread.start()

    def stop(self):
        self.load_thread.DoRun = False
        self.load_thread.join()