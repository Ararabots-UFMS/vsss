from ..View.LoadingView import LoadingView
from threading import Thread


class LoadingController:
    def __init__(self):
        self.load = None
        self.t = Thread(target=self.run, args=())
        self.t.start()

    def run(self):
        self.load = LoadingView()
