import numpy as np
from abc import ABC, abstractmethod

from vision_module.seekers.seeker_data_structures import ObjState

# python 3.4+


class ObjDetector(ABC):

    def __init__(self):
        self.should_calculate_size = True

    @abstractmethod
    def seek(self, segments:np.ndarray, objs_per_segment:[int], full_image:np.ndarray) -> ObjState:
        raise Exception("subclass must override seek")
        pass
    
    @abstractmethod
    def update_obj_size(self, size) -> None:
        raise Exception("subclass must override update_obj_size")
        pass

    def turn_off_size_calculation(self):
        self.should_calculate_size = False
