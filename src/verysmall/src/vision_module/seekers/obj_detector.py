import numpy as np
from abc import ABC, abstractmethod

from vision_module.seekers.seeker_data_structures import ObjState

# python 3.4+

class ObjDetector(ABC):
    
    @abstractmethod
    def seek(self, segments:np.ndarray, objs_per_segment:[int]) -> ObjState:
        raise Exception("subclass must override seek")
        pass
    
    @abstractmethod
    def update_obj_size(self, size) -> None:
        raise Exception("subclass must override update_obj_size")
        pass
    
