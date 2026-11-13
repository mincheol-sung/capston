import numpy as np
from scipy.ndimage import binary_dilation

class Costmap:
    def __init__(self, raw_map, robot_size=3):

        self.raw_map = np.array(raw_map)
        self.height, self.width = self.raw_map.shape 
        
        self.robot_size = robot_size
        
        self.inflated_map = self._inflate_map()

    def _inflate_map(self):
        if self.robot_size <= 1:
            return self.raw_map.copy()
        
        square_mask = np.ones((self.robot_size, self.robot_size)) 
        
        inflated = binary_dilation(self.raw_map, structure=square_mask).astype(int)
        return inflated

    def is_out_of_bounds(self, x, y):
        return not (0 <= x < self.width and 0 <= y < self.height) 

    def is_collision(self, x, y):
        if self.is_out_of_bounds(x, y):
            return True 
        
        return self.inflated_map[y, x] == 1