import numpy as np

def get_euclidean_distance(node1, node2):
    point1 = np.array(node1)
    point2 = np.array(node2)
    
    return np.linalg.norm(point1 - point2)

def is_line_of_sight_clear(p1, p2, costmap):
    p1 = np.array(p1, dtype=int)
    p2 = np.array(p2, dtype=int)
    
    dim = len(p1)
    delta = p2 - p1
    abs_delta = np.abs(delta)
    
    primary_axis = np.argmax(abs_delta)
    primary_step = 1 if delta[primary_axis] > 0 else -1
    
    error = np.zeros(dim, dtype=int) 
    delta2 = 2 * abs_delta
    
    steps = abs_delta[primary_axis]
    current = p1.copy() 
    
    for _ in range(steps):
        if costmap.is_collision(*current): 
            return False
            
        current[primary_axis] += primary_step
        
        for d in range(dim):
            if d == primary_axis: continue
            
            error[d] += delta2[d]
            if error[d] > abs_delta[primary_axis]:
                current[d] += 1 if delta[d] > 0 else -1
                error[d] -= delta2[primary_axis]
                
    return not costmap.is_collision(*p2)