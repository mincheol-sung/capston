import heapq

try:
    from global_planner.theta_star import utils
except ImportError:
    import utils

def get_neighbors(node, costmap):
    neighbors = []
    directions = [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]
    for dx, dy in directions:
        nx, ny = node[0] + dx, node[1] + dy     
        if 0 <= nx < costmap.width and 0 <= ny < costmap.height:   
            if not costmap.is_collision(nx, ny):
                neighbors.append((nx, ny)) 
    return neighbors


def theta_star(start, goal, costmap):
    # 1. Main()
    
    # 2. open := closed := empty
    open_set = []      
    open_dict = {}      
    closed_set = set()  

    g_score = {}    
    parent = {}
    
    # 3. g(s_start) := 0
    g_score[start] = 0.0
    # 4. parent(s_start) := s_start
    parent[start] = start
    
    # 5. open.Insert(s_start, g(s_start) + h(s_start))
    h_start = utils.get_euclidean_distance(start, goal)
    heapq.heappush(open_set, (g_score[start] + h_start, start)) 
    open_dict[start] = g_score[start] + h_start
    
    # 19. UpdateVertex(s, s')
    def update_vertex(s, s_prime):
        # 20. gold := g(s')
        g_old = g_score.get(s_prime, float('inf'))
        
        # 21. ComputeCost(s, s')
        compute_cost(s, s_prime)
        
        # 22. if g(s') < gold then
        if g_score[s_prime] < g_old:
            
            # 25. open.Insert(s', g(s') + h(s'))
            f_score = g_score[s_prime] + utils.get_euclidean_distance(s_prime, goal)
            heapq.heappush(open_set, (f_score, s_prime))
            open_dict[s_prime] = f_score 
            
    # 27. ComputeCost(s, s')
    def compute_cost(s, s_prime):
        # 28. if lineofsight(parent(s), s') then
        if utils.is_line_of_sight_clear(parent[s], s_prime, costmap):
            # 29. /* Path 2 */
            c_dist = utils.get_euclidean_distance(parent[s], s_prime)
            # 30. if g(parent(s)) + c(parent(s), s') < g(s') then
            if g_score[parent[s]] + c_dist < g_score.get(s_prime, float('inf')):
                # 31. parent(s') := parent(s)
                parent[s_prime] = parent[s]
                # 32. g(s') := g(parent(s)) + c(parent(s), s')
                g_score[s_prime] = g_score[parent[s]] + c_dist
        else:
            # 34. /* Path 1 */
            c_dist = utils.get_euclidean_distance(s, s_prime)
            # 35. if g(s) + c(s, s') < g(s') then
            if g_score[s] + c_dist < g_score.get(s_prime, float('inf')):
                # 36. parent(s') := s
                parent[s_prime] = s
                # 37. g(s') := g(s) + c(s, s')
                g_score[s_prime] = g_score[s] + c_dist


    # 6. while open != empty do
    while open_set:
        # 7. s := open.Pop()
        current_f, s = heapq.heappop(open_set)
        
        if s in open_dict and current_f > open_dict[s]:
            continue
            
        if s in open_dict:
            del open_dict[s]
            
        # 8. if s = s_goal then
        if s == goal:
            # 9. return "path found" 
            path = []
            curr = goal
            while curr != start:
                path.append(curr)
                curr = parent[curr]
            path.append(start)
            return path[::-1] 
            
        # 10. closed := closed U {s}
        closed_set.add(s)
        
        # 11. foreach s' in nghbr_vis(s) do
        for s_prime in get_neighbors(s, costmap):
            # 12. if s' not in closed then
            if s_prime not in closed_set:
                # 13. if s' not in open then
                if s_prime not in open_dict:
                    # 14. g(s') := inf
                    g_score[s_prime] = float('inf')
                    # 15. parent(s') := NULL
                    parent[s_prime] = None
                    
                # 16. UpdateVertex(s, s')
                update_vertex(s, s_prime)
                
    # 17. return "no path found"
    return None
