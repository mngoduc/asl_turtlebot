# import numpy as np
# import matplotlib.pyplot as plt
# import matplotlib.patches as patches

# # Represents a motion planning problem to be solved using A*
# class AStar(object):

#     def __init__(self, statespace_lo, statespace_hi, x_init, x_goal, occupancy, resolution=1):
#         self.statespace_lo = statespace_lo         # state space lower bound (e.g., (-5, -5))
#         self.statespace_hi = statespace_hi         # state space upper bound (e.g., (5, 5))
#         self.occupancy = occupancy                 # occupancy grid
#         self.resolution = resolution               # resolution of the discretization of state space (cell/m)
#         self.x_init = self.snap_to_grid(x_init)    # initial state
#         self.x_goal = self.snap_to_grid(x_goal)    # goal state

#         self.closed_set = []    # the set containing the states that have been visited
#         self.open_set = []      # the set containing the states that are condidate for future expension

#         self.f_score = {}       # dictionary of the f score (estimated cost from start to goal passing through state)
#         self.g_score = {}       # dictionary of the g score (cost-to-go from start to state)
#         self.came_from = {}     # dictionary keeping track of each state's parent to reconstruct the path

#         self.open_set.append(x_init)
#         self.g_score[x_init] = 0
#         self.f_score[x_init] = self.distance(x_init,x_goal)

#         self.path = None        # the final path as a list of states

#     # Checks if a give state is free, meaning it is inside the bounds of the map and
#     # is not inside any obstacle
#     # INPUT: (x)
#     #          x - tuple state
#     # OUTPUT: Boolean True/False
#     def is_free(self, x):
#         if x==self.x_init or x==self.x_goal:
#             return True
#         for dim in range(len(x)):
#             if x[dim] < self.statespace_lo[dim]:
#                 return False
#             if x[dim] >= self.statespace_hi[dim]:
#                 return False
#         if not self.occupancy.is_free(x):
#             return False
#         return True

#     # computes the euclidean distance between two states
#     # INPUT: (x1, x2)
#     #          x1 - first state tuple
#     #          x2 - second state tuple
#     # OUTPUT: Float euclidean distance
#     def distance(self, x1, x2):
#         return np.linalg.norm(np.array(x1)-np.array(x2))

#     # returns the closest point on a discrete state grid
#     # INPUT: (x)
#     #          x - tuple state
#     # OUTPUT: A tuple that represents the closest point to x on the discrete state grid
#     def snap_to_grid(self, x):
#         return (self.resolution*round(x[0]/self.resolution), self.resolution*round(x[1]/self.resolution))

#     # gets the FREE neighbor states of a given state. Assumes a motion model
#     # where we can move up, down, left, right, or along the diagonals by an
#     # amount equal to self.resolution.
#     # Use self.is_free in order to check if any given state is indeed free.
#     # Use self.snap_to_grid (see above) to ensure that the neighbors you compute
#     # are actually on the discrete grid, i.e., if you were to compute neighbors by
#     # simply adding/subtracting self.resolution from x, numerical error could
#     # creep in over the course of many additions and cause grid point equality
#     # checks to fail. To remedy this, you should make sure that every neighbor is
#     # snapped to the grid as it is computed.
#     # INPUT: (x)
#     #           x - tuple state
#     # OUTPUT: List of neighbors that are free, as a list of TUPLES
#     def get_neighbors(self, x):
#         list_neighbors = []
#         x_left = x[0] - self.resolution
#         x_right = x[0] + self.resolution 
#         y_bottom = x[1] - self.resolution
#         y_top = x[1] + self.resolution
#         x_neigh_candidates = (x_left, x[0], x_right)
#         y_neigh_candidates = (y_bottom, x[1], y_top)

#         for i in x_neigh_candidates: 
#             for j in y_neigh_candidates: 
#                 if not ((i == 1) and (j == 1)): 
#                     neigh_candidate = self.snap_to_grid((i,j))
#                     if self.is_free(neigh_candidate): 
#                         list_neighbors.append(neigh_candidate)

#         return list_neighbors


#     # Gets the state in open_set that has the lowest f_score
#     # INPUT: None
#     # OUTPUT: A tuple, the state found in open_set that has the lowest f_score
#     def find_best_f_score(self):
#         return min(self.open_set, key=lambda x: self.f_score[x])

#     # Use the came_from map to reconstruct a path from the initial location
#     # to the goal location
#     # INPUT: None
#     # OUTPUT: A list of tuples, which is a list of the states that go from start to goal
#     def reconstruct_path(self):
#         path = [self.x_goal]
#         current = path[-1]
#         while current != self.x_init:
#             path.append(self.came_from[current])
#             current = path[-1]
#         return list(reversed(path))

#     # Plots the path found in self.path and the obstacles
#     # INPUT: None
#     # OUTPUT: None
#     def plot_path(self):
#         if not self.path:
#             return

#         fig = plt.figure()

#         self.occupancy.plot(fig.number)

#         solution_path = np.array(self.path) * self.resolution
#         plt.plot(solution_path[:,0],solution_path[:,1], color="green", linewidth=2, label="solution path", zorder=10)
#         plt.scatter([self.x_init[0]*self.resolution, self.x_goal[0]*self.resolution], [self.x_init[1]*self.resolution, self.x_goal[1]*self.resolution], color="green", s=30, zorder=10)
#         plt.annotate(r"$x_{init}$", np.array(self.x_init)*self.resolution + np.array([.2, 0]), fontsize=16)
#         plt.annotate(r"$x_{goal}$", np.array(self.x_goal)*self.resolution + np.array([.2, 0]), fontsize=16)
#         plt.legend(loc='upper center', bbox_to_anchor=(0.5, -0.03), fancybox=True, ncol=3)

#         plt.axis('equal')
#         plt.show()

#     # Solves the planning problem using the A* search algorithm. It places
#     # the solution as a list of of tuples (each representing a state) that go
#     # from self.x_init to self.x_goal inside the variable self.path
#     # INPUT: None
#     # OUTPUT: Boolean, True if a solution from x_init to x_goal was found
#     def solve(self):
#         while len(self.open_set)>0:
#             x_current = self.find_best_f_score()
#             x_current = self.snap_to_grid(x_current)
#             if (x_current == self.x_goal): 
#                 self.path = self.reconstruct_path()
#                 return self.path
#             (self.open_set).remove(x_current)
#             (self.closed_set).append(x_current)
#             for x_neigh in self.get_neighbors(x_current): 
#                 if x_neigh in self.closed_set: 
#                     continue 
#                 tentative_g_score = self.g_score[x_current] + self.distance(x_current, x_neigh)
#                 if (x_neigh not in self.open_set): 
#                     self.open_set.append(x_neigh)
#                 elif tentative_g_score > self.g_score[x_neigh]: 
#                     continue
#                 self.came_from[x_neigh] = x_current 
#                 self.g_score[x_neigh] = tentative_g_score
#                 self.f_score[x_neigh] = tentative_g_score + self.distance(x_neigh,self.x_goal)
#         return False

# # A 2D state space grid with a set of rectangular obstacles. The grid is fully deterministic
# class DetOccupancyGrid2D(object):
#     def __init__(self, width, height, obstacles):
#         self.width = width
#         self.height = height
#         self.obstacles = obstacles

#     def is_free(self, x):
#         for obs in self.obstacles:
#             inside = True
#             for dim in range(len(x)):
#                 if x[dim] < (obs[0][dim] -4) or x[dim] > (obs[1][dim] + 4):
#                     inside = False
#                     break
#             if inside:
#                 return False
#         return True

#     def plot(self, fig_num=0):
#         fig = plt.figure(fig_num)
#         for obs in self.obstacles:
#             ax = fig.add_subplot(111, aspect='equal')
#             ax.add_patch(
#             patches.Rectangle(
#             obs[0],
#             obs[1][0]-obs[0][0],
#             obs[1][1]-obs[0][1],))

# ### TESTING

# # A simple example
# '''
# width = 10
# height = 10
# x_init = (0,0)
# x_goal = (8,8)
# obstacles = [((6,6),(8,7)),((2,1),(4,2)),((2,4),(4,6)),((6,2),(8,4))]
# occupancy = DetOccupancyGrid2D(width, height, obstacles)
# '''
# if __name__ == '__main__':
#     # A large random example
#     width = 101
#     height = 101
#     num_obs = 15
#     min_size = 5
#     max_size = 25
#     obs_corners_x = np.random.randint(0,width,num_obs)
#     obs_corners_y = np.random.randint(0,height,num_obs)
#     obs_lower_corners = np.vstack([obs_corners_x,obs_corners_y]).T
#     obs_sizes = np.random.randint(min_size,max_size,(num_obs,2))
#     obs_upper_corners = obs_lower_corners + obs_sizes
#     obstacles = zip(obs_lower_corners,obs_upper_corners)
#     occupancy = DetOccupancyGrid2D(width, height, obstacles)
#     x_init = tuple(np.random.randint(0,width-2,2).tolist())
#     x_goal = tuple(np.random.randint(0,height-2,2).tolist())
#     while not (occupancy.is_free(x_init) and occupancy.is_free(x_goal)):
#          x_init = tuple(np.random.randint(0,width-2,2).tolist())
#          x_goal = tuple(np.random.randint(0,height-2,2).tolist())

#     astar = AStar((0, 0), (width, height), x_init, x_goal, occupancy)

#     if not astar.solve():
#         print "No path found"
#         exit(0)

#     astar.plot_path()
import numpy as np
import rospy
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# Represents a motion planning problem to be solved using A*
class AStar(object):

    def __init__(self, statespace_lo, statespace_hi, x_init, x_goal, occupancy, resolution=1):
        self.statespace_lo = statespace_lo         # state space lower bound (e.g., (-5, -5))
        self.statespace_hi = statespace_hi         # state space upper bound (e.g., (5, 5))
        self.occupancy = occupancy                 # occupancy grid
        self.resolution = resolution               # resolution of the discretization of state space (cell/m)
        self.x_init = self.snap_to_grid(x_init)    # initial state
        self.x_goal = self.snap_to_grid(x_goal)    # goal state

        self.closed_set = []    # the set containing the states that have been visited
        self.open_set = []      # the set containing the states that are condidate for future expension

        self.f_score = {}       # dictionary of the f score (estimated cost from start to goal passing through state)
        self.g_score = {}       # dictionary of the g score (cost-to-go from start to state)
        self.came_from = {}     # dictionary keeping track of each state's parent to reconstruct the path

        self.open_set.append(x_init)
        self.g_score[x_init] = 0
        self.f_score[x_init] = self.distance(x_init,x_goal)

        self.path = None        # the final path as a list of states

    # Checks if a give state is free, meaning it is inside the bounds of the map and
    # is not inside any obstacle
    # INPUT: (x)
    #          x - tuple state
    # OUTPUT: Boolean True/False
    def is_free(self, x):
        if x==self.x_init or x==self.x_goal:
            return True
        for dim in range(len(x)):
            #rospy.loginfo("x[dim]")
            #rospy.loginfo(x[dim])
            if x[dim] < self.statespace_lo[dim]:
                #rospy.loginfo("too small xdim")
                return False
            if x[dim] >= self.statespace_hi[dim]:
                return False
        if not self.occupancy.is_free(x):
            #rospy.loginfo("too many obstacles")
            return False
        return True

    # computes the euclidean distance between two states
    # INPUT: (x1, x2)
    #          x1 - first state tuple
    #          x2 - second state tuple
    # OUTPUT: Float euclidean distance
    def distance(self, x1, x2):
        return np.linalg.norm(np.array(x1)-np.array(x2))

    # returns the closest point on a discrete state grid
    # INPUT: (x)
    #          x - tuple state
    # OUTPUT: A tuple that represents the closest point to x on the discrete state grid
    def snap_to_grid(self, x):
        return (self.resolution*round(x[0]/self.resolution), self.resolution*round(x[1]/self.resolution))

    # gets the FREE neighbor states of a given state. Assumes a motion model
    # where we can move up, down, left, right, or along the diagonals by an
    # amount equal to self.resolution.
    # Use self.is_free in order to check if any given state is indeed free.
    # Use self.snap_to_grid (see above) to ensure that the neighbors you compute
    # are actually on the discrete grid, i.e., if you were to compute neighbors by
    # simply adding/subtracting self.resolution from x, numerical error could
    # creep in over the course of many additions and cause grid point equality
    # checks to fail. To remedy this, you should make sure that every neighbor is
    # snapped to the grid as it is computed.
    # INPUT: (x)
    #           x - tuple state
    # OUTPUT: List of neighbors that are free, as a list of TUPLES
    def get_neighbors(self, x):
        # TODO: fill me in!
        free_neighbors = []
        x, y, r = x[0], x[1], self.resolution
        c = np.sqrt(2)/2
        all_neighbors = [ (x+r, y), (x+r*c, y-r*c),
            (x, y-r), (x-r*c, y-r*c),
            (x-r, y), (x-r*c, y+r*c),
            (x, y+r), (x+r*c, y+r*c) ]

        # POSSIBLE CONCERN- MIGHT HAVE TO SNAP_TO_GRID IN THE ABOVE LINE INSTEAD OF BELOW
        # snap neighbors to grid, then check if free
        for i in range(len(all_neighbors)):
            all_neighbors[i] = self.snap_to_grid(all_neighbors[i])
            if self.is_free(all_neighbors[i]):
                free_neighbors.append(all_neighbors[i])

        #rospy.loginfo("Number of free neighbors")
        #rospy.loginfo(len(free_neighbors))
        return free_neighbors


    # Gets the state in open_set that has the lowest f_score
    # INPUT: None
    # OUTPUT: A tuple, the state found in open_set that has the lowest f_score
    def find_best_f_score(self):
        return min(self.open_set, key=lambda x: self.f_score[x])

    # Use the came_from map to reconstruct a path from the initial location
    # to the goal location
    # INPUT: None
    # OUTPUT: A list of tuples, which is a list of the states that go from start to goal
    def reconstruct_path(self):
        path = [self.x_goal]
        current = path[-1]
        while current != self.x_init:
            path.append(self.came_from[current])
            current = path[-1]
        return list(reversed(path))

    # Plots the path found in self.path and the obstacles
    # INPUT: None
    # OUTPUT: None
    def plot_path(self):
        if not self.path:
            return

        fig = plt.figure()

        self.occupancy.plot(fig.number)

        solution_path = np.array(self.path) * self.resolution
        plt.plot(solution_path[:,0],solution_path[:,1], color="green", linewidth=2, label="solution path", zorder=10)
        plt.scatter([self.x_init[0]*self.resolution, self.x_goal[0]*self.resolution], [self.x_init[1]*self.resolution, self.x_goal[1]*self.resolution], color="green", s=30, zorder=10)
        plt.annotate(r"$x_{init}$", np.array(self.x_init)*self.resolution + np.array([.2, 0]), fontsize=16)
        plt.annotate(r"$x_{goal}$", np.array(self.x_goal)*self.resolution + np.array([.2, 0]), fontsize=16)
        plt.legend(loc='upper center', bbox_to_anchor=(0.5, -0.03), fancybox=True, ncol=3)

        plt.axis('equal')
        plt.show()

    # Solves the planning problem using the A* search algorithm. It places
    # the solution as a list of of tuples (each representing a state) that go
    # from self.x_init to self.x_goal inside the variable self.path
    # INPUT: None
    # OUTPUT: Boolean, True if a solution from x_init to x_goal was found
    def solve(self):
        while len(self.open_set)>0:

            # find state in OPEN set with lowest f_score
            x_current = self.find_best_f_score()
            
            # if x_current = x_goal, exit
            if x_current == self.x_goal:
                # update path
                self.path = self.reconstruct_path() 
                return True

            # remove x_current from the OPEN set and add to CLOSED set
            self.open_set.remove(x_current)
            self.closed_set.append(x_current)

            # get neighbors of x_current
            free_neighbors = self.get_neighbors(x_current)

            # looping through all neighbors...
            for i in range(len(free_neighbors)):

                # if neighbor is in CLOSED set, skip to the next neighbor
                x_neigh = free_neighbors[i]
                if x_neigh in self.closed_set:
                    continue

                # calculate tentative g_score
                tentative_g_score = self.g_score[x_current] + self.distance(x_current, x_neigh)

                # if neighbor is not in OPEN set, add it to OPEN set
                if not x_neigh in self.open_set:
                    self.open_set.append(x_neigh)
                # if tentative g_score exceeds neighbor's g_score, skip to next neighbor
                elif tentative_g_score > self.g_score[x_neigh]:
                    continue

                # if pass all the if constructs, update dicts
                self.came_from[x_neigh] = x_current
                self.g_score[x_neigh] = tentative_g_score
                self.f_score[x_neigh] = tentative_g_score + self.distance(self.x_init,self.x_goal)

        return False

# A 2D state space grid with a set of rectangular obstacles. The grid is fully deterministic
class DetOccupancyGrid2D(object):
    def __init__(self, width, height, obstacles):
        self.width = width
        self.height = height
        self.obstacles = obstacles

    def is_free(self, x):
        for obs in self.obstacles:
            inside = True
            for dim in range(len(x)):
                if x[dim] < obs[0][dim] or x[dim] > obs[1][dim]:
                    inside = False
                    break
            if inside:
                return False
        return True

    def plot(self, fig_num=0):
        fig = plt.figure(fig_num)
        for obs in self.obstacles:
            ax = fig.add_subplot(111, aspect='equal')
            ax.add_patch(
            patches.Rectangle(
            obs[0],
            obs[1][0]-obs[0][0],
            obs[1][1]-obs[0][1],))

### TESTING
'''
# A simple example
width = 10
height = 10
x_init = (0,0)
x_goal = (8,8)
obstacles = [((6,6),(8,7)),((2,1),(4,2)),((2,4),(4,6)),((6,2),(8,4))]
occupancy = DetOccupancyGrid2D(width, height, obstacles)

# A large random example
# width = 101
# height = 101
# num_obs = 15
# min_size = 5
# max_size = 25
# obs_corners_x = np.random.randint(0,width,num_obs)
# obs_corners_y = np.random.randint(0,height,num_obs)
# obs_lower_corners = np.vstack([obs_corners_x,obs_corners_y]).T
# obs_sizes = np.random.randint(min_size,max_size,(num_obs,2))
# obs_upper_corners = obs_lower_corners + obs_sizes
# obstacles = zip(obs_lower_corners,obs_upper_corners)
# occupancy = DetOccupancyGrid2D(width, height, obstacles)
# x_init = tuple(np.random.randint(0,width-2,2).tolist())
# x_goal = tuple(np.random.randint(0,height-2,2).tolist())
# while not (occupancy.is_free(x_init) and occupancy.is_free(x_goal)):
#     x_init = tuple(np.random.randint(0,width-2,2).tolist())
#     x_goal = tuple(np.random.randint(0,height-2,2).tolist())

astar = AStar((0, 0), (width, height), x_init, x_goal, occupancy)

if not astar.solve():
    print "No path found"
    exit(0)

astar.plot_path()
'''