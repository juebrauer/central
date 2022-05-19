class rrt:

    def __init__(self, map, start, goal):
        self.map = map
        self.start = start
        self.goal = goal
            

    def run_single_step(self):
        print("RRT single step")        