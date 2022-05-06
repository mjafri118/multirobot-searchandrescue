'''
Harvard CS 286 Spring 2022 - Two robots
'''

import random
# from typing import DefaultDict
from ninja_turtles import NinjaTurtles
import rospy
import time
from pprint import pprint

class Robot:
    def __init__(self, x, y, name):
        # index for row position in grid env
        self.x = x
        # index for column position in grid env
        self.y = y
        self.neighbors = set()
        self.sim_bot = NinjaTurtles(x,y,name)
        # self.sim_bot.remove_bot('turtle1')
        self.sim_bot.add_bot()

class Env:
    def __init__(self, bots, size=10):
        # number of rows and columns in square grid
        self.size = size 
        # list of Robot objects
        self.bots = bots 
        # number of Robots in this Env
        self.num_bots = len(bots)
        # 2D list containing sets, empty or otherwise, of Robot objects at each coordinate location
        self.grid = self.update_grid()
        
        # List to store different flocks
        self.flocks = []
        
        self.steps = 0



    ################ General Helper Functions ######################        
    def move_bot(self, bot, move_cmd):
        '''
        Update position of bot (Robot obj) using move_cmd (tuple).
        Note that move_cmd = (x,y) where x and y are each integers between -1 and 1, inclusive.
        '''
        bot.x += move_cmd[0]
        bot.y += move_cmd[1]
        if bot.x >= self.size:
            bot.x = self.size-1
        if bot.x < 0:
            bot.x = 0
        if bot.y >= self.size:
            bot.y = self.size-1
        if bot.y < 0:
            bot.y = 0
        
        bot.sim_bot.go_to(bot.x,bot.y)


    
    def update_grid(self):
        grid = [[set() for i in range(self.size)] for i in range(self.size)]
        for b in self.bots:
            grid[b.x][b.y].add(b)
        return grid


    def display_grid(self):
        self.grid = self.update_grid()
        # prints grid with number of bots in each coordinate location
        print("Grid["+("%d" %self.size)+"]["+("%d" %self.size)+"]")
        for j in range(self.size-1,-1,-1):
            print(j ,'|')
            for i in range(0,self.size):
                print(len(self.grid[i][j]))
            print()
        
        print("--")
        for i in range(0,self.size):
            print("-")
        print()

        print("  ")
        for i in range(0,self.size):
            print(i)
        print()
    
    
    def _move_towards_step(self, bot, loc):
        '''
        Moves a bot (Robot obj) by one step towards the location.
        '''
        x,y = 0,0
        '''
          Your code to update x,y
        '''
        if bot.x - loc[0] < 0:
            x+=1
        elif bot.x - loc[0] > 0:
            x-=1
        if bot.y - loc[1] < 0:
            y+=1
        elif bot.y - loc[1] > 0:
            y-=1
       
        self.move_bot(bot, [x,y])

        return (x,y)


    def _move_away_step(self, bot, loc):
        '''
        Moves a bot (Robot obj) by one step away from a location.
        '''
        x,y = 0,0
        '''
          Your code to update x,y
        '''
        if bot.x - loc[0] > 0:
            x+=1
        elif  bot.x - loc[0] < 0:
            x-=1
        elif bot.x == loc[0]:
            x = [-1,1][random.randrange(2)]
        if bot.y - loc[1] > 0:
            y+=1
        elif bot.y - loc[1] < 0:
            y-=1
        elif bot.y == loc[1]:
            y = [-1,1][random.randrange(2)]
       
        rand = [1,2,3][random.randrange(3)]
        if rand == 1:
            self.move_bot(bot, [x,0])
            return (x,0)
        elif rand == 2:
            self.move_bot(bot, [0,y])
            return (0,y)
        else:
            self.move_bot(bot, [x,y])
            return (x,y)


    def _move_random_step(self):
        '''
        Moves a bot (Robot obj) by one step towards a random location.
        '''
        x_step, y_step = 0 ,0
        '''
          Your code to update y_step, y_step
        '''
        x_step += [-1,0,1][random.randrange(3)]
        if x_step == 0:
            y_step += [-1,1][random.randrange(2)]
        else:
            y_step += [-1,0,1][random.randrange(3)]

        return (x_step, y_step)
    

    def get_centroid(self, bots=[]):
        '''
        Calulcate the centroid of a flock using bot (Robot Obj) positions
        '''
        x_c, y_c, x_sum, y_sum = 0,0,0,0
        '''
        Your code to update x_c, y_c
        '''
        if bots == [] or bots is None:
            bots = self.bots

        for b in bots:
            x_sum+=b.x
            y_sum+=b.y
        
        x_c = int(x_sum/self.num_bots)
        y_c = int(y_sum/self.num_bots)

        return (x_c, y_c)

    
    def bot_sense(self, bot, sense_r):
        '''
        Get the neighboring robots of a bot (Robot obj) within its sensing radius
        Hint: self.grid stores the positions of all the bots (Robot obj) in a given iteration. This can be used to find the neightbors of a bot using its position.
        Note: A bot is not a neighbor of itself.
        '''
        for b in self.bots:
            b.neighbors = set()
            if abs(b.x - bot.x)<=sense_r and abs(b.y - bot.y)<=sense_r:
                bot.neighbors.add(b)
               
        


    def update_flocks(self, sense_r=0):
        '''
        Generate flock(s) at each timestep based on the position of each robot and the robots within its neighborhood
        '''
        self.flocks = []

        '''
            Your code to update self.flocks
        '''
        flat_flocks = set()

        for b in self.bots:
            if b not in flat_flocks:
                self.bot_sense(b,sense_r)
                flock = set()
                flock.add(b)
                flat_flocks.add(b)
                flock = flock.union(set(b.neighbors))
                flat_flocks = flat_flocks.union(set(b.neighbors))
                self.flocks.append(flock)
    
    
    ################ General Helper Functions ######################


    ################ Centralized communication ######################
    def flock(self, loc, t=5):
        '''
        Aggregate all bots to grid coordinate loc (tuple)
        Then have the flock safe wander for t (int) steps.
        Afterwards, disperse. 
        Display the grid after each of these steps, including after each safe wander interation.
        '''
        print("AGGREGATE")
        self.aggregate(loc)
        self.display_grid()
        time.sleep(3)
        
        for count in range(t):
            print("SAFE WANDER", count)
            self.safe_wander(True)
            self.display_grid()
        
        time.sleep(3)
        print("DISPERSE")
        self.disperse()
        self.display_grid()


    def aggregate(self, loc):
        '''
        Move all bots to grid coordinate loc (tuple).
        After this method is called, all aggregation should be complete (each bot will have taken all
        steps, likely more than one, to completely aggregate.)
        
        Use move_bot() and _move_towards() functions
        '''
        for bot in self.bots:
            while (bot.x != loc[0] and bot.y != loc[1]):
                self._move_towards_step(bot, loc)
                # pprint(vars(bot))

    
    def safe_wander(self, flock=False):
        '''
        Move each bot one step. 
        If flock, all bots in a flock move in same random direction.
        Otherwise, each bot moves in its own random direction

        Use move_bot() and _move_random_step() functions
        '''
        x,y = self._move_random_step()
        if flock:
            for bot in self.bots:
                self.move_bot(bot, [x,y])
        else:
            for bot 2 copy.move_bot(bot, [x_temp,y_temp])

            
    def disperse(self):
        '''
        Move all bots away from centroid, each in a random direction, for 3 steps.
        Use the move_bot(), _move_away_step() and get_centroid() functions.
        '''
        x_c,y_c = se2 copyve_away_step(bot, [x_c,y_c])
            self._move_away_step(bot, [x_c,y_c])
    
    ################ Centralized communication ######################


    ################ Decentralized Communication ######################
    def flock_sense(self, sense_r, t=5):
        '''
        Aggregate all bots using sensing radius sense_r.
        Then have the flock(s) safe wander for t (int) steps.
        Afterwards, disperse flock/s beyond aggregation centroid/s. 
        Display the grid after each of these steps, including after each safe wander interation.
        '''
        print("AGGREGATE")
        self.aggregate_sense(sense_r)
        time.sleep(3)

        for count in range(t):
            print("SAFE WANDER", count)
            self.safe_wander(True)
        
        time.sleep(3)
        print("DISPERSE")
        self.disperse_sense()
        self.display_grid()


    def aggregate_sense(self, sense_r):
        '''
        Aggregate bots into one or more flocks, each using sensing radius of sense_r (int).
        Use bot_sense() and update_flocks() functions
        '''
        self.update_flocks(sense_r)
        


    def safe_wander_sense(self, flock=False):
        '''
        Move each bot one step. 
        If flock, all bots in a flock move in same random direction.
        Otherwise, each bot moves in its own random direction
        '''
        self.safe_wander(flock)
        



    def disperse_sense(self):
        '''
        Move all bots away from their respective flock's centroid.
        '''
        for bot in self.bots:
            for f in self.flocks:
                if bot in f:
                    x_c,y_c = self.get_centroid(list(f))
                    self._move_away_step(bot, [x_c,y_c])


    ################ Decentralized Communication ######################
    def display_grid(self):
        self.grid = self.update_grid()
        # prints grid with number of bots in each coordinate location
        print("Grid["+("%d" %self.size)+"]["+("%d" %self.size)+"]")
        for j in range(self.size-1,-1,-1):
            print(j ,'|')
            for i in range(0,self.size):
                print(len(self.grid[i][j]))
            print()
        
        print("--")
        for i in range(0,self.size):
            print("-")
        print()

        print("  ")
        for i in range(0,self.size):
            print(i)
        print()

if __name__ == "__main__":
    rospy.init_node("Cowabunga")
    
    #Use the same names when generating results for part HW1 Q1.(d)
    bot1 = Robot(9,1,'t1')
    bot1.sim_bot.remove_bot("turtle1") #remove initial bot
    bots = [bot1]
    env1 = Env(bots)
    
    # env1.display_grid()

    bot1.sim_bot.rotate(30,45,0)
    bot1.sim_bot.go_to(8.5,1.5)
    time.sleep(1)
    bot1.sim_bot.go_to(7,2.5)
    time.sleep(1)
    bot1.sim_bot.go_to(6.5,3.5)
    time.sleep(1)
    bot1.sim_bot.go_to(5,4.5)
    time.sleep(1)
    bot1.sim_bot.go_to(5,5) #TODO: move2goal
    bot1.sim_bot.rotate(30,360,0)
    print('Step 1 ----------------Coverage: 1 Bot')

    time.sleep(3)
    bot2 = Robot(9,1,'t2')
    bots = [bot2]
    env2 = Env(bots)
    bot1.sim_bot.go_to(5,5.2)
    time.sleep(1)
    bot1.sim_bot.go_to(4.5,5.1)
    time.sleep(1)
    bot1.sim_bot.go_to(3.5,5)
    time.sleep(1)
    bot1.sim_bot.go_to(2.5,5)
    bot1.sim_bot.rotate(30,360,0)
    time.sleep(3)
    bot2.sim_bot.go_to(9,1.5)
    time.sleep(1)
    bot2.sim_bot.go_to(8,2.5)
    time.sleep(1)
    bot2.sim_bot.go_to(7.5,3.5)
    time.sleep(1)
    bot2.sim_bot.go_to(7.5,5)
    bot2.sim_bot.rotate(30,360,0)
    print('Step 2 ----------------Coverage Split: 2 Bots')

    time.sleep(3)
    bot3 = Robot(9,9,'t3')
    bot3.sim_bot.rotate(60,180,0)
    bot1.sim_bot.rotate(30,360,0)
    bot2.sim_bot.rotate(30,360,0)
    time.sleep(3)
    bot2.sim_bot.go_to(7.5,5.5)
    time.sleep(1)
    bot2.sim_bot.go_to(8,6)
    time.sleep(1)
    bot2.sim_bot.go_to(8.5,7)
    time.sleep(1)
    bot2.sim_bot.go_to(8.8,8.2)
    print('Step 3 ----------------Hunting: Based on AOA signal strength')

    # only need remove once
    # bot1.sim_bot.remove_bot('turtle1') 

    # bots = [bot1, bot2, bot3, bot4]

    # env = Env(bots)

    # env.display_grid()

    # env.flock((5,5))

    # env.flock_sense(2,10)