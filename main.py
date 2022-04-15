import random


class Agent: 
    def __init__(self, type, name):
        # 'searcher' or 'target'
        self.type = type

        # any human readable name
        self.name = name

        self.location = (random.uniform(0,1), random.uniform(0,1), random.uniform(0,1))
        
        self.start_mapping()
        self.update_aoa_reading()
    
    def update_aoa_reading(self):
        self.aoa_reading = (random.uniform(-180,180), random.uniform(0,1))
    
    def start_mapping(self):
        print("Mapping has started!")
    
    def move_robot(self, goal_location):
        total_steps = 100
        steps_taken = 0
        while steps_taken != total_steps:
            self.update_aoa_reading()
            steps_taken += 1
        self.location = goal_location
        print(f'Moving {self.type} {self.name} to {goal_location}')


# input:-searcherToPlace is the searcher agent you need to return the target location of. 
#       -array of searchers/cats of type Agent 
# output: array of length of input (and same order), each element being a 
# location of where that index agent should move to. 
def task1algo1(searcherToPlace, searchers):
    return (0,0,0)


def task3algo(searchers):
    return searchers[0], (0,1,2)


searcher1 = Agent('searcher', 'searcher1')
searcher2 = Agent('searcher', 'searcher2')
searcher3 = Agent('searcher', 'searcher3')
target1 = Agent('target', 'target1')
searchers = [searcher1, searcher2, searcher3]

#  task 1
for searcher in searchers:
    goal_location = task1algo1(searcher, searchers)
    searcher.move_robot(goal_location)

# task 2
#  nothing to be done here. robot has update_aoa_reading() to cover this

# task 3
searcherToMove, targetLocation = task3algo(searchers)
searcherToMove.move_robot(targetLocation)

# task 4
def checkIfTargetAcquired(searcherToMove, target):
    return random.randint(0,1)
searcher_reached_target = checkIfTargetAcquired(searcherToMove, target1)
# stay here until we remove robot
while searcher_reached_target:
    checkIfTargetAcquired(searcherToMove, target1)

# restart program

    



