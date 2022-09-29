# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).

"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem: SearchProblem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:
    """
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()
    

def breadthFirstSearch(problem: SearchProblem):
    from util import Queue

    startCoord = problem.getStartState()
    queue = Queue()
    queue.push((startCoord,[]))
    visited = [startCoord]

    while queue:
        curCoord, path = queue.pop()
        if problem.isGoalState(curCoord):
            return path

        nextNodes = problem.getSuccessors(curCoord)
        for nextNode in nextNodes:
            coord, way, cost = nextNode
            if coord not in visited:
                queue.push((coord,path+[way]))
                visited.append(coord)
    return []

def leeSearch(problem: SearchProblem):
    from util import Queue

    startCoord = problem.getStartState()
    queue = Queue()
    queue.push((startCoord,[]))
    visited = [startCoord]

    while queue:
        curCoord, path = queue.pop()
        if problem.isGoalState(curCoord):
            return path

        nextNodes = problem.getSuccessors(curCoord)
        for nextNode in nextNodes:
            coord, way, cost = nextNode
            if coord not in visited:
                queue.push((coord,path+[way]))
                visited.append(coord)
    return []

def uniformCostSearch(problem: SearchProblem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem: SearchProblem, heuristic=nullHeuristic):
    from util import PriorityQueue;

    start_coord = problem.getStartState()
    queue = PriorityQueue()
    queue.push((start_coord, []),0)
    costVisited = { start_coord: 0 }

    while queue:
        cur_coord, path = queue.pop()

        if problem.isGoalState(cur_coord):
            return path

        next_nodes = problem.getSuccessors(cur_coord)
        for next_node in next_nodes:
            neighCoord, way, price = next_node
            new_cost = costVisited[cur_coord] + price
            if neighCoord not in costVisited or new_cost < costVisited[neighCoord]:
                priority = new_cost + heuristic(neighCoord, problem)
                queue.push((neighCoord, path + [way]), priority)
                costVisited[neighCoord]= new_cost
    return []

def greedySearch(problem: SearchProblem, heuristic=nullHeuristic):
    from util import PriorityQueue;

    start_coord = problem.getStartState()
    queue = PriorityQueue()
    queue.push((start_coord, []),0)
    visited = []

    while queue:
        cur_coord, path = queue.pop()

        if problem.isGoalState(cur_coord):
            return path

        next_nodes = problem.getSuccessors(cur_coord)
        for next_node in next_nodes:
            neighCoord, way, price = next_node
            if neighCoord not in visited:
                priority = heuristic(neighCoord, problem)
                queue.push((neighCoord, path + [way]), priority)
                visited.append(neighCoord)
    return []


# Abbreviations
bfs = breadthFirstSearch
lee = leeSearch
dfs = depthFirstSearch
astar = aStarSearch
greedy = greedySearch
ucs = uniformCostSearch
