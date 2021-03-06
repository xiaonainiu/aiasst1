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

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:
    """

    dfsstack = util.Stack()
    visited = [problem.getStartState() ]
    res,_,_ = dfs_helper(problem, problem.getStartState(), visited, dfsstack)

    return res.list

def dfs_helper(problem, state, visited, dfsstack, success = False):
    if (problem.isGoalState(state)):
        success = True
        return (dfsstack, visited, success)
    else:
        for pos in problem.getSuccessors( state ):
            if pos[0] not in visited:
                visited.append( pos[0] )
                dfsstack.push(pos[1])
                tmp_dfsstack, visited, tmp_success = dfs_helper(problem, pos[0], visited, dfsstack)
                if(tmp_success):
                    return (tmp_dfsstack, visited, tmp_success)
                else:
                    dfsstack.pop()
            else:
                continue
        return (dfsstack, visited, success)

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    bfsqueue = util.Queue()
    path = []
    visited = [problem.getStartState()]
    bfsqueue.push( ( problem.getStartState(), path ) )

    while not bfsqueue.isEmpty():

        node, path = bfsqueue.pop()
        if problem.isGoalState(node):
            return path
        for pos in problem.getSuccessors(node):
            if pos[0] not in visited:
                visited.append(pos[0])
                path_tmp = list( path )
                path_tmp.append(pos[1])
                bfsqueue.push( ( pos[0], path_tmp ))
            else:
                continue
    return "error"

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    ucspriorityqueue = util.PriorityQueue()
    path = []
    visited = [problem.getStartState()]
    best_cost = [0]
    ucspriorityqueue.push( (problem.getStartState(), path) ,  problem.getCostOfActions(path))

    while not ucspriorityqueue.isEmpty():
        node, path = ucspriorityqueue.pop()
        if problem.isGoalState(node):
            return path
        for pos in problem.getSuccessors(node):
            path_tmp = list(path)
            path_tmp.append(pos[1])
            if pos[0] not in visited:
                visited.append(pos[0])
                ucspriorityqueue.push((pos[0], path_tmp), problem.getCostOfActions(path_tmp))
                best_cost.append( problem.getCostOfActions(path_tmp) )
            else:
                idx = visited.index( pos[0] )
                if best_cost[idx] > problem.getCostOfActions(path_tmp):
                    ucspriorityqueue.push((pos[0], path_tmp), problem.getCostOfActions(path_tmp))
                    best_cost[idx] = problem.getCostOfActions(path_tmp)
    return "error"


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    astarpriorityqueue = util.PriorityQueue()
    path = []
    astar_closed_list = [problem.getStartState()]
    best_cost = [ problem.getCostOfActions(path) +  heuristic(problem.getStartState(), problem) ]
    astarpriorityqueue.push((problem.getStartState(), path), problem.getCostOfActions(path) +  heuristic(problem.getStartState(), problem) )

    while not astarpriorityqueue.isEmpty():
        node, path = astarpriorityqueue.pop()

        if problem.isGoalState(node):
            return path
        for pos in problem.getSuccessors(node):
            path_tmp = list(path)
            path_tmp.append(pos[1])
            if pos[0] not in astar_closed_list:
                astar_closed_list.append(pos[0])
                astarpriorityqueue.push((pos[0], path_tmp), problem.getCostOfActions(path_tmp) + heuristic(pos[0], problem) )
                best_cost.append(problem.getCostOfActions(path_tmp)  + heuristic(pos[0], problem) )
            else:
                idx = astar_closed_list.index(pos[0])
                if best_cost[idx] > problem.getCostOfActions(path_tmp) + heuristic(pos[0], problem):
                    astarpriorityqueue.push((pos[0], path_tmp), problem.getCostOfActions(path_tmp) + heuristic(pos[0], problem) )
                    best_cost[idx] = problem.getCostOfActions(path_tmp) + heuristic(pos[0], problem)
    return "error"

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
