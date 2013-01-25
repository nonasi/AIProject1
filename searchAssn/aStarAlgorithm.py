def aStarSearch(problem, heuristic):

    """
    Definitions:

    state = ( (x,y), "dir", pathCost )

    stateNode = (state, g(n))
    
    frontierNode = (stateNode, f(n))

    explored = [ childStateNode, parentStateNode ]
    """

    initialState = (problem.getStartState(), "No Direction", 0)

    initialStateNode = (inState, 0)

    frontier = PriorityQueue()
    
    """ heuristics take (position=(x,y), problem) """
    frontier += (initialStateNode, heuristic(problem.getStartState(), problem))

    explored += [(initialState),(initialState)]
    
    path, foundSoln = aStarHelper(problem=problem, curState=initialState,frontier=frontier,heuristic=heuristic)

    """ ... do formatting ... """

    return path


def aStarHelper(problem, curState, frontier, heuristic):
    
    while not frontier.isEmpty():

        explored = []

        # get state with lowest f(n)

        curState = frontier.pop()

        # NOTE: curState is-a frontierNode (look at Definition)

        # store its g(n)
        nowGN = curState[0][1]

        currentCoordinates = curState[0][0][0]

        # successors is a LIST of "state"s
        successors = problem.getSuccessors(currentCoordinates)

        for successor in range(len(successors)):

            # parent g(n) + pathCost
            successorGN = nowGN + successors[successor][2]

            successorFN = successorGN + heuristic(currentCoordinates, problem)

            # if this (x,y) has already been explored
            if successors[successor][0] in [i[0][0][0] for i in explored]:
                
                # there can only be MAX 1 of these already in there
                # because we REPLACE, not APPEND

                # not sure how to do this
                oldFN = FN of node in explored

                if oldFN > successorFN:
                    remove the element in explored containing oldFN
                    append successors[successor]

                else: continue # Required to go to next successor in for-loop

            else:
                successorNode = (successors[successor], successorGN)
                explored += [successorNode, curState[0]]
                frontier += (successorNode, successorFN)
                if successorNode is goalState:
                    return explored, true

def aStarPathHelper(explored):
    """
    pretty much the same as ucsPathHelper() and bfsPathHelper()
    only difference is the format of the LIST named explored
    """


