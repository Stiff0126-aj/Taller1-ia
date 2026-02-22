from algorithms.problems import SearchProblem
import algorithms.utils as utils
from world.game import Directions
from algorithms.heuristics import  nullHeuristic


def tinyHouseSearch(problem: SearchProblem):
    """
    Returns a sequence of moves that solves tinyHouse. For any other building, the
    sequence of moves will be incorrect, so only use this for tinyHouse.
    """
    s = Directions.SOUTH
    w = Directions.WEST
    return [s, s, w, s, w, w, s, w]


def depthFirstSearch(problem: SearchProblem):
    """
    Search the deepest nodes in the search tree first.
    Punto 1: Respuesta a Baliza de Emergencia
    """
    frontera = utils.Stack()
    frontera.push((problem.getStartState(), []))
    
    visitados = set()

    while not frontera.isEmpty():
        estado, acciones = frontera.pop()

        if problem.isGoalState(estado):
            return acciones

        if estado not in visitados:
            visitados.add(estado)
            for sucesor, accion, costo in problem.getSuccessors(estado):
                if sucesor not in visitados:
                    # Construimos el nuevo camino sumando la dirección actual
                    nuevo_camino = acciones + [accion]
                    frontera.push((sucesor, nuevo_camino))
    
    return []

def breadthFirstSearch(problem: SearchProblem):
    """
    Search the shallowest nodes in the search tree first.
    """
    #Usamos Queue (Cola) para BFS (FIFO)
    frontera = utils.Queue()
    frontera.push((problem.getStartState(), []))
    
    visitados = set()

    while not frontera.isEmpty():
        estado, acciones = frontera.pop()

        if problem.isGoalState(estado):
            return acciones

        if estado not in visitados:
            visitados.add(estado)
            for sucesor, accion, costo in problem.getSuccessors(estado):
                if sucesor not in visitados:
                    nuevo_camino = acciones + [accion]
                    frontera.push((sucesor, nuevo_camino))
                    
    return []


def uniformCostSearch(problem: SearchProblem):

    """
    Search the node of least total cost first.
    """
      
    colaPrioridad = utils.PriorityQueue()

    inicioRobot = problem.getStartState()
    colaPrioridad.push((inicioRobot, [], 0),0)

    # Diccionario con cada estado y sus costos
    visitado = {}
    visitado [inicioRobot] = 0 

    while not colaPrioridad.isEmpty():

        state, actions, costSoFar = colaPrioridad.pop()

        if problem.isGoalState(state):
            return actions

        if state in visitado and costSoFar > visitado[state]:
            continue

        successors = problem.getSuccessors(state)
        for successor in successors:
            siguienteState = successor[0]
            successorAction = successor[1]
            siguenteCost = successor[2]

            nuevoCosto = costSoFar + siguenteCost
            nuevoAction = actions + [successorAction]

            if siguienteState not in visitado:
                visitado [siguienteState] = nuevoCosto
                colaPrioridad.push((siguienteState, nuevoAction, nuevoCosto), nuevoCosto)

            #Ya visitamos este estado pero encontramos un camino más barato
            elif nuevoCosto < visitado [siguienteState]:
                visitado [siguienteState] = nuevoCosto
                colaPrioridad.push((siguienteState, nuevoAction, nuevoCosto), nuevoCosto)
    return []


def aStarSearch(problem: SearchProblem, heuristic=nullHeuristic):
    """
    Search the node that has the lowest combined cost and heuristic first.
    """
    # Cola de prioridad: ordena por f(n) = g(n) + h(n)
    frontier = utils.PriorityQueue()

    # Estado inicial
    startState = problem.getStartState()
    frontier.push((startState, [], 0), heuristic(startState, problem))

    # Diccionario de costos mínimos encontrados hasta cada estado
    bestCost = {}

    while not frontier.isEmpty():
        state, actions, costSoFar = frontier.pop()

        # Si llegamos al objetivo, devolvemos el plan
        if problem.isGoalState(state):
            return actions

        # Si ya llegamos a este estado con menor costo, ignoramos
        if state in bestCost and bestCost[state] <= costSoFar:
            continue

        bestCost[state] = costSoFar

        # Expandir sucesores
        for successor, action, stepCost in problem.getSuccessors(state):
            newCost = costSoFar + stepCost
            newActions = actions + [action]
            priority = newCost + heuristic(successor, problem)

            frontier.push((successor, newActions, newCost), priority)

    return []


# Abbreviations (you can use them for the -f option in main.py)
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
