import heapq
import math

def planPath(img, carWidthPixel, pixelStep):
    # x, y, theta
    start = (len(img[0]) / 2, len(img), 0)
    # priority, cost, state
    pq = [(heuristic(img, start), 0, start, None)]

    parents = {start: None}
    best = (start, stateScore(img, start))
    visited = set([])

    while pq:
        priority, cost, state, parent = heapq.heappop(pq)
        if state not in visited:
            visited.add(state)
            parents[state] = parent

            if isGoalState(img, state):
                return reconstructPath(parents, state)
            score = stateScore(img, state)
            if score > best[1]:
                best = (state, score)
            for successor in successors(img, state, carWidthPixel, pixelStep):
                newCost = cost + 1
                heapq.heappush(pq, (heuristic(img, state) + newCost, newCost, successor, state))
    return reconstructPath(parents, best[0])


def isGoalState(img, state):
    if state[1] < len(img)/2:
        return True
    return False

def stateScore(img, state):
    return len(img) - state[1]

def heuristic(img, state):
    return state[1] + state[2] * 100# + abs(state[0] - len(img[0]))

def successors(img, state, carWidth, pixelStep):
    successorDeltas = []
    # Straight
    successorDeltas.append((0, pixelStep, 0))
    # Turns
    thetas = [math.pi/6, math.pi/3]
    for theta in thetas:
        r = pixelStep / theta
        dx = r * math.sin(theta)
        dy = r * math.cos(theta)
        successorDeltas.append((-dx, dy, -theta))
        successorDeltas.append((dx, dy, theta))

    # Convert to world frame
    successors = []
    for successorDelta in successorDeltas:
        dx, dy, dtheta = successorDelta
        t = state[2]
        newX = math.cos(t) * dx + math.sin(t) * dy + state[0]
        newY = math.sin(t) * dx - math.cos(t) * dy + state[1]
        newTheta = (t + dtheta) % (2 * math.pi)
        if newX < len(img[0]) and newX > 0 and newY < len(img) \
                and newY > 0 and checkSurroundings(img, (newX, newY), carWidth):
            successors.append((newX, newY, newTheta))
    return successors


def isPoint(img, pos, val):
    """ img is y,x 2D array, pos is x,y point """
    intX = int(pos[0])
    intY = int(pos[1])
    if intX < 0 or intX >= len(img[0]) or intY < 0 or intY >= len(img[1]):
        return False
    return img[intY][intX] == val

def checkSurroundings(img, pos, carWidth):
    on = '\x01'
    if not isPoint(img, pos, on):
        return False
    r = carWidth/2
    t = 0
    while t < 2 * math.pi:
        checkX = int(r*math.cos(t)+pos[0])
        checkY = int(r*math.sin(t)+pos[1])
        
        if not isPoint(img, (checkX, checkY), on):
            return False
        t += math.pi /4
    return True

def reconstructPath(parents, state):
    path = [state]
    while parents[state]:
        state = parents[state]
        path.append(state)
    path.reverse()
    return path
