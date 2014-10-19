import Queue
import math

def planPath(img):
    pq = Queue.PriorityQueue()
    # x, y, theta
    start = (len(img[0]) / 2, len(img), 0)
    # priority, cost, state
    pq.put((heuristic(start), 0, start))

    parents = {start: None}
    best = (start, stateScore(img, start))
    visited = set([])

    while pq:
        priority, cost, state = pq.get()
        if state not in visited:
            visited.add(state)

            if isGoalState(img, state):
                return reconstructPath(parents, state)
            score = stateScore(img, state)
            if score > best[1]:
                best = (state, score)

            for successor in successors(img, state):
                newCost = cost + 1
                pq.put((heuristic(img, state) + newCost, newCost, state))

    return reconstructPath(parents, state[0])


def isGoalState(img, state):
    # Within 15 pixels of the top
    if state[1] < 15:
        return True
    return False

def stateScore(img, state):
    return len(img) - state[1]

def heuristic(img, state):
    return state[1]

def successors(img, state):
    # Turning angles
    thetas = [math.pi/6, math.pi/3]
    # 10 pixels at a time
    r = 10

    successorDeltas = []
    # Straight
    successorDeltas.append((0, r, 0))
    for theta in thetas:
        dx = r * math.sin(theta)
        dy = r * math.scos(theta)
        successorDeltas.append((-dx, dy, -theta))
        successorDeltas.append((dx, dy, theta))

    # Convert to world frame
    successors = []
    for successorDelta in successorDeltas:
        dx, dy, dtheta = successorDelta
        t = state[2]
        newX = math.cos(t) * dx - math.sin(t) * dy + state[0]
        newY = math.sin(t) * dx + math.cos(t) * dy + state[1]
        newTheta = (t + dtheta) % (2 * math.pi)
        if newX < len(img[0]) and newX > 0 and newY < len(img) and newY > 0 and img[newY][newX]:
            successors.append((newX, newY, newTheta))
    return successors

def reconstructPath(parents, state):
    path = [state]
    while parents[state]:
        state = parents[state]
        path.append(state)
    path.reverse()
    return path