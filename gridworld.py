from utils import vec2raster, raster2vec, read_input, write_output, myStruct, find_neighbors_astar, find_neighbors_ucs, find_neighbors_bfs
from queue import PriorityQueue
import time


def bfs(inputs, sourceIndex, targetIndex):
    bfsTargetFound = False
    q = PriorityQueue()
    sourceNode = myStruct(sourceIndex, 0, str(sourceIndex), '0')
    q.put((0, sourceNode))
    visited = {}  # dictionary for fast lookup
    currentNode = sourceNode
    costToReachCurrentNode = 0
    while q.empty() is False and bfsTargetFound is False:
        queueItem = q.get()
        currentNode = queueItem[1]
        costToReachCurrentNode = queueItem[0]

        if currentNode.index == targetIndex:
            bfsTargetFound = True
            break

        if currentNode.index in visited:
            continue
        visited[currentNode.index] = '1'

        children, cost = find_neighbors_bfs(currentNode.index, inputs.numRows, inputs.numCols, inputs.jumpsDict)
        for ii in range(len(children)):
            if children[ii] not in visited:
                totalCost = costToReachCurrentNode + cost[ii]
                childNode = myStruct(children[ii], totalCost, currentNode.path + " " + str(children[ii]),
                                     currentNode.stepCost + " " + str(cost[ii]))
                q.put((totalCost, childNode))

    output = []
    if bfsTargetFound:
        pathList = [int(x) for x in currentNode.path.split()]
        costList = [int(y) for y in currentNode.stepCost.split()]
        for ii in range(len(pathList)):
            vectorIndex = raster2vec(inputs.numRows, inputs.numCols, pathList[ii])
            output.append(vectorIndex + [costList[ii]])

        totalCost = costToReachCurrentNode
        numSteps = len(currentNode.path.split())
    else:
        output = "FAIL"
        totalCost = 0
        numSteps = 0

    return bfsTargetFound, output, totalCost, numSteps


def ucs(inputs, sourceIndex, targetIndex):
    ucsTargetFound = False
    q = PriorityQueue()
    sourceNode = myStruct(sourceIndex, 0, str(sourceIndex), '0')
    q.put((0, sourceNode))
    visited = {}  # dictionary for fast lookup
    currentNode = sourceNode
    costToReachCurrentNode = 0
    while q.empty() is False and ucsTargetFound is False:
        queueItem = q.get()
        currentNode = queueItem[1]
        costToReachCurrentNode = queueItem[0]

        if currentNode.index == targetIndex:
            ucsTargetFound = True
            break

        if currentNode.index in visited:
            continue
        visited[currentNode.index] = '1'

        children, cost = find_neighbors_ucs(currentNode.index, inputs.numRows, inputs.numCols, inputs.jumpsDict)
        for ii in range(len(children)):
            if children[ii] not in visited:
                totalCost = costToReachCurrentNode + cost[ii]
                childNode = myStruct(children[ii], totalCost, currentNode.path + " " + str(children[ii]),
                                     currentNode.stepCost + " " + str(cost[ii]))
                q.put((totalCost, childNode))

    output = []
    if ucsTargetFound:
        pathList = [int(x) for x in currentNode.path.split()]
        costList = [int(y) for y in currentNode.stepCost.split()]
        for ii in range(len(pathList)):
            vectorIndex = raster2vec(inputs.numRows, inputs.numCols, pathList[ii])
            output.append(vectorIndex + [costList[ii]])

        totalCost = costToReachCurrentNode
        numSteps = len(currentNode.path.split())
    else:
        output = "FAIL"
        totalCost = 0
        numSteps = 0

    return ucsTargetFound, output, totalCost, numSteps


def astar(inputs, sourceIndex, targetIndex):
    aStarTargetFound = False
    q = PriorityQueue()
    sourceNode = myStruct(sourceIndex, 0, str(sourceIndex), '0')
    q.put((0, sourceNode))
    visited = {}  # dictionary for fast lookup
    currentNode = sourceNode
    costToReachCurrentNode = 0
    while q.empty() is False and aStarTargetFound is False:
        currentNode = q.get()[1]
        costToReachCurrentNode = currentNode.totalCost

        if currentNode.index == targetIndex:
            aStarTargetFound = True
            break

        if currentNode.index in visited:
            continue
        visited[currentNode.index] = '1'

        children, cost, hValue = find_neighbors_astar(currentNode.index, inputs.numRows, inputs.numCols, inputs.jumpsDict, targetIndex)
        for ii in range(len(children)):
            if children[ii] not in visited:
                totalCost = costToReachCurrentNode + cost[ii]
                childNode = myStruct(children[ii], totalCost, currentNode.path + " " + str(children[ii]),
                                     currentNode.stepCost + " " + str(cost[ii]))
                q.put((totalCost + hValue[ii], childNode))

    output = []
    if aStarTargetFound:
        pathList = [int(x) for x in currentNode.path.split()]
        costList = [int(y) for y in currentNode.stepCost.split()]
        for ii in range(len(pathList)):
            vectorIndex = raster2vec(inputs.numRows, inputs.numCols, pathList[ii])
            output.append(vectorIndex + [costList[ii]])

        totalCost = costToReachCurrentNode
        numSteps = len(currentNode.path.split())
    else:
        output = "FAIL"
        totalCost = 0
        numSteps = 0

    return aStarTargetFound, output, totalCost, numSteps


if __name__ == "__main__":
    tStart1 = time.time()
    for fileNum in range(1,51):
        tStart2 = time.time()
        folderName = "UCS"
        if fileNum<16: folderName = "BFS"
        elif fileNum>30: folderName = "A_Star"

        inputs = read_input(fileNum, folderName)
        sourceIndex = vec2raster(inputs.numRows, inputs.numCols, inputs.source)
        targetIndex = vec2raster(inputs.numRows, inputs.numCols, inputs.target)

        targetFound = False
        totalCost = 0
        numSteps = 0
        outputPath = "FAIL"
        if inputs.algo=="A*":
            targetFound, outputPath, totalCost, numSteps = astar(inputs, sourceIndex, targetIndex)
        elif inputs.algo=="UCS":
            targetFound, outputPath, totalCost, numSteps = ucs(inputs, sourceIndex, targetIndex)
        elif inputs.algo=="BFS":
            targetFound, outputPath, totalCost, numSteps = bfs(inputs, sourceIndex, targetIndex)

        write_output(targetFound, outputPath, totalCost, numSteps, fileNum, folderName)
        print("Finished file #%2d, time=%4.3f seconds"% (fileNum, time.time() - tStart2))

    print(time.time() - tStart1)

