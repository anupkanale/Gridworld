import numpy as np


class inputVars:
    def __init__(self, algo, numRows, numCols, source, target, jumpStartYear, jumpEndYear, jumpX, jumpY, numJumpChannels):
        self.algo = algo
        self.numRows = numRows
        self.numCols = numCols
        self.source = source
        self.target = target
        self.jumpStartYear = jumpStartYear
        self.numJumpChannels = numJumpChannels

        jumpLocations = [jumpStartYear, jumpX, jumpY, jumpEndYear]
        self.jumpsDict = {}
        for ii in range(numJumpChannels):
            jumpStartIndex1 = vec2raster(numRows, numCols,
                                         [jumpLocations[0][ii], jumpLocations[1][ii], jumpLocations[2][ii]])
            if jumpStartIndex1 in self.jumpsDict:
                self.jumpsDict[jumpStartIndex1] += [jumpLocations[3][ii]]
            else:
                self.jumpsDict[jumpStartIndex1] = [jumpLocations[3][ii]]

            jumpStartIndex2 = vec2raster(numRows, numCols,
                                         [jumpLocations[3][ii], jumpLocations[1][ii], jumpLocations[2][ii]])
            if jumpStartIndex2 in self.jumpsDict:
                self.jumpsDict[jumpStartIndex2] += [jumpLocations[0][ii]]
            else:
                self.jumpsDict[jumpStartIndex2] = [jumpLocations[0][ii]]


class myStruct:
    def __init__(self, index, totalCost, path, stepCost):
        self.index = index  # raster index of the current node
        self.totalCost = totalCost # cost to arrive at current node from the Source
        self.path = path  # path to the current node from the source
        self.stepCost = stepCost  # cost of each step in the path

    def __lt__(self, other):
        return other


def vec2raster(numRows, numCols, currentNode):
    return currentNode[0]*numCols*numRows + currentNode[1]*numCols + currentNode[2]


def raster2vec(numRows, numCols, rasterIndex):
    year = int(rasterIndex / (numCols*numRows))
    colIndex = (rasterIndex - year*numRows*numCols) % numCols
    rowIndex = int((rasterIndex - year*numRows*numCols) / numCols)
    return [year, rowIndex, colIndex]


def read_input(fileNum, folderName):
    fHandle = open("GradingTestcases/" + folderName + "/input" + str(fileNum) + ".txt")
    file_lines = fHandle.readlines()
    algo = file_lines[0].strip()

    numRows = int(file_lines[1].split()[0])
    numCols = int(file_lines[1].split()[1])

    source = [int(file_lines[2].split()[0]), int(file_lines[2].split()[1]), int(file_lines[2].split()[2])]
    target = [int(file_lines[3].split()[0]), int(file_lines[3].split()[1]), int(file_lines[3].split()[2])]

    jumpStartYear = []
    jumpEndYear = []
    jumpX = []
    jumpY = []
    numJumpChannels = int(file_lines[4])
    for ii in range(numJumpChannels):
        jumpStartYear.append(int(file_lines[5+ii].split()[0]))
        jumpX.append(int(file_lines[5+ii].split()[1]))
        jumpY.append(int(file_lines[5+ii].split()[2]))
        jumpEndYear.append(int(file_lines[5+ii].split()[3]))

    fHandle.close()

    params = inputVars(algo, numRows, numCols, source, target, jumpStartYear, jumpEndYear, jumpX, jumpY, numJumpChannels)
    return params


def write_output(targetFound, outputPath, totalCost, numSteps, fileNum, folderName):
    fHandle = open("myOutputs/" + folderName + "/myOutput" + str(fileNum)+ ".txt", 'w')

    if targetFound:
        # convert to string a write to file
        fHandle.write(str(totalCost) + '\n')
        fHandle.write(str(numSteps) + '\n')
        for ii in range(len(outputPath)):
            line = list(map(str, outputPath[ii]))
            for jj in range(4):
                fHandle.write(line[jj] + ' ')
            fHandle.write('\n')
    else:
        fHandle.write('FAIL')
    fHandle.close()


def find_neighbors_bfs(currentNodeIndex, numRows, numCols, jumpsDict):
    currentNode = raster2vec(numRows, numCols, currentNodeIndex)
    neighbors = []
    dRow = [-1, -1, -1, 0, 0, 1, 1, 1]
    dCol = [-1, 0, 1, -1, 1, -1, 0, 1]
    cost = []
    for ii in range(8):
        neighborRow = currentNode[1] + dRow[ii]
        neighborCol = currentNode[2] + dCol[ii]
        if 0 <= neighborRow < numRows and 0 <= neighborCol < numCols:
            cost.append(1)
            neighborIndex = vec2raster(numRows, numCols, [currentNode[0], neighborRow, neighborCol])
            neighbors.append(neighborIndex)

    if currentNodeIndex in jumpsDict:
        for ii in range(len(jumpsDict[currentNodeIndex])):
            neighborIndex = vec2raster(numRows, numCols, [jumpsDict[currentNodeIndex][ii], currentNode[1], currentNode[2]])
            neighbors.append(neighborIndex)
            cost.append(1)

    return neighbors, cost


def find_neighbors_ucs(currentNodeIndex, numRows, numCols, jumpsDict):
    currentNode = raster2vec(numRows, numCols, currentNodeIndex)
    neighbors = []
    dRow = [-1, -1, -1, 0, 0, 1, 1, 1]
    dCol = [-1, 0, 1, -1, 1, -1, 0, 1]

    cost = []
    for ii in range(8):
        neighborRow = currentNode[1] + dRow[ii]
        neighborCol = currentNode[2] + dCol[ii]
        if 0 <= neighborRow < numRows and 0 <= neighborCol < numCols:
            if ii in [0, 2, 5, 7]:
                cost.append(14)
            else:
                cost.append(10)
            neighborIndex = vec2raster(numRows, numCols, [currentNode[0], neighborRow, neighborCol])
            neighbors.append(neighborIndex)

    if currentNodeIndex in jumpsDict:
        for ii in range(len(jumpsDict[currentNodeIndex])):
            neighborIndex = vec2raster(numRows, numCols, [jumpsDict[currentNodeIndex][ii], currentNode[1], currentNode[2]])
            neighbors.append(neighborIndex)
            cost.append(abs(currentNode[0] - jumpsDict[currentNodeIndex][ii]))

    return neighbors, cost


def heuristic(node, target):
    return np.linalg.norm(np.array(node) - np.array(target), ord=2)


def find_neighbors_astar(currentNodeIndex, numRows, numCols, jumpsDict, targetIndex):
    currentNode = raster2vec(numRows, numCols, currentNodeIndex)
    targetVectorIndex = raster2vec(numRows, numCols, targetIndex)
    neighbors = []
    dRow = [-1, -1, -1, 0, 0, 1, 1, 1]
    dCol = [-1, 0, 1, -1, 1, -1, 0, 1]

    cost = []
    hValue = []
    for ii in range(8):
        neighborRow = currentNode[1] + dRow[ii]
        neighborCol = currentNode[2] + dCol[ii]
        neighborVectorIndex = [currentNode[0], neighborRow, neighborCol]
        if 0 <= neighborRow < numRows and 0 <= neighborCol < numCols:
            hValue.append(heuristic(neighborVectorIndex, targetVectorIndex))
            if ii in [0, 2, 5, 7]:
                cost.append(14)
            else:
                cost.append(10)
            neighborRasterIndex = vec2raster(numRows, numCols, neighborVectorIndex)
            neighbors.append(neighborRasterIndex)

    if currentNodeIndex in jumpsDict:
        for ii in range(len(jumpsDict[currentNodeIndex])):
            neighborVectorIndex = [jumpsDict[currentNodeIndex][ii], currentNode[1], currentNode[2]]
            neighborRasterIndex = vec2raster(numRows, numCols, neighborVectorIndex)
            neighbors.append(neighborRasterIndex)
            cost.append(abs(currentNode[0] - jumpsDict[currentNodeIndex][ii]))

            hValue.append(heuristic(neighborVectorIndex, targetVectorIndex))

    return neighbors, cost, hValue
