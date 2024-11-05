angles = [
    [0, 1, 0],
    [1, 3, 2]
]

minScore = 5
for path in range(2):
    compareAngle = angles[path][0]
    currAngle = compareAngle
    currScore = 0
    for axis in range(3):
      minDist = 5
      minLoc = -1
      for option in range(2):
        if (abs(angles[option][axis] - compareAngle) < minDist):
          minDist = abs(angles[option][axis] - compareAngle)
          minLoc = option
      currScore += pow(minDist, 2)
      compareAngle = angles[minLoc][axis]
      currAngle += compareAngle
    if (currScore < minScore):
      minScore = currScore
      finalAngle = currAngle/3

print(finalAngle)