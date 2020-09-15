import numpy as np


def swapRowCol(matrix, location, target):
    # print("1:", matrix)
    matrix[:, [location, target]] = matrix[:, [target, location]]
    # print("2:", matrix)
    matrix[[location, target], :] = matrix[[target, location], :]
    # print("3:", matrix)
    return matrix


orientation = [1, 0, 2]
orientation = [0, 1, 2]

c, s = np.cos(np.pi/2), np.sin(np.pi/2)
R = np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]])
P = np.identity(3)

rotationAxis = [element for element, index in enumerate(orientation) if element == index]
print("rotationAxis =", rotationAxis)
if len(rotationAxis) == 1:
    rotationAxis = rotationAxis[0]
    otherAxes = orientation.copy()
    otherAxes.remove(rotationAxis)
    print("rotationAxis =", rotationAxis)
    print("otherAxes =", otherAxes)
    if rotationAxis != 2:
        # Swap some columns and rows to make a correct rotation
        R = swapRowCol(R, rotationAxis, 2)
    P[:, [otherAxes[0], otherAxes[1]]] = P[:, [otherAxes[1], otherAxes[0]]]
elif rotationAxis == orientation:
    R = np.identity(3)
else:
    P[:, [orientation[0], orientation[1]]] = P[:, [orientation[1], orientation[0]]]
    P[:, [orientation[1], orientation[2]]] = P[:, [orientation[2], orientation[1]]]
    R = P
if P.sum() > 3:
    raise ValueError

print(R)