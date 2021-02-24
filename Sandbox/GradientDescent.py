import numpy as np


def objective(x):
    return (x+1)*(x+1) + 0.5  # Optimum at (-1, 0.5)


def descend():
    x = 0
    dx = 1.0e-4
    lr = 1.0e-1

    iteration = 1
    MAX_ITERATIONS = 400
    TOLERANCE = 1.0e-4
    while True:
        if iteration > MAX_ITERATIONS:
            print("Maximum number of iterations met.")
            break

        y = objective(x)
        grad = (objective(x + dx) - y)/dx
        x_new = x - lr * grad

        if abs(x - x_new) < TOLERANCE:
            print("Found it! Iterations:", iteration)
            break
        x = x_new

        iteration += 1


def fit_poly():
    x = 1000

    iteration = 1
    MAX_ITERATIONS = 100
    TOLERANCE = 1.0e-5

    # Need at least three points:
    data = np.empty((2, MAX_ITERATIONS))
    data[:] = np.nan
    data[:, 0] = np.array([x, objective(x)])
    data[:, 1] = np.array([x+0.0001, objective(x+0.0001)])
    data[:, 2] = np.array([x-0.0001, objective(x-0.0001)])
    index = 3

    while True:
        if iteration > MAX_ITERATIONS:
            print("Maximum number of iterations met.")
            break

        poly = np.polyfit(data[0, 0:index], data[1, 0:index], 2)
        x_new = -poly[1]/(2*poly[0])  # -b/2a

        if abs(x - x_new) < TOLERANCE:
            print("Found it! Iterations:", iteration)
            break
        x = x_new
        data[:, index] = np.array([x, objective(x)])

        iteration += 1
        index += 1


if __name__ == '__main__':
    print("The objective lies at:", -1, objective(-1))
    # descend()
    fit_poly()
    # This works within one or two iterations, depending on the initial conditions. What is most
    # useful however is that the method works for maxima AND minima!

