import numpy as np


def objective(x):
    return (x+1)*(x+1) + 0.5  # Optimum at (-1, 0.5)


def descend():
    x = 0
    dx = 1.0e-4
    lr = 1.0e-1

    iteration = 0
    MAX_ITERATIONS = 400
    TOLERANCE = 1.0e-4
    while True:
        if iteration > MAX_ITERATIONS:
            print("Maximum number of iterations met.")
            break
        y = objective(x)
        grad = (objective(x + dx) - y)/dx
        x_new = x - lr * grad
        print(round(x_new, 4), round(y, 4), round(grad, 4))
        if abs(x - x_new) < TOLERANCE:
            print("Found it! Iterations:", iteration)
            break
        x = x_new

        iteration += 1


if __name__ == '__main__':
    print("The objective lies at:", -1, objective(-1))
    descend()

