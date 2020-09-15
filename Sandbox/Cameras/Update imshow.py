import matplotlib.pyplot as plt
import numpy as np

images = []
for i in range(20):
    if i % 2 == 0:
        images.append(np.zeros((2000, 2000)))
    else:
        images.append(np.ones((2000, 2000)))

fg = plt.figure()
ax = fg.gca()
imstart = np.diag(np.ones(2000))
h = ax.imshow(imstart)
print(imstart.shape, type(imstart), type(imstart[0, 0]), imstart[0, 0])
for image in images:
    print(image.shape, type(image), type(image[0, 0]), image[0, 0])

    h.set_data(image)
    plt.draw()
    plt.pause(1e-1)

plt.close()