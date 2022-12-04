import matplotlib.pyplot as plt

x_list = list(range(0, 12))
lambda_list = [0.001, 699.051, 1864.14, 1242.76, 414.252, 138.084, 46.028, 15.3427, 5.11423, 1.70474, 0.568247, 0.378832]

plt.figure('lambda iteration')
ax = plt.gca()

ax.set_xlabel('x')
ax.set_ylabel('lambda')

ax.scatter(x_list, lambda_list, c='r', s=20, alpha=0.5)
ax.plot(x_list, lambda_list, c='g', linewidth=1, alpha=0.6)

plt.show()
