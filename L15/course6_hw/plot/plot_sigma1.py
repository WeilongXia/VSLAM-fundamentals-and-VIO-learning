import matplotlib.pyplot as plt

x_list = list(range(1, 11))
lambda_list = [1.07474e-06, 4.3e-6, 9.68e-6, 1.72e-5, 2.69e-5, 3.88e-5, 5.28e-5, 6.89e-5, 8.73e-5, 0.000107808]

plt.figure('lambda iteration')
ax = plt.gca()

ax.set_xlabel('noise')
ax.set_ylabel('sigma4/sigma3')

ax.scatter(x_list, lambda_list, c='r', s=20, alpha=0.5)
ax.plot(x_list, lambda_list, c='g', linewidth=1, alpha=0.6)

plt.show()
