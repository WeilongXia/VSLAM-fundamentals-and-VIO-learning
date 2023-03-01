import matplotlib.pyplot as plt

x_list = list(range(1, 10))
lambda_list = [1.33254e-05, 1.84001e-05, 2.69058e-05, 3.44701e-05, 3.28267e-05, 4.30012e-05, 4.82993e-05, 5.05506e-05, 0.885821]

plt.figure('lambda iteration')
ax = plt.gca()

ax.set_xlabel('noise')
ax.set_ylabel('sigma4/sigma3')

ax.scatter(x_list, lambda_list, c='r', s=20, alpha=0.5)
ax.plot(x_list, lambda_list, c='g', linewidth=1, alpha=0.6)

plt.show()
