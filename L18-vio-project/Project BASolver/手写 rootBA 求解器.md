# 手写 rootBA 求解器

## 项目背景
在 VO 和 VIO 问题求解中，我们通常会构建 Hessian 矩阵，并用舒尔补来求解 BA 问题。大家可能没有注意到用 ceres 求解时默认仅支持 double 类型，这是因为 H 矩阵求逆时如果用 float 类型可能会有失败的情况。
在论文 **Square Root Bundle Adjustment for Large-Scale Reconstruction** 中，作者提出了一种有别于传统求解 BA 问题的新方法。其特点包括:

1. 利用 QR 分解来取代通过 H 矩阵求解 BA 的问题。
2. 即使采用 float 类型依然能保证求解的稳定性，从而提升求解的速度。

 ## 项目任务
在本项目中，我们将试图复现论文 Square Root Bundle Adjustment for Large-Scale Reconstruction（原文见最后链接 1 ）所述方法。为此，我们提供了求解器的大部分框架代码，以及用于验证的仿真数据。需要你完成布置的任务才能让求解器最后生效。主要作业任务包括(在代码中搜索关键词 TODO: task)：

1. 雅克比的计算

   不管是采用传统基于 schur 补的方式，还是新的基于 QR 分解的求解方法，BA 问题的求解通常都需要进行误差函数的构建，以及雅可比的计算。因此，首先请大家根据代码中的残差计算完成相应雅可比的计算。
   note：特征点的优化变量通常有两种选择，逆深度和 xyz。这里，我们将采用特征点的 xyz 作为优化变量。
   
2. QR 分解的执行
   rootBA 最为关键的操作之一就是进行 QR 分解。请在 PerformQR 函数中补充 QR 分解代码，并将相应的 QR 矩阵作用在 landmark 的 storage 矩阵上，参考论文 Figure 2。
   
3. 在 rootBA 中，我们依旧会用 LM 法进行迭代求解。论文中 Figure 3 介绍了利用 6 次 givens 旋转将 lambda 添加到原问题的方式，请在 landmark.hpp 的 Damp() 中实现它。
   (提示:1. 可以打印 givens 旋转后矩阵的 0 元素分布来验证结果的合理性; 2. 为了保证迭代无效后的状态返回，请保留旋转的中间结果来实现状态回归)。

## 其他说明
关于代码的编译和运行请参考代码的 readme.md

## 参考资料
[1] https://arxiv.org/abs/2103.01843
[2] rootBA 代码流程 见项目文件夹 /doc