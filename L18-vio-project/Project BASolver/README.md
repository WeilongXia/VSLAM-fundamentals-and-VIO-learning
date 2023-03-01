# BASolver
本项目旨在重写一套基于 QR 分解的 Bundle Adjustment 求解器。


## 作业要求
1. 完成雅可比和残差函数的计算
2. QR 分解及storage矩阵的填充
3. 完成 Givens旋转的构建

当代码中的TODO没有被更新时，rootba结果将无效，而在完成任务后，rootba将提供合理的优化结果。另外，代码中我们提供了 ceres 的优化结果作为对比。

## 项目依赖
ceres 2.1.0，Eigen 3.4，Tbb (sudo apt-get install libtbb-dev)

## 编译方式
```
cd rootBA
mkdir build
cd build
cmake ..
make
```
## 运行
```
cd build
./benchmark_rootba
```

