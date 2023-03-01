#pragma once

#include <utility.hpp>

// 全局命名空间定义为 rootBA
namespace rootBA {

    // 定义鲁棒核函数的基类
    template<typename Scalar>
    class KernelBase {
    public:
        // 存储核函数的输入
        Scalar x;
        // 存储核函数的输出，以及其一阶导数
        Scalar y;
        Scalar y_;

    public:
        /* 构造函数与析构函数 */
        KernelBase() {}
        KernelBase(Scalar x) {
            this->ComputeOneOrder(x);
            this->ComputeZeroOrder(x);
        }
        virtual ~KernelBase() = default;

    public:
        /* 计算鲁棒核函数的值，由子类完成 */
        virtual Scalar &ComputeZeroOrder(const Scalar &x) = 0;
        /* 计算鲁棒核函数的一阶导数，由子类完成 */
        virtual Scalar &ComputeOneOrder(const Scalar &x) = 0;
        /* 计算鲁棒核函数的值和一阶导数 */
        void Compute(const Scalar &x);
    };

    // 定义基类的方法
    template<typename Scalar>
    void KernelBase<Scalar>::Compute(const Scalar &x) {
        this->x = x;
        this->ComputeZeroOrder(x);
        this->ComputeOneOrder(x);
    }


    // 继承基类，定义 Trival 核函数（本质上就是不加核函数）
    template<typename Scalar>
    class TrivalKernel : public KernelBase<Scalar> {
    public:
        /* 计算 Trival 鲁棒核函数的值 */
        Scalar &ComputeZeroOrder(const Scalar &x) override {
            this->y = x;
            return this->y;
        }

        /* 计算 Trival 鲁棒核函数的一阶导数 */
        Scalar &ComputeOneOrder(const Scalar &x) override {
            this->y_ = Scalar(1);
            return this->y;
        }
    };


    // 继承基类，定义 Huber 核函数
    template<typename Scalar>
    class HuberKernel : public KernelBase<Scalar> {
    private:
        // 参数
        Scalar delta;
        Scalar dsqr;
    public:
        /* 构造函数需要配置参数 */
        explicit HuberKernel(Scalar delta) {
            this->delta = delta;
            this->dsqr = delta * delta;
        }
    public:
        /* 计算 Huber 鲁棒核函数的值 */
        Scalar &ComputeZeroOrder(const Scalar &x) override {
            this->y = x;
            if (this->x > this->dsqr) {
                this->y = 2 * std::sqrt(this->x) * this->delta - this->dsqr;
            }
            return this->y;
        }

        /* 计算 Huber 鲁棒核函数的一阶导数 */
        Scalar &ComputeOneOrder(const Scalar &x) override {
            this->y_ = Scalar(1);
            if (this->x > this->dsqr) {
                this->y_ = this->delta / std::sqrt(this->x);
            }
            return this->y;
        }
    };
}