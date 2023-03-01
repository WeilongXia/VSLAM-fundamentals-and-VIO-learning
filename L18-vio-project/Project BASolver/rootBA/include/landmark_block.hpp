#pragma once

#include <camera_observe.hpp>
#include <kernel_function.hpp>
#include <timer.hpp>
#include <vertex.hpp>

// 全局命名空间定义为 rootBA
namespace rootBA
{

// 定义 landmark block 类，管理数据存储与相关运算方法
template <typename Scalar> class LandmarkBlock
{
  public:
    // storage 矩阵的拆分输出体
    struct DividedResult
    {
        MatrixX<Scalar> Q2T_Jex;
        VectorX<Scalar> Q2T_r;
        std::unordered_map<size_t, MatrixX<Scalar>> Q2T_Jps;
    };

  private:
    // 整个块的存储矩阵 [ Jp | Jl | residual ]
    MatrixX<Scalar> storage;
    // 保存用于回退的 Q_lambda 矩阵
    MatrixX<Scalar> Q_lambda;

    // 所关联的 landmark 对象
    std::shared_ptr<VertexLandmarkPosi<Scalar>> landmark;
    // 观测到此特征点的相机 pose 节点 id，相机节点的指针，以及对应的归一化平面观测结果
    std::unordered_map<size_t, std::shared_ptr<CameraObserve<Scalar>>> observes;
    // 整个 VIO 的相机与 IMU 之间的相对位姿
    std::shared_ptr<VertexExPose<Scalar>> exPose;
    // cameraID 和在 storage 矩阵中索引的映射表
    std::unordered_map<size_t, size_t> cameraID_idx;
    // 归一化平面的观测误差
    std::unordered_map<size_t, Vector2<Scalar>> residuals;
    // 每一个观测误差对应的权重，即 rho'(r.T * S * r)，会保存在 CameraObserve 中的 kernel 里面的 y_ 中
    // 此 Landmark block 提供的 reduced camera problem 增量方程的权重
    Scalar weight;

  public:
    /* 构造函数与析构函数 */
    LandmarkBlock();
    LandmarkBlock(const std::shared_ptr<VertexLandmarkPosi<Scalar>> &landmark,
                  const std::unordered_map<size_t, std::shared_ptr<CameraObserve<Scalar>>> &observes,
                  const std::shared_ptr<VertexExPose<Scalar>> &exPose);
    ~LandmarkBlock();

  public:
    /* 为此 landmark block 对象绑定 landmark */
    bool SetLandmark(const std::shared_ptr<VertexLandmarkPosi<Scalar>> &landmark);
    /* 为此 landmark block 对象绑定观测 */
    bool SetObserves(const std::unordered_map<size_t, std::shared_ptr<CameraObserve<Scalar>>> &observes);
    /* 为此 landmark block 对象绑定相机外参 */
    bool SetExPose(const std::shared_ptr<VertexExPose<Scalar>> &exPose);
    /* 提取此 landmark block 对象绑定的观测信息（可用此提取误差的马氏距离） */
    std::unordered_map<size_t, std::shared_ptr<CameraObserve<Scalar>>> &GetObserves(void);
    /* 提取此 landmark block 对象绑定的 landmark */
    std::shared_ptr<VertexLandmarkPosi<Scalar>> &GetLandmark(void);
    /* 提取此 landmark block 对象绑定的相机外参 */
    std::shared_ptr<VertexExPose<Scalar>> &GetExPose(void);

    /* 重新初始化 storage 的尺寸，并清空其中的数据，返回作用于 problem 中大雅可比矩阵中的行数 */
    /* 此时得到一个确定尺寸的 0 矩阵：
        [ 0 | 0 | 0 ] */
    size_t ResizeStorage(void);

    /* 根据 camera 和 landmark 对象的参数与 observe，计算观测误差 r，暂时不填入到 storage 中 */
    /* 此时 storate 矩阵保持不变 */
    bool PrecomputeResidual(void);

    /* 提取每一帧相机观测对应的观测误差，即返回 this->residuals */
    std::unordered_map<size_t, Vector2<Scalar>> &GetResiduals(void);

    /* 根据 camera 和 landmark 对象的参数与 observe，计算观测误差 r，填入到 storage 中 */
    /* 此时填充了 storage 的一部分：
        [ 0 | 0 | r ]
        [ 0 | 0 | 0 ] */
    bool ComputeResidual(void);

    /* 计算雅可比矩阵，填充 storage，完成线性化操作 */
    /* 此时填充完成 storage 矩阵:
        [ Jp(format diag block matrix) | Jl | r ]
        [     0                        | 0  | 0 ] */
    bool ComputeJacobian(void);

    /* 对 storage 中的 Jl 进行 QR 分解，基于分解结果调整 storage 矩阵 */
    /* 此时基于 QR 分解调整后的 storage 矩阵：
        [ Q1.T * Jp |   R1   | Q1.T * r ]
        [ Q2.T * Jp | R2(=0) | Q2.T * r ]
        [     0     |    0   |     0    ] */
    bool PerformQR(void);

    /* 给 storage 矩阵添加阻尼因子，并计算 givens 矩阵使得 storage 矩阵形式上保持不变，记录下 givens 矩阵在
     * this->Q_lambda */
    /* 添加阻尼因子后的 storage 矩阵：
        [ Q1.T * Jp |        R1        | Q1.T * r ]
        [ Q2.T * Jp |      R2(=0)      | Q2.T * r ]
        [     0     | sqrt(lambda) * I |     0    ] */
    /* 经过 Givens 矩阵调整后的 storage 矩阵，维度和上面的矩阵保持一致：
        [ Q1_.T * Jp |  R1_  | Q1_.T * r ]
        [ Q2_.T * Jp |   0   | Q2_.T * r ]
        [     0     |    0   |     0    ] */
    bool Damp(Scalar lambda);

    /* 提取出此 landmark block 提供的增量方程的权重 */
    Scalar GetWeight(void);

    /* 在 storage 矩阵被 damp 的前提下 */
    /* 按照 observe 的索引顺序，提取出指定于 cameraID 的 Q2.T * Jex、Q2.T * Jp 和 Q2.T * r 矩阵块 */
    /* 用于 problem 填充大雅可比矩阵 */
    DividedResult GetDampedDividedStorage(void);

    /* 返回 damp 之后的 storage 矩阵有效行数 */
    size_t GetDampedDividedStorageRows(void);

    /* 在 storage 矩阵没有被 damp 的前提下 */
    /* 按照 observe 的索引顺序，提取出指定于 cameraID 的 Q2.T * Jex、Q2.T * Jp 和 Q2.T * r 矩阵块 */
    /* 用于 problem 填充大雅可比矩阵 */
    DividedResult GetUndampedDividedStorage(void);

    /* 返回 damp 之后的 storage 矩阵有效行数 */
    size_t GetUndampedDividedStorageRows(void);

    /* 返回 delta_Xl 增量方程的 H 矩阵 */
    /* 本质上就是从 storage 矩阵中提取出 R1 */
    MatrixX<Scalar> Get_R1(void);

    /* 返回 delta_Xl 增量方程的 b 向量 */
    /* 本质上就是从 storage 矩阵中提取出 Q1_.T * r */
    VectorX<Scalar> Get_Q1T_r(void);

    /* 基于已知的相机 pose 增量的结果 delta_Xp，求解此特征点 position 对应的增量 */
    /* 就是计算 delta_Xl = - R1.inv * (Q1T * r + Q1T * Jp * delta_Xp) */
    Vector3<Scalar> ComputeDeltaXl(const VectorX<Scalar> &delta_Xp);

    /* 基于保存的 Q_lambda 矩阵，回退 storage 矩阵至原来的状态 */
    /* 同时，为了方便再次调用 Damp 方法，此处在回退后需要去除因 damp 而增加的空白行 */
    /* 回退之前的 storage 矩阵：
        [ Q1_.T * Jp |  R1_  | Q1_.T * r ]
        [ Q2_.T * Jp |   0   | Q2_.T * r ] */
    /* 回退之后的 storage 矩阵：
        [ Q1.T * Jp |  R1  | Q1.T * r ]
        [ Q2.T * Jp |   0  | Q2.T * r ]
        [     0     |   0  |     0    ] */
    bool RollBack(void);
};
} // namespace rootBA

// 此处为模板类的成员的定义
namespace rootBA
{
/* 构造函数与析构函数 */
template <typename Scalar> LandmarkBlock<Scalar>::LandmarkBlock()
{
}
template <typename Scalar>
LandmarkBlock<Scalar>::LandmarkBlock(const std::shared_ptr<VertexLandmarkPosi<Scalar>> &landmark,
                                     const std::unordered_map<size_t, std::shared_ptr<CameraObserve<Scalar>>> &observes,
                                     const std::shared_ptr<VertexExPose<Scalar>> &exPose)
{
    this->SetLandmark(landmark);
    this->SetObserves(observes);
    this->SetExPose(exPose);
}
template <typename Scalar> LandmarkBlock<Scalar>::~LandmarkBlock()
{
}

/* 为此 landmark block 对象绑定 landmark */
template <typename Scalar>
bool LandmarkBlock<Scalar>::SetLandmark(const std::shared_ptr<VertexLandmarkPosi<Scalar>> &landmark)
{
    if (landmark == nullptr)
    {
        return false;
    }
    else
    {
        this->landmark = landmark;
        return true;
    }
}

/* 为此 landmark block 对象绑定观测 */
template <typename Scalar>
bool LandmarkBlock<Scalar>::SetObserves(
    const std::unordered_map<size_t, std::shared_ptr<CameraObserve<Scalar>>> &observes)
{
    if (observes.empty())
    {
        return false;
    }
    else
    {
        this->observes = observes;
        this->cameraID_idx.clear();
        size_t idx = 0;
        for (auto &ob : this->observes)
        {
            this->cameraID_idx.insert(std::make_pair(ob.first, idx));
            ++idx;
        }
        return true;
    }
}

/* 为此 landmark block 对象绑定相机外参 */
template <typename Scalar> bool LandmarkBlock<Scalar>::SetExPose(const std::shared_ptr<VertexExPose<Scalar>> &exPose)
{
    if (exPose == nullptr)
    {
        return false;
    }
    else
    {
        this->exPose = exPose;
        return true;
    }
}

/* 提取此 landmark block 对象绑定的观测信息（可用此提取误差的马氏距离） */
template <typename Scalar>
std::unordered_map<size_t, std::shared_ptr<CameraObserve<Scalar>>> &LandmarkBlock<Scalar>::GetObserves(void)
{
    return this->observes;
}

/* 提取此 landmark block 对象绑定的 landmark */
template <typename Scalar> std::shared_ptr<VertexLandmarkPosi<Scalar>> &LandmarkBlock<Scalar>::GetLandmark(void)
{
    return this->landmark;
}

/* 提取此 landmark block 对象绑定的相机外参 */
template <typename Scalar> std::shared_ptr<VertexExPose<Scalar>> &LandmarkBlock<Scalar>::GetExPose(void)
{
    return this->exPose;
}

/* 重新初始化 storage 的尺寸，并清空其中的数据 */
/* 此时得到一个确定尺寸的 0 矩阵：
    [ 0 | 0 | 0 ] */
template <typename Scalar> size_t LandmarkBlock<Scalar>::ResizeStorage(void)
{
    // T_bc, T_wb_1, T_wb_2, ... , T_wb_n, p_w, r
    // observe1, observe2, ... , observe_n, damp
    size_t cameraNums = static_cast<size_t>(this->observes.size());
    size_t cols = 6 + cameraNums * 6 + 3 + 1;
    size_t rows = cameraNums * 2 + 3;
    if (this->storage.rows() == rows)
    {
        this->storage.setZero();
    }
    else
    {
        this->storage.setZero(rows, cols);
    }
    // std::cout << "LandmarkBlock<Scalar>::ResizeStorage :\n" << this->storage << std::endl;
    return rows - 3;
}

/* 根据 camera 和 landmark 对象的参数与 observe，计算观测误差 r，暂时不填入到 storage 中 */
/* 此时 storate 矩阵保持不变 */
template <typename Scalar> bool LandmarkBlock<Scalar>::PrecomputeResidual(void)
{
    this->residuals.clear();
    this->weight = 0;
    for (auto &observe : this->observes)
    {
        // 提取相关信息
        size_t cameraID = observe.first;
        auto &camera = observe.second->camera;
        Vector2<Scalar> &measure = observe.second->norm;
        auto &kernel = observe.second->kernel;
        Matrix2<Scalar> &sqrtInfo = observe.second->sqrtInfo;
        // 提取位姿参数
        Quaternion<Scalar> &q_wb = camera->Get_q_wb();
        Vector3<Scalar> &t_wb = camera->Get_t_wb();
        Quaternion<Scalar> &q_bc = this->exPose->Get_q_bc();
        Vector3<Scalar> &t_bc = this->exPose->Get_t_bc();
        Vector3<Scalar> &p_w = this->landmark->Get_p_w();

        // 计算重投影的归一化平面坐标 norm = [ux, uy, 1].T
        Vector3<Scalar> p_b = q_wb.inverse() * (p_w - t_wb);
        Vector3<Scalar> p_c = q_bc.inverse() * (p_b - t_bc);
        Scalar invDep = 1.0 / p_c.z();
        Vector2<Scalar> norm = p_c.template head<2>() / p_c.z();

        // 计算重投影误差
        Vector2<Scalar> residual;
        if (std::isnan(invDep) || std::isinf(invDep))
        { // 判断这个点是否正常
            residual.setZero();
        }
        else
        {
            residual = norm - measure;
        }

        // 计算 rho(r.T * S * r) 即 y，以及鲁棒核权重 rho'(r.T * S * r) 即 y_
        Scalar x = residual.transpose() * sqrtInfo * sqrtInfo * residual;
        kernel->Compute(x);
        this->weight += kernel->y_;

        // 计算 sqrt(S) * r，作为误差输出结果
        Vector2<Scalar> weightedResidual = sqrtInfo * residual;
        this->residuals.insert(std::make_pair(cameraID, weightedResidual));
    }
    return true;
}

/* 提取每一帧相机观测对应的观测误差，即返回 this->residuals */
template <typename Scalar> std::unordered_map<size_t, Vector2<Scalar>> &LandmarkBlock<Scalar>::GetResiduals(void)
{
    return this->residuals;
}

/* 根据 camera 和 landmark 对象的参数与 observe，计算观测误差 r，填入到 storage 中 */
/* 此时填充了 storage 的一部分：
    [ 0 | 0 | r ]
    [ 0 | 0 | 0 ] */
template <typename Scalar> bool LandmarkBlock<Scalar>::ComputeResidual(void)
{
    // 如果还没有预计算过误差，则需要预计算，然后才能进行填充
    if (this->residuals.empty())
    {
        this->PrecomputeResidual();
    }
    for (auto &residual : this->residuals)
    {
        size_t idx = (*this->cameraID_idx.find(residual.first)).second;
        this->storage.template block<2, 1>(idx * 2, this->storage.cols() - 1) = residual.second;
    }
    this->residuals.clear();
    // std::cout << "LandmarkBlock<Scalar>::ComputeResidual :\n" << this->storage << std::endl;
    return true;
}

/* 计算雅可比矩阵，填充 storage，完成线性化操作 */
/* 此时填充完成 storage 矩阵:
    [ Jp(format diag block matrix) | Jl | r ]
    [     0                        | 0  | 0 ] */
template <typename Scalar> bool LandmarkBlock<Scalar>::ComputeJacobian(void)
{
    for (auto &observe : this->observes)
    {
        // 提取相关信息
        auto &camera = observe.second->camera;
        Vector2<Scalar> &measure = observe.second->norm;
        auto &kernel = observe.second->kernel;
        Matrix2<Scalar> &sqrtInfo = observe.second->sqrtInfo;

        // 提取位姿参数
        Quaternion<Scalar> &q_wb = camera->Get_q_wb();
        Matrix3<Scalar> R_bw(q_wb.inverse());
        Vector3<Scalar> &t_wb = camera->Get_t_wb();
        Quaternion<Scalar> &q_bc = this->exPose->Get_q_bc();
        Matrix3<Scalar> R_cb(q_bc.inverse());
        Vector3<Scalar> &t_bc = this->exPose->Get_t_bc();
        Vector3<Scalar> &p_w = this->landmark->Get_p_w();

        // 计算重投影的归一化平面坐标 norm = [ux, uy, 1].T
        Vector3<Scalar> p_b = R_bw * (p_w - t_wb);
        Vector3<Scalar> p_c = R_cb * (p_b - t_bc);
        Scalar dep = p_c.z();

        // 计算雅可比矩阵 d(归一化平面位置) / d(相机坐标系位置)
        Eigen::Matrix<Scalar, 2, 3> reduce;
        if (std::isnan(1.0 / dep) || std::isinf(1.0 / dep))
        {
            reduce.setZero();
        }
        else
        {
            reduce << 1. / dep, 0, -p_c(0) / (dep * dep), 0, 1. / dep, -p_c(1) / (dep * dep);
        }

        // 计算雅可比矩阵 d(相机坐标位置误差) / d(相机 6 自由度位姿)

        //  TODO: task 1 calculate jacobian
        Eigen::Matrix<Scalar, 3, 6> tempJp = Eigen::Matrix<Scalar, 3, 6>::Zero();
        if (camera->IsFixed() == false)
        {
            tempJp.template block<3, 3>(0, 0) = -R_cb * R_bw;
            tempJp.template block<3, 3>(0, 3) = R_cb * SkewSymmetricMatrix(p_b);
        }

        // 计算雅可比矩阵 d(相机坐标位置误差) / d(特征点 3 自由度位置)
        Eigen::Matrix<Scalar, 3, 3> tempJl = Eigen::Matrix<Scalar, 3, 3>::Zero();
        if (this->landmark->IsFixed() == false)
        {
            tempJl = R_cb * R_bw;
        }

        // 计算雅可比矩阵 d(相机坐标位置误差) / d(相机外参 6 自由度位姿)
        // Hint: to calculate skew symmetric matrix, you could use function SkewSymmetricMatrix(*)
        Eigen::Matrix<Scalar, 3, 6> tempJex = Eigen::Matrix<Scalar, 3, 6>::Zero();
        if (this->exPose->IsFixed() == false)
        {
            tempJex.template block<3, 3>(0, 0) = -R_cb;
            tempJex.template block<3, 3>(0, 3) = SkewSymmetricMatrix(p_c);
        }
        // Fill in above this line

        // 通过级联求导的方式，计算雅可比矩阵，填充 storage 矩阵
        size_t idx = (*this->cameraID_idx.find(observe.first)).second;
        this->storage.template block<2, 6>(idx * 2, 0) = sqrtInfo * reduce * tempJex;
        this->storage.template block<2, 3>(idx * 2, this->storage.cols() - 4) = sqrtInfo * reduce * tempJl;
        this->storage.template block<2, 6>(idx * 2, 6 + idx * 6) = sqrtInfo * reduce * tempJp;
    }
    // std::cout << "LandmarkBlock<Scalar>::ComputeJacobian :\n" << this->storage << std::endl;
    return true;
}

/* 对 storage 中的 Jl 进行 QR 分解，基于分解结果调整 storage 矩阵 */
/* 此时基于 QR 分解调整后的 storage 矩阵：
    [ Q1.T * Jp |   R1   | Q1.T * r ]
    [ Q2.T * Jp | R2(=0) | Q2.T * r ]
    [     0     |    0   |     0    ] */
template <typename Scalar> bool LandmarkBlock<Scalar>::PerformQR(void)
{
    // 提取 storage 矩阵中的三个部分
    size_t rows = this->storage.rows();
    size_t cols = this->storage.cols();
    MatrixX<Scalar> &&Jl = this->storage.block(0, cols - 4, rows - 3, 3);
    MatrixX<Scalar> &&Jp = this->storage.block(0, 0, rows - 3, cols - 4);
    VectorX<Scalar> &&r = this->storage.block(0, cols - 1, rows - 3, 1);

    // TODO: task 2 执行QR分解，并更新sorage矩阵
    // 对 Jl 进行 QR 分解
    // HInt: Eigen::HouseholderQR could be used to perform QR decompose.
    Eigen::HouseholderQR<MatrixX<Scalar>> qr;
    qr.compute(Jl);
    MatrixX<Scalar> Q = qr.householderQ();

    // // 对 storage 矩阵左乘 Q.T
    this->storage.block(0, cols - 4, rows - 3, 3) = Q.transpose() * Jl;
    this->storage.block(0, 0, rows - 3, cols - 4) = Q.transpose() * Jp;
    this->storage.block(0, cols - 1, rows - 3, 1) = Q.transpose() * r;

    // Fill in above this line
    // std::cout << "LandmarkBlock<Scalar>::PerformQR :\n" << this->storage << std::endl;
    return true;
}

/* 给 storage 矩阵添加阻尼因子，并计算 givens 矩阵使得 storage 矩阵形式上保持不变，记录下 givens 矩阵在 this->Q_lambda
 */
/* 添加阻尼因子后的 storage 矩阵：
    [ Q1.T * Jp |        R1        | Q1.T * r ]
    [ Q2.T * Jp |      R2(=0)      | Q2.T * r ]
    [     0     | sqrt(lambda) * I |     0    ] */
/* 经过 Givens 矩阵调整后的 storage 矩阵，维度和上面的矩阵保持一致：
    [ Q1_.T * Jp |  R1_  | Q1_.T * r ]
    [ Q2_.T * Jp |   0   | Q2_.T * r ] */
template <typename Scalar> bool LandmarkBlock<Scalar>::Damp(Scalar lambda)
{
    // 将 storage 矩阵的最下面三行置为零
    size_t rows = this->storage.rows();
    size_t cols = this->storage.cols();
    this->storage.block(rows - 3, 0, 3, cols).setZero();

    // 在指定位置添加阻尼因子
    // this->storage.block(rows - 3, cols - 4, 3, 3) = Matrix3<Scalar>::Identity() * std::sqrt(lambda);
    this->storage.block(rows - 3, cols - 4, 3, 3).diagonal().array() += std::sqrt(lambda);

    // 为更正 storage 矩阵中对应于 Jl_ 位置的格式（上为上三角，下为全零），计算 Givens 矩阵
    Eigen::JacobiRotation<Scalar> gr;
    if (this->Q_lambda.rows() == rows)
    {
        this->Q_lambda.setIdentity();
    }
    else
    {
        this->Q_lambda.setIdentity(rows, rows);
    }
    for (size_t i = 0; i < 3; ++i)
    {
        size_t n = i + this->storage.cols() - 4; // 列索引
        for (size_t j = 0; j <= i; ++j)
        {
            size_t m = rows - 3 + i - j;

            // TODO: task 3 进行givens旋转，让storage矩阵指定位置数值为0,并存储中间结果
            // Hint: makeGivens()函数可以进行givens旋转，其方法本质上就是传入 x1 和 x2，计算出旋转矩阵对应的 sin 和 cos
            gr.makeGivens(storage(i, n), storage(m, n));

            // 对 this->storage 矩阵进行初等变换，并将连乘结果记录下来
            this->storage.applyOnTheLeft(i, m, gr.adjoint());
            this->Q_lambda.applyOnTheLeft(i, m, gr.adjoint());

            // Fill in above this line
        }
    }

    // 运算精度会导致部分位置不为零，因此这些位置需要强制给零
    for (size_t i = 0; i < 3; ++i)
    {
        size_t n = i + cols - 4;
        for (size_t m = i + rows - 3; m > i; --m)
        {
            this->storage(m, n) = 0;
        }
    }
    // std::cout << "LandmarkBlock<Scalar>::Damp :\n" << this->storage << std::endl;
    return true;
}

/* 提取出此 landmark block 提供的增量方程的权重 */
template <typename Scalar> Scalar LandmarkBlock<Scalar>::GetWeight(void)
{
    return this->weight;
}

/* 按照 observe 的索引顺序，提取出指定于 cameraID 的 Q2.T * Jex、Q2.T * Jp 和 Q2.T * r 矩阵块 */
/* 用于 problem 填充大雅可比矩阵 */
template <typename Scalar>
typename LandmarkBlock<Scalar>::DividedResult LandmarkBlock<Scalar>::GetDampedDividedStorage(void)
{
    DividedResult res;
    size_t rows = this->storage.rows();
    size_t cols = this->storage.cols();
    res.Q2T_Jex = this->storage.block(3, 0, rows - 3, 6);
    res.Q2T_r = this->storage.block(3, cols - 1, rows - 3, 1);
    for (auto &observe : this->observes)
    {
        size_t idx = (*this->cameraID_idx.find(observe.first)).second;
        MatrixX<Scalar> &&Q2T_Jp = this->storage.block(3, idx * 6 + 6, rows - 3, 6);
        res.Q2T_Jps.insert(std::make_pair(observe.first, Q2T_Jp));
    }
    return res;
}

/* 返回 damp 之后的 storage 矩阵有效行数 */
template <typename Scalar> size_t LandmarkBlock<Scalar>::GetDampedDividedStorageRows(void)
{
    return this->storage.rows() - 3;
}

/* 在 storage 矩阵没有被 damp 的前提下 */
/* 按照 observe 的索引顺序，提取出指定于 cameraID 的 Q2.T * Jex、Q2.T * Jp 和 Q2.T * r 矩阵块 */
/* 用于 problem 填充大雅可比矩阵 */
template <typename Scalar>
typename LandmarkBlock<Scalar>::DividedResult LandmarkBlock<Scalar>::GetUndampedDividedStorage(void)
{
    DividedResult res;
    size_t rows = this->storage.rows();
    size_t cols = this->storage.cols();
    res.Q2T_Jex = this->storage.block(3, 0, rows - 6, 6);
    res.Q2T_r = this->storage.block(3, cols - 1, rows - 6, 1);
    for (auto &observe : this->observes)
    {
        size_t idx = (*this->cameraID_idx.find(observe.first)).second;
        MatrixX<Scalar> &&Q2T_Jp = this->storage.block(3, idx * 6 + 6, rows - 6, 6);
        res.Q2T_Jps.insert(std::make_pair(observe.first, Q2T_Jp));
    }
    return res;
}

/* 返回 damp 之后的 storage 矩阵有效行数 */
template <typename Scalar> size_t LandmarkBlock<Scalar>::GetUndampedDividedStorageRows(void)
{
    return this->storage.rows() - 6;
}

/* 返回 delta_Xl 增量方程的 H 矩阵 */
/* 本质上就是从 storage 矩阵中提取出 R1 */
template <typename Scalar> MatrixX<Scalar> LandmarkBlock<Scalar>::Get_R1(void)
{
    MatrixX<Scalar> &&R1 = this->storage.block(0, this->storage.cols() - 4, 3, 3);
    return this->weight * R1;
}

/* 返回 delta_Xl 增量方程的 b 向量 */
/* 本质上就是从 storage 矩阵中提取出 Q1_.T * r */
template <typename Scalar> VectorX<Scalar> LandmarkBlock<Scalar>::Get_Q1T_r(void)
{
    VectorX<Scalar> &&Q1T_r = this->storage.block(0, this->storage.cols() - 1, 3, 1);
    return this->weight * Q1T_r;
}

/* 基于已知的相机 pose 增量的结果 delta_Xp，求解此特征点 position 对应的增量 */
/* 就是计算 delta_Xl = - R1.inv * (Q1T * r + Q1T * Jp * delta_Xp) */
template <typename Scalar> Vector3<Scalar> LandmarkBlock<Scalar>::ComputeDeltaXl(const VectorX<Scalar> &delta_Xp)
{
    Vector3<Scalar> res;
    size_t rows = this->storage.rows();
    size_t cols = this->storage.cols();
    // 首先需要从全局的 delta_Xp 中提取出此 landmark block 对应的 delta_Xps
    VectorX<Scalar> delta_Xps;
    delta_Xps.resize(cols - 4);
    delta_Xps.head(6) = delta_Xp.head(6);
    for (auto &item : this->observes)
    {
        size_t idx = (*this->cameraID_idx.find(item.first)).second;
        delta_Xps.segment(idx * 6 + 6, 6) = delta_Xp.segment(item.first * 6 + 6, 6);
    }
    // 然后再去求解 delta_Xl
    MatrixX<Scalar> &&Q1T_Jp = this->storage.block(0, 0, 3, cols - 4);
    VectorX<Scalar> &&Q1T_r = this->storage.block(0, cols - 1, 3, 1);
    MatrixX<Scalar> &&R1 = this->storage.block(0, cols - 4, 3, 3);
    res = -R1.inverse() * (Q1T_r + Q1T_Jp * delta_Xps);
    return res;
}

/* 基于保存的 Q_lambda 矩阵，回退 storage 矩阵至原来的状态 */
/* 同时，为了方便再次调用 Damp 方法，此处在回退后需要去除因 damp 而增加的空白行 */
/* 回退之前的 storage 矩阵：
    [ Q1_.T * Jp |  R1_  | Q1_.T * r ]
    [ Q2_.T * Jp |   0   | Q2_.T * r ] */
/* 回退之后的 storage 矩阵：
    [ Q1.T * Jp |  R1  | Q1.T * r ]
    [ Q2.T * Jp |   0  | Q2.T * r ]
    [     0     |   0  |     0    ] */
template <typename Scalar> bool LandmarkBlock<Scalar>::RollBack(void)
{
    size_t rows = this->storage.rows();
    size_t cols = this->storage.cols();

    //  TODO: task 3 对storage矩阵实现状态回退
    this->storage = this->Q_lambda.transpose() * this->storage;

    // Fill in above this line

    // 运算精度导致部分为零的地方不为零，这里强制归零
    for (size_t i = 0; i < 3; ++i)
    {
        size_t n = i + cols - 4;
        for (size_t m = rows - 1; m > i; --m)
        {
            this->storage(m, n) = 0;
        }
    }
    this->storage.block(rows - 3, 0, 3, cols).setZero();
    // std::cout << "LandmarkBlock<Scalar>::RollBack :\n" << this->storage << std::endl;
    return true;
}
} // namespace rootBA