#include <iostream>
#include <iomanip>
#include <ctime>

#include <Eigen/Core>
#include <Eigen/Dense>

int main(int argc, char** argv)
{
    Eigen::MatrixXd A(100, 100);
    A.setRandom();
    // make A positive semidefinite
    A = A.transpose() * A;

    Eigen::VectorXd b(100);
    b = 5*Eigen::VectorXd::Random(100);

    // std::cout << "A: " << std::endl << std::setprecision(3) << A << std::endl;
    // std::cout << "b: " << std::endl << std::setprecision(3) << b << std::endl;

    std::clock_t start;
    double duration;

    // QR decomposition
    // none requirements to the matrix A
    start = std::clock();
    Eigen::VectorXd x_1(100);
    x_1 = A.householderQr().solve(b);
    duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
    std::cout << "QR decomposition solver uses " << duration << " s" << std::endl;

    std::cout << "x_1 is: " << std::setprecision(3) << std::endl;
    for(int i = 0; i < 10; i++)
    {
        for(int j = 0; j < 10; j++)
        {
            std::cout << x_1(10*i+j) << "\t";
        }
        std::cout << std::endl;
    }

    // Cholesky decomposition
    // requirements on the matrix A is positive definite
    start = std::clock();
    Eigen::VectorXd x_2(100);
    x_2 = A.llt().solve(b);
    duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
    std::cout << "cholesky decomposition solver uses " << duration << " s" << std::endl;

    std::cout << "x_2 is: " << std::setprecision(3) << std::endl;
    for(int i = 0; i < 10; i++)
    {
        for(int j = 0; j < 10; j++)
        {
            std::cout << x_2(10*i+j) << "\t";
        }
        std::cout << std::endl;
    }

    return 0;
}