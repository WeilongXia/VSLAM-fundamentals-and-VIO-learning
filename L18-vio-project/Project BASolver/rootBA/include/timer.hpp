#pragma once

#include <ctime>
#include <cstdlib>
#include <chrono>


//命名空间为 rootBA
namespace rootBA {
class TicToc
{
public:
    TicToc()
    {
        tic();
    }

    void tic()
    {
        start = std::chrono::system_clock::now();
    }

    double toc()
    {
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        return elapsed_seconds.count() * 1000;
    }

private:
    std::chrono::time_point<std::chrono::system_clock> start, end;
};

}
// // 命名空间为 rootBA
// namespace rootBA {
// template <class Clock = std::chrono::high_resolution_clock>
// class Timer {
//  public:
//   /// start timer
//   Timer() : start_(Clock::now()), last_t_(Clock::now()) {}

//   void start() {
//     const auto now = Clock::now();
//     start_ = now;
//     last_t_ = now;
//     }

//   /// return elapsed time in seconds
//   double elapsed() const {
//     return std::chrono::duration<double>(Clock::now() - start_).count() *1000;
//   }

//   /// return elapsed time in seconds and reset timer
//   double reset() {
//     const auto now = Clock::now();
//     const double elapsed = std::chrono::duration<double>(now - last_t_).count() *1000;
//     last_t_ = now;
//     return elapsed;
//   }

//  private:
//   std::chrono::time_point<Clock> start_;
//   std::chrono::time_point<Clock> last_t_;
// };

// }