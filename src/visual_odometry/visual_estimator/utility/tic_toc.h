#pragma once

#include <ctime>
#include <cstdlib>
#include <chrono>

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
        double elapsed = std::chrono::duration_cast<double, std::milli>(end - start).count()
        return elapsed;
    }

  private:
    std::chrono::time_point<std::chrono::steady_clock> start, end;
};
