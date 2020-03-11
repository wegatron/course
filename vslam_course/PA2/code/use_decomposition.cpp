#include <Eigen/Dense>
#include <iostream>
#include <chrono>

int main(int argc, char * argv[])
{
  Eigen::Matrix<double, 100, 100> A;
  A = A.Random();
  A = A * A.transpose();
  Eigen::Matrix<double, 100, 1> b, x0, x1;
  b = b.Random();

  std::chrono::high_resolution_clock::time_point tp0 = std::chrono::high_resolution_clock::now();
  Eigen::FullPivHouseholderQR<Eigen::Matrix<double, 100, 100>> fpqr;
  fpqr.compute(A);
  x0 = fpqr.solve(b);
  auto tp1 = std::chrono::high_resolution_clock::now();

  Eigen::LDLT<Eigen::Matrix<double, 100, 100>> ldlt;
  ldlt.compute(A);
  x1 = ldlt.solve(b);
  auto tp2 = std::chrono::high_resolution_clock::now();

  std::cout << "FullPivHouseholderQR time:"
            << std::chrono::duration_cast<std::chrono::milliseconds>(tp1-tp0).count()
            << " ms" << std::endl;
  std::cout << "LDLT time:"
            << std::chrono::duration_cast<std::chrono::milliseconds>(tp2-tp1).count()
            << " ms" << std::endl;
  Eigen::Matrix<double, 100, 1> b0 = A * x0;
  Eigen::Matrix<double, 100, 1> b1 = A * x1;
  std::cout << "FullPivHouseholderQR error: " << (b0-b).norm() << std::endl;
  std::cout << "LDLT error: " << (b1-b).norm() << std::endl;
  return 0;
}
