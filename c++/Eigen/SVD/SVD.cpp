#include <iostream>
#include <Eigen/Dense>

int main()
{
   Eigen::MatrixXf A = Eigen::MatrixXf::Random(3, 2);
   std::cout << "A: " << A << std::endl;

   Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
   std::cout << "U: " << svd.matrixU() << std::endl;;
   std::cout << "V: " << svd.matrixV() << std::endl;;

}