#include <iostream>
#include <complex>

int main (int argc, char *argv[])
{
  std::complex<double> a(3.4, 5.7); // 3.4 + 5.7i

  std::cout << a << std::endl;

  std::cout << std::conj(a) << std::endl; // complex conjugate
  std::cout << std::abs(a) << std::endl; // magnitude
  std::cout << std::real(a) << std::endl; // real part
  std::cout << std::imag(a) << std::endl; // imaginary part
  std::cout << std::norm(a) << std::endl; // norm (magnitude squared)

  return 0;
}