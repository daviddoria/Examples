#include <iostream>
#include <iomanip> // for setfill

int main(int argc, char *argv[])
{
  double number1 = 5.346;
  double number2 = 15.346;

  std::cout << std::setprecision(3) << std::fixed << std::setfill('0');

  std::cout << std::setw(6) << number1 << std::endl;

  std::cout << std::setw(6) << number2 << std::endl;

  // If setprecision(4) and setw(6), 5.3460 (the .3460 is because set precision 4 - it sets the number of digits after the decimal)
  // is display as 5.3460 rather than 05.3460 because the '.' counts as a character for setw and the maximum width is 6, so it refuses
  // to zero pad it.
  return 0;
}
