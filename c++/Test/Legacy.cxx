#include <iostream>
#include <fstream>
#include <iomanip>
using namespace std;


int main () {

	  int row, col;

//     cout << "\n" << input;
     
     int **loc = 0;
     
     loc = new int*[input];
     
     for (row = 0; row < input; row++)
         loc[row] = new int[input];
     
     for (row = 0; row < input; row++)
     {
         for (col = 0; col < input; col++)
             cout << loc[row][col];
//             *(loc[row] + col) = col;  
             
         cout << "\n";
     }    
     
                                      
     row = 1;                              
     col = input/2 + 1;                    
     otherdiag = 0;
     
//     Calculate(input, &*loc);
     
     for (row = 0; row < input; row++)
         delete [] loc[row];
     
     delete [] loc;
}
