#include <iostream>
#include <string>
using namespace std;
		
void ReadFromKeyboard(void);

int main()
{
  ReadFromKeyboard();
  return 0;
}

void ReadFromKeyboard(void)
{
    int choice = 0;
    string itemDesc;
    double itemPrice;

    cout << "1. Process a sale" << endl;
    cout << "2. View sales totals" << endl;
    cout << "3. Change the GST percentage" << endl;
    cout << "4. Quit" << endl;
    cin >> choice;
    cin.ignore();
    
    cout << "Enter Item Description " << endl;
    getline(cin, itemDesc);
    

    cout << "Enter Item Price" << endl;
    cin >> itemPrice;

}

