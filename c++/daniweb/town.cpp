//town
#include "library.h"

#include <ctime>
#include <cstdio>

//Town Functions
void inn();

bool town(PLAYER*)
{      
     int choice = 0;
     
     while (choice != 5)
     {
          cout << "You stand thn the middle of the town.\n";
          cout << "All around you there are peeople going about their daily lives.\n";
          cout << "To your left you spot an INN and to your right you spot the ARMORY and"
               << "\nITEM SHOP.\n\n";
          
          wait();

          cout << "What would you like to do?\n";
          cout << "1: Enter INN\n";
          cout << "2: Enter ARMORY\n";
          cout << "3: Enter ITEM SHOP\n";
          cout << "4: Check Stats\n";
          cout << "5: Leave Town\n";
          cout << ">";

          cin >> choice;
        
          cout << "\n";

          switch (choice)
          {
               case 1:
               {
                    //inn
                    break;
               }
               case 2:
               {
                    //armory
                    break;
               }
               case 3:
               {
                    //item shop
                    break;
               }
               case 4:
               {
                    //stats
                    break;
               }
               default:
                    break;
          }
     }
     return true;
}

void inn()
{
     cout << "You stand in a small almost vancent inn. Exept for tan old man standing behind"
          << "\na counter  there is no one else currently staying here.\n\n";
     cout << "You walk over to the old man. He grins, delighted to see a visitor.\n"
          << "Hello and welcome, would you like to saty in my inn for only 10 gold coins?";
     
     char answer[5];
     
     while (answer[0] != 'y' || answer[0] != 'n')
     {
          cout << "Stay? (10 gold) y/n\n";
          cin >> answer;
     }

     cout << endl;

     if (answer[0] == 'y')
     {
          if (player.getGold() > 10)
          {
               player.spendGold(10);
               cout << "You find a nice bed and fall; fast asleep.\n";
               
               wait();
               
               cout << "\n(HP and MP restored)/n\n";
               cout << "HP: " << player.getHealth() << "/" << player.getMaxHealth();
               cout << " MP: " << player.getMana() << "/" << player.getMaxMana();
               player.setHealth (player.getMaxHealth());
               player.setMana (player.getMaxMana());
               wait();
          }
          else
          {
               cout << "It looks like you don't have enough gold.\n";
               
               wait();
               
               answer[0] = 'n';
          }
     }
     
     if (answer[0] == 'n')
     {
          cout << "You turn around and walk out of the inn.\n";
     }
}
