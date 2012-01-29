//crossroads
#include "library.h"

//Global Variables
bool glcomplete  = false;
bool fcomplete = false;
bool scomplete = false;
bool gcomplete = false;
bool ccomplete = false;

//crossroads
bool crossroads()
{
    int choice = 0;

     while (choice != 10)
     {
          cout << "You stand at the corner of several roads going off in many directions.\n\n";
          cout << "There is a small town behind you full of shops and people.\n";
               
          

          cout << "Which path do you choose?\n";
          cout << "1: Town\n";
          cout << "2: Summoning Portal\n";
     
          if (!glcomplete)
          {
               cout << "3: Grasslands (QUEST)\n";
          }

          if (glcomplete && !fcomplete)
          {
               cout << "4: Forest Road (QUEST)\n";
          }

          if (fcomplete && !scomplete)
          {
               cout << "5: Swamp Road (QUEST)\n";
          }

          if (scomplete && !gcomplete)
          {
               cout << "6: Graveyard Road (QUEST)\n";
          }
     
          if (gcomplete && !ccomplete)
          {
               cout << "7: Castle Road (QUEST)\n";
          }

          cout << "10: Exit Game\n";
          cout << ">";

          cin >> choice;
          
          cout << endl;

          switch (choice)
          {
               case 1:
               {
                    cout << "You walk into the town.\n\n";
                    wait();
                    town(&player);
                    break;
               }
               case 2:
               {
                    //summoningportal()
                    break;
               }
               case 3:
               {
                    if (!gcomplete)
                    //grasslands()
                    break;
               }
               case 4:
               {
                    if (glcomplete && !fcomplete)
                    //forest()
                    break;
               }
               case 5:
               {
                    if (fcomplete && !scomplete)
                    //swamp()
                    break;
               }
               case 6:
               {
                    if (scomplete && !gcomplete)
                    //graveyard()
                    break;
               }
               case 7:
               {
                    if (gcomplete && !ccomplete)
                    //castle()
                    break;
               }
               default:
                    break;
          }
     }
     
     return true;
}
