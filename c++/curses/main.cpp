#include <ncurses.h>
#include <iostream>
//#include <termios.h>
//#include <term.h>
//#include <unistd.h>

using namespace std;

int kbhit()
{
  char ch;
  int nread;

  if(peek_character != -1)
    return 1;
  new_settings.c_cc[VMIN]=0;
  tcsetattr(0, TCSANOW, &new_settings);
  nread = read(0,&ch,1);
  new_settings.c_cc[VMIN]=1;
  tcsetattr(0, TCSANOW, &new_settings);

  if(nread == 1) {
    peek_character = ch;
    return 1;
  }
  return 0;
}

int readch()
{
    char ch;

    if(peek_character != -1) {
        ch = peek_character;
        peek_character = -1;
        return ch;
    }
    read(0,&ch,1);
    return ch;
}

int main()
{

  initscr();
  printw("Hi.");
  getch();
  refresh();
  //  getch();
  int i =0;
  for(int i = 0; i < 20000; i ++)
    {
      cout << i << endl;
      if(kbhit())
	break;
    }
  endwin();
  return 0;
}
