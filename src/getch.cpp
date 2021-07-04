#include "stdafx.h"
#include "getch.h"

#ifndef _WIN32
#include <stdio.h>
#include <unistd.h>
#include <termios.h>

// https://stackoverflow.com/questions/7469139/what-is-the-equivalent-to-getch-getche-in-linux
char getch() {
  char buf=0;
  struct termios old={0};
  fflush(stdout);
  if(tcgetattr(0, &old)<0)
    perror("tcsetattr()");
  old.c_lflag&=~ICANON;
  old.c_lflag&=~ECHO;
  old.c_cc[VMIN]=1;
  old.c_cc[VTIME]=0;
  if(tcsetattr(0, TCSANOW, &old)<0)
    perror("tcsetattr ICANON");
  if(read(0,&buf,1)<0)
    perror("read()");
  old.c_lflag|=ICANON;
  old.c_lflag|=ECHO;
  if(tcsetattr(0, TCSADRAIN, &old)<0)
    perror ("tcsetattr ~ICANON");
  printf("%c\n",buf);
  return buf;
}
#endif

// vim:ts=2 sw=2 sts=2 et ci
