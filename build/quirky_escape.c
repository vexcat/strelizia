#include <stdio.h>

int main(int argc, char** argv) {
  FILE* input = fopen(argv[1], "r");
  int c;
  while((c = fgetc(input)) != EOF) {
    if(c == '~') {
      putchar('~');
      putchar('~');
    } else if(c == 'p') {
      putchar('~');
      putchar('o');
    } else if(c == '\n') {
      putchar('~');
      putchar('n');
    } else {
      putchar(c);
    }
  }
  fclose(input);
}