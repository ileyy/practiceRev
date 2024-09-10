#include "Var_str.h"

void main1();           // Gen_file.c
map* Alg();             // Alg.c
void output(map* map);  // File_out.c

int main() {
  char choice;
  printf(
      "Please choose what you want to do: type g to generate map, type p to "
      "print map with path.\n");
  scanf("%c", &choice);
  switch (choice) {
    case 'g':
      main1();  // generating
      break;
    case 'p':
      output(Alg());  // printing
      break;
    default:
      printf("Error.\n");
      break;
  }
}
