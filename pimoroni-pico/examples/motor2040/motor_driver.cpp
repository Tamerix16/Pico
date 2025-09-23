#include <cstdio>
#include "pico/stdlib.h"
#include <string>
#include "motor2040.hpp"
#include "button.hpp"

/*
Demonstrates how to read the angles of Motor 2040's four encoders.

Press "Boot" to exit the program.
*/
volatile char test[20] = "1";



int main() {
  stdio_init_all();

  // Fill the array of motors, and initialise them. Up to 8 motors can be created


  // Uncomment the below lines to reverse
  // the counting direction of an encoder
  // encoders[0].direction(REVERSED_DIR);
  // encoders[1].direction(REVERSED_DIR);
  // encoders[2].direction(REVERSED_DIR);
  // encoders[3].direction(REVERSED_DIR);

  // Read the encoders until the user button is pressed
  while(true) {
    // Print out the angle of each encoder
 
    scanf("%s", test)
    printf("hi\n");

    sleep_ms(100);
  }
}
