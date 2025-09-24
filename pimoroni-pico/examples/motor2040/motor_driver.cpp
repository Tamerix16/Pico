
#include <cstdio>
#include "pico/stdlib.h"

    volatile char test[5];
    int position = 0;

    int main() 
    {
        stdio_init_all();
        // Your other initialization code
        while (1) 
        {
            int c = getchar_timeout_us(0); // Check for a character immediately
            if (c != PICO_ERROR_TIMEOUT) 
            {
            // Process the received character 'c'
            if (c == '\n')
                {
                    char backup[]="test";
                printf("Received: %s\n", backup);
                position = 0;
                }
                else
                {
                    test[position] = c;
                    position ++;
                    printf("Received: %c\n", (char)c);
                }
            }
        }
        return 0;
    }
