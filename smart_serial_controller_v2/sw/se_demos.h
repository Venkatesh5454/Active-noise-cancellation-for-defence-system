/* =============================================================================
 * se_demos.h - serial-engine demos for the Way 2 console (main.c)
 * ===========================================================================*/
#ifndef SE_DEMOS_H
#define SE_DEMOS_H

#include <stdint.h>

void se_demo(const char *arg);       /* "se help", "se uart", "se spi", ... */
void se_demo_switch(void);           /* slide 14: JD from SPI to the SM0 UART program */
void print_temp(int32_t t_e4);       /* in main.c: prints a 1e-4 C temperature */

#endif /* SE_DEMOS_H */
