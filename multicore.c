#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"

#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "blink.pio.h"

void blink_pin_forever(PIO pio, uint sm, uint offset, uint pin, uint freq);

// by default flash leds on gpios 3-4
#ifndef PIO_BLINK_LED1_GPIO
#define PIO_BLINK_LED1_GPIO 1
#endif

#define FLAG_VALUE 123

void core1_entry() {
  multicore_fifo_push_blocking(FLAG_VALUE);
  uint32_t g = multicore_fifo_pop_blocking();

  if (g != FLAG_VALUE)
    printf("Hmm, that's not right on core 1!\n");
  else
    printf("Its all gone well on core 1!");


  printf("\nClock Hz: %d\n",clock_get_hz(clk_sys));

  assert(PIO_BLINK_LED1_GPIO < 31);

  PIO pio[2];
  uint sm[2];
  uint offset[2];

  // Find a free pio and state machine and add the program
  bool rc = pio_claim_free_sm_and_add_program_for_gpio_range(&blink_program, &pio[0], &sm[0], &offset[0], PIO_BLINK_LED1_GPIO, 2, true);
  hard_assert(rc);
  printf("Loaded program at %u on pio %u\n", offset[0], PIO_NUM(pio[0]));

  // Claim the next state machine and start led2 flashing
  pio_sm_claim(pio[0], sm[0] + 1);
  // Claim the next state machine and start led3 flashing
  pio_sm_claim(pio[0], sm[0] + 2);

  // Start led1 flashing
//  blink_pin_forever(pio[0], sm[0], offset[0], PIO_BLINK_LED1_GPIO, 62400000);
  blink_program_init(pio[0], sm[0], offset[0], PIO_BLINK_LED1_GPIO);
  pio_sm_set_enabled(pio[0], sm[0], true);
  // PIO counter program takes 3 more cycles in total than we pass as
  // input (wait for n + 1; mov; jmp)
  pio[0]->txf[sm[0]] = 0;

//  blink_pin_forever(pio[0], sm[0] + 1, offset[0], PIO_BLINK_LED1_GPIO + 2, 1);
//  blink_pin_forever(pio[0], sm[0] + 2, offset[0], PIO_BLINK_LED1_GPIO + 3, 62400000);

  pio_sm_unclaim(pio[0], sm[0] + 1);
  pio_sm_unclaim(pio[0], sm[0] + 2);
  pio_remove_program_and_unclaim_sm(&blink_program, pio[0], sm[0], offset[0]);

  // the program exits but the pio keeps running!
  printf("All leds should be flashing\n");

//  while (1)
//    tight_loop_contents();
}

int main() {
  stdio_init_all();
  printf("Hello, multicore!\n");

  multicore_launch_core1(core1_entry);

  uint32_t g = multicore_fifo_pop_blocking();

  if (g != FLAG_VALUE)
    printf("Hmm, that's not right on core 0!\n");
  else {
    multicore_fifo_push_blocking(FLAG_VALUE);
    printf("It's all gone well on core 0!");
  }

  while (1)
    tight_loop_contents();
}

void blink_pin_forever(PIO pio, uint sm, uint offset, uint pin, uint freq) {
  blink_program_init(pio, sm, offset, pin);
  pio_sm_set_enabled(pio, sm, true);

  printf("Blinking pin %d at %d Hz\n", pin, freq);

  // PIO counter program takes 3 more cycles in total than we pass as
  // input (wait for n + 1; mov; jmp)
  pio->txf[sm] = (clock_get_hz(clk_sys) / (2 * freq)) - 1;
}
