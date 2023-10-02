#include <cstdint>
#include <iostream>
#include <stdint.h>

using namespace std;

// Я ЯЩЕР, Я ЯЩЕР, Я ЯЩЕР ПРЕЖИВШИЙ ЗАХВАТ БУЛКАМИ!

typedef struct {
  volatile uint32_t val1;
  volatile uint8_t val2;
  volatile uint32_t val3;
} Test;

typedef struct {
  Test fuck[2];
} Test2;

int main() {
  Test2 test2;
  test2.fuck[1].val1 = 100;
  test2.fuck[0].val1 = 100;
  // test2.Test[1] = 1000;


}
