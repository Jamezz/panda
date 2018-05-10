#ifdef STM32F4
  #include "stm32f4xx_hal_gpio_ex.h"
#else
  #include "stm32f2xx_hal_gpio_ex.h"
#endif
// ********************* GPIO (partial) *********************
void set_can_mode(int can, int use_gmlan);