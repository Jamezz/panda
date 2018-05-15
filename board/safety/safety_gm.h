#include "../drivers/drivers.h"
#include "../gpio.hh"

#define GPIO_BSRR_BS12                      ((uint32_t)0x00001000U)
#define GPIO_BSRR_BS13                      ((uint32_t)0x00002000U)
#define GPIO_BSRR_BR12                      ((uint32_t)0x10000000U)
#define GPIO_BSRR_BR13                      ((uint32_t)0x20000000U)
// board enforces
//   in-state
//      accel set/resume
//   out-state
//      cancel button
//      regen paddle
//      accel rising edge
//      brake rising edge
//      brake > 0mph

// gm_: poor man's namespacing
int gm_brake_prev = 0;
int gm_gas_prev = 0;
int gm_speed = 0;

// silence everything if stock ECUs are still online
int gm_ascm_detected = 0;

int gm_ignition_started = 0;

static void gm_rx_hook(CAN_FIFOMailBox_TypeDef *to_push) {
  int bus_number = (to_push->RDTR >> 4) & 0xFF;
  uint32_t addr;
  if (to_push->RIR & 4) {
    // Extended
    // Not looked at, but have to be separated
    // to avoid address collision
    addr = to_push->RIR >> 3;
  } else {
    // Normal
    addr = to_push->RIR >> 21;
  }

  if (addr == 0x135 && bus_number == 0) {
    //Gear selector (used for determining ignition)
    int gear = to_push->RDLR & 0x7;
    gm_ignition_started = gear > 0; //Park = 0. If out of park, we're "on."
  }

  // sample speed, really only care if car is moving or not
  // rear left wheel speed
  if (addr == 842) {
    gm_speed = to_push->RDLR & 0xFFFF;
  }

  // check if stock ASCM ECU is still online
  if (bus_number == 0 && addr == 715) {
    gm_ascm_detected = 1;
    controls_allowed = 0;
  }

  // ACC steering wheel buttons
  if (addr == 481) {
    int buttons = (to_push->RDHR >> 12) & 0x7;
    // res/set - enable, cancel button - disable
    if (buttons == 2 || buttons == 3) {
      controls_allowed = 1;
    } else if (buttons == 6) {
      controls_allowed = 0;
    }
  }

  // exit controls on rising edge of brake press or on brake press when
  // speed > 0
  if (addr == 241) {
    int brake = (to_push->RDLR & 0xFF00) >> 8;
    // Brake pedal's potentiometer returns near-zero reading
    // even when pedal is not pressed
    if (brake < 10) {
      brake = 0;
    }
    if (brake && (!gm_brake_prev || gm_speed)) {
       controls_allowed = 0;
    }
    gm_brake_prev = brake;
  }

  // exit controls on rising edge of gas press
  if (addr == 417) {
    int gas = to_push->RDHR & 0xFF0000;
    if (gas && !gm_gas_prev) {
      controls_allowed = 0;
    }
    gm_gas_prev = gas;
  }

  // exit controls on regen paddle
  if (addr == 189) {
    int regen = to_push->RDLR & 0x20;
    if (regen) {
      controls_allowed = 0;
    }
  }
}

static int gm_msg_is_destined_for_gmlan(CAN_FIFOMailBox_TypeDef *to_send) {
  int bus_number = (to_send->RDTR >> 4) & 0xFF;
  return bus_number == can_gmlan_bus;
}

static int gm_gmlan_bitbang(CAN_FIFOMailBox_TypeDef *to_send) {

  int pin = 12;
  uint32_t addr;
  uint8_t isExtended = 0;
  uint32_t rdlr = to_send->RDLR;
  uint32_t rdhr = to_send->RDHR;

  if (to_send->RIR & 4) {
    // Extended
    addr = to_send->RIR >> 3;
    isExtended = 1;
  } else {
    // Normal
    addr = to_send->RIR >> 21;
  }
    // B12,B13: gmlan
  
  enter_critical_section();
  
  //STEP 1: Send address
  for(int i = 0; i < (isExtended ? 29 : 11); i++) {
      // consider leftmost bit
      // set line high if bit is 1, low if bit is 0
      if (addr & 0x80000000) {
          GPIOB->BSRR = GPIO_BSRR_BS12;
      }
      else {
          GPIOB->BSRR = GPIO_BSRR_BR12;
      }
      set_gpio_mode(GPIOB, pin, MODE_OUTPUT);

      set_gpio_mode(GPIOB, pin, MODE_INPUT);
      if ((addr & 0x80000000) && !get_gpio_input(GPIOB, pin)) {
        //We got stepped on (Transmit a recessive 1 and someone dominated 0)
        goto fail;
      }

      // shift byte left so next bit will be leftmost
      addr <<= 1;
  }

  //STEP 2: Send RDLR
  for(int i = 0; i < 31; i++) {
      // consider leftmost bit
      // set line high if bit is 1, low if bit is 0
      if (rdlr & 0x80000000) {
          GPIOB->BSRR = GPIO_BSRR_BS12;
      }
      else {
          GPIOB->BSRR = GPIO_BSRR_BR12;
      }
      set_gpio_mode(GPIOB, pin, MODE_OUTPUT);

      set_gpio_mode(GPIOB, pin, MODE_INPUT);
      if ((addr & 0x80000000) && !get_gpio_input(GPIOB, pin)) {
        //We got stepped on (Transmit a recessive 1 and someone dominated 0)
        goto fail;
      }


      // shift byte left so next bit will be leftmost
      rdlr <<= 1;
  }

  //STEP 3: If not extended address, send RDHR
  if (!isExtended) {
    //Not extended means we have room for the HR
    for(int i = 0; i < 31; i++) {
        // consider leftmost bit
        // set line high if bit is 1, low if bit is 0
        if (rdhr & 0x80000000) {
            GPIOB->BSRR = GPIO_BSRR_BS12;
        }
        else {
            GPIOB->BSRR = GPIO_BSRR_BR12;
        }
        set_gpio_mode(GPIOB, pin, MODE_OUTPUT);

        set_gpio_mode(GPIOB, pin, MODE_INPUT);
        if ((addr & 0x80000000) && !get_gpio_input(GPIOB, pin)) {
          //We got stepped on (Transmit a recessive 1 and someone dominated 0)
          goto fail;
        }

        // shift byte left so next bit will be leftmost
        rdhr <<= 1;
    }
  }

  //Restore gmlan pins to CAN xcvr
  set_can_mode(can_gmlan_bus, 1);
  exit_critical_section();
  return 1; //Success

fail:
//We got stepped on, return failure.
//Expectation: Caller to call us again to attempt retransmit
  //Restore gmlan pins to CAN xcvr
  set_can_mode(can_gmlan_bus, 1);
  exit_critical_section();
  return 0; //Failure

}

// all commands: gas/regen, friction brake and steering
// if controls_allowed and no pedals pressed
//     allow all commands up to limit
// else
//     block all commands that produce actuation

static int gm_tx_hook(CAN_FIFOMailBox_TypeDef *to_send) {

  // There can be only one! (ASCM)
  if (gm_ascm_detected) {
    return 0;
  }

  // disallow actuator commands if gas or brake (with vehicle moving) are pressed
  // and the the latching controls_allowed flag is True
  int pedal_pressed = gm_gas_prev || (gm_brake_prev && gm_speed);
  int current_controls_allowed = controls_allowed && !pedal_pressed;

  uint32_t addr;
  if (to_send->RIR & 4) {
    // Extended
    addr = to_send->RIR >> 3;
  } else {
    // Normal
    addr = to_send->RIR >> 21;
  }

  // BRAKE: safety check
  if (addr == 789) {
    int rdlr = to_send->RDLR;
    int brake = ((rdlr & 0xF) << 8) + ((rdlr & 0xFF00) >> 8);
    brake = (0x1000 - brake) & 0xFFF;
    if (current_controls_allowed) {
      if (brake > 255) return 0;
    } else {
      if (brake != 0) return 0;
    }
  }

  // LKA STEER: safety check
  if (addr == 384) {
    int rdlr = to_send->RDLR;
    int steer = ((rdlr & 0x7) << 8) + ((rdlr & 0xFF00) >> 8);
    int max_steer = 255;
    if (current_controls_allowed) {
      // Signed arithmetic
      if (steer & 0x400) {
        if (steer < (0x800 - max_steer)) return 0;
      } else {
        if (steer > max_steer) return 0;
      }
    } else {
      if (steer != 0) return 0;
    }
  }

  // PARK ASSIST STEER: unlimited torque, no thanks
  if (addr == 823) return 0;

  // GAS/REGEN: safety check
  if (addr == 715) {
    int rdlr = to_send->RDLR;
    int gas_regen = ((rdlr & 0x7F0000) >> 11) + ((rdlr & 0xF8000000) >> 27);
    int apply = rdlr & 1;
    if (current_controls_allowed) {
      if (gas_regen > 3072) return 0;
    } else {
      // Disabled message is !engaed with gas
      // value that corresponds to max regen.
      if (apply || gas_regen != 1404) return 0;
    }
  }

  if (gm_msg_is_destined_for_gmlan(to_send)) {
    //Bitbang time!
    int successful = 0;
    int attempts = 0;
    while (!successful && attempts++ < 200) {
      successful = gm_gmlan_bitbang(to_send);
    }
  }

  // 1 allows the message through
  return true;
}

static int gm_tx_lin_hook(int lin_num, uint8_t *data, int len) {
  // LIN is not used in Volt
  return false;
}

static void gm_init(int16_t param) {
  controls_allowed = 0;
  gm_ignition_started = 0;
}

static int gm_ign_hook() {
  return gm_ignition_started;
}

static int gm_fwd_hook(int bus_num, CAN_FIFOMailBox_TypeDef *to_fwd) {
  return -1;
}

const safety_hooks gm_hooks = {
  .init = gm_init,
  .rx = gm_rx_hook,
  .tx = gm_tx_hook,
  .tx_lin = gm_tx_lin_hook,
  .ignition = gm_ign_hook,
  .fwd = gm_fwd_hook,
};

