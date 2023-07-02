#include "main.h"
#include <stdint.h>

void configure_board_led(PortRegisters *port_registers) {
  // board led is driven by IO pin PB30
  port_registers->groups[1].direction = 1 << 30;
}
void toggle_board_led(PortRegisters *port_registers) {
  // board led is driven by IO pin PB30
  port_registers->groups[1].output_toggle = 1 << 30;
}

void configure_rtc_1hz_interrupt(GCLKRegisters *gclk_registers,
                                 uint32_t *nvic_iser,
                                 RTCRegistersMode0 *rtc_registers) {
  // Configure generator 4 with no divider
  uint32_t gclk_gen_id_4 = 4;
  gclk_registers->gendiv = gclk_gen_id_4;
  // Enable generator 4 and set source to onboard low power 32k clock
  uint32_t gclk_gen_enable = 1 << 16;
  uint32_t gclk_gen_source_ulp32k = 3 << 8;
  gclk_registers->genctrl =
      gclk_gen_enable | gclk_gen_source_ulp32k | gclk_gen_id_4;
  // Enable GCLK_RTC and set generator 4 as source
  uint32_t gclk_rtc = 4;
  uint32_t gclk_source_gen_4 = gclk_gen_id_4 << 8;
  uint32_t gclk_enable = 1 << 14;
  gclk_registers->clkctrl = gclk_enable | gclk_source_gen_4 | gclk_rtc;

  // Enable RTC interrupts
  *NVIC_ISER |= 1 << 3;

  rtc_registers->count = 0x00000000;
  // takes 1 second for a 32kHz counter to reach 32K
  rtc_registers->comp0 = 0x00008000;
  // fire interrupt when count reaches comp0
  uint8_t int_comp0 = 1;
  rtc_registers->intenset = int_comp0;
  uint16_t rtc_enable = 1 << 1;
  // mode 0 is "just count" mode
  uint16_t rtc_mode = 0 << 2;
  // reset count when we reach comp0 value
  uint16_t rtc_matchclear = 1 << 7;
  rtc_registers->ctrl = rtc_enable | rtc_mode | rtc_matchclear;
}

void configure_sercom0_uart(uint32_t *nvic_iser, GCLKRegisters *gclk_registers, PortRegisters *port_registers) {
  // TODO Enable bus clock
  // The SERCOM bus clock (CLK_SERCOMx_APB) can be enabled and disabled in the Power Manager. Refer to
  // Peripheral Clock Masking for details and default status of this clock.

  // Configure generator 5 with no divider
  uint32_t gclk_gen_id_5 = 5;
  gclk_registers->gendiv = gclk_gen_id_5;
  // Enable generator 5 and set source to onboard low power 32k clock
  uint32_t gclk_gen_enable = 1 << 16;
  uint32_t gclk_gen_source_ulp32k = 3 << 8;
  gclk_registers->genctrl =
      gclk_gen_enable | gclk_gen_source_ulp32k | gclk_gen_id_5;
  // Enable GCLK_SERCOM0_CORE and set generator 5 as source
  uint32_t gclk_sercom0_core = 0x14;
  uint32_t gclk_source_gen_5 = gclk_gen_id_5 << 8;
  uint32_t gclk_enable = 1 << 14;
  gclk_registers->clkctrl = gclk_enable | gclk_source_gen_5 | gclk_sercom0_core;

  // Configure PA10 as Rx
  port_registers->groups[0].direction_clear = 1 << 10;
  // TODO do I need to set pmuxen?
  // set INEN
  port_registers->groups[0].pin_config[10] = 1 << 1;

  // Configure PA11 as Tx
  port_registers->groups[0].direction = 1 << 11;
}

int main() {
  PortRegisters *port_registers = ((PortRegisters *)PORT_REGISTERS_ADDRESS);
  configure_board_led(port_registers);
  toggle_board_led(port_registers);

  GCLKRegisters *gclk_registers = ((GCLKRegisters *)GCLK_REGISTERS_ADDRESS);
  RTCRegistersMode0 *rtc_registers = ((RTCRegistersMode0 *)RTC_REGISTERS_ADDRESS);
  configure_rtc_1hz_interrupt(gclk_registers, NVIC_ISER, rtc_registers);

  return 0;
}

void rtc_handler() {
  PortRegisters *port_registers = ((PortRegisters *)PORT_REGISTERS_ADDRESS);
  RTCRegistersMode0 *rtc_register = ((RTCRegistersMode0 *)RTC_REGISTERS_ADDRESS);

  toggle_board_led(port_registers);

  // clear interrupt flag in rtc
  uint8_t int_comp0 = 1;
  rtc_register->intflag = int_comp0;
  // clear interrupt flag in nvic
  *NVIC_ICPR |= 1 << 3;
}

extern uint32_t _svectors;
extern uint32_t _sidata;
extern uint32_t _sdata;
extern uint32_t _edata;
extern uint32_t _sbss;
extern uint32_t _ebss;
extern uint32_t _estack;

void empty_handler(void) {
  while (1)
    ;
}

void reset_handler(void) {
  // move initialized data to ram
  for (uint32_t *src = &_sdata, *dest = &_sidata; src < &_edata;
       src++, dest++) {
    *dest = *src;
  }
  // zero bss
  for (uint32_t *bss = &_sbss; bss < &_ebss; bss++) {
    *bss = 0;
  }

  main();

  while (1)
    ;
}

__attribute__((section(".vectors"))) const VectorTable VECTOR_TABLE = {
    .stack = (void *)(&_estack),

    .reset_handler = (void *)reset_handler,
    .nonmaskable_interrupt_handler = (void *)empty_handler,
    .hard_fault_handler = (void *)empty_handler,
    .reserved_c12 = (void *)(0),
    .reserved_c11 = (void *)(0),
    .reserved_c10 = (void *)(0),
    .reserved_c9 = (void *)(0),
    .reserved_c8 = (void *)(0),
    .reserved_c7 = (void *)(0),
    .reserved_c6 = (void *)(0),
    .svc_handler = (void *)empty_handler,
    .reserved_c4 = (void *)(0),
    .reserved_c3 = (void *)(0),
    .pendsv_handler = (void *)empty_handler,
    .systick_handler = (void *)empty_handler,

    .power_manager_handler = (void *)empty_handler,
    .system_control_handler = (void *)empty_handler,
    .watchdog_timer_handler = (void *)empty_handler,
    .realtime_counter_handler = (void *)rtc_handler,
    .external_interrupt_controller_handler = (void *)empty_handler,
    .nonvolatile_mem_controller_handler = (void *)empty_handler,
    .dma_controller_handler = (void *)empty_handler,
    .usb_handler = (void *)empty_handler,
    .event_system_interface_handler = (void *)empty_handler,
    .sercom0_handler = (void *)empty_handler,
    .sercom1_handler = (void *)empty_handler,
    .sercom2_handler = (void *)empty_handler,
    .sercom3_handler = (void *)empty_handler,
    .sercom4_handler = (void *)empty_handler,
    .sercom5_handler = (void *)empty_handler,
    .timer_counter_control_0_handler = (void *)empty_handler,
    .timer_counter_control_1_handler = (void *)empty_handler,
    .timer_counter_control_2_handler = (void *)empty_handler,
    .timer_counter_3_handler = (void *)empty_handler,
    .timer_counter_4_handler = (void *)empty_handler,
    .timer_counter_5_handler = (void *)empty_handler,
    .timer_counter_6_handler = (void *)empty_handler,
    .timer_counter_7_handler = (void *)empty_handler,
    .adc_handler = (void *)empty_handler,
    .analog_comparator_handler = (void *)empty_handler,
    .dac_handler = (void *)empty_handler,
    .peripheral_touch_controller_handler = (void *)empty_handler,
    .i2s_handler = (void *)empty_handler,
};