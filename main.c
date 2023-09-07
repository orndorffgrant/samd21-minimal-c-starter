#include "main.h"
#include <cstdint>
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

void configure_sercom0_uart(uint32_t *nvic_iser, PowerManagerRegisters *power_manager_registers,
                            GCLKRegisters *gclk_registers,
                            PortRegisters *port_registers,
                            SercomUartRegisters *sercom0_uart_registers) {
  // Enable bus clock in Power Manager
  uint32_t apbc_mask_sercom0 = 1 << 2;
  power_manager_registers->apbcmask |= apbc_mask_sercom0;

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

  // Configure PA8 as Rx
  port_registers->groups[0].direction_clear = 1 << 8;
  // TODO do I need to set pmuxen?
  // set INEN
  port_registers->groups[0].pin_config[8] = 1 << 1;

  // Configure PA9 as Tx
  port_registers->groups[0].direction = 1 << 9;


  // Configure SERCOM0 peripheral as UART
  // 1. Select either external (0x0) or internal clock (0x1) by writing the Operating Mode value in the CTRLA register
  //    (CTRLA.MODE).
  uint32_t sercom_internal_clock = 1 << 2;
  // 2. Select either asynchronous (0) or or synchronous (1) communication mode by writing the Communication
  //    Mode bit in the CTRLA register (CTRLA.CMODE).
  uint32_t sercom_async = 0 << 28;
  // 3. Select pin for receive data by writing the Receive Data Pinout value in the CTRLA register (CTRLA.RXPO).
  uint32_t pa8_pad_id = 0;
  uint32_t sercom_rx_pa8 = pa8_pad_id << 20;
  // 4. Select pads for the transmitter and external clock by writing the Transmit Data Pinout bit in the CTRLA register
  //    (CTRLA.TXPO).
  uint32_t pa9_pad_id = 1;
  uint32_t sercom_tx_pa9 = pa9_pad_id << 16;
  sercom0_uart_registers->ctrla = sercom_internal_clock | sercom_async | sercom_rx_pa8 | sercom_tx_pa9;

  // 5. Configure the Character Size field in the CTRLB register (CTRLB.CHSIZE) for character size.
  uint32_t byte_char = 8 << 0;
  sercom0_uart_registers->ctrlb = byte_char;

  // 6. Set the Data Order bit in the CTRLA register (CTRLA.DORD) to determine MSB- or LSB-first data
  //    transmission.
  uint32_t msb = 0 << 30;
  sercom0_uart_registers->ctrla |=  msb;

  // 7. To use parity mode:
  //    a. Enable parity mode by writing 0x1 to the Frame Format field in the CTRLA register (CTRLA.FORM).
  //    b. Configure the Parity Mode bit in the CTRLB register (CTRLB.PMODE) for even or odd parity.
  // Skipping parity for now

  // 8. Configure the number of stop bits in the Stop Bit Mode bit in the CTRLB register (CTRLB.SBMODE).
  uint32_t one_stop_bit = 0 << 6;
  sercom0_uart_registers->ctrlb |= one_stop_bit;
  
  // 9. When using an internal clock, write the Baud register (BAUD) to generate the desired baud rate.
  sercom0_uart_registers->baud = 1234; // TODO
  // 10. Enable the transmitter and receiver by writing '1' to the Receiver Enable and Transmitter Enable bits in the
  //     CTRLB register (CTRLB.RXEN and CTRLB.TXEN).
  uint32_t rxen = 1 << 17;
  uint32_t txen = 1 << 16;
  sercom0_uart_registers->ctrlb |= rxen | txen;

  // This peripheral is enabled by writing '1' to the Enable bit in the Control A register (CTRLA.ENABLE)
  uint32_t enable = 1 << 1;
  sercom0_uart_registers->ctrla |=  enable;
}

int main() {
  PortRegisters *port_registers = ((PortRegisters *)PORT_REGISTERS_ADDRESS);
  GCLKRegisters *gclk_registers = ((GCLKRegisters *)GCLK_REGISTERS_ADDRESS);
  RTCRegistersMode0 *rtc_registers =
      ((RTCRegistersMode0 *)RTC_REGISTERS_ADDRESS);
  SercomUartRegisters *sercom0_uart_registers =
      ((SercomUartRegisters *)SERCOM0_REGISTERS_ADDRESS);
  PowerManagerRegisters *power_manager_registers = ((PowerManagerRegisters *)POWER_MANAGER_REGISTERS_ADDRESS);

  configure_board_led(port_registers);
  toggle_board_led(port_registers);
  configure_rtc_1hz_interrupt(gclk_registers, NVIC_ISER, rtc_registers);
  configure_sercom0_uart(NVIC_ISER, power_manager_registers, gclk_registers, port_registers,
                         sercom0_uart_registers);

  return 0;
}

void rtc_handler() {
  PortRegisters *port_registers = ((PortRegisters *)PORT_REGISTERS_ADDRESS);
  RTCRegistersMode0 *rtc_register =
      ((RTCRegistersMode0 *)RTC_REGISTERS_ADDRESS);

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