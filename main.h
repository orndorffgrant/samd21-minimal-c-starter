#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>

typedef struct _SercomUartRegisters {
  volatile uint32_t ctrla;
  volatile uint32_t ctrlb;
  volatile const uint8_t _reserved0[4];
  volatile uint16_t baud;
  volatile uint8_t rxpl;
  volatile const uint8_t _reserved1[5];
  volatile uint8_t intenclr;
  volatile const uint8_t _reserved2;
  volatile uint8_t intenset;
  volatile const uint8_t _reserved3;
  volatile uint8_t intflag;
  volatile const uint8_t _reserved4;
  volatile uint16_t status;
  volatile uint32_t syncbusy;
  volatile const uint8_t _reserved5[8];
  volatile uint8_t data[2];
  volatile const uint8_t _reserved6[6];
  volatile uint8_t dbgctrl;
} SercomUartRegisters;
#define SERCOM0_REGISTERS_ADDRESS 0x42000800

typedef struct _PowerManagerRegisters {
  volatile uint8_t ctrl;
  volatile uint8_t sleep;
  volatile const uint8_t _reserved0[6];
  volatile uint8_t cpusel;
  volatile uint8_t apbasel;
  volatile uint8_t apbbsel;
  volatile uint8_t apbcsel;
  volatile const uint8_t _reserved1[8];
  volatile uint32_t ahbmask;
  volatile uint32_t apbamask;
  volatile uint32_t apbbmask;
  volatile uint32_t apbcmask;
  volatile const uint8_t _reserved2[16];
  volatile uint8_t intenclr;
  volatile uint8_t intenset;
  volatile uint8_t intflag;
  volatile const uint8_t _reserved3;
  volatile uint8_t rcause;
} PowerManagerRegisters;
#define POWER_MANAGER_REGISTERS_ADDRESS 0x40000400

typedef struct _RTCRegistersMode0 {
  volatile uint16_t ctrl;
  volatile uint16_t readreq;
  volatile uint16_t evctrl;
  volatile uint8_t intenclr;
  volatile uint8_t intenset;
  volatile uint8_t intflag;
  volatile const uint8_t _reserved0;
  volatile uint8_t status;
  volatile uint8_t dbgctrl;
  volatile uint8_t freqcorr;
  volatile const uint8_t _reserved1[3];
  volatile uint32_t count;
  volatile const uint8_t _reserved2[4];
  volatile uint32_t comp0;
} RTCRegistersMode0;
#define RTC_REGISTERS_ADDRESS 0x40001400

typedef struct _GCLKRegisters {
  volatile uint8_t ctrl;
  volatile uint8_t status;
  volatile uint16_t clkctrl;
  volatile uint32_t genctrl;
  volatile uint32_t gendiv;
} GCLKRegisters;
#define GCLK_REGISTERS_ADDRESS 0x40000C00

typedef struct _PortRegisterGroup {
  volatile uint32_t direction;
  volatile uint32_t direction_clear;
  volatile uint32_t direction_set;
  volatile uint32_t direction_toggle;
  volatile uint32_t output;
  volatile uint32_t output_clear;
  volatile uint32_t output_set;
  volatile uint32_t output_toggle;
  volatile const uint32_t input;
  volatile uint32_t control;
  volatile uint32_t write_config;
  volatile const uint8_t _reserved0[4];
  volatile uint8_t peripheral_mux[16];
  volatile uint8_t pin_config[32];
  volatile const uint8_t _reserved1[32];
} PortRegisterGroup;

typedef struct _PortRegisters {
  PortRegisterGroup groups[2];
} PortRegisters;
#define PORT_REGISTERS_ADDRESS 0x41004400

#define NVIC_ISER ((uint32_t *)0xE000E100)
#define NVIC_ICPR ((uint32_t *)0xE000E280)

typedef struct _VectorTable {
  void *stack;

  void *reset_handler;
  void *nonmaskable_interrupt_handler;
  void *hard_fault_handler;
  void *reserved_c12;
  void *reserved_c11;
  void *reserved_c10;
  void *reserved_c9;
  void *reserved_c8;
  void *reserved_c7;
  void *reserved_c6;
  void *svc_handler;
  void *reserved_c4;
  void *reserved_c3;
  void *pendsv_handler;
  void *systick_handler;

  void *power_manager_handler;
  void *system_control_handler;
  void *watchdog_timer_handler;
  void *realtime_counter_handler;
  void *external_interrupt_controller_handler;
  void *nonvolatile_mem_controller_handler;
  void *dma_controller_handler;
  void *usb_handler;
  void *event_system_interface_handler;
  void *sercom0_handler;
  void *sercom1_handler;
  void *sercom2_handler;
  void *sercom3_handler;
  void *sercom4_handler;
  void *sercom5_handler;
  void *timer_counter_control_0_handler;
  void *timer_counter_control_1_handler;
  void *timer_counter_control_2_handler;
  void *timer_counter_3_handler;
  void *timer_counter_4_handler;
  void *timer_counter_5_handler;
  void *timer_counter_6_handler;
  void *timer_counter_7_handler;
  void *adc_handler;
  void *analog_comparator_handler;
  void *dac_handler;
  void *peripheral_touch_controller_handler;
  void *i2s_handler;
} VectorTable;

#endif
