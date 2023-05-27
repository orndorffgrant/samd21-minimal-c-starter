build:
  @mkdir -p build/artifacts
  @mkdir -p build/dist
  /usr/bin/arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -c -Os -Wall -ffunction-sections -o build/artifacts/main.o main.c
  /usr/bin/arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -T link.ld -Wl,--gc-sections --specs=nosys.specs -o build/dist/main.elf build/artifacts/main.o
  /usr/bin/arm-none-eabi-objcopy -O ihex -R .eeprom -R .fuse -R .lock -R .signature build/dist/main.elf build/dist/main.hex

program: build
  openocd -f ./openocd/atmel_samd21_xplained_pro.cfg -c "program build/dist/main.hex verify reset exit"

clean:
  rm -rf ./build