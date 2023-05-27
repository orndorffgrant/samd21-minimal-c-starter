# Minimal Atmel SAMD21 Xplained Pro board bare metal C starter

## Development environment
This was developed and tested on Ubuntu 23.04.

```
sudo apt install openocd gcc-arm-none-eabi
snap install --edge --classic just
```

## Developing

Hook up the dev board debug USB port to your machine.

```
just program
```