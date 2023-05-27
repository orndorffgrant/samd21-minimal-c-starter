OUTPUT_FORMAT(elf32-littlearm)
OUTPUT_ARCH(arm)

MEMORY
{
    flash (rx) : ORIGIN = 0x00000000, LENGTH = 0x00040000
    ram (rwx) : ORIGIN = 0x20000000, LENGTH = 0x00008000
}

_estack = ORIGIN(ram) + LENGTH(ram);

SECTIONS
{
    .vectors :
    {
        . = ALIGN(4);
        _svectors = .;
        KEEP(*(.vectors))
    } > flash

    .text :
    {
        *(.text*)
    } > flash

    .rodata :
    {
        *(.rodata*)
    } > flash

    .data : {
        . = ALIGN(4);
        _sdata = .;
        *(.data SORT(.data.*))
        . = ALIGN(4);
        _edata = .;
    } > ram AT > flash
    _sidata = LOADADDR(.data);

    .bss (NOLOAD) :
    {
        . = ALIGN(4);
        _sbss = . ;
        *(.bss .bss.*)
        . = ALIGN(4);
        _ebss = . ;
    } > ram

    . = ALIGN(4);
    _end = . ;
}
