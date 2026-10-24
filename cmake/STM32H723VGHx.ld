/* GNU ld layout for STM32H723VGHx. DMA buffers stay in D2 SRAM. */

ENTRY(Reset_Handler)

MEMORY
{
  FLASH   (rx)  : ORIGIN = 0x08000000, LENGTH = 1M
  DTCMRAM (xrw) : ORIGIN = 0x20000000, LENGTH = 128K
  AXI_SRAM (xrw): ORIGIN = 0x24000000, LENGTH = 320K
  RAM_D2  (xrw) : ORIGIN = 0x30000000, LENGTH = 32K
}

_estack = ORIGIN(DTCMRAM) + LENGTH(DTCMRAM);
_Min_Heap_Size = 0;
_Min_Stack_Size = 0x1000;

SECTIONS
{
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } > FLASH

  .text :
  {
    . = ALIGN(4);
    *(.text*)
    *(.rodata*)
    *(.glue_7)
    *(.glue_7t)
    KEEP(*(.init))
    KEEP(*(.fini))
    . = ALIGN(4);
    _etext = .;
  } > FLASH

  .ARM.extab :
  {
    *(.ARM.extab* .gnu.linkonce.armextab.*)
  } > FLASH

  .ARM :
  {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } > FLASH

  .preinit_array :
  {
    PROVIDE_HIDDEN(__preinit_array_start = .);
    KEEP(*(.preinit_array*))
    PROVIDE_HIDDEN(__preinit_array_end = .);
  } > FLASH

  .init_array :
  {
    PROVIDE_HIDDEN(__init_array_start = .);
    KEEP(*(SORT(.init_array.*)))
    KEEP(*(.init_array*))
    PROVIDE_HIDDEN(__init_array_end = .);
  } > FLASH

  .fini_array :
  {
    PROVIDE_HIDDEN(__fini_array_start = .);
    KEEP(*(SORT(.fini_array.*)))
    KEEP(*(.fini_array*))
    PROVIDE_HIDDEN(__fini_array_end = .);
  } > FLASH

  _sidata = LOADADDR(.data);
  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data*)
    . = ALIGN(4);
    _edata = .;
  } > AXI_SRAM AT> FLASH

  .bss (NOLOAD) :
  {
    . = ALIGN(4);
    _sbss = .;
    __bss_start__ = _sbss;
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
    __bss_end__ = _ebss;
  } > AXI_SRAM

  /* newlib's nosys sbrk stub uses this traditional heap boundary symbol. */
  end = _ebss;
  _end = end;
  __heap_end__ = ORIGIN(AXI_SRAM) + LENGTH(AXI_SRAM);

  .RAM_D2 (NOLOAD) :
  {
    . = ALIGN(4);
    __ram_d2_start__ = .;
    *(RAM_D2)
    *(RAM_D2*)
    . = ALIGN(4);
    __ram_d2_end__ = .;
  } > RAM_D2

  .ramfunc :
  {
    . = ALIGN(4);
    *(.RamFunc)
    *(.RamFunc*)
    . = ALIGN(4);
  } > AXI_SRAM AT> FLASH

  .stack (NOLOAD) :
  {
    . = ALIGN(8);
    . += _Min_Stack_Size;
    . = ALIGN(8);
  } > DTCMRAM

  /DISCARD/ :
  {
    *(.note.GNU-stack)
    *(.comment)
  }
}

ASSERT(__ram_d2_end__ <= ORIGIN(RAM_D2) + LENGTH(RAM_D2), "RAM_D2 overflow")
ASSERT(_ebss <= ORIGIN(AXI_SRAM) + LENGTH(AXI_SRAM), "AXI SRAM overflow")
ASSERT(SIZEOF(.stack) <= LENGTH(DTCMRAM), "DTCM stack overflow")
