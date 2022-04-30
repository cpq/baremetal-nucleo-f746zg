ENTRY(_reset);
MEMORY {
  flash(rx)  : ORIGIN = 0x08000000, LENGTH = 1024k
  sram(rwx) : ORIGIN = 0x20000000, LENGTH = 320k
}
_estack     = ORIGIN(sram) + LENGTH(sram);    /* boot.s and init_heap() */
SECTIONS {
  .vectors  : { KEEP(*(.vectors)) } > flash
  .text     : { *(.text*) } > flash
  .rodata   : { *(.rodata*) } > flash

  .data : {
    _sdata = .;   /* for init_ram() */
    *(.first_data)
    *(.data SORT(.data.*))
    _edata = .;  /* for init_ram() */
  } > sram AT > flash
  _sidata = LOADADDR(.data);

  .bss : {
    _sbss = .;              /* for init_ram() */
    __bss_start__ = _sbss;  /* for libc */
    *(.bss SORT(.bss.*) COMMON)
    _ebss = .;              /* for init_ram() */
    __bss_end__ = _ebss;    /* for libc */
  } > sram

  . = ALIGN(8);
  end = .;      /* for syscalls.c */
  _end = .;     /* for cmsis_gcc.h and init_ram() */
  __end__ = .;  /* for libc */
}
