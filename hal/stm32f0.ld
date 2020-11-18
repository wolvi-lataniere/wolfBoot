MEMORY
{
    FLASH (rx) : ORIGIN = 0x8000000, LENGTH = ##WOLFBOOT_PARTITION_BOOT_ADDRESS## - 0x08000000
    REFLASH (rx) : ORIGIN = ##WOLFBOOT_PARTITION_BOOT_ADDRESS## LENGTH = ##WOLFBOOT_PARTITION_SIZE##
    RAM (rwx) : ORIGIN = 0x20000100, LENGTH = 0x00000F00
}

SECTIONS
{
    .text :
    {
        _start_text = .;
        KEEP(*(.isr_vector))
        *(.text*)
        *(.rodata*)
        . = ALIGN(4);
        _end_text = .;
    } > FLASH
    .edidx :
    {
        . = ALIGN(4);
        *(.ARM.exidx*)
    } > FLASH

    .edidx :
    {
        . = ALIGN(4);
        *(.ARM.exidx*)
    } > FLASH


    _stored_data = .;
    .data : AT (_stored_data)
    {
        _start_data = .;
        KEEP(*(.data*))
        . = ALIGN(4);
        KEEP(*(.ramcode))
        . = ALIGN(4);
        _end_data = .;
    } > RAM

    .bss (NOLOAD) :
    {
        _start_bss = .;
        __bss_start__ = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _end_bss = .;
        __bss_end__ = .;
        _end = .;
    } > RAM
    . = ALIGN(4);
    
    .usercode :
    {
	  . = ALIGN(4);
	  FILL(0xFFFFFFFF);
	  . = ORIGIN(REFLASH) + LENGTH(REFLASH) - 1;
    } > REFLASH
}

END_STACK = ORIGIN(RAM) + LENGTH(RAM);
