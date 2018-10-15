# Hardware abstraction layer

In order to run wolfBoot on a target microcontroller, an implementation of the HAL
must be provided.

The HAL only purposes are allowing write/erase operations from the bootloader
and the application initiating the firmware upgrade through the bootutil library, and
ensuring that the MCU is running at full speed during boot, to optimize the
verification of the signatures.

The implementation of the hardware-specific calls for each platform are grouped in 
a single c file in the [hal](hal) directory.

The directory also contains a platform-specific linker script for each supported MCU,  
with the same name and  the `.ld` extension. This is used to link the bootloader's 
firmware on the specific hardware, exporting all the necessary symbols for flash 
and RAM boundaries.

## Supported platforms

The following platforms are supported in the current version:
  - STM32F4
  - nRF52

## API

The Hardware Abstraction Layer (HAL) consists of six function calls
be implemented for each supported target:

`void hal_init(void)`

This function is called by the bootloader at the very beginning of the execution.
Ideally, the implementation provided configures the clock settings for the target 
microcontroller, to ensure that it runs at at the required speed to shorten the 
time required for the cryptography primitives to verify the firmware images.

`void hal_flash_unlock(void)`

If the IAP interface of the flash memory of the target requires it, this function
is called before every write and erase operations to unlock write access to the
flash. On some targets, this function may be empty.

`int hal_flash_write(uint32_t address, const uint8_t *data, int len)`

This function provides an implementation of the flash write function, using the
target's IAP interface. `address` is the offset from the beginning of the
flash area, `data` is the payload to be stored in the flash using the IAP interface,
and `len` is the size of the payload. `hal_flash_write` should return 0 upon success,
or a negative value in case of failure.

`void hal_flash_lock(void)`

If the IAP interface of the flash memory requires locking/unlocking, this function
restores the flash write protection by excluding write accesses. This function is called
by the bootloader at the end of every write and erase operations.

`int hal_flash_erase(uint32_t address, int len)`

Called by the bootloader to erase part of the flash memory to allow subsequent boots.
Erase operations must be performed via the specific IAP interface of the target microcontroller.
`address` marks the start of the area that the bootloader wants to erase, and `len` specifies
the size of the area to be erased. This function must take into account the geometry of the flash
sectors, and erase all the sectors in between.

`void hal_prepare_boot(void)`

This function is called by the bootloader at a very late stage, before chain-loading the firmware
in the next stage. This can be used to revert all the changes made to the clock settings, to ensure
that the state of the microcontroller is restored to its original settings.

