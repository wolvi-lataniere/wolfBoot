/**
Here, we trying to port wolfboot HAL lib from stm32l0 to stm32F0
this file shall include only validated values


***/



#include <stdint.h>
#include <image.h>
#include <hal.h>
/* STM32 F0 register configuration */

/* Assembly helpers */
#define DMB() __asm__ volatile ("dmb")

/*** RCC ***/

#define RCC_BASE                    (0x40021000)
#define RCC_CR                      (*(volatile uint32_t *)(RCC_BASE + 0x00))
#define RCC_CFGR                    (*(volatile uint32_t *)(RCC_BASE + 0x04))
#define RCC_CFGR2                   (*(volatile uint32_t *)(RCC_BASE + 0x2C))
#define RCC_CR_HSIRDY               (1 << 1)
#define RCC_CR_HSION                (1 << 0)
#define RCC_CR_PLLRDY               (1 << 25)
#define RCC_CR_PLLON                (1 << 24)

#define RCC_PRESCALER_DIV_NONE      0

#define RCC_CFGR_SW_PLL             2
#define RCC_CFGR_SW_HSI             0
#define RCC_CFGR_PLLSRC_HSIDIV      0
#define RCC_CFGR_PLLMUL             10
#define RCC_CFGR_PPRE               0
#define RCC_CFGR_HPRE               0

#define RCC_CFGR2_PREDIV            0

/*** FLASH ***/

#define FLASH_BASE                  (0x40022000)
#define FLASH_ACR                   (*(volatile uint32_t *)(FLASH_BASE + 0x00))
#define FLASH_KEYR                  (*(volatile uint32_t *)(FLASH_BASE + 0x04))
#define FLASH_SR                    (*(volatile uint32_t *)(FLASH_BASE + 0x0C))
#define FLASH_CR                    (*(volatile uint32_t *)(FLASH_BASE + 0x10))
#define FLASH_AR                    (*(volatile uint32_t *)(FLASH_BASE + 0x14))

#define SYSCFG_CFGR1                (*(volatile uint32_t *)( 0x40010000))

#define FLASH_SR_EOP                (1 << 5)
#define FLASH_SR_BSY                (1 << 0)
#define FLASH_SR_WRPRTERR           (1 << 4)
#define FLASH_SR_PGERR              (1 << 2)

#define FLASH_CR_LOCK               (1 << 7)
#define FLASH_CR_PG                 (1 << 0)
#define FLASH_CR_STRT               (1 << 6)
#define FLASH_CR_PER                (1 << 1)

#define FLASH_MEM_OFFSET            (0x08000000)
#define FLASH_MEM_END               (0x08040000)
#define FLASH_PAGE_SIZE             (0x800)

#define FLASH_KEY1                  (0x45670123)
#define FLASH_KEY2                  (0xCDEF89AB)



/**
 Technical notes:
 
clock source : HSI (8Mhz)
*((uint32_t*)(0x40010000))
After each device reset, all peripheral clocks are disabled (except for the SRAM and Flash). Before using a peripheral you have to enable its clock in the RCC_AHBENR, RCC_APB2ENR or RCC_APB1ENR register


The STRT bit in the FLASH_CR register should be reset before starting a programming operation
 
 **/



static void RAMFUNCTION flash_set_waitstates(unsigned int waitstates)
{
  
}



static RAMFUNCTION void flash_wait_complete(void)
{
    while ((FLASH_SR & FLASH_SR_BSY) == FLASH_SR_BSY);
}



static void RAMFUNCTION clear_errors(void)
{
    /// clear PGERR and WRRTERR
    FLASH_SR |= FLASH_SR_PGERR | FLASH_SR_WRPRTERR | FLASH_SR_EOP;
}


int RAMFUNCTION hal_flash_write(uint32_t address, const uint8_t *data, int len)
{
  int i=0;
  uint16_t *src, *dst;

  clear_errors();


  /* Set programming mode */
  FLASH_CR |= FLASH_CR_PG;

  while(i<len)
    {
      clear_errors();
      if(((len-i) < 2) ||
         (((((uint32_t)data)+i)&0x01)!=0) ||
         (((address+i)&0x01)!= 0))
        {
          uint16_t buffer;
          uint8_t* bufBytes = (uint8_t*)&buffer;
          int offset = (address+i)&0x00000001;
          dst = (uint16_t*)(address & 0xfffffffe);

          // STM32F0 & F1 are not able to write over non-erased block, except writting a 16bits 0
          // Thus, in case of write_success, we have to write 2 bytes to 0.
          buffer = dst[i>>1];
          if ((buffer != 0xFFFF) &&
              (data[i] == 0))
            buffer = 0;

          bufBytes[offset] = data[i];
          // Write the data
          flash_wait_complete();
          dst[i>>1] = buffer;
          flash_wait_complete();
          i++;
        }
      else
        {
          src=(uint16_t*)(data);
          dst=(uint16_t*)(address);

          flash_wait_complete();
          dst[i>>1] = src[i>>1];
          flash_wait_complete();
          
          i+=2;
        }
    }

  /* unset programming mode */
  FLASH_CR &= ~FLASH_CR_PG;

  return 0;//((FLASH_SR&FLASH_SR_PGERR)==0)?0:1;
}

void RAMFUNCTION hal_flash_unlock(void)
{
    /*
     If the IAP interface of the flash memory of the target requires it, this function is called before every write and erase operations to unlock write access to the flash. On some targets, this function may be empty.
     */
  flash_wait_complete();
  FLASH_KEYR = FLASH_KEY1;
  DMB();
  FLASH_KEYR = FLASH_KEY2;
  DMB();
  while ((FLASH_CR & FLASH_CR_LOCK) != 0);
}


void RAMFUNCTION hal_flash_lock(void)
{
    /*
     If the IAP interface of the flash memory requires locking/unlocking, this function restores the flash write protection by excluding write accesses. This function is called by the bootloader at the end of every write and erase operations.
     */
    FLASH_CR |= FLASH_CR_LOCK;
    DMB();
}


int RAMFUNCTION hal_flash_erase(uint32_t address, int len)
{
    /*
     Called by the bootloader to erase part of the flash memory to allow subsequent boots. Erase operations must be performed via the specific IAP interface of the target microcontroller. address marks the start of the area that the bootloader wants to erase, and len specifies the size of the area to be erased. This function must take into account the geometry of the flash sectors, and erase all the sectors in between.
     */
    
    /*  SEE DOCUMENTATION PAGE 60   */
    
    uint32_t end, cadd;
    uint32_t i;
    if( address >= 0x20000000 && address <= 0x20002000)
        return 0;

    //    address &= 0x00FFF000;
    ///cadd = FLASH_MEM_OFFSET + address;

    /* Check bound compliance
    if((FLASH_MEM_OFFSET + address + len) > FLASH_MEM_END)
        return -1;*/
    
    /*Find start sector*/
   // for(i = 0; i < address; i+= FLASH_PAGE_SIZE); /// <= ?
    /* start at previous sector*/
    cadd = address;//FLASH_MEM_OFFSET + i; //- FLASH_SECTOR_SIZE;
    /* compute last sector to erase*/
    end = cadd + len - 1;

    flash_wait_complete();
    clear_errors();
    
    for( cadd ; cadd < end; cadd+=FLASH_PAGE_SIZE) // <= ?
    {
        /*Set erase mode*/
        FLASH_CR |= FLASH_CR_PER;
        /* set page address to erase*/
        FLASH_AR = cadd;
        /* start erase */
        FLASH_CR |= FLASH_CR_STRT;
        DMB();
        /*wait for it*/
        flash_wait_complete();
        /* /\*wair end of operation and clear it*\/ */
        /* while((FLASH_SR & FLASH_SR_EOP)  != FLASH_SR_EOP); */
        /* FLASH_SR &= ~FLASH_SR_EOP; */
        /*Unset erase mode */
        FLASH_CR &= ~FLASH_CR_PER;
    }

    return 0;
}



static void clock_pll_off(void)
{
    /* Select HSI as SYSCLK source */
     uint32_t reg32 = RCC_CFGR;
     reg32 &= ~((1 << 1) | (1 << 0));
     RCC_CFGR = (reg32 | RCC_CR_HSION);
     DMB();
     
     /* Turn off PLL */
    RCC_CR &= ~RCC_CR_PLLON;
    DMB();
}


static void clock_pll_on(int powersave)
{
  uint32_t reg32;
  /*Enable Power Controller*/
  //APB1_CLOCK_ER |= PWR_APB1_CLOCK_ER_VAL;
  
  
  /* CPU Speed = 48MHz */
  
  
  /* Enable internal high-speed oscillator (HSI=8Mhz) */
  RCC_CR |= RCC_CR_HSION;
  DMB();
  while ((RCC_CR & RCC_CR_HSIRDY) == 0) {};
  
  /* Select HSI as SYSCLK source */
  reg32 = RCC_CFGR;
  reg32 &= ~((1 << 1) | (1 << 0));
  RCC_CFGR = (reg32 | RCC_CR_HSION);
  
  /* Set Predivider PLL*/
  reg32 = RCC_CFGR2;
  reg32 &= ~((1 << 3) | (1 << 2) | (1 << 1) | (1 << 0));
  RCC_CFGR2 = (reg32 | RCC_CFGR2_PREDIV);
  DMB();
  
  /* Set HSI as PLL source */
  reg32 = RCC_CFGR;
  reg32 &= ~((1 << 16) | (1 << 15));
  RCC_CFGR = (reg32 | RCC_CFGR_PLLSRC_HSIDIV << 15);
  DMB();
  
  /* Set PLL multiplier */
  reg32 = RCC_CFGR;
  reg32 &= ~((1 << 21) | (1 << 20) | (1 << 19) | (1 << 18));
  RCC_CFGR = (reg32 | RCC_CFGR_PLLMUL << 18);
  DMB();
  
  /* Set PCLK Prescaler (APB) */
  reg32 = RCC_CFGR;
  reg32 &= ~((1 << 10) | (1 << 9) | (1 << 8));
  RCC_CFGR = (reg32 | RCC_CFGR_PPRE << 8);
  DMB();
  
  /* Set HCLK Prescaler (AHB) */
  reg32 = RCC_CFGR;
  reg32 &= ~((1 << 7) | (1 << 6) | (1 << 5) | (1 << 4));
  RCC_CFGR = (reg32 | RCC_CFGR_HPRE << 4);
  DMB();
  
  /* Enable PLL */
  RCC_CR |= RCC_CR_PLLON;
  DMB();
  while ((RCC_CR & RCC_CR_PLLRDY) == 0) {};
  
  /* Select PLL as SYSCLK source */
  reg32 = RCC_CFGR;
  reg32 &= ~((1 << 1) | (1 << 0));
  RCC_CFGR = (reg32 | RCC_CFGR_SW_PLL);
  DMB();
}



void hal_init(void)
{
    /*
    This function is called by the bootloader at the very beginning of the execution. Ideally, the implementation provided configures the clock settings for the target microcontroller, to ensure that it runs at at the required speed to shorten the time required for the cryptography primitives to verify the firmware images.
    */
  FLASH_ACR = (FLASH_ACR & (~((1<<5) | (1<<4)))) | 1;
  DMB();
  clock_pll_on(0);
}


void hal_prepare_boot(void)
{
  /*
    This function is called by the bootloader at a very late stage, before chain-loading the firmware in the next stage. This can be used to revert all the changes made to the clock settings, to ensure that the state of the microcontroller is restored to its original settings.
  */
#ifdef SPI_FLASH
  spi_release();
#endif
  asm volatile("cpsid i");
  volatile uint32_t *src, *dst;

  dst = (uint32_t *)0x20000000;
	src = (uint32_t *)(WOLFBOOT_PARTITION_BOOT_ADDRESS + IMAGE_HEADER_SIZE);

  while( dst < (uint32_t*)0x200000C0)
	{
		*dst = *src;
		dst++;
		src++;
	}
	///// remap SRAM at 0x00000000 using SYSCFG
	SYSCFG_CFGR1 |= (uint32_t)3; //&= ~(uint32_t)3;///

  clock_pll_off();
}
