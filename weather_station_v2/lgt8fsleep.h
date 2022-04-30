#include <lgt8f328p_spec.h>
#define PMU_IOCD (1 << 7) | (1 << 6) 
#define PMU_SWDD 1 
#define PMU_DPS2 1 
#define PMU_LDOPD  1 
#define PMU_CLKPR  1 

#define PMU_SLEEP_IDLE  0      // 待机模式 
#define PMU_SLEEP_ANRM  1      // ADC噪声抑制休眠模式 
#define PMU_SLEEP_SAVE  2      // 省电模式 
#define PMU_SLEEP_DPS1  3      // 掉电DPS1模式 
#define PMU_SLEEP_DPS0  6      // 掉电DPS0模式 
#define PMU_SLEEP_DPS2  7      // 掉电DPS2模式 
 
// DPS2寄存器控制 
#define pmuIOCD(pid) IOCWK = pid 
#define pmuEnableDPS2() DPS2R |= ( 1 << DPS2E) 
 
// PMU休眠管理函数  
void pmuSleep(u8 mode) 
{ 
 u8 CLKPR_reg = CLKPR; 
 u8 LDOCR_reg = LDOCR; 
 u8 MCUSR_reg = MCUSR | 0x7f; 
 u8 SREG_reg = SREG; 
 
 // global interrupt disable 
 CLI(); 
 
#if PMU_SWDD == 1 
 // disable SWD interface to release PE0/2 
 MCUSR = 0xff; 
 MCUSR = 0xff; 
#endif 
 
#if PMU_DPS2EN == 1 
 pmuIOCD(PMU_IOCD); 
 pmuEnableDPS2(); 
#endif 
 
#if (PMU_CLKPR == 1) || (PMU_LDOPD == 1) 
 // minimize system power before LDO power/down 
 CLKPR = 0x80; 
 CLKPR = 0x08; 
#endif 
 
 //NOP(); NOP(); NOP(); NOP(); 
#if PMU_LDOPD == 1 
 LDOCR = 0x80; 
 LDOCR = 0x02; // LDO power/down 
#endif 
 
 //NOP(); NOP(); NOP(); NOP(); 
 // bring system to sleep 
 SMCR = ((mode & 0x7) << 1) | 0x1; 
 SLEEP(); 
 
 NOP(); NOP(); // NOP(); NOP(); 
 
#if PMU_LDOPD == 1 
 // restore LDO settings 
 LDOCR = 0x80; 
 LDOCR = LDOCR_reg; 
#endif 
 NOP(); NOP(); NOP(); NOP(); 
 
#if PMU_CLKPR == 1 
 // restore system clock prescaler 
 CLKPR = 0x80; 
 CLKPR = CLKPR_reg; 
#endif 
 // restore SWD/SWC interface 
 MCUSR = 0x80; 
 MCUSR = MCUSR_reg; 
 
 // restore interrupt settings 
 SREG = SREG_reg; 
} 
