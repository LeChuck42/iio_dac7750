#ifndef __DAC_7750_H__
#define __DAC_7750_H__

#define DAC7750_WDPD_10MS		    0x0
#define DAC7750_WDPD_51MS		    0x1
#define DAC7750_WDPD_102MS		    0x2
#define DAC7750_WDPD_204MS		    0x3
                                   
#define DAC7750_SRSTEP_0_0625       0x0
#define DAC7750_SRSTEP_0_125        0x1
#define DAC7750_SRSTEP_0_25         0x2
#define DAC7750_SRSTEP_0_5          0x3
#define DAC7750_SRSTEP_1            0x4
#define DAC7750_SRSTEP_2            0x5
#define DAC7750_SRSTEP_4            0x6
#define DAC7750_SRSTEP_8            0x7
                                    
#define DAC7750_SRCLK_258065        0x0
#define DAC7750_SRCLK_200000        0x1
#define DAC7750_SRCLK_153845        0x2
#define DAC7750_SRCLK_131145        0x3
#define DAC7750_SRCLK_115940        0x4
#define DAC7750_SRCLK_69565         0x5
#define DAC7750_SRCLK_37560         0x6
#define DAC7750_SRCLK_25805         0x7
#define DAC7750_SRCLK_20150         0x8
#define DAC7750_SRCLK_16030         0x9
#define DAC7750_SRCLK_10295         0xA
#define DAC7750_SRCLK_8280          0xB
#define DAC7750_SRCLK_6900          0xC
#define DAC7750_SRCLK_5530          0xD
#define DAC7750_SRCLK_4240          0xE
#define DAC7750_SRCLK_3300          0xF
                                    
#define DAC7750_RANGE_4_20          0x5
#define DAC7750_RANGE_0_20          0x6
#define DAC7750_RANGE_0_24          0x7

#define DAC7750_ADDR(a)             ((a)<<16)
#define DAC7750_ADDR_NOP            0x00
#define DAC7750_ADDR_WR_REG         0x01
#define DAC7750_ADDR_READ           0x02
#define DAC7750_ADDR_WR_CTRL        0x55
#define DAC7750_ADDR_WR_RESET       0x56
#define DAC7750_ADDR_WR_CFG         0x57
#define DAC7750_ADDR_WR_DAC_GAIN    0x58
#define DAC7750_ADDR_WR_DAC_ZERO    0x59
#define DAC7750_ADDR_WD_TIMER       0x95 

#define DAC7750_READ_STATUS         0x00
#define DAC7750_READ_DATA           0x01
#define DAC7750_READ_CTRL           0x02
#define DAC7750_READ_CFG            0x0B
#define DAC7750_READ_DAC_GAIN       0x13
#define DAC7750_READ_DAC_ZERO       0x17

#define DAC7750_SHIFT_CTRL_REXT     13
#define DAC7750_SHIFT_CTRL_OUTEN    12
#define DAC7750_SHIFT_CTRL_SRCLK    8
#define DAC7750_SHIFT_CTRL_SRSTEP   5
#define DAC7750_SHIFT_CTRL_SREN     4
#define DAC7750_SHIFT_CTRL_DCEN     3
#define DAC7750_SHIFT_CTRL_RANGE    0

#define DAC7750_MASK_CTRL_REXT      (0x1<<DAC7750_SHIFT_CTRL_REXT)
#define DAC7750_MASK_CTRL_OUTEN     (0x1<<DAC7750_SHIFT_CTRL_OUTEN)
#define DAC7750_MASK_CTRL_SRCLK     (0xF<<DAC7750_SHIFT_CTRL_SRCLK)
#define DAC7750_MASK_CTRL_SRSTEP    (0x7<<DAC7750_SHIFT_CTRL_SRSTEP)
#define DAC7750_MASK_CTRL_SREN      (0x1<<DAC7750_SHIFT_CTRL_SREN)
#define DAC7750_MASK_CTRL_DCEN      (0x1<<DAC7750_SHIFT_CTRL_DCEN)
#define DAC7750_MASK_CTRL_RANGE     (0x7<<DAC7750_SHIFT_CTRL_RANGE)

#define DAC7750_IIO_ADDRESS(mask, shift) ((mask)<<8 | (shift))

#endif
