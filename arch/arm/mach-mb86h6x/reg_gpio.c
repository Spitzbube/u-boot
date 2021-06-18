
#define GPIO_DataOut ((volatile unsigned*)0xc3000000)
#define GPIO_DataIn ((volatile unsigned*)0xc3000200)
#define GPIO_IrqCfg ((volatile unsigned*)0xc30004c0)
#define GPIO_Config ((volatile unsigned*)0xc30004c4)
#define GPIO_IrqStatus ((volatile unsigned*)0xc3000400)
#define GPIO_Input_31_0 ((volatile unsigned*)0xc3000404)
#define GPIO_Input_63_32 ((volatile unsigned*)0xc3000408)
#define GPIO_Input_95_64 ((volatile unsigned*)0xc300040c)
#define REG9 ((volatile unsigned*)0xc30004ec)


static volatile int m_gpio_dataout[96]; //21f6aa08
static volatile int m_gpio_datain[67]; //21f6ab88
static volatile int m_gpio_irqcfg; //21f6ac94
static volatile int m_gpio_config; //21f6ac98
static volatile int Data_21f6ac9c; //21f6ac9c


/* 21c7059c - complete */
void FREG_GPIO_Init(void)
{
   unsigned i;

   for (i = 0; i < 96; i++)
   {
      m_gpio_dataout[i] = m_gpio_dataout[i];
   }

   for (i = 0; i < 96; i++)
   {
      m_gpio_dataout[i] = m_gpio_dataout[i];
   }

   for (i = 0; i < 96; i++)
   {
      m_gpio_dataout[i] = m_gpio_dataout[i];
   }

   for (i = 0; i < 96; i++)
   {
      m_gpio_dataout[i] = m_gpio_dataout[i] | (1 << 8);
   }

   for (i = 0; i < 96; i++)
   {
      m_gpio_dataout[i] = m_gpio_dataout[i];
   }

   for (i = 0; i < 67; i++)
   {
      m_gpio_datain[i] = m_gpio_datain[i];
   }

   m_gpio_irqcfg = m_gpio_irqcfg | (1 << 9);
   m_gpio_irqcfg = m_gpio_irqcfg;
   m_gpio_irqcfg = m_gpio_irqcfg;

   m_gpio_config = m_gpio_config;
   m_gpio_config = m_gpio_config;
}


/* 21c70680 - complete */
void func_21c70680(int a, int b)
{
   m_gpio_dataout[a] = b;
   (GPIO_DataOut)[a] = b;
}


/* 21c7069c - complete */
void FREG_GPIO_SetDataOut_DataEnableInv(int a, int b)
{
   int c = (m_gpio_dataout[a] & ~(1 << 18)) |
      ((b << 18) & (1 << 18));
    m_gpio_dataout[a] = c;
   (GPIO_DataOut)[a] = c;
}


/* 21c706cc - complete */
void FREG_GPIO_SetDataOut_DataOutInv(int a, int b)
{
   int c = (m_gpio_dataout[a] & ~(1 << 17)) |
      ((b << 17) & (1 << 17));
   m_gpio_dataout[a] = c;
   (GPIO_DataOut)[a] = c;
}


/* 21c706fc - complete */
void FREG_GPIO_SetDataOut_DataEnExchange(int a, int b)
{
   int c = (m_gpio_dataout[a] & ~(1 << 16)) |
      ((b << 16) & (1 << 16));
   m_gpio_dataout[a] = c;
   (GPIO_DataOut)[a] = c;
}


/* 21c7072c - complete */
void FREG_GPIO_SetDataOut_DataEnSel(int a, int b)
{
   int c = (m_gpio_dataout[a] & ~(0xf << 8)) |
      ((b << 8) & (0xf << 8));
   m_gpio_dataout[a] = c;
   (GPIO_DataOut)[a] = c;
}


/* 21c7075c - complete */
void FREG_GPIO_SetDataOut_DataOutSel(int a, int b)
{
   b = (m_gpio_dataout[a] & ~0x7f) | (b & (0x7f));
   m_gpio_dataout[a] = b;
   (GPIO_DataOut)[a] = b;
}


/* 21c70788 - complete */
void func_21c70788(int a, int b)
{
   m_gpio_datain[a] = b;
   (GPIO_DataIn)[a] = b;
}


/* 21c707a4 - complete */
void FREG_GPIO_SetDataIn_DataInSel(int a, int b)
{
   b = (b & 0x7f) | (m_gpio_datain[a] & ~0x7f);
   m_gpio_datain[a] = b;
   (GPIO_DataIn)[a] = b;
}


/* 21c707d0 - complete */
void FREG_GPIO_SetIrqCfg_IrqClear(int a)
{
   a = ((a << 9) & (1 << 9)) | (m_gpio_irqcfg & ~(1 << 9));
   m_gpio_irqcfg = a;
   *(GPIO_IrqCfg) = a;
}


/* 21c707fc - complete */
void FREG_GPIO_SetIrqCfg_IrqMode(int a)
{
   a = ((a << 8) & (1 << 8)) | (m_gpio_irqcfg & ~(1 << 8));
   m_gpio_irqcfg = a;
   *(GPIO_IrqCfg) = a;
}


/* 21c70828 - complete */
void FREG_GPIO_SetIrqCfg_IrqInput(int a)
{
   a = (a & 0x7f) | (m_gpio_irqcfg & ~0x7f);
   m_gpio_irqcfg = a;
   *(GPIO_IrqCfg) = a;
}


/* 21c70850 - complete */
void func_21c70850(int a)
{
   a = ((a << 3) & (1 << 3)) | (m_gpio_config & ~(1 << 3));
   m_gpio_config = a;
   *(GPIO_Config) = a;
}


/* 21c7087c - complete */
void FREG_GPIO_SetConfig_FlashAddrEn(int a)
{
   a = (a & 1) | (m_gpio_config & ~1);
   m_gpio_config = a;
   *(GPIO_Config) = a;
}


/* 21c708a4 - complete */
unsigned FREG_GPIO_GetIrqStatus_GpioIrq(void)
{
   return *(GPIO_IrqStatus) & 1;
}


/* 21c708b4 - complete */
unsigned FREG_GPIO_GetInput_31_0(void)
{
   return *(GPIO_Input_31_0);
}


/* 21c708c0 - complete */
unsigned FREG_GPIO_GetInput_63_32(void)
{
   return *(GPIO_Input_63_32);
}


/* 21c708cc - complete */
unsigned FREG_GPIO_GetInput_95_64(void)
{
   return *(GPIO_Input_95_64);
}


/* 21c708d8 - complete */
void func_21c708d8(int a)
{
   Data_21f6ac9c = a;
   *REG9 = a;
}


/* 21c708f0 - complete */
unsigned func_21c708f0(void)
{
   return Data_21f6ac9c;
}





