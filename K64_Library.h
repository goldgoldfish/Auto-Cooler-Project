//Ben Wedemire
//3515624
//
//A compliation of usefull functions for the MK64FN1M0VLL12
//
//2017/2/16

#ifndef ____K64_Library__
#define ____K64_Library__

//Structure Definition
typedef struct rpm_struc{
  unsigned int rpm_fb;
  unsigned int capture_time;
} rpm_struc;

//UART0 Functions
void UART0_Init(void);
void UART0_Putchar(char t);
int UART0_Getchar(void);
void UART0_PutString(char *s);

//ADC0 Functions
void ADC0_Init(void);
int ADC0_Convert(void);

//Switch Functions
void Switch_Init(void);
void Switch_Input(void);

//LED Functions
void LED_Init(void);

//Buffer Functions
unsigned int *InitBuf(unsigned *array);
int AddBuf(unsigned int temp, unsigned int *array);
unsigned int RemoveBuf(unsigned int *array);
void check_array(unsigned int *array);

//FTM Functions
void flextimer0_init(void);
unsigned int Measure_Pulse(void);

#endif
