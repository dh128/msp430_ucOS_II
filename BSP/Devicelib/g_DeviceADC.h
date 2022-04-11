#ifndef G_DEVICEADC_H
#define G_DEVICEADC_H

#ifdef BAT_LITHIUM
#define Max_BatADC_Value  737      //10位AD最大1024,3.6V电池
#define Min_BatADC_Value  614
#else
#define Max_BatADC_Value  850      //10位AD最大1024
#define Min_BatADC_Value  650
#endif
extern uint32_t TempADC_Value;
extern uint32_t BatADC_Value;
extern uint8_t Flag_FanOPEN;
extern uint8_t PowerQuantity[4];
//extern uint16_t  LastPowerQuantity;
extern uint16_t SUMPowerQuantity;
//extern uint8_t  AVGPowerQuantity;

void g_Device_ADC_Init(void);
void ADCValueHandle(void);
void ADCRead1000Routine(void);

void GetADCValue(void);


#endif /* G_DEVICEADC_H */



