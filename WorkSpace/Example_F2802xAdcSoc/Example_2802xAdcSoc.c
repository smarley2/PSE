//#############################################################################
//
//  Controlador de carga com MPPT
//
///////////////////////////////////////////////////////////////////////////////
//
// Definições dos pinos
//
//	LED 1 - GPIO0
//	LED 3 - GPIO2
//	LED 4 - GPIO3
//
//	Medições
//	Corrente - ADCINA0
//	Tensão - ADCINA1
//	Potenciometro - ADCINA2 (simular variação da corrente)
//
//	Saídas
//	PWM - GPIO1
//
//	Entradas
//	Button - GPIO12
//
///////////////////////////////////////////////////////////////////////////////
//
//	Definição dos fundos de escala dos AD´s
//
//	Fundo de escala da medição de corrente - 4096 bits - 15A (0,003662109375A/bit)
//	Corrente nominal i_limit = 2730; bits - 9,99A
//
//	Fundo de escala da tensão - 4096 bits - 50V (0,01220703125V/bit)
//
//	Sistema 12V:
//	Tensão flutuação, tensao_bat_max = 1179 bits - 14,392V
//	Tensão nominal de carga, tensao_bat_nom = 1130 bits - 13,79V
//	Tensão mínima, tensao_bat_min = 885 bits - 10,80V
//
//	Tensão mínima do pv, tensao_pv_min = 1392 bits - 16,99V
//
//	Sistema 24V:
//	Tensão flutuação, tensao_bat_max = 2358 bits - 28,784V
//	Tensão nominal de carga, tensao_bat_nom = 2260 bits - 27,58V
//	Tensão mínima, tensao_bat_min = 1770 bits - 21,60V
//
//	Tensão mínima do pv, tensao_pv_min = 2784 bits - 33,98V
//
//	tensao_bat_min, tensao_bat_nom, tensao_bat_max
//	tensao_pv_min
//#############################################################################

#include "DSP28x_Project.h"
#include "f2802x_common/include/adc.h"
#include "f2802x_common/include/clk.h"
#include "f2802x_common/include/flash.h"
#include "f2802x_common/include/gpio.h"
#include "f2802x_common/include/pie.h"
#include "f2802x_common/include/pll.h"
#include "f2802x_common/include/pwm.h"
#include "f2802x_common/include/timer.h"
#include "f2802x_common/include/wdog.h"

// Configuração do registrados do PWM
#define EPWM1_TIMER_TBPRD  1000  // Período do PWM
//////////////////////////////////////////////////

// Interrupções.
__interrupt void adc_isr(void);
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
///////////////////////////////////////

// Funções
void Adc_init(void);
void Gpio_init(void);
void Pwm_init(void);
void Timer_init(void);
void temp_interna(void);
void verifica_painel(void);
void detecta_bat(void);
void rampa(void);
void pert_obs(void);
void aguarda(void);
void delay_loop(void);
//////////////////////////////////////

// Variáveis globais
unsigned long estado = 1;
// 1 - stand_by - aguarda sem PWM até que haja tensão suficiente nos painéis
// 2 - track_rampa - verredura do ciclo ativo
// 3 - track - pertubar e observa
// 4 - hold - aguarda com o mesmo duty cicle

unsigned long tensao = 0, corrente = 0, potenciometro = 0, pot = 0, j = 1, count_15min = 0;
unsigned long temperatura = 0, tensao_pv_min = 0, tensao_pv = 0, count_s = 0, count_init_delay = 15;
unsigned long tensao_bat = 0, tensao_bat_max = 0, tensao_bat_nom = 0, tensao_bat_min = 0;
unsigned long i_limit = 2730, pot_anterior = 0, PWM = 0, PWM_max = 0;
/////////////////////////////////////

ADC_Handle myAdc;
CLK_Handle myClk;
FLASH_Handle myFlash;
GPIO_Handle myGpio;
PIE_Handle myPie;
PWM_Handle myPwm1, myPwm2, myPwm3;
TIMER_Handle myTimer0, myTimer1;

void main(void)
{
    CPU_Handle myCpu;
    PLL_Handle myPll;
    WDOG_Handle myWDog;

    // Initialize all the handles needed for this application
    myAdc = ADC_init((void *)ADC_BASE_ADDR, sizeof(ADC_Obj));
    myClk = CLK_init((void *)CLK_BASE_ADDR, sizeof(CLK_Obj));
    myCpu = CPU_init((void *)NULL, sizeof(CPU_Obj));
    myFlash = FLASH_init((void *)FLASH_BASE_ADDR, sizeof(FLASH_Obj));
    myGpio = GPIO_init((void *)GPIO_BASE_ADDR, sizeof(GPIO_Obj));
    myPie = PIE_init((void *)PIE_BASE_ADDR, sizeof(PIE_Obj));
    myPll = PLL_init((void *)PLL_BASE_ADDR, sizeof(PLL_Obj));
    myPwm1 = PWM_init((void *)PWM_ePWM1_BASE_ADDR, sizeof(PWM_Obj));
    myPwm2 = PWM_init((void *)PWM_ePWM2_BASE_ADDR, sizeof(PWM_Obj));
    myWDog = WDOG_init((void *)WDOG_BASE_ADDR, sizeof(WDOG_Obj));
    myTimer0 = TIMER_init((void *)TIMER0_BASE_ADDR, sizeof(TIMER_Obj));
    myTimer1 = TIMER_init((void *)TIMER1_BASE_ADDR, sizeof(TIMER_Obj));

    // Perform basic system initialization
    WDOG_disable(myWDog);
    CLK_enableAdcClock(myClk);
    (*Device_cal)();

    // Selecioando oscilador interno
    CLK_setOscSrc(myClk, CLK_OscSrc_Internal);

    // Clock interno em 60MHz
    PLL_setup(myPll, PLL_Multiplier_12, PLL_DivideSelect_ClkIn_by_2);

    // Disable the PIE and all interrupts
    PIE_disable(myPie);
    PIE_disableAllInts(myPie);
    CPU_disableGlobalInts(myCpu);
    CPU_clearIntFlags(myCpu);

    // If running from flash copy RAM only functions to RAM
	#ifdef _FLASH
    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (size_t)&RamfuncsLoadSize);
	#endif

    // Setup a debug vector table and enable the PIE
    PIE_setDebugIntVectorTable(myPie);
    PIE_enable(myPie);

    // Register interrupt handlers in the PIE vector table
    PIE_registerPieIntHandler(myPie, PIE_GroupNumber_10, PIE_SubGroupNumber_1,(intVec_t)&adc_isr);
    PIE_registerPieIntHandler(myPie, PIE_GroupNumber_1, PIE_SubGroupNumber_7, (intVec_t)&cpu_timer0_isr);
    PIE_registerSystemIntHandler(myPie, PIE_SystemInterrupts_TINT1, (intVec_t)&cpu_timer1_isr);

    // Inicialização dos timers
    Timer_init();

    // Inicialização dos GPIO
    Gpio_init();

    // Inicialização dos ADs
    Adc_init();

    // Inicialização do PWM
    Pwm_init();

    // Enable ADCINT1 in PIE
    PIE_enableAdcInt(myPie, ADC_IntNumber_1);
    // Enable CPU Interrupt 1
    CPU_enableInt(myCpu, CPU_IntNumber_10);
    CPU_enableInt(myCpu, CPU_IntNumber_1);
    CPU_enableInt(myCpu, CPU_IntNumber_13);
    PIE_enableTimer0Int(myPie);

    // Enable Global interrupt INTM
    CPU_enableGlobalInts(myCpu);
    // Enable Global realtime interrupt DBGM
    CPU_enableDebugInt(myCpu);


	CLK_disablePwmClock(myClk, PWM_Number_1);
	CLK_disablePwmClock(myClk, PWM_Number_2);
    // Detecta se o sistema é 12 ou 24V.
    detecta_bat();

    // Wait for ADC interrupt
    for(;;){}
}

/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
// Rotina que detecta se o banco de baterias está em 12 ou 24V para carregar as constantes.
void detecta_bat(void)
{
	unsigned long tensao_bat_buff[9];
	int i=0;

	for(i=0;i<10;i++){
	ADC_forceConversion(myAdc, ADC_SocNumber_1);
	delay_loop();
	tensao_bat_buff[i] = ADC_readResult(myAdc, ADC_ResultNumber_1);
	tensao_bat = tensao_bat + tensao_bat_buff[i];}

	tensao_bat = tensao_bat / 10;

	if(tensao_bat<1500)
	{
		tensao_bat_max = 1179;
		tensao_bat_nom = 1130;
		tensao_bat_min = 885;
		tensao_pv_min = 1392;
	}else
	{
		tensao_bat_max = 2358;
		tensao_bat_nom = 2260;
		tensao_bat_min = 1770;
		tensao_pv_min = 2784;
	}

	estado = 1;
    TIMER_start(myTimer0);
    TIMER_start(myTimer1);
}

/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
// Timer de 1 segundo
__interrupt void cpu_timer0_isr(void)
{
	GPIO_toggle(myGpio,GPIO_Number_0);
	count_s++;
	count_15min++;

	temp_interna();

	if(count_s > count_init_delay)
	{
		count_s = 0;
		if(estado == 1)
		{
			verifica_painel();
		}
	}

	// A cada 15min verifica a tensão do painel, caso ainda esteja dentro do limite
	// realiza uma nova rampa, caso contrário permanece em stand_by.
	if(count_15min > 900){estado = 1;count_15min=0;}


// Acknowledge this interrupt to receive more interrupts from group 1
//   PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
    PIE_clearInt(myPie, PIE_GroupNumber_1);
}

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// Desliga o PWM e verifica se a tensão do painel está em nível adequado.
void verifica_painel()
{
	unsigned int i = 0;
	CLK_disablePwmClock(myClk, PWM_Number_1);
	CLK_disablePwmClock(myClk, PWM_Number_2);
	delay_loop();

	for(i=0;i<10;i++){
	// Adquire a tensão do painel
    ADC_forceConversion(myAdc, ADC_SocNumber_3);
    delay_loop();
    tensao_pv = tensao_pv + ADC_readResult(myAdc, ADC_ResultNumber_3);}
	tensao_pv = tensao_pv / 10;

    if(tensao_pv>tensao_pv_min)
    {
    	estado = 2;
    	pot_anterior = 0;
    	PWM = 0;
    	PWM_max = 0;
    	count_init_delay = 0;
		j = 1;
    	CLK_enablePwmClock(myClk, PWM_Number_1);
    	CLK_enablePwmClock(myClk, PWM_Number_2);
    }else
    {
    	count_init_delay = 15 * j;
    	j++;
    	if(count_init_delay>900){count_init_delay = 900; j = 60;}
    	estado = 1;
    }
}

__interrupt void adc_isr(void)
{
	unsigned long PWM_SOC = 0;

	// Tensão
	tensao = ADC_readResult(myAdc, ADC_ResultNumber_1);
	// Corrente
	corrente = ADC_readResult(myAdc, ADC_ResultNumber_2);
	// Tensão painel
	tensao_pv = ADC_readResult(myAdc, ADC_ResultNumber_3);

	pot = tensao * corrente;

	switch (estado)
	{
	case 2: rampa();
				   break;
	case 3: aguarda();
				   break;
	case 4: pert_obs();
				   break;
	}

	PWM_SOC = PWM / 2;
	PWM_setCmpA(myPwm1, PWM_SOC);
    PWM_setCmpB(myPwm1, PWM);

    if (corrente>410) // Maior que 1,5A começa a retificação síncrona
    {PWM_setCmpA(myPwm2, PWM);}
    else{PWM_setCmpA(myPwm2, 0);}


    // Clear ADCINT1 flag reinitialize for next SOC
    ADC_clearIntFlag(myAdc, ADC_IntNumber_1);
    // Acknowledge interrupt to PIE
    PIE_clearInt(myPie, PIE_GroupNumber_10);
    return;
}


void rampa()
{
	if(pot>pot_anterior){PWM_max = PWM;}

	PWM = PWM + 1;

	pot_anterior = pot;

	if(PWM > EPWM1_TIMER_TBPRD)
	{
		estado = 3; // Hold
		PWM = PWM_max;
	}

/*	if(corrente > i_limit)
	{
		estado = 2; // Hold
		PWM = PWM_max - 50;
	}*/
}

void aguarda()
{
	delay_loop();
	delay_loop();
	delay_loop();
	delay_loop();
	delay_loop();
	delay_loop();
	delay_loop();
	estado = 4;
}

void pert_obs()
{

}





















void temp_interna(void)
{
	long temp_sensor = 0;
    ADC_forceConversion(myAdc, ADC_SocNumber_4);
    temp_sensor = ADC_readResult(myAdc, ADC_ResultNumber_4);

    temperatura = ADC_getTemperatureC(myAdc, temp_sensor);
    return;
}

__interrupt void cpu_timer1_isr(void)
{
	GPIO_toggle(myGpio,GPIO_Number_3);	// tougle LED a cada meio segundo
    PIE_clearInt(myPie, PIE_GroupNumber_1);
}

void delay_loop()
{
    short      i;
    for (i = 0; i < 1000; i++) {}
}

/////////////////////////////////////////////////////
// Inicialização dos periféricos

void Adc_init(void)
{
    ADC_enableBandGap(myAdc);
    ADC_enableRefBuffers(myAdc);
    ADC_powerUp(myAdc);
    ADC_enable(myAdc);
    ADC_setVoltRefSrc(myAdc, ADC_VoltageRefSrc_Int);
    ADC_enableTempSensor(myAdc);                                            //Connect channel A5 internally to the temperature sensor

    // Configure ADC
    //Note: Channel ADCINA4  will be double sampled to workaround the ADC 1st sample issue for rev0 silicon errata
    ADC_setIntPulseGenMode(myAdc, ADC_IntPulseGenMode_Prior);               //ADCINT1 trips after AdcResults latch
    ADC_enableInt(myAdc, ADC_IntNumber_1);                                  //Enabled ADCINT1
    ADC_setIntMode(myAdc, ADC_IntNumber_1, ADC_IntMode_ClearFlag);          //Disable ADCINT1 Continuous mode
    ADC_setIntSrc(myAdc, ADC_IntNumber_1, ADC_IntSrc_EOC2);                 //setup EOC2 to trigger ADCINT1 to fire
    // Tensão
    ADC_setSocChanNumber (myAdc, ADC_SocNumber_0, ADC_SocChanNumber_A0);    //set SOC0 channel select to ADCINA4
    ADC_setSocChanNumber (myAdc, ADC_SocNumber_1, ADC_SocChanNumber_A0);    //set SOC1 channel select to ADCINA4
    // Corrente
    ADC_setSocChanNumber (myAdc, ADC_SocNumber_2, ADC_SocChanNumber_A1);    //set SOC2 channel select to ADCINA2
    // Tensao_pv
    ADC_setSocChanNumber (myAdc, ADC_SocNumber_3, ADC_SocChanNumber_A2);    //set SOC2 channel select to ADCINA2
    //Temperatura interna
    ADC_setSocChanNumber (myAdc, ADC_SocNumber_4, ADC_SocChanNumber_A5);    //Set SOC0 channel select to ADCINA5

    ADC_setSocTrigSrc(myAdc, ADC_SocNumber_0, ADC_SocTrigSrc_EPWM1_ADCSOCA);    //set SOC0 start trigger on EPWM1A, due to round-robin SOC0 converts first then SOC1
    ADC_setSocTrigSrc(myAdc, ADC_SocNumber_1, ADC_SocTrigSrc_EPWM1_ADCSOCA);    //set SOC1 start trigger on EPWM1A, due to round-robin SOC0 converts first then SOC1
    ADC_setSocTrigSrc(myAdc, ADC_SocNumber_2, ADC_SocTrigSrc_EPWM1_ADCSOCA);    //set SOC2 start trigger on EPWM1A, due to round-robin SOC0 converts first then SOC1, then SOC2
    ADC_setSocTrigSrc(myAdc, ADC_SocNumber_3, ADC_SocTrigSrc_EPWM1_ADCSOCA);    //set SOC2 start trigger on EPWM1A, due to round-robin SOC0 converts first then SOC1, then SOC2

    ADC_setSocSampleWindow(myAdc, ADC_SocNumber_0, ADC_SocSampleWindow_7_cycles);   //set SOC0 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
    ADC_setSocSampleWindow(myAdc, ADC_SocNumber_1, ADC_SocSampleWindow_7_cycles);   //set SOC1 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
    ADC_setSocSampleWindow(myAdc, ADC_SocNumber_2, ADC_SocSampleWindow_7_cycles);   //set SOC2 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
    ADC_setSocSampleWindow(myAdc, ADC_SocNumber_3, ADC_SocSampleWindow_7_cycles);   //set SOC2 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
    ADC_setSocSampleWindow(myAdc, ADC_SocNumber_4, ADC_SocSampleWindow_37_cycles);   //Set SOC1 acquisition period to 37 ADCCLK

}

void Gpio_init(void)
{
    GPIO_setPullUp(myGpio, GPIO_Number_0, GPIO_PullUp_Enable);   // Enable pullup on GPIO0 - LED
    GPIO_setPullUp(myGpio, GPIO_Number_1, GPIO_PullUp_Disable);   // Disable pullup on GPIO1 - Chave principal
    GPIO_setPullUp(myGpio, GPIO_Number_2, GPIO_PullUp_Disable);   // Disable pullup on GPIO2 - Ret syn
    GPIO_setPullUp(myGpio, GPIO_Number_3, GPIO_PullUp_Enable);   // Enable pullup on GPIO3 - LED
    GPIO_setPullUp(myGpio, GPIO_Number_12, GPIO_PullUp_Enable);   // Enable pullup on GPIO4 - Button

    GPIO_setMode(myGpio, GPIO_Number_0, GPIO_0_Mode_GeneralPurpose);  // GPIO0 = LED
    GPIO_setMode(myGpio, GPIO_Number_1, GPIO_1_Mode_EPWM1B);  // GPIO1 = PWM1B - Chave principal
    GPIO_setMode(myGpio, GPIO_Number_2, GPIO_2_Mode_EPWM2A);  // GPIO2 = PWM2A - Retificador síncrono
    GPIO_setMode(myGpio, GPIO_Number_3, GPIO_0_Mode_GeneralPurpose);  // GPIO3 = LED

    GPIO_setDirection(myGpio, GPIO_Number_0, GPIO_Direction_Output);
    GPIO_setDirection(myGpio, GPIO_Number_1, GPIO_Direction_Output);
    GPIO_setDirection(myGpio, GPIO_Number_2, GPIO_Direction_Output);
    GPIO_setDirection(myGpio, GPIO_Number_3, GPIO_Direction_Output);

	GPIO_setLow(myGpio,GPIO_Number_0);
	GPIO_setLow(myGpio,GPIO_Number_3);
}

void Timer_init()
{
    TIMER_stop(myTimer0);
    TIMER_stop(myTimer1);

    // Configure CPU-Timer 0 to interrupt every second:
    // 60MHz CPU Freq, 1 second Period (in uSeconds)
    TIMER_setPeriod(myTimer0, 60 * 1000000);
    TIMER_setPeriod(myTimer1, 30 * 1000000);

    TIMER_setPreScaler(myTimer0, 0);
    TIMER_reload(myTimer0);
    TIMER_setEmulationMode(myTimer0, TIMER_EmulationMode_StopAfterNextDecrement);
    TIMER_enableInt(myTimer0);

    TIMER_setPreScaler(myTimer1, 0);
    TIMER_reload(myTimer1);
    TIMER_setEmulationMode(myTimer1, TIMER_EmulationMode_StopAfterNextDecrement);
    TIMER_enableInt(myTimer1);
}

void Pwm_init()
{
//////////////////////////////////////////////////////
	// PWM da chave principal


	// Enable PWM clock
    CLK_enablePwmClock(myClk, PWM_Number_1);

    // Setup TBCLK
    PWM_setCounterMode(myPwm1, PWM_CounterMode_Up);         // Count up
    PWM_setPeriod(myPwm1, EPWM1_TIMER_TBPRD);               // Set timer period
    PWM_disableCounterLoad(myPwm1);                         // Disable phase loading
    PWM_setPhase(myPwm1, 0x0000);                           // Phase is 0
    PWM_setCount(myPwm1, 0x0000);                           // Clear counter
    PWM_setHighSpeedClkDiv(myPwm1, PWM_HspClkDiv_by_2);     // Clock ratio to SYSCLKOUT
    PWM_setClkDiv(myPwm1, PWM_ClkDiv_by_2);

    // Setup shadow register load on ZERO
    PWM_setShadowMode_CmpA(myPwm1, PWM_ShadowMode_Shadow);
    PWM_setShadowMode_CmpB(myPwm1, PWM_ShadowMode_Shadow);
    PWM_setLoadMode_CmpA(myPwm1, PWM_LoadMode_Zero);
    PWM_setLoadMode_CmpB(myPwm1, PWM_LoadMode_Zero);

    // Set Compare values
    PWM_setCmpA(myPwm1, 0);    // Set compare A value
    PWM_setCmpB(myPwm1, 0);    // Set Compare B value

    // Set actions
    PWM_setActionQual_Zero_PwmA(myPwm1, PWM_ActionQual_Set);            // Set PWM1A on Zero
    PWM_setActionQual_CntUp_CmpA_PwmA(myPwm1, PWM_ActionQual_Clear);    // Clear PWM1A on event A, up count

    PWM_setActionQual_Zero_PwmB(myPwm1, PWM_ActionQual_Set);            // Set PWM1B on Zero
    PWM_setActionQual_CntUp_CmpB_PwmB(myPwm1, PWM_ActionQual_Clear);    // Clear PWM1B on event B, up count

    PWM_enableSocAPulse(myPwm1);                                         // Enable SOC on A group
    PWM_setSocAPulseSrc(myPwm1, PWM_SocPulseSrc_CounterEqualCmpAIncr);   // Select SOC from from CPMA on upcount
    PWM_setSocAPeriod(myPwm1, PWM_SocPeriod_ThirdEvent);                 // Generate pulse on 1st event

//////////////////////////////////////////////////////////////
    // PWM do retificador síncrono

    // Enable PWM clock
    CLK_enablePwmClock(myClk, PWM_Number_2);

    // Setup TBCLK
    PWM_setCounterMode(myPwm2, PWM_CounterMode_Up);         // Count up
    PWM_setPeriod(myPwm2, EPWM1_TIMER_TBPRD);               // Set timer period
    PWM_disableCounterLoad(myPwm2);                         // Disable phase loading
    PWM_setPhase(myPwm2, 0x0000);                           // Phase is 0
    PWM_setCount(myPwm2, 0x0000);                           // Clear counter
    PWM_setHighSpeedClkDiv(myPwm2, PWM_HspClkDiv_by_2);     // Clock ratio to SYSCLKOUT
    PWM_setClkDiv(myPwm2, PWM_ClkDiv_by_2);

    // Setup shadow register load on ZERO
    PWM_setShadowMode_CmpA(myPwm2, PWM_ShadowMode_Shadow);
    PWM_setLoadMode_CmpA(myPwm2, PWM_LoadMode_Zero);

    // Set Compare values
    PWM_setCmpA(myPwm2, 0);    // Set compare A value

    // Set actions
    PWM_setActionQual_Zero_PwmA(myPwm2, PWM_ActionQual_Clear);            // Set PWM1A on Zero
    PWM_setActionQual_CntUp_CmpA_PwmA(myPwm2, PWM_ActionQual_Set);    // Clear PWM1A on event A, up count

    // Active Low complementary PWMs - setup the deadband
    PWM_setDeadBandOutputMode(myPwm2, PWM_DeadBandOutputMode_EPWMxA_Rising_EPWMxB_Falling);
    PWM_setDeadBandPolarity(myPwm2, PWM_DeadBandPolarity_EPWMxA_Inverted_EPWMxB_Inverted);
    PWM_setDeadBandInputMode(myPwm2, PWM_DeadBandInputMode_EPWMxA_Rising_and_Falling);
    PWM_setDeadBandRisingEdgeDelay(myPwm2, 50);
    PWM_setDeadBandFallingEdgeDelay(myPwm2, 50);


    CLK_enableTbClockSync(myClk);
}
