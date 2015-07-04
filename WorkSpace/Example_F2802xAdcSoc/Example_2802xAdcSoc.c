//#############################################################################
//
//  Controlador de carga com MPPT
//
///////////////////////////////////////////////////////////////////////////////
//
// Defini��es dos pinos
//
//	LED 1 - GPIO0
//	LED 3 - GPIO2
//	LED 4 - GPIO3
//
//	Medi��es
//	Corrente - ADCINA0
//	Tens�o - ADCINA1
//	Potenciometro - ADCINA2 (simular varia��o da corrente)
//
//	Sa�das
//	SW - PWM2B
//	Rect - PWM3A
//
//	Entradas
//	Button - GPIO12
//
///////////////////////////////////////////////////////////////////////////////
//
//	Defini��o dos fundos de escala dos AD�s
//
//	Fundo de escala da medi��o de corrente - 4096 bits - 15A (0,003662109375A/bit)
//	Corrente nominal i_limit = 2730; bits - 9,99A
//
//	Fundo de escala da tens�o - 4096 bits - 50V (0,01220703125V/bit)
//
//	Sistema 12V:
//	Tens�o flutua��o, tensao_bat_max = 1179 bits - 14,392V
//	Tens�o nominal de carga, tensao_bat_nom = 1130 bits - 13,79V
//	Tens�o m�nima, tensao_bat_min = 885 bits - 10,80V
//
//	Tens�o m�nima do pv, tensao_pv_min = 1392 bits - 16,99V
//
//	Sistema 24V:
//	Tens�o flutua��o, tensao_bat_max = 2358 bits - 28,784V
//	Tens�o nominal de carga, tensao_bat_nom = 2260 bits - 27,58V
//	Tens�o m�nima, tensao_bat_min = 1770 bits - 21,60V
//
//	Tens�o m�nima do pv, tensao_pv_min = 2784 bits - 33,98V
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

// Configura��o do registrados do PWM
#define EPWM1_TIMER_TBPRD  1000  // Per�odo do PWM
//////////////////////////////////////////////////

// Interrup��es.
__interrupt void adc_isr(void);
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
///////////////////////////////////////

// Fun��es
void Adc_init(void);
void Gpio_init(void);
void Pwm_init(void);
void Timer_init(void);
void temp_interna(void);
void verifica_painel(void);
void detecta_bat(void);
void bootstrap(void);
void rampa(void);
void pert_obs(void);
void aguarda(void);
void delay_loop(void);
//////////////////////////////////////

// Vari�veis globais
unsigned long estado = 1;
// 1 - stand_by - aguarda sem PWM at� que haja tens�o suficiente nos pain�is
// 2 - track_rampa - verredura do ciclo ativo
// 3 - track - pertubar e observa
// 4 - hold - aguarda com o mesmo duty cicle

unsigned long tensao = 0, corrente = 0, potenciometro = 0, pot = 0, count_15min = 0;
unsigned long temperatura = 0, tensao_pv_min = 0, tensao_pv = 0, count_s = 0, count_init_delay = 15;
unsigned long tensao_bat = 0, tensao_bat_max = 0, tensao_bat_nom = 0, tensao_bat_min = 0;
unsigned long i_limit = 2730;
long PWM = 0;
/////////////////////////////////////

ADC_Handle myAdc;
CLK_Handle myClk;
FLASH_Handle myFlash;
GPIO_Handle myGpio;
PIE_Handle myPie;
PWM_Handle myPwm1, myPwm2, myPwm3, myPwm4;
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
    myPwm2 = PWM_init((void *)PWM_ePWM2_BASE_ADDR, sizeof(PWM_Obj));
    myPwm3 = PWM_init((void *)PWM_ePWM3_BASE_ADDR, sizeof(PWM_Obj));
    myPwm4 = PWM_init((void *)PWM_ePWM4_BASE_ADDR, sizeof(PWM_Obj));
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

    // Inicializa��o dos timers
    Timer_init();

    // Inicializa��o dos GPIO
    Gpio_init();

    // Inicializa��o dos ADs
    Adc_init();

    // Inicializa��o do PWM
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

	CLK_enablePwmClock(myClk, PWM_Number_2);
	CLK_enablePwmClock(myClk, PWM_Number_3);
	CLK_enablePwmClock(myClk, PWM_Number_4);

/*	CLK_disablePwmClock(myClk, PWM_Number_2);
	CLK_disablePwmClock(myClk, PWM_Number_3);
	CLK_disablePwmClock(myClk, PWM_Number_4);
*/    // Detecta se o sistema � 12 ou 24V.
    detecta_bat();

    // Wait for ADC interrupt
    for(;;){}
}

/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
// Rotina que detecta se o banco de baterias est� em 12 ou 24V para carregar as constantes.
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

	GPIO_setLow(myGpio,GPIO_Number_17);	// LED - Bat

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

	// A cada 15min verifica a tens�o do painel, caso ainda esteja dentro do limite
	// realiza uma nova rampa, caso contr�rio permanece em stand_by.
	if(count_15min > 900){estado = 1;count_15min=0;}


// Acknowledge this interrupt to receive more interrupts from group 1
//   PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
    PIE_clearInt(myPie, PIE_GroupNumber_1);
}

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// Desliga o PWM e verifica se a tens�o do painel est� em n�vel adequado.
void verifica_painel()
{
	static unsigned long j = 1;
	unsigned int i = 0;

	CLK_disablePwmClock(myClk, PWM_Number_2);
	CLK_disablePwmClock(myClk, PWM_Number_3);
	delay_loop();

	for(i=0;i<10;i++){
	// Adquire a tens�o do painel
    ADC_forceConversion(myAdc, ADC_SocNumber_3);
    delay_loop();
    tensao_pv = tensao_pv + ADC_readResult(myAdc, ADC_ResultNumber_3);}
	tensao_pv = tensao_pv / 10;

    if(tensao_pv>tensao_pv_min)
    {
    	GPIO_setLow(myGpio,GPIO_Number_16);	// LED - PV
    	estado = 2;
    	PWM = 0;
    	count_init_delay = 0;
		j = 1;
    	CLK_enablePwmClock(myClk, PWM_Number_2);
    	CLK_enablePwmClock(myClk, PWM_Number_3);
    	CLK_enablePwmClock(myClk, PWM_Number_4);
    }else
    {
    	GPIO_setHigh(myGpio,GPIO_Number_16);	// LED - PV
    	count_init_delay = 15 * j;
    	j++;
    	if(count_init_delay>900){count_init_delay = 900; j = 60;}
    	estado = 1;
    }
}

__interrupt void adc_isr(void)
{
	static unsigned long PWM_SOC = 0;

	GPIO_setLow(myGpio,GPIO_Number_1);	// LED - Output: Utilizado para medir o tempo dentro da interrup��o

	// Tens�o
	tensao = ADC_readResult(myAdc, ADC_ResultNumber_1);
	// Corrente
	corrente = ADC_readResult(myAdc, ADC_ResultNumber_2);
	// Tens�o painel
	tensao_pv = ADC_readResult(myAdc, ADC_ResultNumber_3);

	pot = tensao * corrente;

	switch (estado)
	{
	case 2: bootstrap();
				   break;
	case 3: rampa();
				   break;
	case 4: aguarda();
				   break;
	case 5: pert_obs();
				   break;
	}

	if(PWM > EPWM1_TIMER_TBPRD)
	{PWM = EPWM1_TIMER_TBPRD;}
	if(PWM < 0)
	{PWM = 0;}

	PWM_SOC = PWM / 2;
	PWM_setCmpA(myPwm4, PWM_SOC);

    PWM_setCmpA(myPwm2, PWM);

    if ((estado!=2)&&(corrente>410)) // Maior que 1,5A come�a a retifica��o s�ncrona
    {PWM_setCmpA(myPwm3, PWM);}
    else{PWM_setCmpA(myPwm3, 1000);}


    // Clear ADCINT1 flag reinitialize for next SOC
    ADC_clearIntFlag(myAdc, ADC_IntNumber_1);
    // Acknowledge interrupt to PIE
    PIE_clearInt(myPie, PIE_GroupNumber_10);

	GPIO_setHigh(myGpio,GPIO_Number_1);	//LED - Output: Utilizado para medir o tempo dentro da interrup��o

    return;
}


void bootstrap()
{
	static unsigned long i = 0;

	i++;	// Delay para estabilizar o conversor ap�s fixar no ciclo ativo de m�xima pot�ncia encontrado.
	PWM_setCmpA(myPwm3, 500);
	if(i > 100)	// Aguarda 100ms - 4 * 1500 = 6000 ciclos.
	{
		estado = 3;
		PWM = 150;
		i = 0;
	}
}

void rampa()
{
	static unsigned long  PWM_max = 0, pot_max = 0, i = 0;

	i++;	// Delay para estabilizar o conversor ap�s a mudan�a do ciclo ativo durante o rastreamento.
	if(i == 300)	// Delay de aprox 1,33s. // 15000 ciclos por segundo. Possui 1000 incrementos no PWM.
	{
		i=0;
		if(pot > pot_max){PWM_max = PWM; pot_max = pot;}

		PWM = PWM + 1;

		if(PWM > (900))
		{
			estado = 4; // Hold
			PWM = PWM_max;
			PWM_max = 0;
			pot_max = 0;
			i=0;
		}

	/*	if(corrente > i_limit)
		{
			estado = 2; // Hold
			PWM = PWM_max - 50;
			PWM_max = 0;
			pot_max = 0;
			i=0;
		}*/
	}
}

void aguarda()
{
	static unsigned long i = 0;

	i++;	// Delay para estabilizar o conversor ap�s fixar no ciclo ativo de m�xima pot�ncia encontrado.
	if(i > 1500)	// Aguarda 100ms - 4 * 1500 = 6000 ciclos.
	{
		estado = 5;
		i = 0;
	}
}

void pert_obs()
{
	static unsigned long  i = 0, pot_anterior = 0, limita = 0;
	static long step_pwm = 1;

	i++;
	if(i == 75) // Delay para estabilizar o conversor ap�s a mudan�a do ciclo ativo.
	{			// Interrup��o a 60kHz/4 = 15kHz. 1 ciclo de delay ~= 66,6us
		i = 0;

		if(pot<pot_anterior)
		{step_pwm = step_pwm * (-1);}

		// Se a corrente passar do Limite, soma limita na conta abaixo.
		// Como limita � maior que step_pwm, o valor do PWM ser� decrementado enquanto a corrente estiver acima do limite.
/*		if(corrente > i_limit)
		{limita = 3;}else{limita = 0;}

		if(corrente > (i_limit + 100)){PWM = PWM - 50;}*/

		PWM = PWM + step_pwm - limita;
		if (PWM>950){PWM=950;} // Limita para carga do bootstrap

		pot_anterior = pot;
	}

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
	GPIO_toggle(myGpio,GPIO_Number_2);	// LED - Fault tougle LED a cada meio segundo
    PIE_clearInt(myPie, PIE_GroupNumber_1);
}

void delay_loop()
{
    short      i;
    for (i = 0; i < 1000; i++) {}
}

/////////////////////////////////////////////////////
// Inicializa��o dos perif�ricos

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
    // Tens�o_bat
    ADC_setSocChanNumber (myAdc, ADC_SocNumber_0, ADC_SocChanNumber_A0);    //set SOC0 channel select to ADCINA4
    ADC_setSocChanNumber (myAdc, ADC_SocNumber_1, ADC_SocChanNumber_A3);    //set SOC1 channel select to ADCINA4
    // Corrente
    ADC_setSocChanNumber (myAdc, ADC_SocNumber_2, ADC_SocChanNumber_A2);    //set SOC2 channel select to ADCINA2
    // Tensao_pv
    ADC_setSocChanNumber (myAdc, ADC_SocNumber_3, ADC_SocChanNumber_A4);    //set SOC2 channel select to ADCINA2
    //Temperatura interna
    ADC_setSocChanNumber (myAdc, ADC_SocNumber_4, ADC_SocChanNumber_A5);    //Set SOC0 channel select to ADCINA5

    ADC_setSocTrigSrc(myAdc, ADC_SocNumber_0, ADC_SocTrigSrc_EPWM4_ADCSOCA);    //set SOC0 start trigger on EPWM2A, due to round-robin SOC0 converts first then SOC1
    ADC_setSocTrigSrc(myAdc, ADC_SocNumber_1, ADC_SocTrigSrc_EPWM4_ADCSOCA);    //set SOC1 start trigger on EPWM2A, due to round-robin SOC0 converts first then SOC1
    ADC_setSocTrigSrc(myAdc, ADC_SocNumber_2, ADC_SocTrigSrc_EPWM4_ADCSOCA);    //set SOC2 start trigger on EPWM2A, due to round-robin SOC0 converts first then SOC1, then SOC2
    ADC_setSocTrigSrc(myAdc, ADC_SocNumber_3, ADC_SocTrigSrc_EPWM4_ADCSOCA);    //set SOC2 start trigger on EPWM2A, due to round-robin SOC0 converts first then SOC1, then SOC2

    ADC_setSocSampleWindow(myAdc, ADC_SocNumber_0, ADC_SocSampleWindow_7_cycles);   //set SOC0 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
    ADC_setSocSampleWindow(myAdc, ADC_SocNumber_1, ADC_SocSampleWindow_7_cycles);   //set SOC1 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
    ADC_setSocSampleWindow(myAdc, ADC_SocNumber_2, ADC_SocSampleWindow_7_cycles);   //set SOC2 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
    ADC_setSocSampleWindow(myAdc, ADC_SocNumber_3, ADC_SocSampleWindow_7_cycles);   //set SOC2 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
    ADC_setSocSampleWindow(myAdc, ADC_SocNumber_4, ADC_SocSampleWindow_37_cycles);   //Set SOC1 acquisition period to 37 ADCCLK

}

void Gpio_init(void)
{
    GPIO_setPullUp(myGpio, GPIO_Number_0, GPIO_PullUp_Enable);   // Enable pullup on GPIO0 - FAN PWR
    GPIO_setPullUp(myGpio, GPIO_Number_1, GPIO_PullUp_Disable);   // Disable pullup on GPIO1 - Output
    GPIO_setPullUp(myGpio, GPIO_Number_2, GPIO_PullUp_Enable);   // Enable pullup on GPIO2 - Led Fault
    GPIO_setPullUp(myGpio, GPIO_Number_3, GPIO_PullUp_Disable);   // Disable pullup on GPIO3 - SW
    GPIO_setPullUp(myGpio, GPIO_Number_4, GPIO_PullUp_Disable);   // Disable pullup on GPIO4 - Rect_sync
    GPIO_setPullUp(myGpio, GPIO_Number_5, GPIO_PullUp_Disable);   // Disable pullup on GPIO4 - Rect_sync
    GPIO_setPullUp(myGpio, GPIO_Number_6, GPIO_PullUp_Disable);   // Disable pullup on GPIO4 - Rect_sync
    GPIO_setPullUp(myGpio, GPIO_Number_7, GPIO_PullUp_Disable);   // Disable pullup on GPIO4 - Rect_sync
    GPIO_setPullUp(myGpio, GPIO_Number_16, GPIO_PullUp_Enable);   // Enable pullup on GPIO16 - Led PV
    GPIO_setPullUp(myGpio, GPIO_Number_17, GPIO_PullUp_Enable);   // Enable pullup on GPIO17 - Led Bat
    GPIO_setPullUp(myGpio, GPIO_Number_33, GPIO_PullUp_Enable);   // Enable pullup on GPIO33 - Led Out

    GPIO_setMode(myGpio, GPIO_Number_0, GPIO_0_Mode_GeneralPurpose);  // GPIO0 - FAN PWR
    GPIO_setMode(myGpio, GPIO_Number_1, GPIO_1_Mode_GeneralPurpose);  // GPIO1 - Output
    GPIO_setMode(myGpio, GPIO_Number_2, GPIO_2_Mode_GeneralPurpose);  // GPIO2 - Led Fault
    GPIO_setMode(myGpio, GPIO_Number_3, GPIO_3_Mode_EPWM2B);  // GPIO3 - SW
    GPIO_setMode(myGpio, GPIO_Number_4, GPIO_4_Mode_EPWM3A);  // GPIO4 - Rect_sync
    GPIO_setMode(myGpio, GPIO_Number_5, GPIO_5_Mode_EPWM3B);  // GPIO4 - Rect_sync
    GPIO_setMode(myGpio, GPIO_Number_6, GPIO_6_Mode_EPWM4A);  // GPIO4 - Rect_sync
    GPIO_setMode(myGpio, GPIO_Number_7, GPIO_7_Mode_EPWM4B);  // GPIO4 - Rect_sync
    GPIO_setMode(myGpio, GPIO_Number_16, GPIO_16_Mode_GeneralPurpose);  // GPIO16 - Led PV
    GPIO_setMode(myGpio, GPIO_Number_17, GPIO_17_Mode_GeneralPurpose);  // GPIO17 - Led Bat
    GPIO_setMode(myGpio, GPIO_Number_33, GPIO_33_Mode_GeneralPurpose);  // GPIO33 - Led Out

    GPIO_setDirection(myGpio, GPIO_Number_0, GPIO_Direction_Output);
    GPIO_setDirection(myGpio, GPIO_Number_1, GPIO_Direction_Output);
    GPIO_setDirection(myGpio, GPIO_Number_2, GPIO_Direction_Output);
    GPIO_setDirection(myGpio, GPIO_Number_3, GPIO_Direction_Output);
    GPIO_setDirection(myGpio, GPIO_Number_4, GPIO_Direction_Output);
    GPIO_setDirection(myGpio, GPIO_Number_5, GPIO_Direction_Output);
    GPIO_setDirection(myGpio, GPIO_Number_6, GPIO_Direction_Output);
    GPIO_setDirection(myGpio, GPIO_Number_7, GPIO_Direction_Output);
    GPIO_setDirection(myGpio, GPIO_Number_16, GPIO_Direction_Output);
    GPIO_setDirection(myGpio, GPIO_Number_17, GPIO_Direction_Output);
    GPIO_setDirection(myGpio, GPIO_Number_33, GPIO_Direction_Output);

	GPIO_setLow(myGpio,GPIO_Number_0);  // GPIO0 - FAN PWR
	GPIO_setLow(myGpio,GPIO_Number_1);  // GPIO1 - Output
	GPIO_setHigh(myGpio,GPIO_Number_2);  // GPIO2 - Led Fault
	GPIO_setHigh(myGpio,GPIO_Number_16);  // GPIO16 - Led PV
	GPIO_setHigh(myGpio,GPIO_Number_17);  // GPIO17 - Led Bat
	GPIO_setHigh(myGpio,GPIO_Number_33);  // GPIO33 - Led Out
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
    CLK_enablePwmClock(myClk, PWM_Number_2);

    // Setup TBCLK
    PWM_setCounterMode(myPwm2, PWM_CounterMode_Up);         // Count up
    PWM_setPeriod(myPwm2, EPWM1_TIMER_TBPRD);               // Set timer period
    PWM_disableCounterLoad(myPwm2);                         // Disable phase loading
    PWM_setPhase(myPwm2, 0x0000);                           // Phase is 0
    PWM_setCount(myPwm2, 0x0000);                           // Clear counter
    PWM_setHighSpeedClkDiv(myPwm2, PWM_HspClkDiv_by_1);     // Clock ratio to SYSCLKOUT
    PWM_setClkDiv(myPwm2, PWM_ClkDiv_by_1);

    // Setup shadow register load on ZERO
    PWM_setShadowMode_CmpA(myPwm2, PWM_ShadowMode_Shadow);
    PWM_setShadowMode_CmpB(myPwm2, PWM_ShadowMode_Shadow);
    PWM_setLoadMode_CmpA(myPwm2, PWM_LoadMode_Zero);
    PWM_setLoadMode_CmpB(myPwm2, PWM_LoadMode_Zero);

    // Set Compare values
    PWM_setCmpA(myPwm2, 0);    // Set compare A value
    PWM_setCmpB(myPwm2, 0);    // Set Compare B value

    // Set actions
    PWM_setActionQual_Zero_PwmA(myPwm2, PWM_ActionQual_Clear);            // Set PWM1A on Zero
    PWM_setActionQual_CntUp_CmpA_PwmA(myPwm2, PWM_ActionQual_Set);    // Clear PWM1A on event A, up count

    PWM_setActionQual_Zero_PwmB(myPwm2, PWM_ActionQual_Clear);            // Set PWM1B on Zero
    PWM_setActionQual_CntUp_CmpB_PwmB(myPwm2, PWM_ActionQual_Set);    // Clear PWM1B on event B, up count

    PWM_setDeadBandOutputMode(myPwm2, PWM_DeadBandOutputMode_EPWMxA_Rising_EPWMxB_Falling);
    PWM_setDeadBandPolarity(myPwm2, PWM_DeadBandPolarity_EPWMxB_Inverted);
    PWM_setDeadBandInputMode(myPwm2, PWM_DeadBandInputMode_EPWMxA_Rising_and_Falling);
    PWM_setDeadBandRisingEdgeDelay(myPwm2, 25);
    PWM_setDeadBandFallingEdgeDelay(myPwm2, 25);

//////////////////////////////////////////////////////////////
    // PWM do retificador s�ncrono

    // Enable PWM clock
    CLK_enablePwmClock(myClk, PWM_Number_3);

    // Setup TBCLK
    PWM_setCounterMode(myPwm3, PWM_CounterMode_Up);         // Count up
    PWM_setPeriod(myPwm3, EPWM1_TIMER_TBPRD);               // Set timer period
    PWM_disableCounterLoad(myPwm3);                         // Disable phase loading
    PWM_setPhase(myPwm3, 0x0000);                           // Phase is 0
    PWM_setCount(myPwm3, 0x0000);                           // Clear counter
    PWM_setHighSpeedClkDiv(myPwm3, PWM_HspClkDiv_by_1);     // Clock ratio to SYSCLKOUT
    PWM_setClkDiv(myPwm3, PWM_ClkDiv_by_1);

    // Setup shadow register load on ZERO
    PWM_setShadowMode_CmpA(myPwm3, PWM_ShadowMode_Shadow);
    PWM_setLoadMode_CmpA(myPwm3, PWM_LoadMode_Zero);
    PWM_setShadowMode_CmpB(myPwm3, PWM_ShadowMode_Shadow);
    PWM_setLoadMode_CmpB(myPwm3, PWM_LoadMode_Zero);

    // Set Compare values
    PWM_setCmpA(myPwm3, EPWM1_TIMER_TBPRD);    // Set compare A value

    // Set actions
    PWM_setActionQual_Zero_PwmA(myPwm3, PWM_ActionQual_Clear);            // Set PWM1A on Zero
    PWM_setActionQual_CntUp_CmpA_PwmA(myPwm3, PWM_ActionQual_Set);    // Clear PWM1A on event A, up count

    PWM_setActionQual_Zero_PwmB(myPwm3, PWM_ActionQual_Clear);            // Set PWM1A on Zero
    PWM_setActionQual_CntUp_CmpA_PwmB(myPwm3, PWM_ActionQual_Set);    // Clear PWM1A on event A, up count

    // Active Low complementary PWMs - setup the deadband
    PWM_setDeadBandOutputMode(myPwm3, PWM_DeadBandOutputMode_EPWMxA_Rising_EPWMxB_Falling);
    PWM_setDeadBandPolarity(myPwm3, PWM_DeadBandPolarity_EPWMxB_Inverted);
    PWM_setDeadBandInputMode(myPwm3, PWM_DeadBandInputMode_EPWMxA_Rising_and_Falling);
    PWM_setDeadBandRisingEdgeDelay(myPwm3, 25);
    PWM_setDeadBandFallingEdgeDelay(myPwm3, 25);


    //////////////////////////////////////////////////////
    // Interrup��o

    // Enable PWM clock
    CLK_enablePwmClock(myClk, PWM_Number_4);

    // Setup TBCLK
    PWM_setCounterMode(myPwm4, PWM_CounterMode_Up);         // Count up
    PWM_setPeriod(myPwm4, EPWM1_TIMER_TBPRD);               // Set timer period
    PWM_disableCounterLoad(myPwm4);                         // Disable phase loading
    PWM_setPhase(myPwm4, 0x0000);                           // Phase is 0
    PWM_setCount(myPwm4, 0x0000);                           // Clear counter
    PWM_setHighSpeedClkDiv(myPwm4, PWM_HspClkDiv_by_1);     // Clock ratio to SYSCLKOUT
    PWM_setClkDiv(myPwm4, PWM_ClkDiv_by_1);

    // Setup shadow register load on ZERO
    PWM_setShadowMode_CmpA(myPwm4, PWM_ShadowMode_Shadow);
    PWM_setShadowMode_CmpB(myPwm4, PWM_ShadowMode_Shadow);
    PWM_setLoadMode_CmpA(myPwm4, PWM_LoadMode_Zero);
    PWM_setLoadMode_CmpB(myPwm4, PWM_LoadMode_Zero);

    // Set Compare values
    PWM_setCmpA(myPwm4, 0);    // Set compare A value
    PWM_setCmpB(myPwm4, 0);    // Set Compare B value

    // Set actions
    PWM_setActionQual_Zero_PwmA(myPwm4, PWM_ActionQual_Set);            // Set PWM1A on Zero
    PWM_setActionQual_CntUp_CmpA_PwmA(myPwm4, PWM_ActionQual_Clear);    // Clear PWM1A on event A, up count


    PWM_enableSocAPulse(myPwm4);                                         // Enable SOC on A group
    PWM_setSocAPulseSrc(myPwm4, PWM_SocPulseSrc_CounterEqualCmpAIncr);   // Select SOC from from CPMA on upcount
    PWM_setSocAPeriod(myPwm4, PWM_SocPeriod_ThirdEvent);                 // Generate pulse on 1st event

    CLK_enableTbClockSync(myClk);
}