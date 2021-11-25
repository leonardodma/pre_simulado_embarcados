
/************************************************************************/
/* INCLUDES                                                            */
/************************************************************************/
#include <asf.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "conf_board.h"
#include "conf_uart_serial.h"
#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"


/***************************************************************************
 * DEFINES ÚTEIS
***************************************************************************/

//  DEFINICOES PLACA OLED
#define MAX_WIDTH 128
#define HEIGHT 12

//AFEC - Potenciometro
#define AFEC_POT AFEC0
#define AFEC_POT_ID ID_AFEC0
#define AFEC_POT_CHANNEL 0 // Canal do pino PD30

// LED PLACA
#define LED_PIO       PIOC
#define LED_PIO_ID    ID_PIOC
#define LED_IDX       8u
#define LED_IDX_MASK  (1u << LED_IDX)

// LED 1
#define LED1_PIO      PIOA
#define LED1_PIO_ID   ID_PIOA
#define LED1_IDX      0
#define LED1_IDX_MASK (1 << LED1_IDX)

//LED 2
#define LED2_PIO      PIOC
#define LED2_PIO_ID   ID_PIOC
#define LED2_IDX      30
#define LED2_IDX_MASK (1 << LED2_IDX)

//LED 3
#define LED3_PIO      PIOB
#define LED3_PIO_ID   ID_PIOB
#define LED3_IDX      2
#define LED3_IDX_MASK (1 << LED3_IDX)

// BUT 1
#define BUT1_PIO            PIOD
#define BUT1_PIO_ID         ID_PIOD
#define BUT1_PIO_IDX        28
#define BUT1_PIO_IDX_MASK   (1u << BUT1_PIO_IDX)

// BUT 2
#define BUT2_PIO            PIOC
#define BUT2_PIO_ID         ID_PIOC
#define BUT2_PIO_IDX        31
#define BUT2_PIO_IDX_MASK   (1u << BUT1_PIO_IDX)

// BUT 3
#define BUT3_PIO            PIOA
#define BUT3_PIO_ID         ID_PIOA
#define BUT3_PIO_IDX        19
#define BUT3_PIO_IDX_MASK   (1u << BUT3_PIO_IDX)

// TASK 
#define TASK_EX1_STACK_SIZE               (1024/sizeof(portSTACK_TYPE))
#define TASK_EX1_STACK_PRIORITY           (tskIDLE_PRIORITY)


/***************************************************************************
 * GLOBAIS ÚTEIS
***************************************************************************/

// SEMAFORO
SemaphoreHandle_t xSemaphore;

// FILA
QueueHandle_t xQueueADC;

// RTC
typedef struct  {
  uint32_t year;
  uint32_t month;
  uint32_t day;
  uint32_t week;
  uint32_t hour;
  uint32_t minute;
  uint32_t second;
} calendar;

int h,m,s;
int yy,mm,dd;

// TC
volatile char flag_tc1 = 0;
volatile char flag_tc3 = 0;
volatile char flag_tc6 = 0;

// BUFFER PARA ESCRITA
char buffer[32];

typedef struct {
  int value;
} adcData;

/***************************************************************************
 * FUNCOES ÚTEIS
***************************************************************************/

void progress_bar(int max_value, int value){
	float percentage = (float)value/max_value * 1.0;
	int width = (int)(MAX_WIDTH*percentage);
	gfx_mono_draw_filled_rect(width,20,MAX_WIDTH,HEIGHT,GFX_PIXEL_CLR);
	gfx_mono_draw_filled_rect(0,20,width,HEIGHT,GFX_PIXEL_SET);
}

void pisca_led(int n, int t, Pio *  p_pio , uint32_t mask){
  for (int i=0;i<n;i++){
    pio_clear(p_pio, mask);
    delay_ms(t);
    pio_set(p_pio, mask);
    delay_ms(t);
  }
}

void pin_toggle(Pio *pio, uint32_t mask){
	if(pio_get_output_data_status(pio, mask))
	pio_clear(pio, mask);
	else
	pio_set(pio,mask);
}

// RTT
uint ul_previous_time = rtt_read_timer_value(RTT);

//RTC
rtc_get_time(RTC,&h,&m,&s);
rtc_set_date_alarm(RTC, 1, rtc_initial.month, 1, rtc_initial.day);
rtc_set_time_alarm(RTC, 1, h, 1, m, 1, s + 10);

// TC - PWM

//Ex: 20% ligado, 80% desligado
void TC1_Handler(void){
	volatile uint32_t ul_dummy;
	ul_dummy = tc_get_status(TC0, 1);
	UNUSED(ul_dummy);

    if (estado_led1 == 1) {
        estado_led1 = 0;
        pio_set(LED_PIO, LED_IDX_MASK);
        TC_init(TC0, ID_TC1, 1, 250);
    } else {
        estado_led1 = 1;
        pio_clear(LED_PIO, LED_IDX_MASK);
        TC_init(TC0, ID_TC1, 1, 1000);
    }
}

// 20 %
estado_led1 = 1;
pio_clear(LED_PIO, LED_IDX_MASK);
TC_init(TC0, ID_TC1, 1, 1000);

/***************************************************************************
 * INITS 
***************************************************************************/

void LED_init(int estado, Pio *p_pio ,uint32_t id, uint32_t mask ){
	pmc_enable_periph_clk(id);
	pio_set_output(p_pio, mask, estado, 0, 0);
};

void BUT_init( Pio *p_pio ,uint32_t id, uint32_t mask, void *p_handler){
	pmc_enable_periph_clk(id);

	pio_configure(p_pio, PIO_INPUT, mask, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(p_pio, mask, 60);
	
	pio_handler_set(p_pio,
	id,
	mask,
	PIO_IT_FALL_EDGE,
	p_handler);

	pio_enable_interrupt(p_pio, mask);
	NVIC_EnableIRQ(id);
	NVIC_SetPriority(id, 4); // Prioridade 4
}

void PIN_init( Pio *p_pio ,uint32_t id, uint32_t mask, void *p_handler){
	pmc_enable_periph_clk(id);

	pio_configure(p_pio, PIO_INPUT, mask, PIO_PULLUP);
	
	pio_handler_set(p_pio,
	id,
	mask,
	PIO_IT_FALL_EDGE,
	p_handler);

	pio_enable_interrupt(p_pio, mask);
	NVIC_EnableIRQ(id);
	NVIC_SetPriority(id, 4); // Prioridade 4
}

/*
Inicializa os TCs
TC_init(TC0, ID_TC1, 1, 5);
TC_init(TC1, ID_TC4, 1, 10);
TC_init(TC2, ID_TC7, 1, 1);
*/
void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq){
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();

	/* Configura o PMC */
	/* O TimerCounter � meio confuso
	o uC possui 3 TCs, cada TC possui 3 canais
	TC0 : ID_TC0, ID_TC1, ID_TC2
	TC1 : ID_TC3, ID_TC4, ID_TC5
	TC2 : ID_TC6, ID_TC7, ID_TC8
	*/
	pmc_enable_periph_clk(ID_TC);

	/** Configura o TC para operar em  freq hz e interrup�c�o no RC compare */
	tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);

	/* Configura e ativa interrup�c�o no TC canal 0 */
	/* Interrup��o no C */
    NVIC_SetPriority(ID_TC, 4);
	NVIC_EnableIRQ((IRQn_Type) ID_TC);
	tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);

	/* Inicializa o canal 0 do TC */
	tc_start(TC, TC_CHANNEL);
}


/*
INICIALIZA RTC
RTC_init(RTC, ID_RTC, rtc_initial, RTC_IER_ALREN | RTC_IER_SECEN);
*/
void RTC_init(Rtc *rtc, uint32_t id_rtc, calendar t, uint32_t irq_type){
	/* Configura o PMC */
	pmc_enable_periph_clk(ID_RTC);

	/* Default RTC configuration, 24-hour mode */
	rtc_set_hour_mode(rtc, 0);

	/* Configura data e hora manualmente */
	rtc_set_date(rtc, t.year, t.month, t.day, t.week);
	rtc_set_time(rtc, t.hour, t.minute, t.second);

	/* Configure RTC interrupts */
	NVIC_DisableIRQ(id_rtc);
	NVIC_ClearPendingIRQ(id_rtc);
	NVIC_SetPriority(id_rtc, 4);
	NVIC_EnableIRQ(id_rtc);

	/* Ativa interrupcao via alarme */
	rtc_enable_interrupt(rtc,  irq_type);
}

/*
INICIALIZA RTT
uint16_t pllPreScale = (int) (((float) 32768) / 5.0);
uint32_t irqRTTvalue = 25;
RTT_init(pllPreScale, irqRTTvalue);
*/
static void RTT_init(uint16_t pllPreScale, uint32_t IrqNPulses)
{
	uint32_t ul_previous_time;

	/* Configure RTT for a 1 second tick interrupt */
	rtt_sel_source(RTT, false);
	rtt_init(RTT, pllPreScale);
	
	ul_previous_time = rtt_read_timer_value(RTT);
	while (ul_previous_time == rtt_read_timer_value(RTT));
	
	rtt_write_alarm_time(RTT, IrqNPulses+ul_previous_time);

	/* Enable RTT interrupt */
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 4);
	NVIC_EnableIRQ(RTT_IRQn);
	rtt_enable_interrupt(RTT, RTT_MR_ALMIEN | RTT_MR_RTTINCIEN);
}

/***************************************************************************
 * HANDLERS / CALLBACKS 
***************************************************************************/

void TC1_Handler(void){
	volatile uint32_t ul_dummy;

	// Devemos indicar ao TC que a interrupção foi satisfeita.
	ul_dummy = tc_get_status(TC0, 1);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	/** Muda o estado do LED */
	flag_tc1 = 1;
}

void TC4_Handler(void){
	volatile uint32_t ul_dummy;

	// Devemos indicar ao TC que a interrupção foi satisfeita.
	ul_dummy = tc_get_status(TC1, 1);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	/** Muda o estado do LED */
	flag_tc4 = 1;
}

void TC7_Handler(void){
	volatile uint32_t ul_dummy;

	// Devemos indicar ao TC que a interrupção foi satisfeita.
	ul_dummy = tc_get_status(TC2, 1);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	/** Muda o estado do LED */
	flag_tc7 = 1;
}



void RTT_Handler(void)
{
	uint32_t ul_status;

	/* Get RTT status - ACK */
	ul_status = rtt_get_status(RTT);

	/* IRQ due to Time has changed */
	if ((ul_status & RTT_SR_RTTINC) == RTT_SR_RTTINC) {
	
	}

	/* IRQ due to Alarm */
	if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
		f_rtt_alarme = 1;                  // flag RTT alarme
	}
}


void RTC_Handler(void)
{
	uint32_t ul_status = rtc_get_status(RTC);

	/* seccond tick	*/
	if ((ul_status & RTC_SR_SEC) == RTC_SR_SEC) {
		
	}
	
	/* Time or date alarm */
	if ((ul_status & RTC_SR_ALARM) == RTC_SR_ALARM) {
		
	}
	
	rtc_clear_status(RTC, RTC_SCCR_SECCLR);
	rtc_clear_status(RTC, RTC_SCCR_ALRCLR);
	rtc_clear_status(RTC, RTC_SCCR_ACKCLR);
	rtc_clear_status(RTC, RTC_SCCR_TIMCLR);
	rtc_clear_status(RTC, RTC_SCCR_CALCLR);
	rtc_clear_status(RTC, RTC_SCCR_TDERRCLR);
}

static void AFEC_pot_Callback(void){
  g_ul_value = afec_channel_get_value(AFEC_POT, AFEC_POT_CHANNEL);
  printf("\n but_callback \n");
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  //libera o semaforo - faz a ponte entre IRQ do botao e semaforo do RTOS
  xSemaphoreGiveFromISR(xSemaphoreADC, &xHigherPriorityTaskWoken);
  
  adcData adc;
  adc.value = g_ul_value;
  xQueueSendFromISR(xQueueADC, &adc, 0);
}