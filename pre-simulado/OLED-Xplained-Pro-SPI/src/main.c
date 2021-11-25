/************************************************************************/
/* Defines                                                              */
/************************************************************************/
#include <asf.h>
#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

// OLED
#define OLED_WIDTH  128
#define OLED_HEIGHT 32

// LED PLACA
#define LED_PLACA_PIO           PIOC                 // periferico que controla o LED
#define LED_PLACA_PIO_ID        12                  // ID do periférico PIOC (controla LED)
#define LED_PLACA_PIO_IDX       8                    // ID do LED no PIO
#define LED_PLACA_PIO_IDX_MASK  (1 << LED_PLACA_PIO_IDX)   // Mascara para CONTROLARMOS o LED

// LED 1
#define LED1_PIO                PIOA
#define LED1_PIO_ID             ID_PIOA
#define LED1_IDX                0
#define LED1_IDX_MASK           (1 << LED1_IDX)

// LED 2
#define LED2_PIO                PIOC
#define LED2_PIO_ID             ID_PIOC
#define LED2_IDX                30
#define LED2_IDX_MASK           (1 << LED2_IDX)

// LED 3
#define LED3_PIO                PIOB
#define LED3_PIO_ID             ID_PIOB
#define LED3_IDX                2
#define LED3_IDX_MASK           (1 << LED3_IDX)

// Botao 1
#define BUT_PIO PIOD
#define BUT_PIO_ID ID_PIOD
#define BUT_IDX 28
#define BUT_IDX_MASK (1 << BUT_IDX)
#define BUT_PIO_PIN_MASK

// Botao 1
#define BUT1_PIO PIOD
#define BUT1_PIO_ID ID_PIOD
#define BUT1_IDX 28
#define BUT1_IDX_MASK (1 << BUT1_IDX)
#define BUT1_PIO_PIN_MASK

// Botao 2
#define BUT2_PIO PIOC
#define BUT2_PIO_ID ID_PIOC
#define BUT2_IDX 31
#define BUT2_IDX_MASK (1 << BUT2_IDX)
#define BUT2_PIO_PIN_MASK

// Botao 3
#define BUT3_PIO PIOA
#define BUT3_PIO_ID ID_PIOA
#define BUT3_IDX 19
#define BUT3_IDX_MASK (1 << BUT3_IDX)
#define BUT3_PIO_PIN_MASK

#define HEIGHT        5
#define MAX_HEIGHT      5
#define MAX_WIDTH     128

typedef struct  {
	uint32_t year;
	uint32_t month;
	uint32_t day;
	uint32_t week;
	uint32_t hour;
	uint32_t minute;
	uint32_t second;
} calendar;

#define PIO_IT_EDGE (1u << 6) /*  Interrupt Edge detection is active. */

/************************************************************************/
/* variaveis globais                                                    */
/************************************************************************/
/* flags */
volatile char but_1 = 0;
volatile char but_2 = 0;
volatile char but_3 = 0;

// Tc
volatile char flag_tc1 = 0;
volatile char flag_tc4 = 0;
volatile char flag_tc7 = 0;

// Rtt
volatile char f_rtt_alarme = 0;
volatile char f_rtt_time_changed = 0;

// Rtc
volatile char flag_rtc = 0;
volatile char flag_sec = 0;

int counter_progress_bar = 0;
int pixels = 0;
uint32_t h, m, s;

/************************************************************************/
/* Funcoes                                                              */
/************************************************************************/
void progress_bar(){
	if (counter_progress_bar > 9)
	{
		gfx_mono_ssd1306_init();
		pixels = 0;
		counter_progress_bar = 0;
	}
	gfx_mono_draw_filled_circle(8+pixels, 25, 2, GFX_PIXEL_SET, GFX_WHOLE);
	pixels += 12;
	counter_progress_bar += 1;
}

void invert_led(Pio *PIO, const uint32_t MASK, volatile char but_flag){
	if (but_flag)
	{
		if (pio_get(PIO, PIO_DEFAULT, MASK))
		{
			pio_clear(PIO, MASK);
		}
		else
		{
			pio_set(PIO, MASK);
		}
	}
	else{
		pio_set(PIO, MASK);
	}
}

void pisca_led(Pio *PIO, const uint32_t MASK){
	pio_clear(PIO, MASK);
	delay_ms(2000);
	pio_set(PIO, MASK);
}
/************************************************************************/
/* handler / callbacks                                                  */
/************************************************************************/
void but_callback(void)
{
	// PINO == 0 --> Borda de descida
	but_1 = !but_1;
}

void but_callback_2(void)
{
	// PINO == 0 --> Borda de descida
	but_2 = !but_2;
}

void but_callback_3(void)
{
	// PINO == 0 --> Borda de descida
	but_3 = !but_3;
}

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
		f_rtt_time_changed = 1;
	}

	/* IRQ due to Alarm */
	if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
		f_rtt_alarme = 1;                  // flag RTT alarme
		//counter = 0;
	}
}

void RTC_Handler(void)
{
	uint32_t ul_status = rtc_get_status(RTC);

	/* seccond tick	*/
	if ((ul_status & RTC_SR_SEC) == RTC_SR_SEC) {
		flag_sec = 1;
	}
	
	/* Time or date alarm */
	if ((ul_status & RTC_SR_ALARM) == RTC_SR_ALARM) {
		flag_rtc = 1;
	}
	
	rtc_clear_status(RTC, RTC_SCCR_SECCLR);
	rtc_clear_status(RTC, RTC_SCCR_ALRCLR);
	rtc_clear_status(RTC, RTC_SCCR_ACKCLR);
	rtc_clear_status(RTC, RTC_SCCR_TIMCLR);
	rtc_clear_status(RTC, RTC_SCCR_CALCLR);
	rtc_clear_status(RTC, RTC_SCCR_TDERRCLR);
}

/************************************************************************/
/* Inits                                                                */
/************************************************************************/
void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq){
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();

	/* Configura o PMC */
	/* O TimerCounter é meio confuso
	o uC possui 3 TCs, cada TC possui 3 canais
	TC0 : ID_TC0, ID_TC1, ID_TC2
	TC1 : ID_TC3, ID_TC4, ID_TC5
	TC2 : ID_TC6, ID_TC7, ID_TC8
	*/
	pmc_enable_periph_clk(ID_TC);

	/** Configura o TC para operar em  freq hz e interrupçcão no RC compare */
	tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);

	/* Configura e ativa interrupçcão no TC canal 0 */
	/* Interrupção no C */
	NVIC_SetPriority(ID_TC, 4);
	NVIC_EnableIRQ((IRQn_Type) ID_TC);
	tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);

	/* Inicializa o canal 0 do TC */
	tc_start(TC, TC_CHANNEL);
}

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
	rtt_enable_interrupt(RTT, RTT_MR_ALMIEN );
}

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

void LED_init(int estado, Pio *p_pio ,uint32_t id, uint32_t mask ){
	pmc_enable_periph_clk(id);
	pio_set_output(p_pio, mask, estado, 0, 0);
};

//inicializa botoes
void BUT_init( Pio *p_pio ,uint32_t id, uint32_t mask, void *p_handler, uint32_t p_edge){
	// Inicializa clock do perif?rico PIO responsavel pelo botao
	pmc_enable_periph_clk(id);

	// Configura PIO para lidar com o pino do bot?o como entrada
	// com pull-up
	pio_configure(p_pio, PIO_INPUT, mask, PIO_PULLUP);
	pio_configure(p_pio, PIO_INPUT, mask, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(p_pio, mask, 60);

	// Configura interrup??o no pino referente ao botao e associa
	// fun??o de callback caso uma interrup??o for gerada
	// a fun??o de callback ? a: but_callback()
	pio_handler_set(p_pio,
	id,
	mask,
	p_edge,
	p_handler);

	// Ativa interrup??o
	pio_enable_interrupt(p_pio, mask);

	// Configura NVIC para receber interrupcoes do PIO do botao
	// com prioridade 4 (quanto mais pr?ximo de 0 maior)
	NVIC_EnableIRQ(id);
	NVIC_SetPriority(id, 4); // Prioridade 4
}

void io_init(void)
{
	board_init();
	sysclk_init();
	/* Disable the watchdog */
	WDT->WDT_MR = WDT_MR_WDDIS;
	delay_init();
	// Init OLED
	gfx_mono_ssd1306_init();
	
	/* Inicializa Leds */
	LED_init(0, LED1_PIO, LED1_PIO_ID, LED1_IDX_MASK);
	LED_init(1, LED2_PIO, LED2_PIO_ID, LED2_IDX_MASK);
	LED_init(0, LED3_PIO, LED3_PIO_ID, LED3_IDX_MASK);

	//Inicializa botoes
	BUT_init(BUT1_PIO, BUT1_PIO_ID, BUT1_IDX_MASK, but_callback, PIO_IT_RISE_EDGE);
	BUT_init(BUT2_PIO, BUT2_PIO_ID, BUT2_IDX_MASK, but_callback_2, PIO_IT_FALL_EDGE);
	BUT_init(BUT3_PIO, BUT3_PIO_ID, BUT3_IDX_MASK, but_callback_3, PIO_IT_RISE_EDGE);
}

/************************************************************************/
/* Main                                                                 */
/************************************************************************/
int main (void)
{
	io_init();
	
	// Inicializa os TCs
	TC_init(TC0, ID_TC1, 1, 5);
	TC_init(TC1, ID_TC4, 1, 10);
	TC_init(TC2, ID_TC7, 1, 1);
	
	/** Configura RTC */
	calendar rtc_initial = {2018, 3, 19, 12, 15, 45 ,1};
	RTC_init(RTC, ID_RTC, rtc_initial, RTC_IER_ALREN | RTC_IER_SECEN);
	
	f_rtt_alarme = 1;

	// Init OLED
	gfx_mono_ssd1306_init();
	char buffer [50];
	// Escreve na tela um circulo e um texto
	// 	gfx_mono_draw_string("5", 1,15, &sysfont);
	// 	gfx_mono_draw_string("10", 60,15, &sysfont);
	// 	gfx_mono_draw_string("1", 120,15, &sysfont);

	/* Insert application code here, after the board has been initialized. */
	while(1) {
		if(flag_tc1){
			invert_led(LED1_PIO, LED1_IDX_MASK, but_1);
			flag_tc1 = 0;
		}
		
		if (flag_tc4)
		{
			invert_led(LED2_PIO, LED2_IDX_MASK, but_2);
			flag_tc4 = 0;
		}
		
		if (flag_tc7)
		{
			invert_led(LED3_PIO, LED3_IDX_MASK, but_3);
			flag_tc7 = 0;
		}
		
		if (f_rtt_alarme){
			/*
			* IRQ (interrupção ocorre) apos 5s => 5 pulsos por sengundo (0,2s) -> 25 pulsos são necessários para dar 5s
			*/
			uint16_t pllPreScale = (int) (((float) 32768) / 5.0);
			uint32_t irqRTTvalue = 25;
			
			// reinicia RTT para gerar um novo IRQ
			RTT_init(pllPreScale, irqRTTvalue);
			
			f_rtt_alarme = 0;
		}
		
		if (flag_sec){
			uint32_t h, m, s;
			rtc_get_time(RTC,&h,&m,&s);
			gfx_mono_draw_string("                ", 35, 2, &sysfont);
			sprintf(buffer, "%lu:%lu:%lu", h, m, s);
			gfx_mono_draw_string(buffer, 35, 2, &sysfont);
			flag_sec= 0;
		}
		
		if (f_rtt_time_changed)
		{
			progress_bar();
		}
		
		pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
	}
}