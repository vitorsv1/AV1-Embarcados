/*
 * main.c
 *
 * Created: 05/03/2019 18:00:58
 *  Author: eduardo
 */ 

#include <asf.h>
#include "tfont.h"
#include "sourcecodepro_28.h"
#include "calibri_36.h"
#include "arial_72.h"
#include "math.h"


#define YEAR        2018
#define MOUNTH      3
#define DAY         19
#define WEEK        12
#define HOUR        15
#define MINUTE      45
#define SECOND      0



struct ili9488_opt_t g_ili9488_display_opt;

//BOTAO
//#define BUT_PIO_ID			  ID_PIOA
//#define BUT_PIO				  PIOA
//#define BUT_IDX				  11
//#define BUT_IDX_MASK		  (1 << BUT_IDX)
//#define BUT_DEBOUNCING_VALUE  79

// Bot?o1
#define BUT_PIO      PIOD
#define BUT_PIO_ID   ID_PIOD
#define BUT_IDX	  28
#define BUT_IDX_MASK (1 << BUT_IDX)



// VARIAVEIS GLOBAIS
volatile int t = 0;
volatile int dis = 0;
volatile int vel = 0;
volatile int but_flag = 0;
volatile Bool f_rtt_alarme = false;
volatile uint8_t flag_RTC   = 0;

void button_callback(void){
	but_flag = 1;
	
}

void RTC_Handler(void)
{
	uint32_t ul_status = rtc_get_status(RTC);
	
	
	/*
	*  Verifica por qual motivo entrou
	*  na interrupcao, se foi por segundo
	*  ou Alarm
	*/
	if ((ul_status & RTC_SR_SEC) == RTC_SR_SEC) {
		rtc_clear_status(RTC, RTC_SCCR_SECCLR);
	}
	
	/* Time or date alarm */
	if ((ul_status & RTC_SR_ALARM) == RTC_SR_ALARM) {
			rtc_clear_status(RTC, RTC_SCCR_ALRCLR);
			flag_RTC = 1;
			
	}
	
	rtc_clear_status(RTC, RTC_SCCR_ACKCLR);
	rtc_clear_status(RTC, RTC_SCCR_TIMCLR);
	rtc_clear_status(RTC, RTC_SCCR_CALCLR);
	rtc_clear_status(RTC, RTC_SCCR_TDERRCLR);
	
}


void RTT_Handler(void)
{
	uint32_t ul_status;

	/* Get RTT status */
	ul_status = rtt_get_status(RTT);

	/* IRQ due to Time has changed */
	if ((ul_status & RTT_SR_RTTINC) == RTT_SR_RTTINC) {  }

	/* IRQ due to Alarm */
	if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
		//pin_toggle(LED_PIO, LED_IDX_MASK);    // BLINK Led
		f_rtt_alarme = true;                  // flag RTT alarme
	}
}

//FUNÇÕES

void pin_toggle(Pio *pio, uint32_t mask){
	if(pio_get_output_data_status(pio, mask))
	pio_clear(pio, mask);
	else
	pio_set(pio,mask);
}


static float get_time_rtt(){
	uint ul_previous_time = rtt_read_timer_value(RTT);
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
	NVIC_SetPriority(RTT_IRQn, 0);
	NVIC_EnableIRQ(RTT_IRQn);
	rtt_enable_interrupt(RTT, RTT_MR_ALMIEN);
}


void configure_lcd(void){
	/* Initialize display parameter */
	g_ili9488_display_opt.ul_width = ILI9488_LCD_WIDTH;
	g_ili9488_display_opt.ul_height = ILI9488_LCD_HEIGHT;
	g_ili9488_display_opt.foreground_color = COLOR_CONVERT(COLOR_WHITE);
	g_ili9488_display_opt.background_color = COLOR_CONVERT(COLOR_WHITE);

	/* Initialize LCD */
	ili9488_init(&g_ili9488_display_opt);
	ili9488_draw_filled_rectangle(0, 0, ILI9488_LCD_WIDTH-1, ILI9488_LCD_HEIGHT-1);
	
}

void font_draw_text(tFont *font, const char *text, int x, int y, int spacing) {
	char *p = text;
	while(*p != NULL) {
		char letter = *p;
		int letter_offset = letter - font->start_char;
		if(letter <= font->end_char) {
			tChar *current_char = font->chars + letter_offset;
			ili9488_draw_pixmap(x, y, current_char->image->width, current_char->image->height, current_char->image->data);
			x += current_char->image->width + spacing;
		}
		p++;
	}	
}

void RTC_init(){
	/* Configura o PMC */
	pmc_enable_periph_clk(ID_RTC);

	/* Default RTC configuration, 24-hour mode */
	rtc_set_hour_mode(RTC, 0);

	/* Configura data e hora manualmente */
	rtc_set_date(RTC, YEAR, MOUNTH, DAY, WEEK);
	rtc_set_time(RTC, HOUR, MINUTE, SECOND);

	/* Configure RTC interrupts */
	NVIC_DisableIRQ(RTC_IRQn);
	NVIC_ClearPendingIRQ(RTC_IRQn);
	NVIC_SetPriority(RTC_IRQn, 0);
	NVIC_EnableIRQ(RTC_IRQn);

	/* Ativa interrupcao via alarme */
	rtc_enable_interrupt(RTC,  RTC_IER_ALREN);

}

void io_init(void){
	// Inicializa clock do periferico
	pmc_enable_periph_clk(BUT_PIO_ID);
		
	//Configura o PIO para lidar com pino do botão como entrada
	pio_configure(BUT_PIO, PIO_INPUT, BUT_IDX_MASK, PIO_PULLUP);
		
	//Configura interrupção o pino referente ao botão e faz o callback com a função dele
	pio_handler_set(BUT_PIO,
	BUT_PIO_ID,
	BUT_IDX_MASK,
	PIO_IT_RISE_EDGE,
	button_callback);
		
	// Ativa interrupção
	pio_enable_interrupt(BUT_PIO, BUT_IDX_MASK);
		
	// Configura NVIC para receber interrupcoes do PIO do botao
	// com prioridade 4 (quanto mais próximo de 0 maior)
	NVIC_EnableIRQ(BUT_PIO_ID);
	NVIC_SetPriority(BUT_PIO_ID, 4); // Prioridade 4
}

int main(void) {
	board_init();
	sysclk_init();
	
	// Desativa watchdog
	WDT->WDT_MR = WDT_MR_WDDIS;
	
	io_init();	
	configure_lcd();
	
	// Inicializa RTT com IRQ no alarme.
	f_rtt_alarme = true;
	
	
	char tempo[32];
	char distancia[32];
	char velocidade[32];
	dis = 0;
	int disNova = 0;
	vel = 0;
	int dis_ = 0;
	int temp_ = 0;
	int vel_ = 0;
	
	
	rtc_set_date_alarm(RTC, 1, MOUNTH, 1, DAY);
	
	
	font_draw_text(&calibri_36, "CicloComputer", 15, 15, 1);
	font_draw_text(&calibri_36, "Velocidade", 15, 60, 1);
	//font_draw_text(&arial_72, "", 15, 90, 2);
	font_draw_text(&calibri_36, "Distancia", 15, 170, 1);
	//font_draw_text(&arial_72,tempo, 15, 205, 2);
	font_draw_text(&calibri_36, "Tempo", 15, 290, 1);
	//font_draw_text(&arial_72, "000", 15, 325, 2);
	while(1) {
		pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
		if(but_flag){
			
			dis++;
			dis_ = (int) 2*M_PI*(0.650/2)*dis;
			disNova++;
			but_flag = 0;
		}
		
		
		
		if (f_rtt_alarme){
			
		
			uint16_t pllPreScale = (int) (((float) 32768) / 2.0);
			uint32_t irqRTTvalue  = 4;
			temp_ += 2;
			vel_ = (int) 2*M_PI*(disNova/temp_)*(0.650/2)*3.6; 
			// reinicia RTT para gerar um novo IRQ
			RTT_init(pllPreScale, irqRTTvalue);
			
			//rtt_read_timer_value();
			
			sprintf(velocidade,"%03d",vel_);
			font_draw_text(&arial_72, velocidade, 15, 90, 2);
			
			sprintf(distancia,"%03d",dis_);
			font_draw_text(&arial_72,distancia, 15, 205, 2);
			
			sprintf(tempo,"%d",temp_);
			font_draw_text(&arial_72, tempo, 15, 325, 2);
			
			f_rtt_alarme = false;
			
			
			
			disNova = 0;
		}
		
	}
}