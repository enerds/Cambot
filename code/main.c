#define F_CPU 16000000
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "lcd_lib.h"

/**************** Global Variables *************/
uint16_t lcount = 0;
uint16_t rcount = 0;

uint16_t seconds = 0;
uint16_t millis = 0;

// ticks per 100ms
int8_t tpmsl = 0;
int8_t tpmsr = 0;

// total ticks
uint16_t ttl = 0;
uint16_t ttr = 0;

// sollticks
int8_t sollticksl = 0;
int8_t sollticksr = 0;

// global speed-variables
int8_t speedl = 0;
int8_t speedr = 0;
int8_t nspeedl = 0;
int8_t nspeedr = 0;

// sum of errors for i-controller
int16_t esuml = 0;
int16_t esumr = 0;

/**************** Forward Decs *****************/
void timer1_init(void);
void set_clockdiv(uint8_t div);
void set_speed(int8_t speed_left, int8_t speed_right);
int16_t pi(float kp, float ki, int error, float ta, int16_t esum);

/**************** DEFINES **********************/
#define RU 37.699 // Radumfang
#define TPU 1800 // Ticks pro Umdrehung
#define SPT RU/TPU // Strecke pro Tick
#define RPM 75 // Rounds per Minute
#define RPS RPM / 60 // Rounds per Second
#define TPS RPS * TPU // Ticks pro Sekunde, wenn volle Geschwindigkeit
#define GAI 1 / TPS // Geschätztes nötiges Abtast-Intervall -> 1 / Ticks Pro Sekunde
                    // Bei 1 / 2250 = 0.0004 wären das ungefähr 400 Mikrosekunden
#define STEPS 16 // Anzahl Steps für Speed
#define SSTEP 65535 / STEPS // One Speedstep in bits

#define TIMESPAN 10
float timespan = 10.0f;


/*************** INTERRUPTS *************************/
// left wheel
ISR( INT0_vect ){
    lcount++;
}

// right wheel
ISR( INT1_vect){
    rcount++;
}

// Time
ISR(TIMER3_COMPA_vect){
    millis++;
	    // count ticks for 10 ms for controller
	    if(millis % 10 == 0 ){
		cli();
		    tpmsl = lcount;
		    lcount = 0;
		    tpmsr = rcount;
		    rcount = 0;
		sei();
		    // increment total ticks (used later for distance measurement)
		    ttl += tpmsl;
		    ttr += tpmsr;

		    int32_t diff = ttl - ttr;

		    esuml += sollticksl - tpmsl;
		    esumr += sollticksr - tpmsr;

		    int16_t new_speedl = OCR1A + pi(5.0f, 0.1f, (sollticksl - tpmsl), 1, -diff);
		    int16_t new_speedr = OCR1B + pi(5.0f, 0.1f, (sollticksr - tpmsr), 1, diff);
			if(new_speedl < 0) new_speedl = 0;
			if(new_speedr < 0) new_speedr = 0;
			if(new_speedl > 65535) new_speedl = 65535;
			if(new_speedr > 65535) new_speedr = 65535;
			OCR1A = new_speedl;
			OCR1B = new_speedr;
	    }

}

/*************** HELPER FUNCTIONS ********************/
int16_t pi(float kp, float ki, signed int error, float ta, int16_t esum){
    float corrected = kp * error + ki * esum;
    int32_t new_value = corrected * 1024;

    if(new_value > 31199) new_value = 31199;
    if(new_value < -31199) new_value = -31199;

    return new_value;
}

void timer1_init(void){
    //Modus und Vorteiler wählen (Phase Correct PWM und Vorteiler 1)
    //TCCR1A |= (1<<COM1A1)|(1<<COM1B1)|(1<<COM1C1)|(1<<WGM11);
    TCCR1A |= (1<<COM1A1)|(1<<COM1B1)|(1<<WGM11);
    TCCR1A &= ~(1<<COM1A0);
    TCCR1A &= ~(1<<COM1B0);

    TCCR1B |= (1<<WGM12)|(1<<WGM13)|(1<<CS10);

    //Auflösung 16-Bit
    ICR1 |= 0xFFFF;
}

void set_clockdiv(uint8_t div){
    // Zunächst muss Clock Prescaler Change Enable gesetzt werden
    CLKPR = 0x80; 
    //Anschließend wird 4x das Register CLKPR gelöscht und damit auf Clock Division Faktor 1 gestellt
    CLKPR = div; 
    CLKPR = div; 
    CLKPR = div; 
    CLKPR = div;
}


void set_direction(int8_t dir){
    // zurück
    if(dir == -1){
        PORTD |= (1 << 4);
        PORTD &= ~(1 << 5);

        PORTD &= ~(1 << 6);
        PORTD |= (1 << 7);
    }
    /* bremsen */
    if(dir == 0){
        PORTD &= ~(1 << 5);
        PORTD &= ~(1 << 4);

        PORTD &= ~(1 << 7);
        PORTD &= ~(1 << 6);
    }
    /* vorwärts */
    if(dir == 1){
        PORTD |= (1 << 5);
        PORTD &= ~(1 << 4);

        PORTD &= ~(1 << 7);
        PORTD |= (1 << 6);
    }
}

void trigger(){
        PORTB &= ~(1 << 1);
        PORTB &= ~(1 << 2);
        PORTB &= ~(1 << 3);
        _delay_ms(1000);
        PORTB |= (1 << 1);
        PORTB |= (1 << 2);
        PORTB |= (1 << 3);
}



/******************** MAIN *****************************/
int main (void){
    LCDinit();
    LCDcursorOFF();
    LCDclr();
    LCDstring("Cambot v0.2", 11);

    DDRB = 0xFF; // ausgang
    
    // Motor-Leitungen auf Ausgang stellen
    DDRD |= (1 << 4) | (1 << 5) | (1 << 6) | (1 << 7);
    DDRD &= ~(1 << 0); // eingang
    DDRD &= ~(1 << 1); // eingang
    PORTD |= (1 << 0); // pullup
    PORTD |= (1 << 1); // pullup

    // interrupts einschalten
    EICRA |= (1 << ISC10) | (1 << ISC00); 
    EIMSK |= (1 << INT0) | (1 << INT1); // INT0 enable

    // Timer 3 für die Uhrzeit
    TCCR3A = 0;
    TCCR3B = 0;
    OCR3A = 15999;
    TCCR3B |= (1 << WGM12);
    TCCR3B |= (1 << CS10);
    TIMSK3 |= (1 << OCIE3A);

	OCR1A = 0;
	OCR1B = 0;

    // Interrupt starten
    sei();

    set_clockdiv(0);
    timer1_init();

    uint16_t shotcount = 0;

    while (1){
        set_direction(1);

        char string[16];

        while(1){
		set_direction(1);
		sollticksl = 10;
		sollticksr = 10;

	    cli();
		uint16_t my_millis = millis;
	    sei();


	    // show time
	    if (my_millis >= 1000){
		cli();
			millis = 0;
		sei();
		seconds++;
		char tmpstring[16];
		LCDGotoXY(0, 1);
		LCDstring("                ", 16);
		LCDGotoXY(0, 1);
		snprintf(tmpstring, 16, "S:%u L%u R%u", seconds, tpmsl, tpmsr);
		LCDstring(tmpstring, strlen(tmpstring));

		LCDGotoXY(0, 0);
		LCDstring("                ", 16);
		LCDGotoXY(0, 0);
		snprintf(tmpstring, 16, "LT%u RT%u", ttl, ttr);
		LCDstring(tmpstring, strlen(tmpstring));
	    }


            if(ttl >= 3600){
		set_direction(0);
		sollticksl = 0;
		sollticksr = 0;
		esumr = 0;
		esuml = 0;

		char tmpstring[16];
		LCDGotoXY(0, 0);
		LCDstring("                ", 16);
		LCDGotoXY(0, 0);
		snprintf(tmpstring, 16, "LT%u RT%u", ttl, ttr);
		LCDstring(tmpstring, strlen(tmpstring));


                _delay_ms(2000);
                trigger();
                shotcount++;
                cli();
                    ttl = 0;
                    ttr = 0;
                sei();
            }
        }
    }
    return 0;
}

