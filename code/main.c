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
uint8_t tpmsl = 0;
uint8_t tpmsr = 0;

// total ticks
uint16_t ttl = 0;
uint16_t ttr = 0;

// global speed-variables
int8_t speedl = 0;
int8_t speedr = 0;
int8_t nspeedl = 0;
int8_t nspeedr = 0;
/**************** Forward Decs *****************/
void timer1_init(void);
void set_clockdiv(uint8_t div);
void set_speed(int8_t speed_left, int8_t speed_right);
int8_t speed_to_ticks(int8_t speed);

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

/*************** INTERRUPTS *************************/
// left wheel
ISR( INT0_vect ){
    lcount++;
    // TODO: remove, only test, set led high
}

// right wheel
ISR( INT1_vect){
    rcount++;
}

// Time
ISR(TIMER3_COMPA_vect){
    millis++;
    // count ticks for 10 ms for controller
    if(millis % 10 == 0){
        cli();
            tpmsl = lcount;
            lcount = 0;
            tpmsr = rcount;
            rcount = 0;
        sei();
            // increment total ticks (used later for distance measurement)
            ttl += tpmsl;
            ttr += tpmsr;

            // desired ticks per 10ms
            int8_t sollticksl = 8;
            int8_t sollticksr = 8;

            // calculate additions for speeds
            float diff1 = (sollticksl - tpmsl) / 8.0f; // diff durch max. ticks pro intervall
            float diff2 = (sollticksr - tpmsr) / 8.0f;
            
            if( sollticksl*4095 + 65535 * diff1 > 65535){
                OCR1A = 65535;
            }else if(sollticksl*4095 + 65535 * diff1 < 0){
                OCR1A = 0;
            }else{
                OCR1A = sollticksl*4095 + 65535 * diff1;
            }

            if(sollticksr*4095 + 65535 * diff2 > 65535){
                OCR1B = 65535;
            }else if(sollticksr*4095 + 65535 * diff2 < 0){
                OCR1B = 0;
            }else{
                OCR1B = sollticksr*4095 + 65535 * diff2;
            }
    }

    // show time
    if (millis >= 500){
        millis = 0;
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
//        snprintf(tmpstring, 16, "LT%u RT%u", ttl, ttr);
        snprintf(tmpstring, 16, "1A%u 1B%u", OCR1A, OCR1B);
        LCDstring(tmpstring, strlen(tmpstring));
    }
}

/*************** HELPER FUNCTIONS ********************/
int8_t speed_to_ticks(int8_t speed){
    int8_t speedlist[17];
    speedlist[0] = 0;
    speedlist[1] = 0;
    speedlist[2] = 5;
    speedlist[3] = 25;
    speedlist[4] = 45;
    speedlist[5] = 70;
    speedlist[6] = 90;
    speedlist[7] = 100;
    speedlist[8] = 120;
    speedlist[9] = 135;
    speedlist[10] = 140;
    speedlist[11] = 155;
    speedlist[12] = 160;
    speedlist[13] = 170;
    speedlist[14] = 177;
    speedlist[15] = 180;
    speedlist[16] = 185;
    return speedlist[speed];
}

void timer1_init(void){
    //Modus und Vorteiler wählen (Phase Correct PWM und Vorteiler 1)
    TCCR1A |= (1<<COM1A1)|(1<<COM1B1)|(1<<COM1C1)|(1<<WGM11);
    TCCR1B |= (1<<WGM13)|(1<<CS10);
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

void set_speed(int8_t speed_left, int8_t speed_right){
    if(speed_left > 16) speed_left = 16;
    if(speed_left < 0) speed_left = 0;
    if(speed_right > 16) speed_right = 16;
    if(speed_right < 0) speed_right = 0;

    if(speed_left * SSTEP > 65535){
        OCR1A = 65535;
    }else{
        OCR1A = speed_left * SSTEP; // speed * bits per volt
    }

    if(speed_right * SSTEP > 65535){
        OCR1B = 65535;
    }else{
        OCR1B = speed_right * SSTEP; // speed * bits per volt
    }
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
        PORTD |= (1 << 5);
        PORTD |= (1 << 4);

        PORTD |= (1 << 7);
        PORTD |= (1 << 6);
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
        PORTB |= (1 << 0); //set pin 0 on port B high
        _delay_ms(1000);
        PORTB |= (1 << 1);
        PORTB |= (1 << 2);
        PORTB |= (1 << 3);
        PORTB &= ~(1 << 0); //set pin 0 on port B low
}



/******************** MAIN *****************************/
int main (void){
    LCDinit();
    LCDcursorOFF();
    LCDclr();
    LCDstring("Cambot v0.2", 11);
    LCDGotoXY(0, 1);
    LCDstring("Zweite Zeile", 12);


    DDRB = 0xFF; // ausgang
    
    // Motor-Leitungen auf Ausgang stellen
    DDRD |= (1 << 4) | (1 << 5) | (1 << 6) | (1 << 7);
    //DDRD &= ~(1 << 0); // eingang
    //DDRD &= ~(1 << 1); // eingang
    PORTD |= (1 << 0); // pullup
    PORTD |= (1 << 1); // pullup

    // interrupts einschalten
    EIMSK |= (1 << INT0) | (1 << INT1); // INT0 enable
    EICRA |= (1 << ISC10) | (1 << ISC00); 

    // Timer 3 für die Uhrzeit
    TCCR3A = 0;
    TCCR3B = 0;
    OCR3A = 15999;
    TCCR3B |= (1 << WGM12);
    TCCR3B |= (1 << CS10);
    TIMSK3 |= (1 << OCIE3A);

    // Interrupt starten
    sei();


    set_clockdiv(0);
    timer1_init();

    uint16_t shotcount = 0;

    while (1){
        // drei mal blinken bevor wir losfahren
        PORTB |= (1 << 0); //set pin 0 on port B high
        _delay_ms(500);
        PORTB &= ~(1 << 0); //set pin 0 on port B low
        _delay_ms(500);
        PORTB |= (1 << 0); //set pin 0 on port B high
        _delay_ms(500);
        PORTB &= ~(1 << 0); //set pin 0 on port B low
        _delay_ms(500);
        PORTB |= (1 << 0); //set pin 0 on port B high
        _delay_ms(500);
        PORTB &= ~(1 << 0); //set pin 0 on port B low
        _delay_ms(500);

        set_direction(1);
        speedl = 10;
        speedr = 10;
        //set_speed(speedl, speedr);
        OCR1A = 50000;
        OCR1B = 50000;

        char string[16];

        // let the motors run on each level
        // to measure ticks per speedvalue
        /*
        while(1){
            uint8_t x=0;
            for(x=0;x<17;x++){
                set_speed(x,x);
                LCDGotoXY(0, 0);
                snprintf(string, 16, "SL: %u", x);
                LCDstring("                ", 16);
                LCDGotoXY(0, 0);
                LCDstring(string, strlen(string));
                _delay_ms(10000);
            }
        }
        */

        while(1){
            // controller for the wheels
            cli();
                uint16_t mytpmsl = tpmsl;
                uint16_t mytpmsr = tpmsr;
            sei();



            // get total ticks left
            cli();
                uint16_t my_ttl = ttl;
            sei();
            if(my_ttl >= 10*TPU){ // halbe umdrehung
                set_direction(0);

                // AUSGABE AUF DISPLAY
                /*
                LCDGotoXY(0, 0);
                snprintf(string, 16, "L:%u R:%u S:%u", ttl, ttr ,shotcount);
                LCDstring("                ", 16);
                LCDGotoXY(0, 0);
                LCDstring(string, strlen(string));
                */

                _delay_ms(2000);
                trigger();
                shotcount++;
                set_direction(1);
                cli();
                    ttl = 0;
                    ttr = 0;
                sei();
            }
        }
    }
    return 0;
}

