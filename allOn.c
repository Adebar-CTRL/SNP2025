#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

volatile uint8_t seconds = 0;
volatile uint8_t minutes = 0;
volatile uint8_t hours = 0;
int currentBrightnessLevel = 1;
const uint8_t brightness_levels[] = {255, 254, 248, 156, 0};

void PWM_Init(void) {


    TCCR1A = (1 << WGM10) | (1 << COM1A1) | (1 << COM1B1); // 8-bit Fast PWM, clear on compare match
    TCCR1B = (1 << WGM12) | (1 << CS11); // Prescaler = 8
	
    OCR1A = 248; // For minutes cathodes (PB1)
    OCR1B = 248; // For hours cathodes (PB2)
}

void setupTimer2(void) {
    // Asynchronmodus aktivieren (externer 32.768 kHz Quarz)
    ASSR |= (1 << AS2);
    
    // Prescaler auf 128 setzen (32.768 kHz / 128 = 256 Hz)
    TCCR2B = (1 << CS22) | (1 << CS20); // Prescaler 128
	TIMSK2 = (1 << TOIE2);           // Enable Timer2 Overflow interrupt
    // (If using an external clock, wait for clock stabilization)
    // Wait for the external clock to stabilize
    while (ASSR & ((1 << TCR2BUB) | (1 << TCR2AUB) | (1 << OCR2AUB) | (1 << TCN2UB))) {
      // Wait until all update busy flags are cleared
    }
}


void enterSleepMode(void) {
    set_sleep_mode(SLEEP_MODE_IDLE); // "Idle"-Modus (CPU stoppt, Timer läuft weiter)
    sleep_enable();
    sei(); // Interrupts aktivieren
    sleep_cpu(); // Jetzt schlafen gehen...
    sleep_disable(); // Nach dem Aufwachen Sleep-Modus deaktivieren
}

void setupPorts(void) {
    // --- PWM Outputs for Cathodes on Port B ---
    // PB1 for minutes, PB2 for hours.
    DDRB |= (1 << PB1) | (1 << PB2);

	// Setze Port C (PC0 bis PC5) und Port D (PD3 bis PD7) als Ausgänge
    DDRC = 0x3F;  // PC0 bis PC5 als Ausgang
    DDRD = 0xF8;  // PD3 bis PD7 als Ausgang (PD0 bis PD2 bleiben Eingang)

    // Initialisiere die Ports mit 0 (alle LEDs aus)
    PORTC = 0x00;
    PORTD = 0x00;

}
//setup von Knöpfen
void INT_Init(void) {
    // Konfiguriere PD2 (INT0) als Eingang mit aktiviertem Pull-Up
    DDRD &= ~(1 << PD2);
    PORTD |= (1 << PD2);
    // Konfiguriere INT0: Auslösung an fallender Flanke
    EICRA |= (1 << ISC01);  // ISC01 = 1, ISC00 = 0 => fallende Flanke
    EICRA &= ~(1 << ISC00);
    // Aktiviere den externen Interrupt INT0
    EIMSK |= (1 << INT0);

	// === PD1 als normaler Button-Eingang mit Pull-Up (kein Interrupt) === druch: if (!(PIND & (1 << PD1))) {} prüfen
    DDRD &= ~(1 << PD1);
    PORTD |= (1 << PD1);
	
	  // === PB0 (Pin-Change-Interrupt) als Eingang mit Pull-Up ===
    DDRB &= ~(1 << PB0);
    PORTB |= (1 << PB0);

    // Aktivieren von Pin-Change-Interrupts für PB0 (PCINT0)
    PCICR |= (1 << PCIE0);  // Aktiviere Pin-Change-Interrupt für PCINT[7:0] (Port B)
    PCMSK0 |= (1 << PCINT0); // PB0 (PCINT0) als Pin-Change-Interrupt aktivieren
}


void displayTime(void) {
    // Minuten auf PC0-PC5 (6 Bits)
    PORTC = minutes & 0x3F;  
    
    // Stunden auf PD3-PD7 (5 Bits)
    PORTD = (PORTD & 0x07) | ((hours & 0x1F) << 3); //& 0x07 bedeutet das nur die unteren drei Bits (PD0, PD1, PD2) erhalten bleiben. 
	//Dann wird hours 3 bits nach links geschoben und mit PortD durch | vereinigt

}

void updateClock(void) {
	seconds++;
	if(seconds >= 60) {
		seconds = 0;
		minutes++;
	    if (minutes >= 60) {
	        minutes = 0;
	        hours++;
	        if (hours >= 24) {
	            hours = 0;
	        }
	    }
	}
    displayTime();
}

//Interrupt der jede Sekunde durch den Quarz ausgelöst wird
ISR(TIMER2_OVF_vect) {
    updateClock();  // Erhöhe Minuten jede Sekunde
	if (!(PIND & (1 << PD1))) {
		currentBrightnessLevel = (currentBrightnessLevel + 1) % 5;
    	OCR1A = brightness_levels[currentBrightnessLevel]; // For minutes cathodes (PB1)
    	OCR1B = brightness_levels[currentBrightnessLevel]; // For hours cathodes (PB2)
	}
}

//Interrupt bei Button 2
ISR(INT0_vect) {
	_delay_ms(5); // Debounce
    if (!(PIND & (1 << PD2))) {  // Prüfen, ob PB0 gedrückt wurde
        minutes++;
	    if (minutes >= 60) {
	        minutes = 0;
	        hours++;
	        if (hours >= 24) {
	            hours = 0;
	        }
	    }
    	displayTime();
    }
}

ISR(PCINT0_vect) {
    _delay_ms(5); // Debounce
    if (!(PINB & (1 << PB0))) {  // Prüfen, ob PB0 gedrückt wurde
        if (hours == 23) hours = 0;
		else {
			hours++;
		}
	displayTime();
    }
}
int main(void)
{
	setupPorts();
	INT_Init();
	PWM_Init(); //initialisiere PWM ausgänge für helligkeitsregulierung
	setupTimer2();
    sei();  // Enable global interrupts
    // Unendliche Schleife
    while (1)
    {
	enterSleepMode();	
	_delay_ms(5); 
    }

    return 0;  // Dies wird nie erreicht
}
