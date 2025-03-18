#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

volatile uint8_t minutes = 0;
volatile uint8_t hours = 0;
int currentBrightnessLevel = 1;

void PWM_Init(void) {


    TCCR1A = (1 << WGM10) | (1 << COM1A1) | (1 << COM1B1); // 8-bit Fast PWM, clear on compare match
    TCCR1B = (1 << WGM12) | (1 << CS11); // Prescaler = 8
	
    OCR1A = 156; // For minutes cathodes (PB1)
    OCR1B = 156; // For hours cathodes (PB2)
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
void INT0_Init(void) {
    // Konfiguriere PD2 (INT0) als Eingang mit aktiviertem Pull-Up
    DDRD &= ~(1 << PD2);
    PORTD |= (1 << PD2);

    // Konfiguriere INT0: Auslösung an fallender Flanke
    EICRA |= (1 << ISC01);  // ISC01 = 1, ISC00 = 0 => fallende Flanke
    EICRA &= ~(1 << ISC00);

    // Aktiviere den externen Interrupt INT0
    EIMSK |= (1 << INT0);
}


void displayTime(void) {
    // Minuten auf PC0-PC5 (6 Bits)
    PORTC = minutes & 0x3F;  
    
    // Stunden auf PD3-PD7 (5 Bits)
    PORTD = (PORTD & 0x07) | ((hours & 0x1F) << 3); //& 0x07 bedeutet das nur die unteren drei Bits (PD0, PD1, PD2) erhalten bleiben. 
	//Dann wird hours 3 bits nach links geschoben und mit PortD durch | vereinigt

}

void updateClock(void) {
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

//Interrupt der jede Sekunde durch den Quarz ausgelöst wird
ISR(TIMER2_OVF_vect) {
    updateClock();  // Erhöhe Minuten jede Sekunde
}

//Interrupt bei Button 2
ISR(INT0_vect) {
	const uint8_t brightness_levels[] = {254, 248, 156, 0};
    _delay_ms(5); // Debounce
    if (!(PIND & (1 << PD2))) { // Confirm button is still pressed
		currentBrightnessLevel = (currentBrightnessLevel + 1) % 4;
    	OCR1A = brightness_levels[currentBrightnessLevel]; // For minutes cathodes (PB1)
    	OCR1B = brightness_levels[currentBrightnessLevel]; // For hours cathodes (PB2)
	}
}

int main(void)
{
	int pd_counter = 1;
	setupPorts();
	INT0_Init();
	PWM_Init(); //initialisiere PWM ausgänge für helligkeitsregulierung
    sei();  // Enable global interrupts
    // Unendliche Schleife
    while (1)
    {
	//updateClock();
	//_delay_ms(5); 
    }

    return 0;  // Dies wird nie erreicht
}
