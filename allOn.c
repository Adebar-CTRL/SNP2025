#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
int currentBrightnessLevel = 1;

void PWM_Init(void) {


    TCCR1A = (1 << WGM10) | (1 << COM1A1) | (1 << COM1B1); // 8-bit Fast PWM, clear on compare match
    TCCR1B = (1 << WGM12) | (1 << CS11); // Prescaler = 8

    OCR1A = 156; // For minutes cathodes (PB1)
    OCR1B = 156; // For hours cathodes (PB2)
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
	
	setupPorts();
	INT0_Init();
//	external_interrupts_Init();
	PWM_Init(); //initialisiere PWM ausgänge für helligkeitsregulierung
    sei();  // Enable global interrupts
    // Unendliche Schleife
    while (1)
    {
        // Zähle für Port C (PC0 bis PC5)
        PORTC++;  // Inkrementiere PORTC
        // Zähle für Port D (PD3 bis PD7)
        PORTD++;  // Inkrementiere PORTD
        _delay_ms(400);  // Warte für 50ms
    }

    return 0;  // Dies wird nie erreicht
}
