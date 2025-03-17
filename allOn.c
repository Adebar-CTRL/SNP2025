#include <avr/io.h>
#include <util/delay.h>

void count_on_port(volatile uint8_t *port, uint8_t value, uint8_t bits)
{
    *port = value;  // Setze den Port-Wert auf die aktuelle Zahl
}

int main(void)
{
    // Setze PB1 und PB2 als Ausgänge (für PWM)
    DDRB |= (1 << PB1) | (1 << PB2);

    // Timer 1 Konfiguration für PB1 (OC1A)
    TCCR1B |= (1 << CS00); // prescaler = 1
    OCR1A = 50;  // Setze die PWM Periode für PB1
    OCR1B = 50;  // Setze die PWM Periode für PB1
    TCCR1A |= (1 << WGM11); // Fast PWM Mode
    TCCR1A |= (1 << COM1A1) | (1 << COM1A0); // Set OC1A on match, clear on top
    TCCR1A |= (1 << COM1B1) | (1 << COM1B0); // Set OC1B on match, clear on top

    // Timer 0 Konfiguration für PB2 (OC0A)
    TCCR0A |= (1 << WGM00) | (1 << WGM01); // Fast PWM Mode
    TCCR0A |= (1 << COM0A1) | (1 << COM0A0); // Set OC0A on match, clear on top
    TCCR0B |= (1 << CS00); // prescaler = 1
    OCR0A = 50;  // Setze die PWM Periode für PB2

     // Setze PC0 bis PC5 und PD3 bis PD7 als Ausgänge
    DDRC = 0x3F;  // PC0 bis PC5 als Ausgang (6 Ausgänge)
    DDRD = 0xF8;  // PD3 bis PD7 als Ausgang (5 Ausgänge)

    // Initialisiere die Ausgänge mit 0 (alle LEDs aus)
    PORTC = 0x00;
    PORTD = 0x00;

    uint8_t minutes = 0;  // Minuten (0 bis 59)
    uint8_t hours = 0;    // Stunden (0 bis 23)

    // Unendliche Schleife
    while (1)
    {
        // Setze Port C auf die Minuten (0 bis 59)
        count_on_port(&PORTC, minutes, 6); // Minuten im Bereich 0 bis 59 (6 Bits)

        // Setze Port D auf die Stunden (0 bis 23)
        count_on_port(&PORTD, hours, 5); // Stunden im Bereich 0 bis 23 (5 Bits)

        _delay_ms(1000);  // Warte 1 Sekunde (Simuliere 1 Sekunde für die Uhr)

        // Erhöhe die Minuten und überprüfe den Überlauf (0 bis 59)
        minutes++;
        if (minutes >= 60)
        {
            minutes = 0;  // Zurücksetzen der Minuten
            // Erhöhe die Stunden und überprüfe den Überlauf (0 bis 23)
            hours++;
            if (hours >= 24)
            {
                hours = 0;  // Zurücksetzen der Stunden
            }
        }
    }

    return 0;  // Dies wird nie erreicht
}
