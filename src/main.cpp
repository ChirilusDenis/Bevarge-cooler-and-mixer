#include <LiquidCrystal_I2C.h>
#include  <Wire.h>
#include <HX711.h>

#define FAN_PWM_DIV 5 // (1/FAN_PWM_DIV) = duty% 
#define W_CUP 179510 // measure full cup weight

volatile int timer_0_ps = 0; // extra counting space for TIMER0
volatile bool interrupt = false; // interupt happened
volatile int state = 0; // state of the system
volatile int READ_FREQ_DIV = 30; // adition prescaler to slow down interupts

LiquidCrystal_I2C lcd(0x27,  16, 2);
const int LOADCELL_DOUT_PIN = 4;
const int LOADCELL_SCK_PIN = 3;
HX711 scale;

ISR(TIMER0_COMPA_vect) // about 60Hz
{
    timer_0_ps ++;
	if(timer_0_ps >= READ_FREQ_DIV) {
		timer_0_ps = 0;
		interrupt = true;
	}
}

ISR(INT0_vect) {
	if (state == 0)
		state = 1;	
}

void adc_init(void)
{
    //Set ADC prescaler to 128 (don't need higher precision for this project)
    ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

    //Set ADC reference voltage to AVCC
    ADMUX |= (1 << REFS0);

    //Enable ADC
    ADCSRA |= (1 << ADEN);
}

uint16_t myAnalogRead(uint8_t channel)
{
    // Force input channel to be between 0 and 7 (as ADC pins are PA0-7)
    channel &= 0b00000111;

    // 1. Clear the old channel value (if any, last 5 bits in ADMUX)
    ADMUX &= ~(31 << MUX0);
    // 2. Select the new channel in ADMUX
    ADMUX |= (channel << MUX0);
    // 3. Start single conversion
    ADCSRA |= (1 << ADSC);
    // 4. Busy wait for conversion to complete
    while(!(ADCSRA & (1<<ADIF)));

    // Return ADC value
    return (ADC);
}

void Timer0_init_ctc(void)
{
    /* Clear previous settings */
    TCCR0A = 0;
    TCCR0B = 0;
	TCNT0 = 0;

    /* Set CTC mode */
    TCCR0A |= (1 << WGM01);

    /* Set prescaler to 1024 */
    TCCR0B |= (1 << CS02) | (1<<CS00);

    /* Activate Compare A interrupt */
    TIMSK0 |= (1 << OCIE0A);
	OCR0A = 255; 
}

void Timer1_init_pwm() {
    DDRB |= (1 << PB1);  // Set PB1 (OC1A) as output

    // Set Fast PWM
    TCCR1A = (1 << COM1A1) | (1 << WGM11);  // Non-inverting, PWM A
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS10);  // Fast PWM, TOP = ICR1, prescaler = 1

    ICR1 = 639;  // TOP for 25kHz

	OCR1A = 640 / FAN_PWM_DIV - 1; // 20% duty
}

int read_temp_sens(int channel) {
	uint16_t t = myAnalogRead(channel); // Read raw value

	// NTC thermistor math to get read in Celsius
	float voltage = t * 5.0 / 1024.0;
	float Rt = 100000.0 * voltage / (5.0 - voltage);
    float tempK = 1.0 / (1.0/298.15 +log(Rt / 100000.0) / 3950.0);
    return int(tempK - 273.15);
}

int get_avg_liquid_temp(int prop, int ch1, int ch2) {
	// get the average temperature between the 2 sensors
	int t0 = read_temp_sens(ch1);
	int t1 = read_temp_sens(ch2);
	return (t0 * prop + t1 * (100 - prop)) / 100;
}

void pots_value(int *t, int *p) {
	int temp, propr, real_t, real_p;
	
	temp = myAnalogRead(2);
	real_t = (temp * 10 / 1024.0) + 10; // transform in 10 - 20 range

	propr = myAnalogRead(3);
	real_p = (propr / 1024.0) * 60 + 20; // transform in 20 - 80 range

	*t = real_t;
	*p = real_p;
}

void lcd_selection(int temp, int propr, int lq_t) {
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.printf("Temp: %d/%d", temp, lq_t);
	lcd.setCursor(0 ,1);
	lcd.printf("Prop: 0.%d", propr);
}

void init_INT0(void) {
	DDRD &= ~(1<<PD2); // PD2 as input
	PORTD |= (1 << PORTD2); // activate pull up

	EICRA &= ~(3 << ISC00); // activate INT0 on LOW
    EIMSK |= (1 << INT0);
}

void active_pump_1(void) {PORTD |= (1<<PD5);}
void dis_pump_1(void) {PORTD &= ~(1<<PD5);}

void active_pump_2(void) {PORTD |= (1<<PD6);}
void dis_pump_2(void) {PORTD &= ~(1<<PD6);}

void active_peltier(void) {
	PORTD &= ~(1<<PD7);
	OCR1A = ICR1; // ramp up fans(duty = 100%)
}
void dis_peltier(void) {
	PORTD |= (1<<PD7);
	OCR1A = (ICR1 + 1) / FAN_PWM_DIV - 1; // slow down fans(duty = 20%)
}

void init_relays(void) {
	DDRD |= (1<<PD5) | (1<<PD6) | (1<<PD7); // output pins
	PORTD |= (1<<PD7); // PD7 goes to pelteir realys, inative on HIGH
	PORTD &= ~((1<<PD5) | (1<<PD6)); // pump relays, inactive on LOW
}

void init_scale(void) {
	scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN, 64);
	// system should be started with the cup in place
	scale.tare(); // zeroes the loadcell with the weight of the empty cup
}

long get_w(void) {
	return scale.get_value(1);
}


int main(void) {
	init();
	lcd.init();
	lcd.backlight();
	lcd.setCursor(0, 0);
	lcd.print("Starting...");
	adc_init();
	Timer0_init_ctc();
	Timer1_init_pwm();
	init_INT0();
	init_relays();
	init_scale();
	sei();
	state = 0;

	int temp = 0 ;
	int proportion = 0;
	int liq_t = 0;

	while(1) {

		if(state == 0) {
			if(interrupt) {
				liq_t = read_temp_sens(0); // read liquids temperature
				
				pots_value(&temp, &proportion); // read selection from potentiometers
				lcd_selection(temp, proportion, liq_t); // print pn LCD
				
				interrupt = false;
			}
		}
		
		else if (state == 1) {
			if(interrupt) {
				liq_t = read_temp_sens(0); // read liquids temperature

				lcd_selection(temp, proportion, liq_t); // rint selection

				if (liq_t > temp)
					active_peltier(); // if liquids are over the set temperature, they are cooled
				else {
					// liquids got to the target temp, stop cooling
					dis_peltier();
					state = 2;
				}
				interrupt = false;
			}
		}

		else if (state == 2) {
			if(interrupt) {

				lcd.clear();
				lcd.setCursor(0, 0);
				lcd.printf("Weight = %ld", get_w());

				active_pump_1();
				// chceck if the first liquid weight treshold is reached
				if(get_w() >= (W_CUP / 100) * proportion) {
					dis_pump_1();
					state = 3;
				}

				interrupt = false;
			}
		}

		else if (state == 3) {
			if(interrupt) {

				lcd.clear();
				lcd.setCursor(0, 0);
				lcd.printf("Weight = %ld", get_w());

				active_pump_2();
				// check if the cup is full with the rest of the second liquid
				if(get_w() >= W_CUP) {
					dis_pump_2();
					state = 4;
				}

				interrupt = false;
			}
		}

		else if (state == 4) {
			if(interrupt) {

				lcd.clear();
				lcd.setCursor(0, 0);
				lcd.print("Please pick up");

				// check if full cup was picked up
				if(get_w() < 200) {
					state = 0;
				}

				interrupt = false;
			}

		} else state = 0;
	}
}