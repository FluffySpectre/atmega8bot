#include <avr/io.h>
#define F_CPU 1000000UL  // 1 MHz
#include <util/delay.h>

#define OFF			   0
#define ON  		   1

#define RWD 		   0
#define FWD            1

#define LEFT         1.1
#define MIDDLE       1.6
#define RIGHT        2.0

#define MAX_DISTANCE 220

int value;

void MSleep(unsigned int t) {
	while(t) {
		_delay_ms(100);
		
		t--;
	}
}

void StatusLED(unsigned char status) {
	PORTB = (PORTB & ~(1 << PB0)) | (status << PB0);
}

/*
uint16_t GetAdc(uint8_t channel){
  	uint16_t z = 300;// z  
	uint16_t j, i;
	uint32_t value = 0, value1 = 0, value2 = 0;
	
	for(j = z; j > 0; j--) {
		for(i = z; i > 0; i--) {
			ADCSRA = 0x80; 		// ADC eingeschaltet, kein Prescale 
			ADMUX = channel;  			// ADC Ref auf Avcc, ADC0 gewaehlt, normale Formatierung
			ADCSRA |= (1 << ADSC); 	// single conversion mode ein
			while (ADCSRA & (1 << ADSC));// auf Abschluss der Konvertierung warten 
			
			value += ADCW;
		}
		
		value1 = value / z;
		value2 += value1;
	}
	
	value = (value2 / z);

	return (value);
}
*/

int GetAdc(int kanal){

  	int i,j,z;// z  
	z= 300;
	i=z, j=z;
  	
	
	long int analogwert=0, analogwert1=0, analogwert2=0 ;
	
	while(j){
  		
		while(i){
			ADCSRA=0x80; 		// ADC eingeschaltet, kein Prescale 
			ADMUX=kanal;  			// ADC Ref auf Avcc, ADC0 gewaehlt, normale Formatierung
			ADCSRA |=_BV(ADSC); 	// single conversion mode ein
			while (ADCSRA & (1<<ADSC)) {;}	// auf Abschluss der Konvertierung warten 
			analogwert+=ADCW;
			i--;
		}
		
		analogwert1 = analogwert/z;
		analogwert2 += analogwert1;
		j--;
	}
	
	analogwert=(analogwert2/z);

	return (analogwert);
}

void Init(void) {	
	// Eingänge für Sensoren etc.
	DDRC &= ~(1 << PC1) | ~(1 << PC2) | ~(1 << PC3) | ~(1 << PC4);
	
	// Ausgang für Schwenkservo
	DDRC = DDRC | (1 << PC0) | (1 << PC5);
	
	// Ausgänge für die Motorsteuerung
	DDRB = DDRB | (1 << PB0) | (1 << PB1) | (1 << PB2);
	DDRD = DDRD | (1 << PD0) | (1 << PD1) | (1 << PD2) | (1 << PD3) | (1 << PD4) | (1 << PD5);
	
	// Timer für PWM konfigurieren und aktivieren
	TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM10);
	TCCR1B = (1 << CS11);	
	
	StatusLED(ON);
	
	// Messwert initialisieren
	value = 0;
}

void MotorDir(unsigned char dir) {
    /* Linker Motor */
	if(dir == FWD) {
		PORTD &= ~(1 << PD0);
		PORTD |= (1 << PD1);
	}else{
		PORTD &= ~(1 << PD1);
		PORTD |= (1 << PD0);
	}
}

void MotorSpeed(unsigned char speed) {
	OCR1A = speed;
}

void MotorFWD(unsigned char speed) {
	MotorDir(FWD);
	MotorSpeed(speed);
}

void MotorRWD(unsigned char speed) {
	MotorDir(RWD);
	MotorSpeed(speed);
}

void MotorStop(void) {
	PORTD = PORTD & ~(1 << PD0) & ~(1 << PD1);
}

void LenkenLeft(void) {
    PORTD &= ~(1 << PD3);
	PORTD |= (1 << PD2);
	
	OCR1B = 255;
}

void LenkenRight(void) {
    PORTD &= ~(1 << PD2);
    PORTD |= (1 << PD3);
	
	OCR1B = 255;
}

void LenkenFree(void) {
    PORTD = PORTD & ~(1 << PD2) & ~(1 << PD3);
	OCR1B = 0;
}

void AvoidObstacle(unsigned char dir, unsigned char speed) {
	/* Anhalten */
	MotorStop();
	MSleep(5);
	
	//MotorRWD(200);
	//MSleep(12);
	
	/* Solange drehen, bis kein Hindernis mehr erkannt wird */
	while(value >= MAX_DISTANCE){
		if(dir == LEFT) {
			LenkenLeft();
		}else{
			LenkenRight();
		}
		
		MotorFWD(220);
		
		value = GetAdc(4);
		
		MSleep(1);
	}
	
	LenkenFree();
}

void TurnSensor(float angle) {
	unsigned char turnTime = 5;
	
	while(turnTime > 0) {
		PORTC |= (1 << PC5); 
		_delay_ms(angle);
		
		PORTC &= ~(1 << PC5); 
		_delay_ms(20-angle);
		
		turnTime--;
	}
}

void TurnSensorLeft(void){
	TurnSensor(LEFT);
	
	value = GetAdc(4);
	
	if(value >= MAX_DISTANCE) {
		AvoidObstacle(RIGHT, 110); 
	}else{
		MotorFWD(130);  
	}
}

void TurnSensorRight(void){
	TurnSensor(RIGHT);
	
	value = GetAdc(4);
	
	if(value >= MAX_DISTANCE) {
		AvoidObstacle(LEFT, 110); 
	}else{
		MotorFWD(130);  
	}
}

void TurnSensorMiddleLeft(void){
	TurnSensor(MIDDLE);
	
	value = GetAdc(4);
	
	if(value >= MAX_DISTANCE) {
		AvoidObstacle(RIGHT, 110); 
	}else{
		MotorFWD(130);  
	}
}

void TurnSensorMiddleRight(void){
	TurnSensor(MIDDLE);
	
	value = GetAdc(4);
	
	if(value >= MAX_DISTANCE) {
		AvoidObstacle(LEFT, 110); 
	}else{
		MotorFWD(130);  
	}
}

int main(void){
	/* Prozessor initialisieren */
	Init(); 
	
	PORTC |= (1 << PC0);
	
	/* Programmschleife */
	while(1) {
		TurnSensorLeft(); 
		TurnSensorMiddleLeft();
		TurnSensorRight();
		TurnSensorMiddleRight();
	}
	
	return 0;
}

