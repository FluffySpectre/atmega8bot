/************************************************************************
 *
 * Datei:  			    RoboControl
 *							
 * Projekt:   			AtmegaBot
 * 
 * Beschreibung:	    Hauptprogramm zur Steuerung des Roboters
 *
 *
 * Version    Datum          Autor                 Kommentare
 * -------  ----------   --------------   ------------------------------
 * 1.00     16.01.2011   Björn Bosse	  Erstes Build
 * 2.00	    10.07.2011   Björn Bosse	  Kommentare überarbeitet
 *
 *
 ***********************************************************************/


#include "RoboControl.h"


uint16_t distanceSensorVal;		// Wert des Abstandssensors
//volatile uint16_t timeCount;	// 


/**
 * Timer Overflow Interrupt
 *
 * @param -
 *
 * @return -
 *
 * @example -
 */
ISR(TIMER2_COMP_vect) {	
	//timeCount++;
	
	// Kollisionstaster abfragen
	CheckCollisionSensor();
}



/**
 * Bereitet den Prozessor für die Verwendung vor
 * - Ein-/Ausgänge
 * - Timer
 *
 * @param -
 *
 * @return -
 *
 * @example int main(void) {
 *				Init();
 *
 *				while(1) {}
 *			}
 */
void Init(void) {	
	// Eingaenge für Sensoren etc.
	DDRC &=  ~(1 << PC0) | ~(1 << PC1) | ~(1 << PC2) | ~(1 << PC3) | ~(1 << PC4);
	
	// Ausgang für Schwenkservo
	DDRC |= (1 << SERVO);
	
	// Ausgang für die Status-LED
	DDRB |= (1 << STATUS_LED);
	
	// Ausgaenge für den Motortreiber
	DDRB |= (1 << PB1) | (1 << PB2);
	DDRD |= (1 << PD0) | (1 << PD1) | (1 << PD2) | (1 << PD3) | (1 << PD4) | (1 << PD5);
	
	// Timer1 für PWM konfigurieren und aktivieren
	TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM10);
	TCCR1B = (1 << CS11);	
	
	// Timer2 fuer Delays aktivieren
	TCCR2 = (1 << WGM21) | (1 << CS20) | (1 << CS21);	
	OCR2 = 125;
	TIMSK |= (1 << OCIE2);
	
	//OCR2   =   F_CPU / (38000 * 2) - 1;     // Modulationsfrequenz
	//TCCR2  =   (1 << WGM21)  				 // CTC-Mode
			 //| (1 << COM20)  				 // Toggle OC2 on Compare Match
			 //| (1 << CS21);                 // Prescaler: 1

	// Interne Pull-Up-Widerstaende für Kollisionssensoren einschalten
	PORTC |= (1 << POLLSWITCH_LEFT) | (1 << POLLSWITCH_RIGHT);
	
	// Status LED einschalten
	StatusLED(ON);
	
	// Messwert initialisieren
	distanceSensorVal = 0;
	
	// Interrupts zulassen
	sei();
}



/**
 * Schaltet die Status-LED ein/aus
 *
 * @param status Der Status der LED (ON/OFF)
 *
 * @return -
 *
 * @example StatusLED(ON);
 */
inline void StatusLED(uint8_t status) {
	PORTB = (PORTB & ~(1 << STATUS_LED)) | (status << STATUS_LED);
}



/**
 * Setzt die Drehrichtung der Motoren
 *
 * @param leftDir Drehrichtung des linken Motors (FWD/RWD)
 * @param rightDir Drehrichtung des rechten Motors (FWD/RWD)
 *
 * @return -
 *
 * @example MotorDir(FWD, FWD);
 */
void MotorDir(uint8_t leftDir, uint8_t rightDir) {
    // Rechter Motor
	if(rightDir == FWD) {
		PORTD &= ~(1 << PD0);
		PORTD |= (1 << PD1);
	}else{
		PORTD &= ~(1 << PD1);
		PORTD |= (1 << PD0);
	}
	
	// Linker Motor
	if(leftDir == FWD) {
		PORTD &= ~(1 << PD2);
		PORTD |= (1 << PD3);
	}else{
		PORTD &= ~(1 << PD3);
		PORTD |= (1 << PD2);
	}
}



/**
 * Setzt die Geschwindigkeit der Motoren
 *
 * @param leftSpeed Geschwindigkeit des linken Motors (0 - 255)
 * @param rightSpeed Geschwindigkeit des rechten Motors (0 - 255)
 *
 * @return -
 *
 * @example MotorSpeed(220, 220);
 */
inline void MotorSpeed(uint8_t leftSpeed, uint8_t rightSpeed) {
	OCR1A = leftSpeed;
	OCR1B = rightSpeed;
}



/**
 * Beide Motoren vorwärts mit bestimmter Geschwindigkeit
 *
 * @param leftSpeed Geschwindigkeit des linken Motors (0 - 255)
 * @param rightSpeed Geschwindigkeit des rechten Motors (0 - 255)
 *
 * @return -
 *
 * @example MotorFWD(220, 220);
 */
void MotorFWD(uint8_t leftSpeed, uint8_t rightSpeed) {
	MotorDir(FWD, FWD);
	MotorSpeed(leftSpeed, rightSpeed);
}



/**
 * Beide Motoren rückwärts mit bestimmter Geschwindigkeit
 *
 * @param leftSpeed Geschwindigkeit des linken Motors (0 - 255)
 * @param rightSpeed Geschwindigkeit des rechten Motors (0 - 255)
 *
 * @return -
 *
 * @example MotorRWD(220, 220);
 */
void MotorRWD(uint8_t leftSpeed, uint8_t rightSpeed) {
	MotorDir(RWD, RWD);
	MotorSpeed(leftSpeed, rightSpeed);
}



/**
 * Beide Motoren stoppen
 *
 * @param -
 *
 * @return -
 *
 * @example MotorStop();
 */
//inline void MotorStop(void) {
//	PORTD &= ~(1 << PD0) & ~(1 << PD1) & ~(1 << PD2) & ~(1 << PD3);
//}



/**
 * Steuert den angegebenen Winkel eines Servos an
 *
 * @param angle Winkel der angefahren werden soll
 *
 * @return -
 *
 * @example TurnSensor(LEFT); // Schwenkt den Sensor nach links
 */
void TurnSensor(float angle) {
	uint8_t turnTime = 25;
	
	while(turnTime > 0) {
		PORTC |= (1 << SERVO); 
		//MSleep(angle);
		_delay_ms(angle);
		
		PORTC &= ~(1 << SERVO); 
		//MSleep(20 - angle);
		_delay_ms(20 - angle);
		
		turnTime--;
	}
}



/**
 * Schwenkt den Abstandssensor nach Links, ermittelt, ob 
 * sich ein Hindernis im Weg befindet und 
 * weicht nötigenfalls nach Rechts aus
 *
 * @param -
 *
 * @return -
 *
 * @example TurnSensorLeft();
 */
void TurnSensorLeft(void) {
	TurnSensor(LEFT);
	
	distanceSensorVal = GetAdc(DETECTION_SENSOR_ADC);
	
	if(distanceSensorVal >= DETECTION_RANGE) {
		AvoidObstacle(LEFT); 
	}else{
		MotorFWD(220, 230);  
	}
}



/**
 * Schwenkt den Abstandssensor nach Rechts, ermittelt, ob 
 * sich ein Hindernis im Weg befindet und 
 * weicht nötigenfalls nach Links aus
 *
 * @param -
 *
 * @return -
 *
 * @example TurnSensorRight();
 */
void TurnSensorRight(void) {
	TurnSensor(RIGHT);
	
	distanceSensorVal = GetAdc(DETECTION_SENSOR_ADC);
	
	if(distanceSensorVal >= DETECTION_RANGE) {
		AvoidObstacle(RIGHT); 
	}else{
		MotorFWD(220, 230);  
	}
}



/**
 * Schwenkt den Abstandssensor von Links zur Mitte, ermittelt, ob 
 * sich ein Hindernis im Weg befindet und 
 * weicht nötigenfalls in eine zufällige Richtung aus
 *
 * @param -
 *
 * @return -
 *
 * @example TurnSensorMiddleLeft();
 */
void TurnSensorMiddleLeft(void) {
	TurnSensor(MIDDLE);
	
	distanceSensorVal = GetAdc(DETECTION_SENSOR_ADC);
	
	if(distanceSensorVal >= DETECTION_RANGE) {
		uint8_t randValue = RandValue01();
		
		// Zufällige Richtung wählen
		if(randValue == 0) { AvoidObstacle(LEFT); }
		if(randValue == 1) { AvoidObstacle(RIGHT); }
	}else{
		MotorFWD(220, 230);  
	}
}



/**
 * Schwenkt den Abstandssensor von Rechts zur Mitte, ermittelt, ob 
 * sich ein Hindernis im Weg befindet und 
 * weicht nötigenfalls in eine zufällige Richtung aus
 *
 * @param -
 *
 * @return -
 *
 * @example TurnSensorMiddleRight();
 */
void TurnSensorMiddleRight(void) {
	TurnSensor(MIDDLE);
	
	distanceSensorVal = GetAdc(DETECTION_SENSOR_ADC);
	
	if(distanceSensorVal >= DETECTION_RANGE) {
		uint8_t randValue = RandValue01();
		
		if(randValue == 0) { AvoidObstacle(LEFT); }
		if(randValue == 1) { AvoidObstacle(RIGHT); }
	}else{
		MotorFWD(220, 230);  
	}
}



/**
 * Umfährt ein Hindernis in angegebener Richtung
 *
 * @param dir Richtung, in die ausgewichen werden soll (LEFT/RIGHT)
 *
 * @return -
 *
 * @example AvoidObstacle(RIGHT);
 */
void AvoidObstacle(uint8_t dir) {
	// Solange drehen, bis kein Hindernis mehr erkannt wird
	while(distanceSensorVal >= DETECTION_RANGE){
		if(dir == LEFT) {
			MotorDir(RWD, FWD);
			MotorSpeed(220, 230);
		}else{
			MotorDir(FWD, RWD);
			MotorSpeed(220, 230);
		}
		
		distanceSensorVal = GetAdc(DETECTION_SENSOR_ADC);
		
		MSleep(1);
	}
}



/**
 * Prüft die Kollisionstaster ob ein Hindernis getroffen wurde
 * (Für Hindernisse die vom Abstandssensor nicht erfasst wurden oder
 * unterhalb des Erkennungsfeldes des Abstandssensors liegen)
 *
 * @param -
 *
 * @return -
 *
 * @example CheckCollisionSensor();
 */
void CheckCollisionSensor(void) {
	// Linker Taster
	if(!(PINC & (1 << POLLSWITCH_LEFT))) {
		MotorRWD(190, 200);
		MSleep(900);
		
		MotorDir(FWD, RWD);
		MotorSpeed(220, 230);
		MSleep(500);
	}
	
	// Rechter Taster
	if(!(PINC & (1 << POLLSWITCH_RIGHT))) {
		MotorRWD(190, 200);
		MSleep(900);
		
		MotorDir(RWD, FWD);
		MotorSpeed(220, 230);
		MSleep(500);
	}
}



/**
 * Timergesteuerte Sleep-Funktion
 * Wartet t ms
 *
 * @param t Zeit in Millisekunden die gewartet werden soll
 *
 * @return -
 *
 * @example MSleep(100); // Wartet 100 ms
 */
void MSleep(uint16_t t) {
	uint16_t z;
	
	for(z = 0; z < t; z++) {
		_delay_ms(1);
		
		//timeCount = 0;
		//while (timeCount < 36);
	}
}



/**
 * Erzeugt einen Zufallswert (0 oder 1)
 *
 * @param -
 *
 * @return Zufallswert
 *
 * @example uint8_t randVal = RandValue01();
 */
uint8_t RandValue01(void) {
	return (uint8_t) (rand() % 2);
}



/**
 * Hauptroutine
 *
 * @param -
 *
 * @return 0
 *
 * @example -
 */
int main(void) {
	// Prozessor initialisieren
	Init(); 
	
	// Programmschleife
	while(1) {
		TurnSensorLeft(); 
		TurnSensorMiddleLeft();
		TurnSensorRight();
		TurnSensorMiddleRight();
	}
	
	return 0;
}
