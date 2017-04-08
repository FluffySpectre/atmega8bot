/************************************************************************
 *
 * Datei:  			    RoboControl.h
 *							
 * Projekt:   			AtmegaBot
 * 
 * Beschreibung:	    Hauptprogramm zur Steuerung des Roboters
 *
 *
 * Version    Datum          Autor                 Kommentare
 * -------  ----------   --------------   ------------------------------
 * 1.00	    10.07.2011   Björn Bosse	  Erstes Build
 *
 *
 ***********************************************************************/


#ifndef ROBOCONTROL_H
#define ROBOCONTROL_H

#define F_CPU 8000000UL			      // Interner 8 MHz Takt
//#define F_CPU 3686400			      // Quarz mit 3.6864 MHz

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <util/delay.h>
#include "AdcLib.h"


#define OFF			        	0	  // 
#define ON  		   			1	  // 

#define RWD 		   			0	  // 
#define FWD            			1	  // 

#define STATUS_LED        		PB0	  // Pin der Status-LED
#define POLLSWITCH_LEFT   		PC0	  // Pin des linken Kollisionstasters
#define POLLSWITCH_RIGHT  		PC1	  // Pin des rechten Kollisionstasters
#define SERVO             		PC5	  // Pin des Servos

#define LEFT        	  		1.0	  // Servoanschlag links
#define MIDDLE      	 	    1.5	  // Servoanschlag mitte
#define RIGHT       	  	    2.0	  // Servoanschlag rechts

#define DETECTION_SENSOR_ADC      4   // Kanal, wo der Abstandssensor angeschlossen ist
#define DETECTION_RANGE         250	  // Erkennungsreichweite des optischen Abstandssensors


// Funktionsdeklaration

void Init(void);
inline void StatusLED(uint8_t status);
void MotorDir(uint8_t leftDir, uint8_t rightDir);
inline void MotorSpeed(uint8_t leftSpeed, uint8_t rightSpeed);
void MotorFWD(uint8_t leftSpeed, uint8_t rightSpeed);
void MotorRWD(uint8_t leftSpeed, uint8_t rightSpeed);
//inline void MotorStop(void);
void TurnSensor(float angle);
void TurnSensorLeft(void);
void TurnSensorRight(void);
void TurnSensorMiddleLeft(void);
void TurnSensorMiddleRight(void);
void AvoidObstacle(uint8_t dir);
void CheckCollisionSensor(void);
void MSleep(uint16_t t);
uint8_t RandValue01(void);

#endif // ROBOCONTROL_H