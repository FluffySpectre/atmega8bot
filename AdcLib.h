/************************************************************************
 *
 * Datei:  			    AdcLib.h
 *							
 * Projekt:   			AtmegaBot
 * 
 * Beschreibung:	    Stellt Funktionen f�r die Auswertung 
 *						der ADC-Kan�le des Atmega8 bereit
 *
 *
 * Version    Datum          Autor                 Kommentare
 * -------  ----------   --------------   ------------------------------
 * 1.00	    16.01.2011   Bj�rn Bosse	  Erstes Build
 *
 *
 ***********************************************************************/


#ifndef ADCLIB_H
#define ADCLIB_H

#include <avr/io.h>


// Funktionsdeklaration
uint16_t GetAdc(uint8_t channel);


#endif // ADCLIB_H
