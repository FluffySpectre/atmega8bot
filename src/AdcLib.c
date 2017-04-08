/************************************************************************
 *
 * Datei:  			    AdcLib.c
 *							
 * Projekt:   			AtmegaBot
 * 
 * Beschreibung:	    Stellt Funktionen für die Auswertung 
 *						der ADC-Kanäle des Atmega8 bereit
 *
 *
 * Version    Datum          Autor                 Kommentare
 * -------  ----------   --------------   ------------------------------
 * 1.00	    10.07.2011   Björn Bosse	  Erstes Build
 *
 *
 ***********************************************************************/


#include "AdcLib.h"


/**
 * Gibt den Durchschnittswert eines Adc-Kanals zurück
 *
 * @param channel ADC-Kanal des Controllers
 *
 * @return Mittelwert des ADC-Kanals
 *
 * @example uint16_t adcVal = GetAdc(1);
 */
uint16_t GetAdc(uint8_t channel) {
  	uint8_t numOfSamples, i;
	uint16_t analogValue = 0, averageValue = 0;
	
	numOfSamples = 32;							// Anzahl der Samples
	i = numOfSamples;							// Samples kopieren, zum herunterzählen 
	
	ADCSRA = (1 << ADEN); 		            	// ADC einschalten, kein Prescale (0x80)
	ADMUX = channel;  			            	// ADC Ref auf Avcc, Kanal wählen, normale Formatierung
	
	while(i) {
		ADCSRA |= (1 << ADSC); 	    	    // 'Single Conversion Mode' ein
		while (ADCSRA & (1 << ADSC));			// Auf Abschluss der Konvertierung warten 
		
		analogValue += ADCW;
		i--;
	}
	
	ADCSRA = 0;									// ADC ausschalten um Strom zu sparen
	
	averageValue = analogValue / numOfSamples;  // Arithmetisches Mittel der Werte berechnen

	return averageValue;
}