/*
  Name:    Sauron.h
  Created: 01.06.2016
  Author:  Roman Schönfeld & Daniel Hausherr
*/

/***********************************************************************************************
  This work is licensed under a Creative Commons Attribution 4.0 International (CC BY 4.0) License.

  You are free to:

  Share — copy and redistribute the material in any medium or format
  Adapt — remix, transform, and build upon the material
  for any purpose, even commercially.

  The licensor cannot revoke these freedoms as long as you follow the license terms.

  Under the following terms:

  Attribution — You must give appropriate credit, provide a link to the license, and indicate if
  changes were made. You may do so in any reasonable manner, but not in any way that suggests
  the licensor endorses you or your use.

  No additional restrictions — You may not apply legal terms or technological measures that
  legally restrict others from doing anything the license permits.

  Credits:
  Laboratory for Physics and Illumination Technologies,
  University of Applied Sciences Southern Westfalia
  Germany

  Make Light! Support by the BMBF in the development of this device is gratefully acknowledged
***************************************************************************************************/
#ifndef _SAURON_h
#define _SAURON_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include <Wire.h>

/***************************************************************************************************
**  Define Constants
**
**  Define directives providing meaningful names for constant values.
***************************************************************************************************/
#define _READY_PIN			    A6		// definiert den ready pin des Saurons zur Abfrage, ob Messwerte verfügbar sind
#define _OSR				    0		// Operational State Register 
#define _OUT                    0     	// Operational State Register 
#define _CMD				    8		// Registereintrag = 8b im Register CREGH (Command Register High Byte), versetzt den Sauron in den Kommandmodus und deaktiviert den Hardwareteiler
#define _CREGL				    6		// Register CREGL (Command Register Low Byte)
#define _CREGH                  7     	// Register CREGH (Command Register HIGH Byte)
#define _CONFIG_MODE		    2		// Registereintrag = 2b, im OSR der Sauron befindet sich im Konfigurationsmodus + Stopmodus
#define _MEASUREMENT_MODE	    131		// Registereintrag = 131b, im OSR der Sauron befindet sich im Messmodus + Startmodus
										// Für weitere Informationen siehe Datenblatt "Mazet MCDC04"
                               

typedef enum {
  TINT_MS_1 = 0,
  TINT_MS_2,
  TINT_MS_4,
  TINT_MS_8,
  TINT_MS_16,
  TINT_MS_32,
  TINT_MS_64,
  TINT_MS_128,
  TINT_MS_256,
  TINT_MS_512,
  TINT_MS_1024,

  TINT_COUNT
} integrationTime;

typedef enum {
  IREF_NA_20 = 0,
  IREF_NA_80,
  IREF_NA_320,
  IREF_NA_1280,
  IREF_NA_5120,

  IREF_COUNT
} referenceCurrent;

typedef enum {
  CH1 = 0,
  CH2,
  CH3,
  CH4,

  CH_COUNT
} outputData; 


extern integrationTime current_time;
extern referenceCurrent current_ref;
extern outputData current_data_output;


/***************************************************************************************************
**  init_i2c
**
**  Initiate the Wire library and join the I2C bus as a master or slave. This should normally be called only once.
**
**  Input:  None
**  Output: None
***************************************************************************************************/
extern void init_i2c();

/***************************************************************************************************
**  set_mode
**
**  beginnt und beendet eine Messung
**
**  Input:   mode TRUE falls Messung starten und FALSE falls Messung beenden
**  Output: -1 - No Succes
**       0 - Succes
***************************************************************************************************/
extern int8_t set_mode(bool mode);

/***************************************************************************************************
**  set_config
**
**  verarbeitet und sendet die Parameter "Integrationszeit" und "Messbereich" an den Sauron
**
**  Input:  tint Integrationszeit, iref Messbereich,  verarbeitete *dev_tint Integrationszeit und *dev_iref Messbereich
**  Output:	-1 - No Succes
**			 0 - Succes
***************************************************************************************************/
extern int8_t set_config(uint8_t  tint, uint8_t iref);

/***************************************************************************************************
**  get_config
**
**  liest die aktuell gesetzten Parameter aus dem Sauron
**
**  Input:	 value Buffer zum einlesen der Werte, size Größe des Buffers
**  Output:	-1 - No Succes
**			 0 - Succes
***************************************************************************************************/
extern int8_t get_config(uint16_t *value, uint8_t b_size);

/***************************************************************************************************
**  get_data
**
**  liest die Messdaten aus
**
**  Input:	 value Buffer zum einlesen der Werte, size Größe des Buffers
**  Output:	-1 - No Succes
**			 0 - Succes
***************************************************************************************************/
extern int8_t get_data(uint16_t *data, uint8_t b_size);
#endif

