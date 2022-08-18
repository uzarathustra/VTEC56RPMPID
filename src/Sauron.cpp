/*
  Name:    Sauron.cpp
  Created: 01.06.2016
  Author:  Roman Schönfeld & Daniel Hausherr
*/

/***************************************************************************************************
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

#include "Sauron.h"

/***************************************************************************************************
**  Constanten
***************************************************************************************************/
static const int I2C_ADDRESS = 0x74;

/***************************************************************************************************
**  locale Functions
***************************************************************************************************/
static int8_t i2c_write_register_sauron(uint8_t *conf, uint8_t b_size);
static int8_t i2c_read_register_sauron(uint8_t *value, uint8_t b_size, uint8_t reg_number);

/***************************************************************************************************
**  external Functions
***************************************************************************************************/
// Initialisiert I2C
void init_i2c() {
  Wire.begin();
}

// Verarbeitet Parameter und schreibt sie ins Register
int8_t set_config(uint8_t  tint, uint8_t iref) {
  uint8_t configx[] = { _CREGL, 0,  _CMD }, configy[] = { _OSR, 0};

   configy[1] = _CONFIG_MODE;

  // versetzt den Mazet in den Configmodus
  if (i2c_write_register_sauron(configy, sizeof(configy))) // ungleich 0 no succes
    return -1;

  configx[1] = ((iref << 4) | tint);

  if (i2c_write_register_sauron(configx, sizeof(configx)))  // ungleich 0 no succes
    return -1;
     
  return 0;
}

// Beginnt oder Beendet Messung
int8_t set_mode(bool mode) {
  uint8_t configx[] = { _OSR, 0};

  if (mode)
    configx[1] = _MEASUREMENT_MODE;
  else
    configx[1] = _CONFIG_MODE;

  if (i2c_write_register_sauron(configx, sizeof(configx))) // ungleich 0 no succes (-1)
    return -1;

  return 0;
}

// Zum Auslesen der Messdaten
int8_t get_data(uint16_t *value, uint8_t b_size) {
  uint8_t data[b_size], i = 0;

  if (i2c_read_register_sauron(data, b_size, _OUT) != 0) // ungleich 0 no succes (-1)
    return -1;

  for (i = 0; i < (b_size / 2); i++)
    value[i] = (uint16_t)(((data[(2 * i) + 1]) << 8) | data[2 * i]);

  return 0;
}

// Zum Auslesen der Register
int8_t get_config(uint16_t *value, uint8_t b_size) {
  uint8_t data[b_size], i = 0;

  if (i2c_read_register_sauron(data, b_size, _CREGL) != 0) // ungleich 0 no succes (-1)
    return -1;

  for (i = 0; i < b_size; i++)
    value[i] = (uint16_t)data[i];

  return 0;
}


/***************************************************************************************************
**  internal Functions
***************************************************************************************************/
/***************************************************************************************************
**  i2c_write_register_sauron
**
**  schreibt in die Register des Saurons
**
**  Input:   command Buffer beinhaltet die Registerwerte, size Größe des Buffers
**  Output: -1 - No Succes
**       0 - Succes
***************************************************************************************************/
static int8_t i2c_write_register_sauron(uint8_t *conf, uint8_t b_size ) {

  Wire.beginTransmission(I2C_ADDRESS);
  Wire.write(conf, b_size);

  return (int8_t)Wire.endTransmission(); // ungleich 0 no succes
}

/***************************************************************************************************
**  i2c_read_register_sauron
**
**  liest aus den Registern des Saurons
**
**  Input:   value Buffer zum einlesen der Werte, t_size Größe des Buffers, register_number Register Nummer aus dem gelesen werden soll
**  Output: -1 - No Succes
**       0 - Succes
***************************************************************************************************/
static int8_t i2c_read_register_sauron(uint8_t *data, uint8_t b_size, uint8_t reg_number) {
  uint16_t i2c_read_timeouts = 0, i = 0;
  uint8_t reg[] = { reg_number };

  if (i2c_write_register_sauron(reg, sizeof(reg)) != 0) // ungleich 0 no succes
    return -1;

  Wire.requestFrom(I2C_ADDRESS, b_size);

  while (Wire.available() < b_size) {
    i2c_read_timeouts++;
    if (i2c_read_timeouts > 100) {
      return -1;
    }
    else {
      delay(1);
    }
  }
  for (i = 0; i < b_size; i++) {
    data[i] = (uint8_t)Wire.read();
  }
  return 0;
}
