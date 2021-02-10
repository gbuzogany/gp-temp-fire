#ifndef _MAX31865_
#define _MAX31865_

typedef enum max31865_numwires {
  MAX31865_2WIRE = 0,
  MAX31865_3WIRE = 1,
  MAX31865_4WIRE = 0
} max31865_numwires_t;

ret_code_t max31865_spi_init(void);
void max31865_spi_uninit(void);
void max31865_init(void);
float max31865_temperature(float RTDnominal, float refResistor);

#endif // _MAX31865_