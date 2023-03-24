#include "Arduino.h"

#ifndef at24c256_h
#define at24c256_h
//---------------------------------------------------------------------------------------------------------
class at24c256 {
  public:
    at24c256(uint8_t);                            // EEPROM (адрес)
    void init();                                  // инициализация на библиотеката
    void write(uint16_t m_addr, uint8_t m_data);  // записва клетка на адрес m_addr с данни m_data
    void update(uint16_t m_addr, uint8_t m_data); // чете клетка и записва данни само при разлика
    uint8_t read(uint16_t m_addr);                // чете съдържанието на клетка с адрес m_addr
  private:
    uint8_t m_chip_addr;                          // I2C адрес
};
//---------------------------------------------------------------------------------------------------------
#endif
