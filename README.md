# VL53L0X-HAL-API
Erstellt wurde eine leichtgewichtigere VL53L0X-API auf Basis der für [Arduino erstellten API von Pololu](https://github.com/pololu/vl53l0x-arduino) und der abgewandelten C-Version von [yetifrisstlama](https://github.com/yetifrisstlama/vl53l0x-non-arduino). Diese API wurde außerdem um die ST-HAL-Library erweitert und die Kommentare auf Deutsch übersetzt.
## Nächste Schritte
Im nächsten Verlauf wird die VL53L0X-HAL-API mit dem STM32F103RB getestet und entsprechend angepasst.

> Für Vorschläge und gefundene Fehler bin ich dankbar.


- [ ] Mit STM32F103RB zum laufen bringen
- [ ] Beispiel-Dateien aufführen
- [ ] Refaktorierung

### Zu überprüfende Abschnitte
#### readReg16Bit & readReg32Bit

##### Codeausschnitt

```c
/*
 *  Liest ein 16-Bit Register
 */
uint16_t readReg16Bit(uint8_t reg)
{
  HAL_I2C_Mem_Read(&hi2c, g_i2cAddr, reg, I2C_MEMADD_SIZE_8BIT,&data, 2, 100);
}

/*
 *  Liest ein 32-Bit Register
 */
uint32_t readReg32Bit(unint8_t reg)
{
  HAL_I2C_Mem_Read(&hi2c, g_i2cAddr, reg, I2C_MEMADD_SIZE_8BIT,&data, 4, 100);
}
```

##### Zu überprüfen

- Kann HAL_I2C_Mem_Read 16-Bit/32-Bit lesen?
- Muss ein Buffer eingerichtet werden?

#### writeMulti & readMulti

##### Codeausschnitt

```c
void writeMulti(uint8_t reg, uint8_t const *src, uint8_t count){
 i2c_start( g_i2cAddr | I2C_WRITE );
 i2c_write( reg );
 while ( count-- > 0 ) {
   i2c_write( *src++ );
 }
 i2c_stop();
}

void readMulti(uint8_t reg, uint8_t * dst, uint8_t count) {
 i2c_start( g_i2cAddr | I2C_WRITE );
 i2c_write( reg );
 i2c_rep_start( g_i2cAddr | I2C_READ );
 while ( count > 0 ) {
   if ( count > 1 ){
     *dst++ = i2c_readAck();
   } else {
     *dst++ = i2c_readNak();
   }
   count--;
 }
 i2c_stop();
}
```

##### Zu überprüfen

- Gibt es eine HAL Funktion dazu?
- Andere Möglichkeit als while() anstreben?
  - Performanceeinbußen?
