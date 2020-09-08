/*
 *      File: C_VL53L0X.c
 *
 *      @brief  Umgeschriebene C-Datei von Pololu die
 *              auf der ST-API für den VL53L0X basiert.
 *
 *      @note   Pololu-API ist für Arduino ausgelegt.
 *              Mehr zur Pololu-API auf:
 *              https://github.com/pololu/vl53l0x-arduino
 * 
 *              Nachtrag [01.09.2020]
 *              Außerdem noch gefunden und verwendet: 
 *              https://github.com/yetifrisstlama/vl53l0x-non-arduino
 */

/*  Includes  */
#include "stm32f1xx_hal.h"

/*  Benutzer Includes */
#include "C_VL53L0X.h"
//#include <stdbool.h>


/*  Zu ersetzende Bibliotheken -----------------------------------------------------------! */
#include <stdint.h>
#include "i2cmaster.h"
#include "millis.h"
#include "VL53L0X.h"

/*
 * ACHTUNG: In C++ wird die implementierung der Memberfunktion mit CLASS::Funktion ausgeführt.
 *          Hierführ muss in C eine neue Funktion geschaffen werden die zuvor in der Header-Datei
 *          deklariert wurde.
 */

/* Lokale Variablen (private) */
uint8_t g_i2cAddr = ADDRESS_DEFAULT;
uint16_t g_ioTimeout = 0;  // kein timeout
uint8_t g_isTimeout = 0;
uint16_t g_timeoutStartMs;
uint8_t g_stopVariable; // von init gelesen und beim Starten der Messung verwendet
uint32_t g_measTimBudUs;



/* Lokale Funktionen (private) */
bool getSpadInfo(uint8_t *count, bool *type_is_aperture);
void getSequenceStepEnables(SequenceStepEnables * enables);
void getSequenceStepTimeouts(SequenceStepEnables const * enables, SequenceStepTimeouts * timeouts);
bool performSingleRefCalibration(uint8_t vhv_init_byte);
static uint16_t decodeTimeout(uint16_t value);
static uint16_t encodeTimeout(uint16_t timeout_mclks);
static uint32_t timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks);
static uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks);

/* I2C */
I2C_HandleTypeDef hi2c2;

/*
 *  Schreibt 8-Bit Register
 */
void writeReg(uint8_t reg, uint8_t value)
{
  HAL_I2C_Mem_Write(&hi2c, g_i2cAddr, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 100);
}

/*
 *  Schreibt 16-Bit Register
 */
void writeReg16Bit(uint8_t reg, uint16_t value)
{
  uint8_t value_array[1];

  value_array[0] = ((value >> 8) & 0xFF);
  value_array[1] = ((value     ) & 0xFF);

  HAL_I2C_Mem_Write(&hi2c, g_i2cAddr, reg, I2C_MEMADD_SIZE_8BIT, value_array, 2, 100);
}

/*
 *  Schreibt 32-Bit Register
 */
 void writeReg32Bit(unint8_t reg, unint32_t value)
 {
   uint8_t value_array[4];

   value_array[0] = (value >> 24) & 0xFF;
   value_array[1] = (value >> 16) & 0xFF;
   value_array[2] = (value >>  8) & 0xFF;
   value_array[3] = (value      ) & 0xFF;

   HAL_I2C_Mem_Write(&hi2c, g_i2cAddr, reg, I2C_MEMADD_SIZE_8BIT, value_array, 4, 100);
 }

/*
 *  Liest ein 8-Bit Register
 */
uint8_t readReg(uint8_t reg)
{
  HAL_I2C_Mem_Read(&hi2c, g_i2cAddr, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 100)
}

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

//_____________________________________________________________________________________________________________________________________

/*
 *  Schreibt eine beliebige Anzahl von Bytes aus dem gegebenen
 *  Array in den Sensor, beginnend mit dem gegebenen Register
 */
void writeMulti(uint8_t reg, uint8_t const *src, uint8_t count){
 i2c_start( g_i2cAddr | I2C_WRITE );
 i2c_write( reg );
 while ( count-- > 0 ) {
   i2c_write( *src++ );
 }
 i2c_stop();
}

/*
 * Liest eine beliebige Anzahl von Bytes aus dem Sensor,
 * beginnend mit dem angegebenen Register, in das angegebene Array
 */
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


//--------------------- Public Methoden -------------------------------------------

void setAddress(uint8_t new_addr) {
 writeReg( I2C_SLAVE_DEVICE_ADDRESS, (new_addr>>1) & 0x7F );
 g_i2cAddr = new_addr;
}

uint8_t getAddress() {
 return g_i2cAddr;
}

/*
 *  Initialisiert den Sensor mit einer Sequenz basierend auf VL53L0X_DataInit(),
 *  VL53L0X_StaticInit() und VL53L0X_PerformRefCalibration().
 *  Diese Funktion führt keine SPAD-Referenzkalibrierung
 *  (VL53L0X_PerformRefSpadManagement()) durch, da im API-Benutzerhandbuch steht,
 *  dass sie von ST auf den blanken Modulen durchgeführt wird; es scheint, dass dies
 *  gut genug funktionieren sollte, wenn kein Deckglas (Durchsichtige Abdeckung)
 *  hinzugefügt wird.
 *  Wenn io_2v8 (optional) wahr ist oder nicht angegeben wird, ist der Sensor
 *  für den 2V8-Modus konfiguriert.
 */
bool initVL53L0X( bool io_2v8 ){
 // Beginnt mit VL53L0X_DataInit()

 // Sensor verwendet den 1V8 Modus für I/O standardmäßig
 // Wenn nötig in 2V8 Modus wechseln
 if (io_2v8)
 {
   writeReg(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV,
     readReg(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV) | 0x01); // set bit 0
 }

 // Setzt den I2C Standard Modus
 writeReg(0x88, 0x00);

 writeReg(0x80, 0x01);
 writeReg(0xFF, 0x01);
 writeReg(0x00, 0x00);
 g_stopVariable = readReg(0x91);
 writeReg(0x00, 0x01);
 writeReg(0xFF, 0x00);
 writeReg(0x80, 0x00);

 // Deaktiviert die Grenzwertprüfungen (limit checks) SIGNAL_RATE_MSRC (bit 1)
 // und SIGNAL_RATE_PRE_RANGE (bit 4)
 writeReg(MSRC_CONFIG_CONTROL, readReg(MSRC_CONFIG_CONTROL) | 0x12);

 // Setzt die Grenze der Signalrate im Endbereich
 // auf 0,25 MCPS (million counts per second)
 setSignalRateLimit(0.25);

 writeReg(SYSTEM_SEQUENCE_CONFIG, 0xFF);

 // Ende VL53L0X_DataInit()

 // Beginnt mit VL53L0X_StaticInit()

 uint8_t spad_count;
 bool spad_type_is_aperture;
 if (!getSpadInfo(&spad_count, &spad_type_is_aperture)) { return false; }

// Die SPAD-Map (RefGoodSpadMap) wird von VL53L0X_get_info_from_device()
// in der API gelesen, aber die gleichen Daten scheinen von
// GLOBAL_CONFIG_SPAD_ENABLES_REF_0 bis _6 leichter lesbar zu sein,
// also werden diese von dort gelesen.
 uint8_t ref_spad_map[6];
 readMulti(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

 // -- Beginnt mit VL53L0X_set_reference_spads() (sofern die NVM-Werte gültig sind)

 writeReg(0xFF, 0x01);
 writeReg(DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
 writeReg(DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
 writeReg(0xFF, 0x00);
 writeReg(GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

 uint8_t first_spad_to_enable = spad_type_is_aperture ? 12 : 0; // 12 ist die erste "aperture spad"
 uint8_t spads_enabled = 0;

 for (uint8_t i = 0; i < 48; i++)
 {
   if (i < first_spad_to_enable || spads_enabled == spad_count)
   {
     // Dieses Bit ist niedriger als das erste, das aktiviert werden sollte,
     // oder (reference_spad_count)-Bits wurden bereits aktiviert, so dass
     // dieses Bit auf Null gesetzt wird.
     ref_spad_map[i / 8] &= ~(1 << (i % 8));
   }
   else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1)
   {
     spads_enabled++;
   }
 }

 writeMulti(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

 // -- Ende VL53L0X_set_reference_spads()

 // -- Beginnt mit VL53L0X_load_tuning_settings()
 // DefaultTuningSettings aus der vl53l0x_tuning.h

 writeReg(0xFF, 0x01);
 writeReg(0x00, 0x00);

 writeReg(0xFF, 0x00);
 writeReg(0x09, 0x00);
 writeReg(0x10, 0x00);
 writeReg(0x11, 0x00);

 writeReg(0x24, 0x01);
 writeReg(0x25, 0xFF);
 writeReg(0x75, 0x00);

 writeReg(0xFF, 0x01);
 writeReg(0x4E, 0x2C);
 writeReg(0x48, 0x00);
 writeReg(0x30, 0x20);

 writeReg(0xFF, 0x00);
 writeReg(0x30, 0x09);
 writeReg(0x54, 0x00);
 writeReg(0x31, 0x04);
 writeReg(0x32, 0x03);
 writeReg(0x40, 0x83);
 writeReg(0x46, 0x25);
 writeReg(0x60, 0x00);
 writeReg(0x27, 0x00);
 writeReg(0x50, 0x06);
 writeReg(0x51, 0x00);
 writeReg(0x52, 0x96);
 writeReg(0x56, 0x08);
 writeReg(0x57, 0x30);
 writeReg(0x61, 0x00);
 writeReg(0x62, 0x00);
 writeReg(0x64, 0x00);
 writeReg(0x65, 0x00);
 writeReg(0x66, 0xA0);

 writeReg(0xFF, 0x01);
 writeReg(0x22, 0x32);
 writeReg(0x47, 0x14);
 writeReg(0x49, 0xFF);
 writeReg(0x4A, 0x00);

 writeReg(0xFF, 0x00);
 writeReg(0x7A, 0x0A);
 writeReg(0x7B, 0x00);
 writeReg(0x78, 0x21);

 writeReg(0xFF, 0x01);
 writeReg(0x23, 0x34);
 writeReg(0x42, 0x00);
 writeReg(0x44, 0xFF);
 writeReg(0x45, 0x26);
 writeReg(0x46, 0x05);
 writeReg(0x40, 0x40);
 writeReg(0x0E, 0x06);
 writeReg(0x20, 0x1A);
 writeReg(0x43, 0x40);

 writeReg(0xFF, 0x00);
 writeReg(0x34, 0x03);
 writeReg(0x35, 0x44);

 writeReg(0xFF, 0x01);
 writeReg(0x31, 0x04);
 writeReg(0x4B, 0x09);
 writeReg(0x4C, 0x05);
 writeReg(0x4D, 0x04);

 writeReg(0xFF, 0x00);
 writeReg(0x44, 0x00);
 writeReg(0x45, 0x20);
 writeReg(0x47, 0x08);
 writeReg(0x48, 0x28);
 writeReg(0x67, 0x00);
 writeReg(0x70, 0x04);
 writeReg(0x71, 0x01);
 writeReg(0x72, 0xFE);
 writeReg(0x76, 0x00);
 writeReg(0x77, 0x00);

 writeReg(0xFF, 0x01);
 writeReg(0x0D, 0x01);

 writeReg(0xFF, 0x00);
 writeReg(0x80, 0x01);
 writeReg(0x01, 0xF8);

 writeReg(0xFF, 0x01);
 writeReg(0x8E, 0x01);
 writeReg(0x00, 0x01);
 writeReg(0xFF, 0x00);
 writeReg(0x80, 0x00);

 // -- Ende VL53L0X_load_tuning_settings()

 // "Interrupt-Konfiguration auf 'Neue Probe bereit' setzen"
 // -- Beginnt mit VL53L0X_SetGpioConfig()

 writeReg(SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
 writeReg(GPIO_HV_MUX_ACTIVE_HIGH, readReg(GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10); // active low
 writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01);

 // -- Ende VL53L0X_SetGpioConfig()

 g_measTimBudUs = getMeasurementTimingBudget();

 // MSRC und TCC standardmäßig deaktiviert
 // MSRC = Minimum Signal Rate Check
 // TCC = Target CentreCheck

 // -- Beginnt mit VL53L0X_SetSequenceStepEnable()

 writeReg(SYSTEM_SEQUENCE_CONFIG, 0xE8);

 // -- Ende VL53L0X_SetSequenceStepEnable()

 // Rekalkulation von timing budget
 setMeasurementTimingBudget(g_measTimBudUs);

 // Ende VL53L0X_StaticInit()

 // Beginnt mit VL53L0X_PerformRefCalibration() (VL53L0X_perform_ref_calibration())

 // -- Beginnt mit VL53L0X_perform_vhv_calibration()

 writeReg(SYSTEM_SEQUENCE_CONFIG, 0x01);
 if (!performSingleRefCalibration(0x40)) { return false; }

 // -- Ende VL53L0X_perform_vhv_calibration()

 // -- Beginnt mit VL53L0X_perform_phase_calibration()

 writeReg(SYSTEM_SEQUENCE_CONFIG, 0x02);
 if (!performSingleRefCalibration(0x00)) { return false; }

 // -- Ende VL53L0X_perform_phase_calibration()

 // "Stellt die vorherige Sequenzkonfiguration wieder her"
 writeReg(SYSTEM_SEQUENCE_CONFIG, 0xE8);

 // Ende VL53L0X_PerformRefCalibration()

 return true;
}


// Setzt den Grenzwert für die Rücksignalratenprüfung in Einheiten
// von MCPS (mega counts per second). "Dies stellt die Amplitude des
// Signals dar, das vom Ziel reflektiert und vom Gerät erkannt wird";
// die Einstellung dieses Grenzwertes bestimmt vermutlich die minimale
// Messung, die erforderlich ist, damit der Sensor einen gültigen
// Messwert meldet.
// Die Einstellung eines unteren Grenzwertes erhöht den möglichen
// Bereich des Sensors, aber scheint die Wahrscheinlichkeit zu erhöhen,
// eine ungenaue Messung zu erhalten, aufgrund von unerwünschte
// Reflexionen anderer Objekten als dem beabsichtigten Ziel.
// Standardwert ist 0,25 MCPS, wie von ST-API und dieser Bibliothek
// initialisiert wurde.


bool setSignalRateLimit(float limit_Mcps)
{
 if (limit_Mcps < 0 || limit_Mcps > 511.99) { return false; }

 // Q9.7 fixed point format (9 integer bits, 7 fractional bits)
 writeReg16Bit(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, limit_Mcps * (1 << 7));
 return true;
}

// Get the return signal rate limit check value in MCPS
float getSignalRateLimit(void)
{
 return (float)readReg16Bit(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT) / (1 << 7);
}

// Legt das Zeitbudget für die Messung in Mikrosekunden fest, d.h. die
// für eine Messung zulässige Zeit; die ST API und diese Bibliothek kümmern
// sich um die Aufteilung des Zeitbudgets auf die Teilschritte in der
// Messbereichssequenz. Ein längeres Zeitbudget ermöglicht genauere Messungen.
// Eine Erhöhung des Budgets um den Faktor N verringert die Standardabweichung
// der Bereichsmessung um einen Faktor von sqrt(N). Der Standardwert liegt bei
// etwa 33 Millisekunden; das Minimum beträgt 20 ms. basierend auf
// VL53L0X_set_measurement_timing_budget_micro_seconds()

bool setMeasurementTimingBudget(uint32_t budget_us)
{
 SequenceStepEnables enables;
 SequenceStepTimeouts timeouts;

 uint16_t const StartOverhead      = 1320; // Bitte berücksichtigen, dass dies ein anderer Wert ist als der in get_
 uint16_t const EndOverhead        = 960;
 uint16_t const MsrcOverhead       = 660;
 uint16_t const TccOverhead        = 590;
 uint16_t const DssOverhead        = 690;
 uint16_t const PreRangeOverhead   = 660;
 uint16_t const FinalRangeOverhead = 550;

 uint32_t const MinTimingBudget = 20000;

 if (budget_us < MinTimingBudget) { return false; }

 uint32_t used_budget_us = StartOverhead + EndOverhead;

 getSequenceStepEnables(&enables);
 getSequenceStepTimeouts(&enables, &timeouts);

 if (enables.tcc)
 {
   used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
 }

 if (enables.dss)
 {
   used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
 }
 else if (enables.msrc)
 {
   used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
 }

 if (enables.pre_range)
 {
   used_budget_us += (timeouts.pre_range_us + PreRangeOverhead);
 }

 if (enables.final_range)
 {
   used_budget_us += FinalRangeOverhead;

     // "Zu Beachten ist, dass die endgültige Bereichszeitüberschreitung durch
     // das Zeitbudget und die Summe aller anderen Zeitüberschreitungen innerhalb
     // der Sequenz bestimmt wird. Wenn es keinen Platz für den endgültigen
     // Bereichs-Timeout gibt, wird ein Fehler gesetzt. Andernfalls wird die
     // verbleibende Zeit auf den endgültigen Bereich angewendet."

   if (used_budget_us > budget_us)
   {
     // "Geforderter Timeout zu groß."
     return false;
   }

   uint32_t final_range_timeout_us = budget_us - used_budget_us;

   // set_sequence_step_timeout() beginnt
   //(SequenzSchrittId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)
   //
   // "Für die endgültige Bereichszeitüberschreitung muss die
   // Zeitüberschreitung vor der Bereichsüberschreitung hinzugefügt
   // werden. Dazu müssen sowohl die Zeitüberschreitungen im End-
   // als auch im Vorbereich in Makroperioden MClks ausgedrückt werden,
   // da sie unterschiedliche vcsel-Perioden haben.

   uint16_t final_range_timeout_mclks =
     timeoutMicrosecondsToMclks(final_range_timeout_us,
                                timeouts.final_range_vcsel_period_pclks);

   if (enables.pre_range)
   {
     final_range_timeout_mclks += timeouts.pre_range_mclks;
   }

   writeReg16Bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
     encodeTimeout(final_range_timeout_mclks));

   // Ende set_sequence_step_timeout()

   g_measTimBudUs = budget_us; // zur internen Wiederverwendung speichern
 }
 return true;
}

// Erhält das Zeitbudget für die Messung in Mikrosekunden, basierend auf
// VL53L0X_get_measurement_timing_budget_micro_seconds() in us
uint32_t getMeasurementTimingBudget(void)
{
 SequenceStepEnables enables;
 SequenceStepTimeouts timeouts;

 uint16_t const StartOverhead     = 1910; // Bitte berücksichtigen, dass dies ein anderer Wert ist als der in set_
 uint16_t const EndOverhead        = 960;
 uint16_t const MsrcOverhead       = 660;
 uint16_t const TccOverhead        = 590;
 uint16_t const DssOverhead        = 690;
 uint16_t const PreRangeOverhead   = 660;
 uint16_t const FinalRangeOverhead = 550;

 // "Start- und End-Overhead-Zeiten immer vorhanden"
 uint32_t budget_us = StartOverhead + EndOverhead;

 getSequenceStepEnables(&enables);
 getSequenceStepTimeouts(&enables, &timeouts);

 if (enables.tcc)
 {
   budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
 }

 if (enables.dss)
 {
   budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
 }
 else if (enables.msrc)
 {
   budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
 }

 if (enables.pre_range)
 {
   budget_us += (timeouts.pre_range_us + PreRangeOverhead);
 }

 if (enables.final_range)
 {
   budget_us += (timeouts.final_range_us + FinalRangeOverhead);
 }

 g_measTimBudUs = budget_us; // zur internen Wiederverwendung speichern
 return budget_us;
}

// Setzt die VCSEL-Pulsperiode (VCSEL = vertical cavity surface emitting laser)
// für den gegebenen Periodentyp (pre-range oder final-range) auf den gegebenen
// Wert in PCLKs.
// Längere Perioden scheinen den potentiellen Bereich des Sensors zu vergrößern.
// Gültige Werte sind (nur gerade Zahlen):
//  pre: 12 bis 18 (initialisierte Voreinstellung: 14)
//  final: 8 bis 14 (initialisierte Vorgabe: 10)
// basierend auf VL53L0X_set_vcsel_pulse_period()
bool setVcselPulsePeriod(vcselPeriodType type, uint8_t period_pclks)
{
 uint8_t vcsel_period_reg = encodeVcselPeriod(period_pclks);

 SequenceStepEnables enables;
 SequenceStepTimeouts timeouts;

 getSequenceStepEnables(&enables);
 getSequenceStepTimeouts(&enables, &timeouts);

 // "Spezifische Einstellungen für die angeforderte Taktperiode anwenden"
 // "Timeouts neu berechnen und anwenden, in Makro-Perioden"
 //
 // "Wenn die VCSEL-Periode für den pre- oder final-Bereich geändert wird,
 // muss die entsprechende Zeitüberschreitung mit der aktuellen VCSEL-Periode
 // aus dem Gerät ausgelesen werden, dann kann die neue VCSEL-Periode angewendet werden. Die Zeitüberschreitung muss dann unter Verwendung der neuen
 // VCSEL-Periode in das Gerät zurückgeschrieben werden.
 //
 // Für das MSRC-Timeout gilt das Gleiche - dieser Timeout ist abhängig von
 // der VCSEL-Periode vor dem Bereich".

 if (type == VcselPeriodPreRange)
 {
   // "Set phase check limits"
   switch (period_pclks)
   {
     case 12:
       writeReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18);
       break;

     case 14:
       writeReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30);
       break;

     case 16:
       writeReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40);
       break;

     case 18:
       writeReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50);
       break;

     default:
       // ungültige Periode
       return false;
   }
   writeReg(PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);

   // neue VCSEL-Periode anwenden
   writeReg(PRE_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

   // Update Timeouts

   // Beginnt set_sequence_step_timeout()
   // (SequenceStepId == VL53L0X_SEQUENCESTEP_PRE_RANGE)

   uint16_t new_pre_range_timeout_mclks =
     timeoutMicrosecondsToMclks(timeouts.pre_range_us, period_pclks);

   writeReg16Bit(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI,
     encodeTimeout(new_pre_range_timeout_mclks));

   // Ende set_sequence_step_timeout()

   // Beginnt set_sequence_step_timeout()
   // (SequenceStepId == VL53L0X_SEQUENCESTEP_MSRC)

   uint16_t new_msrc_timeout_mclks =
     timeoutMicrosecondsToMclks(timeouts.msrc_dss_tcc_us, period_pclks);

   writeReg(MSRC_CONFIG_TIMEOUT_MACROP,
     (new_msrc_timeout_mclks > 256) ? 255 : (new_msrc_timeout_mclks - 1));

   // Ende set_sequence_step_timeout()
 }
 else if (type == VcselPeriodFinalRange)
 {
   switch (period_pclks)
   {
     case 8:
       writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10);
       writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
       writeReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x02);
       writeReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C);
       writeReg(0xFF, 0x01);
       writeReg(ALGO_PHASECAL_LIM, 0x30);
       writeReg(0xFF, 0x00);
       break;

     case 10:
       writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28);
       writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
       writeReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
       writeReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09);
       writeReg(0xFF, 0x01);
       writeReg(ALGO_PHASECAL_LIM, 0x20);
       writeReg(0xFF, 0x00);
       break;

     case 12:
       writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
       writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
       writeReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
       writeReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08);
       writeReg(0xFF, 0x01);
       writeReg(ALGO_PHASECAL_LIM, 0x20);
       writeReg(0xFF, 0x00);
       break;

     case 14:
       writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48);
       writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
       writeReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
       writeReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07);
       writeReg(0xFF, 0x01);
       writeReg(ALGO_PHASECAL_LIM, 0x20);
       writeReg(0xFF, 0x00);
       break;

     default:
       // ungültige Periode
       return false;
   }

   // neue VCSEL-Periode anwenden
   writeReg(FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

   // Update Timeouts

   // Beginnt set_sequence_step_timeout()
   // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

    // "Für die endgültige Bereichszeitüberschreitung muss die
    // Zeitüberschreitung vor der Bereichsüberschreitung hinzugefügt
    // werden. Dazu müssen sowohl die Zeitüberschreitungen im
    // final-Bereich als auch im pre-Bereich in Makroperioden MClks
    // ausgedrückt werden, da sie unterschiedliche vcsel-Perioden haben.""

   uint16_t new_final_range_timeout_mclks =
     timeoutMicrosecondsToMclks(timeouts.final_range_us, period_pclks);

   if (enables.pre_range)
   {
     new_final_range_timeout_mclks += timeouts.pre_range_mclks;
   }

   writeReg16Bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
     encodeTimeout(new_final_range_timeout_mclks));

   // Ende set_sequence_step_timeout
 }
 else
 {
   // Ungültige Type
   return false;
 }

 // "Schließlich muss das Zeitbudget wieder eingesetzt werden"

 setMeasurementTimingBudget(g_measTimBudUs);

 // "Führt die Phasenkalibrierung durch. Dies ist nach dem Wechsel
 // auf vcsel-Periode erforderlich."
 // Beginnt VL53L0X_perform_phase_calibration()

 uint8_t sequence_config = readReg(SYSTEM_SEQUENCE_CONFIG);
 writeReg(SYSTEM_SEQUENCE_CONFIG, 0x02);
 performSingleRefCalibration(0x0);
 writeReg(SYSTEM_SEQUENCE_CONFIG, sequence_config);

 // Ende VL53L0X_perform_phase_calibration()

 return true;
}

// Ermitteln der VCSEL-Impulsperiode in PCLKs für den gegebenen Periodentyp.
// Basierend auf VL53L0X_get_vcsel_pulse_period()
uint8_t getVcselPulsePeriod(vcselPeriodType type)
{
 if (type == VcselPeriodPreRange)
 {
   return decodeVcselPeriod(readReg(PRE_RANGE_CONFIG_VCSEL_PERIOD));
 }
 else if (type == VcselPeriodFinalRange)
 {
   return decodeVcselPeriod(readReg(FINAL_RANGE_CONFIG_VCSEL_PERIOD));
 }
 else { return 255; }
}

// Startet fortlaufende Bereichsmessungen. Wenn period_ms (optional) 0
// oder nicht angegeben ist, wird der kontinuierliche Back-to-Back-Modus
// verwendet (der Sensor führt so oft wie möglich Messungen durch);
// andernfalls wird der kontinuierliche zeitgesteuerte Modus verwendet,
// wobei die angegebene Zwischenmessperiode in Millisekunden bestimmt,
// wie oft der Sensor eine Messung durchführt.
// Basierend auf VL53L0X_StartMessung()
void startContinuous(uint32_t period_ms)
{
 writeReg(0x80, 0x01);
 writeReg(0xFF, 0x01);
 writeReg(0x00, 0x00);
 writeReg(0x91, g_stopVariable);
 writeReg(0x00, 0x01);
 writeReg(0xFF, 0x00);
 writeReg(0x80, 0x00);

 if (period_ms != 0)
 {
   // Kontinuierlich zeitgesteuerter Modus

   // Beginnt VL53L0X_SetInterMeasurementPeriodMilliSeconds()

   uint16_t osc_calibrate_val = readReg16Bit(OSC_CALIBRATE_VAL);

   if (osc_calibrate_val != 0)
   {
     period_ms *= osc_calibrate_val;
   }

   writeReg32Bit(SYSTEM_INTERMEASUREMENT_PERIOD, period_ms);

   // Ende VL53L0X_SetInterMeasurementPeriodMilliSeconds()

   writeReg(SYSRANGE_START, 0x04); // VL53L0X_REG_SYSRANGE_MODE_TIMED
 }
 else
 {
   // Kontinuierlicher back-to-back Modus
   writeReg(SYSRANGE_START, 0x02); // VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK
 }
}

// Stoppt kontinuierliche Messung
// Basierend auf VL53L0X_StopMeasurement()
void stopContinuous(void)
{
 writeReg(SYSRANGE_START, 0x01); // VL53L0X_REG_SYSRANGE_MODE_SINGLESHOT

 writeReg(0xFF, 0x01);
 writeReg(0x00, 0x00);
 writeReg(0x91, 0x00);
 writeReg(0x00, 0x01);
 writeReg(0xFF, 0x00);
}

// Gibt einen Messbereich in Millimetern zurück, wenn der kontinuierliche
// Modus aktiv ist (readRangeSingleMillimeters() ruft diese Funktion auch
// nach dem Start einer Einzelschussmessung auf). extraStats liefert
// zusätzliche Informationen für diese Messung. Wenn nicht benötigt, auf
// 0 setzen.
uint16_t readRangeContinuousMillimeters( statInfo_t *extraStats ) {
 uint8_t tempBuffer[12];
 uint16_t temp;
 startTimeout();
 while ((readReg(RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
   if (checkTimeoutExpired())
   {
     g_isTimeout = true;
     return 65535;
   }
 }
 if( extraStats == 0 ){
   // Annahmen: Die Linearitätskorrekturverstärkung beträgt 1000 (Voreinstellung);
   // fractional ranging ist nicht aktiviert
   temp = readReg16Bit(RESULT_RANGE_STATUS + 10);
 } else {
   // GGF. GENAUER NACHSCHAUEN -----!
   // Register map beginnt bei 0x14
   //     0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F
   //    5A 06 BC 04 00 85 00 38 00 19 06 B6 00 00 00 00
   //   0: Ranging status, uint8_t
   //   1: ???
   // 3,2: Effective SPAD return count, uint16_t, fixpoint8.8
   //   4: 0 ?
   //   5: ???
   // 6,7: signal count rate [mcps], uint16_t, fixpoint9.7
   // 9,8: AmbientRateRtnMegaCps  [mcps], uint16_t, fixpoimt9.7
   // A,B: uncorrected distance [mm], uint16_t
   readMulti(0x14, tempBuffer, 12);
   extraStats->rangeStatus =  tempBuffer[0x00]>>3;
   extraStats->spadCnt     = (tempBuffer[0x02]<<8) | tempBuffer[0x03];
   extraStats->signalCnt   = (tempBuffer[0x06]<<8) | tempBuffer[0x07];
   extraStats->ambientCnt  = (tempBuffer[0x08]<<8) | tempBuffer[0x09];
   temp                    = (tempBuffer[0x0A]<<8) | tempBuffer[0x0B];
   extraStats->rawDistance = temp;
 }
 writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01);
 return temp;
}

// Führt eine Single-Shot-Messung durch und gibt den Messwert in
// Millimetern zurück
// Basierend auf VL53L0X_PerformSingleRangingMeasurement()
// extraStats liefert zusätzliche Informationen für diese Messung.
// Wenn nicht benötigt, auf 0 setzen.
uint16_t readRangeSingleMillimeters( statInfo_t *extraStats ) {
 writeReg(0x80, 0x01);
 writeReg(0xFF, 0x01);
 writeReg(0x00, 0x00);
 writeReg(0x91, g_stopVariable);
 writeReg(0x00, 0x01);
 writeReg(0xFF, 0x00);
 writeReg(0x80, 0x00);
 writeReg(SYSRANGE_START, 0x01);
 // "Warten, bis das Start-Bit gelöscht wurde"
 startTimeout();
 while (readReg(SYSRANGE_START) & 0x01){
   if (checkTimeoutExpired()){
     g_isTimeout = true;
     return 65535;
   }
 }
 return readRangeContinuousMillimeters( extraStats );
}

// Ist in einer der Lesefunktionen ein Timeout aufgetreten seit dem
// letzten Aufruf von timeoutOccurred()?
bool timeoutOccurred()
{
 bool tmp = g_isTimeout;
 g_isTimeout = false;
 return tmp;
}

void setTimeout(uint16_t timeout){
 g_ioTimeout = timeout;
}

uint16_t getTimeout(void){
 return g_ioTimeout;
}

// Private Methoden /////////////////////////////////////////////////////////////

// Abrufen von SPAD-Referenz (single photon avalanche diode)
// Anzahl und Typ basierend auf VL53L0X_get_info_from_device(),
// aber nur Abrufen von SPAD-Referenzanzahl und -typ
bool getSpadInfo(uint8_t * count, bool * type_is_aperture)
{
 uint8_t tmp;

 writeReg(0x80, 0x01);
 writeReg(0xFF, 0x01);
 writeReg(0x00, 0x00);

 writeReg(0xFF, 0x06);
 writeReg(0x83, readReg(0x83) | 0x04);
 writeReg(0xFF, 0x07);
 writeReg(0x81, 0x01);

 writeReg(0x80, 0x01);

 writeReg(0x94, 0x6b);
 writeReg(0x83, 0x00);
 startTimeout();
 while (readReg(0x83) == 0x00)
 {
   if (checkTimeoutExpired()) { return false; }
 }
 writeReg(0x83, 0x01);
 tmp = readReg(0x92);

 *count = tmp & 0x7f;
 *type_is_aperture = (tmp >> 7) & 0x01;

 writeReg(0x81, 0x00);
 writeReg(0xFF, 0x06);
 writeReg(0x83, readReg(0x83)  & ~0x04);
 writeReg(0xFF, 0x01);
 writeReg(0x00, 0x01);

 writeReg(0xFF, 0x00);
 writeReg(0x80, 0x00);

 return true;
}

// Get sequence step enables
// Basierend auf VL53L0X_GetSequenceStepEnables()
void getSequenceStepEnables(SequenceStepEnables * enables)
{
 uint8_t sequence_config = readReg(SYSTEM_SEQUENCE_CONFIG);

 enables->tcc          = (sequence_config >> 4) & 0x1;
 enables->dss          = (sequence_config >> 3) & 0x1;
 enables->msrc         = (sequence_config >> 2) & 0x1;
 enables->pre_range    = (sequence_config >> 6) & 0x1;
 enables->final_range  = (sequence_config >> 7) & 0x1;
}

// Get sequence step timeouts
// Basierend auf get_sequence_step_timeout(),
// erhält aber alle Timeouts anstelle nur des angeforderten und
// speichert dabei Zwischenwerte
void getSequenceStepTimeouts(SequenceStepEnables const * enables, SequenceStepTimeouts * timeouts)
{
 timeouts->pre_range_vcsel_period_pclks = getVcselPulsePeriod(VcselPeriodPreRange);

 timeouts->msrc_dss_tcc_mclks = readReg(MSRC_CONFIG_TIMEOUT_MACROP) + 1;
 timeouts->msrc_dss_tcc_us =
   timeoutMclksToMicroseconds(timeouts->msrc_dss_tcc_mclks,
                              timeouts->pre_range_vcsel_period_pclks);

 timeouts->pre_range_mclks =
   decodeTimeout(readReg16Bit(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
 timeouts->pre_range_us =
   timeoutMclksToMicroseconds(timeouts->pre_range_mclks,
                              timeouts->pre_range_vcsel_period_pclks);

 timeouts->final_range_vcsel_period_pclks = getVcselPulsePeriod(VcselPeriodFinalRange);

 timeouts->final_range_mclks =
   decodeTimeout(readReg16Bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));

 if (enables->pre_range)
 {
   timeouts->final_range_mclks -= timeouts->pre_range_mclks;
 }

 timeouts->final_range_us =
   timeoutMclksToMicroseconds(timeouts->final_range_mclks,
                              timeouts->final_range_vcsel_period_pclks);
}


// Dekodiert Sequenzschritt-Timeout in MCLKs vom Registerwert
// Basierend auf VL53L0X_decode_timeout()
// Hinweis: Die ursprüngliche Funktion gab eine uint32_t zurück,
// aber der Rückgabewert wird immer in einem uint16_t gespeichert.
uint16_t decodeTimeout(uint16_t reg_val)
{
 // Format: "(LSByte * 2^MSByte) + 1"
 return (uint16_t)((reg_val & 0x00FF) <<
        (uint16_t)((reg_val & 0xFF00) >> 8)) + 1;
}

// Kodiert Sequenzschritt-Timeout-Registerwert ab Zeitüberschreitung in MCLKs
// Basiert auf VL53L0X_encode_timeout()
// Hinweis: Die ursprüngliche Funktion nahm ein uint16_t an,
// aber das an diese übergebene Argument ist immer ein uint16_t.
uint16_t encodeTimeout(uint16_t timeout_mclks)
{
 // Format: "(LSByte * 2^MSByte) + 1"

 uint32_t ls_byte = 0;
 uint16_t ms_byte = 0;

 if (timeout_mclks > 0)
 {
   ls_byte = timeout_mclks - 1;

   while ((ls_byte & 0xFFFFFF00) > 0)
   {
     ls_byte >>= 1;
     ms_byte++;
   }

   return (ms_byte << 8) | (ls_byte & 0xFF);
 }
 else { return 0; }
}

// Konvertieren der Sequenzschritt-Timeout von MCLKs in
// Mikrosekunden mit gegebener VCSEL-Periode in PCLKs
// Basiert auf VL53L0X_calc_timeout_us()
uint32_t timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks)
{
 uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

 return ((timeout_period_mclks * macro_period_ns) + (macro_period_ns / 2)) / 1000;
}

// Konvertieren der Sequenzschritt-Timeout von Mikrosekunden
// in MCLKs mit gegebener VCSEL-Periode in PCLKs
// Basiert auf VL53L0X_calc_timeout_mclks()
uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks)
{
 uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

 return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}


// Basiert auf VL53L0X_perform_single_ref_calibration()
bool performSingleRefCalibration(uint8_t vhv_init_byte)
{
 writeReg(SYSRANGE_START, 0x01 | vhv_init_byte); // VL53L0X_REG_SYSRANGE_MODE_START_STOP

 startTimeout();
 while ((readReg(RESULT_INTERRUPT_STATUS) & 0x07) == 0)
 {
   if (checkTimeoutExpired()) { return false; }
 }

 writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01);

 writeReg(SYSRANGE_START, 0x00);

 return true;
}
