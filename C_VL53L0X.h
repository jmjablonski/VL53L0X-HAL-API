/**
 *        File: C_VL53L0X.h
 *
 *      Author: J. Jablonski
 *
 *      @brief  Umgeschriebene Header-Datei von Pololu die
 *              auf der ST-API für den VL53L0X basiert.
 *
 *      Pololu-API ist für Arduino ausgelegt.
 *      Die Kommentare zu den Funktionen wurden großteils
 *      übernommen und auf deutsch übersetzt (vgl. README.md
 *      Pololu).
 *
 *      Mehr zur Pololu-API auf:
 *              https://github.com/pololu/vl53l0x-arduino
 *
 *      Abkürzungen:
 *                  VCSEL = vertical cavity surface emitting laser
 *                  PCLK  = Peripheral Clock || Processor CLock
 *                  TCC   = Target CentreCheck
 *                  MSRC  = Minimum Signal Rate Check
 *                  DSS   = Dynamic Spad Selection
 *
 */

 // Wenn alles klappt Refaktorierung vornehmen:
 //                                             bool  -> bool_t
 //                                             true  -> TRUE
 //                                             false -> FALSE

#ifndef C_VL53L0X_H_
#define C_VL53L0X_H_

/*  Defines */                                           // Aus der C++-Datei entnommen

/*
 *  Das two-wire interface von Arduino verwendet eine 7-Bit-Zahl für die Adresse,
 *  und setzt das letzte Bit korrekt basierend auf Lese- und Schreibvorgängen
 */
#define ADDRESS_DEFAULT 0b0101001 //Schauen ob Adresse richtig ist

/*
 *  Zeichnet die aktuelle Zeit auf, um eine bevorstehende Zeitüberschreitung zu prüfen
 */
#define startTimeout() (g_timeoutStartMs = millis())

/*
 *  Prüft, ob das Timeout aktiviert (auf einen Wert ungleich Null gesetzt) und abgelaufen ist
 */
#define checkTimeoutExpired() (g_ioTimeout > 0 && ((uint16_t)millis() - g_timeoutStartMs) > g_ioTimeout)


/*
 *  Decodiert die VCSEL-Pulsperiode  in PCLKs aus dem Registerwert
 *  basierend auf VL53L0X_decode_vcsel_period()
 */
#define decodeVcselPeriod(reg_val)      (((reg_val) + 1) << 1)

/*
 *  Kodiert den VCSEL-Impulsperiodenregisterwert anhand der PCLKs Periode
 *  basierend auf VL53L0X_encode_vcsel_period()
 */
#define encodeVcselPeriod(period_pclks) (((period_pclks) >> 1) - 1)

/*
 *  Berechnet die Makroperiode in *Nanosekunden* aus der VCSEL-Periode in PCLKs
 *  basierend auf VL53L0X_calc_macro_period_ps()
 *  PLL_period_ps = 1655; macro_period_vclks = 2304 ----------------- Nachschauen
 */
#define calcMacroPeriod(vcsel_period_pclks) ((((uint32_t)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000)


/*  Register Adressen aus ST-API vl53l0x_device.h (Reiehnfolge übernommen)*/
enum regAddr
{
    SYSRANGE_START                              = 0x00,

    SYSTEM_THRESH_HIGH                          = 0x0C,
    SYSTEM_THRESH_LOW                           = 0x0E,

    SYSTEM_SEQUENCE_CONFIG                      = 0x01,
    SYSTEM_RANGE_CONFIG                         = 0x09,
    SYSTEM_INTERMEASUREMENT_PERIOD              = 0x04,

    SYSTEM_INTERRUPT_CONFIG_GPIO                = 0x0A,

    GPIO_HV_MUX_ACTIVE_HIGH                     = 0x84,

    SYSTEM_INTERRUPT_CLEAR                      = 0x0B,

    RESULT_INTERRUPT_STATUS                     = 0x13,
    RESULT_RANGE_STATUS                         = 0x14,

    RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN       = 0xBC,
    RESULT_CORE_RANGING_TOTAL_EVENTS_RTN        = 0xC0,
    RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF       = 0xD0,
    RESULT_CORE_RANGING_TOTAL_EVENTS_REF        = 0xD4,
    RESULT_PEAK_SIGNAL_RATE_REF                 = 0xB6,

    ALGO_PART_TO_PART_RANGE_OFFSET_MM           = 0x28,

    I2C_SLAVE_DEVICE_ADDRESS                    = 0x8A,

    MSRC_CONFIG_CONTROL                         = 0x60,

    PRE_RANGE_CONFIG_MIN_SNR                    = 0x27,
    PRE_RANGE_CONFIG_VALID_PHASE_LOW            = 0x56,
    PRE_RANGE_CONFIG_VALID_PHASE_HIGH           = 0x57,
    PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT          = 0x64,

    FINAL_RANGE_CONFIG_MIN_SNR                  = 0x67,
    FINAL_RANGE_CONFIG_VALID_PHASE_LOW          = 0x47,
    FINAL_RANGE_CONFIG_VALID_PHASE_HIGH         = 0x48,
    FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44,

    PRE_RANGE_CONFIG_SIGMA_THRESH_HI            = 0x61,
    PRE_RANGE_CONFIG_SIGMA_THRESH_LO            = 0x62,

    PRE_RANGE_CONFIG_VCSEL_PERIOD               = 0x50,
    PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI          = 0x51,
    PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO          = 0x52,

    SYSTEM_HISTOGRAM_BIN                        = 0x81,
    HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT       = 0x33,
    HISTOGRAM_CONFIG_READOUT_CTRL               = 0x55,

    FINAL_RANGE_CONFIG_VCSEL_PERIOD             = 0x70,
    FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI        = 0x71,
    FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO        = 0x72,
    CROSSTALK_COMPENSATION_PEAK_RATE_MCPS       = 0x20,

    MSRC_CONFIG_TIMEOUT_MACROP                  = 0x46,

    SOFT_RESET_GO2_SOFT_RESET_N                 = 0xBF,
    IDENTIFICATION_MODEL_ID                     = 0xC0,
    IDENTIFICATION_REVISION_ID                  = 0xC2,

    OSC_CALIBRATE_VAL                           = 0xF8,

    GLOBAL_CONFIG_VCSEL_WIDTH                   = 0x32,
    GLOBAL_CONFIG_SPAD_ENABLES_REF_0            = 0xB0,
    GLOBAL_CONFIG_SPAD_ENABLES_REF_1            = 0xB1,
    GLOBAL_CONFIG_SPAD_ENABLES_REF_2            = 0xB2,
    GLOBAL_CONFIG_SPAD_ENABLES_REF_3            = 0xB3,
    GLOBAL_CONFIG_SPAD_ENABLES_REF_4            = 0xB4,
    GLOBAL_CONFIG_SPAD_ENABLES_REF_5            = 0xB5,

    GLOBAL_CONFIG_REF_EN_START_SELECT           = 0xB6,
    DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD         = 0x4E,
    DYNAMIC_SPAD_REF_EN_START_OFFSET            = 0x4F,
    POWER_MANAGEMENT_GO1_POWER_FORCE            = 0x80,

    VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV           = 0x89,

    ALGO_PHASECAL_LIM                           = 0x30,
    ALGO_PHASECAL_CONFIG_TIMEOUT                = 0x30,
};

enum vcselPeriodType
{
    VcselPeriodPreRange,
    VcselPeriodFinalRange
};

  uint8_t last_status; //Schauen weil nicht in nonArduine


/** @brief bool als Typdefinition, wie es in C++ gegeben ist
 *         Für spätere Integration in C++, nur noch ausklammern.
 */
#define bool    uint8_t
#define false   0
#define true    1

/*
 * Zusätzliche Informationen für eine Messung
 */
typedef struct
{
  uint16_t rawDistance; //unkorrigierte Distanz  [mm],   uint16_t
  uint16_t signalCnt;   //Signal Counting Rate [mcps], uint16_t, fixpoint9.7
  uint16_t ambientCnt;  //Ambient Counting Rate [mcps], uint16_t, fixpoint9.7
  uint16_t spadCnt;     //Effective SPAD return count,  uint16_t, fixpoint8.8
  uint8_t  rangeStatus; //Ranging status (0-15)
} statInfo_t;

/*
 *  ------------------------------------------
 *  Teil der in der C++-Header in public stand
 *  ------------------------------------------
 */

/** @brief Konfigurierung des i2c für neue Adresse.
 *
 *  @param new_addr (8 Bit, LSB = 0)
 */
void setAddress(uint8_t new_addr);

/** @brief Gibt die aktuelle i2c Adresse wieder.
 *
 *  In C++ mit inline verwendet worden, schauen ob {return address;} nötig ist
 */
uint8_t getAddress(void) { return address; } //return nötig?


/** @brief
 *  @param
 *  @return
 */
bool init(bool io_2v8 = true);

/** @brief i2c Kommunikationsfunktionen
 *
 *  Muss später an die HAL-Lib angepasst werden.
 *
 *  @param  reg     Registeradresse aus regAddr
 *  @param  value   Wert an das Register
 *  @param  src     Source -> Quelle
 *  @param  count   Anzahl der Bytes
 *  @param  dst     Destination -> Zielort
 */
void writeReg(uint8_t reg, uint8_t value);        // Schreibt ein 8-Bit Register
void writeReg16Bit(uint8_t reg, uint16_t value);  // Schreibt ein 16-Bit Register
void writeReg32Bit(uint8_t reg, uint32_t value);  // Schreibt ein 32-Bit Register
uint8_t readReg(uint8_t reg);                     // Liest ein 8-Bit Register
uint16_t readReg16Bit(uint8_t reg);               // Liest ein 16-Bit Register
uint32_t readReg32Bit(uint8_t reg);               // Liest ein 32-Bit Register

/** @brief Schreibt "count" Anzahl von Bytes von "src" auf den Sensor, beginnt bei "reg"
 */
void writeMulti(uint8_t reg, uint8_t const * src, uint8_t count);

/** @brief Liest "count" Anzahl von Bytes vom Sensor, beginnt bei "reg" und endet bei "dst"
 */
void readMulti(uint8_t reg, uint8_t * dst, uint8_t count);


/** @brief Setzt die Rückgabe des Signalratenlimits auf den angegebenen Wert
 *         in der Einheiten MCPS (Megazählungen pro Sekunde).
 *
 *  Dies ist die Mindestamplitude des vom Zielobjekt reflektierten und vom
 *  Sensor empfangenen Signals, die erforderlich ist, damit der Sensor einen
 *  gültigen Messwert meldet. Die Einstellung einer unteren Grenze erhöht den
 *  möglichen Bereich des Sensors, sondern erhöht auch die Wahrscheinlichkeit,
 *  eine ungenaue Messung zu erhalten, weil Reflexionen von anderen Objekten
 *  als dem beabsichtigten Ziel. Dieser Grenzwert ist auf 0,25 MCPS
 *  initialisiert standardmäßig.
 *
 *  Der Rückgabewert ist ein boolescher Wert, der angibt, ob das
 *  angeforderte Limit gültig war.
 */
bool setSignalRateLimit(float limit_Mcps);

/** @brief Gibt die aktuelle Rückgabe des Signalratenlimits in MCPS wieder.
 */
float getSignalRateLimit(void);

/** @brief Legt das Timing-Budget (Zeitfenster) für die Messung in Mikrosekunden
 *         fest.
 *
 *  D.h. die Zeit, die für eine Messung zur Verfügung steht; die ST-API und
 *  diese Bibliothek sorgen für die Aufteilung des Timing-Budgets auf die
 *  Teilschritte in der Messbereichssequenz. Ein längeres Zeitspannenbudget
 *  ermöglicht genauere Messungen. Eine Erhöhung des Budgets um den Faktor N
 *  verringert die Standardabweichung der Entfernungsmessung um den Faktor
 *  sqrt(N). Standardwert ist etwa 33 Millisekunden; das Minimum beträgt 20 ms
 *  basierend auf L53L0X_Messsatz_Zeitplan_Budget_Mikro_Sekunden().
 *
 *  Der Rückgabewert ist ein boolescher Wert, der angibt, ob das angeforderte
 *  Zeitfenster gültig war.
 */
bool setMeasurementTimingBudget(uint32_t budget_us);

/** @brief Gibt das aktuelle Messzeitbudget in Mikrosekunden zurück.
 */
uint32_t getMeasurementTimingBudget(void);

/** @brief Setzt die VCSEL-Pulsperiode für den gegebenen Periodentyp
 *         (VcselPeriodPreRange oder VcselPeriodFinalRange) auf den gegebenen
 *         Wert (in PCLKs).
 *
 *  Längere Zeiträume erhöhen den möglichen Bereich des Sensors. Gültige Werte
 *  sind (nur gerade Zahlen):
 *
 *  Vorher: 12 bis 18 (standardmäßig auf 14 initialisiert)
 *  Final: 8 bis 14 (standardmäßig auf 10 initialisiert)
 *
 *  Der Rückgabewert ist ein boolescher Wert, der angibt, ob der angeforderte
 *  Zeitraum gültig war.
 */
bool setVcselPulsePeriod(vcselPeriodType type, uint8_t period_pclks);

/** @brief Gibt die aktuelle VCSEL-Impulsperiode für den gegebenen Periodentyp
 *         zurück.
 */
uint8_t getVcselPulsePeriod(vcselPeriodType type);

/** @brief Beginnt mit kontinuierlichen Messungen.
 *
 *  Wenn das Argument period_ms gleich 0 ist, wird der kontinuierliche
 *  Back-to-Back-Modus verwendet (der Sensor nimmt so oft wie möglich Messungen
 *  vor); wenn es ungleich Null ist, wird der kontinuierliche zeitgesteuerte
 *  Modus verwendet, wobei die angegebene Zwischenmessperiode in Millisekunden
 *  bestimmt, wie oft der Sensor eine Messung vornimmt.
 */
void startContinuous(uint32_t period_ms = 0);

/** @brief Stoppt den kontinuierlichen Messmoduns.
*/
void stopContinuous(void);

/** @brief Gibt bei aktiviertem kontinuierlichen Modus einen Messbereich in
 *         Millimetern zurück.
 *
 *  Zusätzliche Messdaten werden in `extraStats` kopiert, wenn sie ungleich
 *  Null sind. Schauen 
 */
uint16_t readRangeContinuousMillimeters(void); //ggf. mit übergabe

/** @brief Führt eine Einzelmessung durch und gibt den Messwert in Millimetern
 *         zurück.
 *
 *  Zusätzliche Messdaten werden in `extraStats` kopiert, wenn sie ungleich
 *  Null sind. **********************************************************************Schauen ob dies angewendet werden sollte,
 *                                                                                   Wenn ja, schaue nonArduino[...].h Line 203 etc.
 */
uint16_t readRangeSingleMillimeters(void); //***************************************************************************Schauen weil mit übergabe vgl l. 198 in nonArduine

/** @brief Legt eine Zeitabschaltungsperiode (Timeout-Periode) in Millisekunden
 *         fest, nach der die Lesevorgänge abgebrochen werden, wenn der Sensor
 *         nicht bereit ist.
 *
 *  Ein Wert von 0 deaktiviert die Zeitüberschreitung.
 *  War in C++ als inline Funktion verwendet worden.
 */
void setTimeout(uint16_t timeout) //Gleichsetzen nötig?
{
  io_timeout = timeout;
}

/** @brief Gibt die aktuelle Einstellung der Timeout-Periode zurück.
 *
 *  War in C++ als inline Funktion verwendet worden.
 */
uint16_t getTimeout(void)
{
  return io_timeout;
}

/** @brief Gibt an, ob seit dem letzten Aufruf von timeoutOccurred() eine
 *         Lesezeitüberschreitung aufgetreten ist.
 */
bool timeoutOccurred(void);



// Teil der in der C++-Header in private stand *********************************



/** @brief Status der Sequenzfolge
 *
 *  @param  tcc           Target Centre Check
 *  @param  msrc          Minimum Signal Rate Check
 *  @param  dss           Dynamic Spad Selection
 *  @param  pre_range     Vorherige Entfernung
 *  @param  final_range   Finale Entfernung
 */
 typedef struct
 {
   bool tcc, msrc, dss, pre_range, final_range; // in C++-Datei als boolean deklariert. boolean war in Arduino.h als bool deklariert
 } SequenceStepEnables;

 typedef struct
 {
   uint16_t pre_range_vcsel_period_pclks, final_range_vcsel_period_pclks;

   uint16_t msrc_dss_tcc_mclks, pre_range_mclks, final_range_mclks;
   uint32_t msrc_dss_tcc_us,    pre_range_us,    final_range_us;
 }SequenceStepTimeouts;

/*

 ---------------------------------------------------------------------------
|																		    |
|		der Folgende Teil                             						|
|		--> anschauen und ggf rauslöschen	                                |
|   --> oder drinnen lassen													|																	
|																			|
 ---------------------------------------------------------------------------

*/
 
 uint8_t address;
 uint16_t io_timeout;
 bool did_timeout;
 uint16_t timeout_start_ms;

 uint8_t stop_variable; // read by init and used when starting measurement; is StopVariable field of VL53L0X_DevData_t structure in API
 uint32_t measurement_timing_budget_us;

 bool getSpadInfo(uint8_t * count, bool * type_is_aperture);

 void getSequenceStepEnables(SequenceStepEnables * enables);
 void getSequenceStepTimeouts(SequenceStepEnables const * enables, SequenceStepTimeouts * timeouts);

 bool performSingleRefCalibration(uint8_t vhv_init_byte);

 static uint16_t decodeTimeout(uint16_t value);
 static uint16_t encodeTimeout(uint32_t timeout_mclks);
 static uint32_t timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks);
 static uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks);

#endif
