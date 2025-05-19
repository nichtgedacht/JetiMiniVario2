#include "JetiExBusProtocol.h"  //	https://github.com/nichtgedacht/JetiExBus
#include "config.h"
#include <DPS.h>                //	https://github.com/nichtgedacht/DPS
config_t cfg;

JetiExBusProtocol exBus;
DPS dpsA;
DPS dpsB;

#define DPS_ADDRESS_A 0x77  // for both values or altitude only if DUAL sensor
#define DPS_ADDRESS_B 0x76  // for vario only if DUAL sensor


#define T1 150000.0 // base time constant
#define T2 200000.0 // base time constant
double t1 = T1;
double t2 = T2;

#define deltaTA 30992  //measured average value, see commented out measurement
//#define deltaTB 15977
#define deltaTB 30992
//char input;

// set neutral
uint16_t channelValue = 1500;
uint16_t prevChannelValue = 1500;

bool servoRun = false;
bool wdRun = false;
ulong rcWatch;

bool resetHome = true;
double latHome;
double lonHome;
double latLast;
double lonLast;
uint32_t travel = 0;

ulong blink;

volatile bool wdTimeout = false;

long mue = 0;
long mueDiff = 0;
long prevMue = 0;
long worstMueDiff = 0;
unsigned long loopCount = 0;


uint32_t mic, prevmic, micDiff;
double micDiffAvr;
char micDiffAvr_string[20];

long realPressureA, realPressureB;
int32_t pressureA[1], pressureB[1];
int32_t temperatureA[1], temperatureB[1];

#ifdef GPS
// sets refresh rate
char refresh_10hz[] = { 0xb5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00,
                        0x01, 0x00, 0x01, 0x00, 0x7a, 0x12 };

// sets messages 
char ubx_nav_pvt[] = { 0xb5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x07,
                       0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x18, 0xe1 };

// sets port
char prt_ubx_only_19200[] = { 0xb5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00,
                              0x00, 0x00, 0xd0, 0x08, 0x00, 0x00, 0x00, 0x4b,
                              0x00, 0x00, 0x07, 0x00, 0x01, 0x00, 0x00, 0x00,
                              0x00, 0x00, 0x46, 0x4b };


char enable_galileo[] = { 0xb5, 0x62, 0x06, 0x3e, 0x0c, 0x00, 0x00, 0x00,
                          0x20, 0x01, 0x02, 0x04, 0x08, 0x00, 0x01, 0x00,
                          0x01, 0x01, 0x82, 0x56 };
#endif

enum {
    ID_DUMMY = 0,
#ifdef BARO
    ID_VARIOM,
#ifdef DUAL
    ID_VARIOB,
    ID_TEKTAS,
#endif
    ID_ALTITU,
#endif
#ifdef GPS
    ID_GPSLON,
    ID_GPSLAT,
    ID_GPSSPD,
    ID_GPSALT,
    ID_GPSTIM,
    ID_GPSVEH,
    ID_GPSDIS,
    ID_GPSTRA,
    ID_GPSHAC,
    ID_GPSVAC,
    ID_GPSHEA,
#endif
#ifdef VOLT
    ID_VOLTA1,
    ID_VOLTA2,
#endif
};

#ifdef BARO
double referencePressureA = 0, referencePressureB = 0, offsetPressureB = 0;
double r_altitudeA = 0, r_altitude0A = 0, r_altitudeB = 0, r_altitude0B = 0; 
double climbA = 0, climb0A = 0, climbB = 0, climb0B = 0; 
double dyn_alfaA, dyn_alfaB, alfa_1A, alfa_2A, alfa_1B, alfa_2B, factor;

uint32_t diff_t_A, max_diff_t_A, diff_t_B, max_diff_t_B;
double relativeAltitudeA = 0, relativeAltitudeB = 0;
#endif

#ifdef VOLT
double avar1 = 0;
double avar2 = 0;
uint16_t GAIN_CORR;
uint16_t OFFSET_CORR;
bool adcStart = true;
#endif
int j=0;

#ifdef GPS
//-------------------- UBX ----------------------------

typedef union {

    struct {
                                /* Comments stohlen from PX4 */
	    uint32_t iTOW;          /**< GPS Time of Week [ms] */
	    uint16_t year;          /**< Year (UTC)*/
	    uint8_t  month;         /**< Month, range 1..12 (UTC) */
	    uint8_t  day;           /**< Day of month, range 1..31 (UTC) */
	    uint8_t  hour;          /**< Hour of day, range 0..23 (UTC) */
	    uint8_t  min;           /**< Minute of hour, range 0..59 (UTC) */
	    uint8_t  sec;           /**< Seconds of minute, range 0..60 (UTC) */
	    uint8_t  valid;         /**< Validity flags (see UBX_RX_NAV_PVT_VALID_...) */
	    uint32_t tAcc;          /**< Time accuracy estimate (UTC) [ns] */
	    int32_t  nano;          /**< Fraction of second (UTC) [-1e9...1e9 ns] */
	    uint8_t  fixType;       /**< GNSSfix type: 0 = No fix, 1 = Dead Reckoning only, 2 = 2D fix, 3 = 3d-fix, 4 = GNSS + dead reckoning, 5 = time only fix */
	    uint8_t  flags;         /**< Fix Status Flags (see UBX_RX_NAV_PVT_FLAGS_...) */
	    uint8_t  flags2;        /**< Additional flags */
	    uint8_t  numSV;         /**< Number of SVs used in Nav Solution */
	    int32_t  lon;           /**< Longitude [1e-7 deg] */
	    int32_t  lat;           /**< Latitude [1e-7 deg] */
	    int32_t  height;        /**< Height above ellipsoid [mm] */
	    int32_t  hMSL;          /**< Height above mean sea level [mm] */
	    uint32_t hAcc;          /**< Horizontal accuracy estimate [mm] */
	    uint32_t vAcc;          /**< Vertical accuracy estimate [mm] */
	    int32_t  velN;          /**< NED north velocity [mm/s]*/
	    int32_t  velE;          /**< NED east velocity [mm/s]*/
	    int32_t  velD;          /**< NED down velocity [mm/s]*/
	    int32_t  gSpeed;        /**< Ground Speed (2-D) [mm/s] */
	    int32_t  headMot;       /**< Heading of motion (2-D) [1e-5 deg] */
	    uint32_t sAcc;          /**< Speed accuracy estimate [mm/s] */
	    uint32_t headAcc;       /**< Heading accuracy estimate (motion and vehicle) [1e-5 deg] */
	    uint16_t pDOP;          /**< Position DOP [0.01] */
	    uint16_t flags3;        /**< Additional flags */
	    uint32_t reserved1;     /**< reserved */
	    int32_t  headVeh;       /**< (ubx8+ only) Heading of vehicle (2-D) [1e-5 deg] */
	    int16_t  magDec;        /**< magnetc declination */
        uint16_t magAcc;        /**< magnetic accelaration */

    } Val;

    uint8_t UBXBuffer[92];

} NavPvt_t;

typedef union {

    struct {
                         /* Comments stohlen from PX4 */
	    uint32_t iTOW;   /**< GPS Time of Week [ms] */
	    int32_t  lon;    /**< Longitude [1e-7 deg] */
	    int32_t  lat;    /**< Latitude [1e-7 deg] */
	    int32_t  height; /**< Height above ellipsoid [mm] */
	    int32_t  hMSL;   /**< Height above mean sea level [mm] */
	    uint32_t hAcc;   /**< Horizontal accuracy estimate [mm] */
	    uint32_t vAcc;   /**< Vertical accuracy estimate [mm] */

    } Val;

    uint8_t UBXBuffer[28];

} NavPosllh_t;

enum UBXFrameState
{
	WaitStart1,
    WaitStart2,
	WaitClass,
    WaitID,
	WaitLen1,
    WaitLen2,
    WaitPayload,
    WaitChk2
};

UBXFrameState state = WaitStart1;
NavPvt_t NavPvt;
NavPosllh_t NavPosllh;

uint16_t distanceHome(double lonRemote, double latRemote ) {

    double dx = 1113000 * cos( ( latHome * PI ) / 180 ) * ( lonHome - lonRemote); 
    double dy = 1113000 * ( latHome - latRemote);
    return round ( sqrt( dx * dx + dy * dy ) ); // distance in decimeter
}

uint16_t distanceTravel(double lonCurrent, double latCurrent ) {

    uint16_t step;

    double dx = 1113000 * cos( ( latLast * PI ) / 180 ) * ( lonLast - lonCurrent); 
    double dy = 1113000 * ( latLast - latCurrent);

    step =  round ( sqrt( dx * dx + dy * dy ) ); // distance in decimeter

    if (step > 20) {
        latLast = latCurrent;
        lonLast = lonCurrent;
        travel += step ;
    }
    return travel / 10;
}

void DecodeUBX(uint8_t Class, uint8_t ID) {

    bool valid = false;
    double lat = 0, lon = 0;

    if ( Class == 1 && ID == 7 ) {

        if ( NavPvt.Val.fixType == 3 ) {
            valid = true;
            lat = (double)NavPvt.Val.lat/10000000; // degree
            lon = (double)NavPvt.Val.lon/10000000; // degree
            if ( resetHome ) {
                resetHome = false;
                latLast = latHome = lat;
                lonLast = lonHome = lon;
                travel = 0;
            }
        }

        exBus.SetSensorValueGPS (ID_GPSLAT, false, lat, valid);
        exBus.SetSensorValueGPS (ID_GPSLON, true,  lon, valid);
        exBus.SetSensorValue (ID_GPSALT, round(NavPvt.Val.hMSL/100), valid);
        exBus.SetSensorValue (ID_GPSSPD, round(NavPvt.Val.gSpeed/10), valid);
        exBus.SetSensorValueTime(ID_GPSTIM, NavPvt.Val.hour, NavPvt.Val.min, NavPvt.Val.sec, valid);
        exBus.SetSensorValue (ID_GPSVEH, NavPvt.Val.numSV, valid);
        exBus.SetSensorValue (ID_GPSDIS, distanceHome(lon, lat), valid);
        exBus.SetSensorValue (ID_GPSTRA, distanceTravel(lon, lat), valid);
        exBus.SetSensorValue (ID_GPSHAC, round(NavPvt.Val.hAcc/100), valid);
        exBus.SetSensorValue (ID_GPSVAC, round(NavPvt.Val.vAcc/100), valid);
        exBus.SetSensorValue (ID_GPSHEA, round(NavPvt.Val.headMot/10000), valid);
    }

#ifdef DEBUG
    SerialUSB.print(NavPvt.Val.numSV);
    SerialUSB.println(" vehicles");    

    switch ( NavPvt.Val.fixType ) {

        case 0: 
            SerialUSB.println("no fix");
            break;
        case 2:
            SerialUSB.println("2d fix");
            break;
        case 3:
            SerialUSB.println("3d fix");
            break;
    } 
#endif
}

void parse_UBX_NAV(char c) {

    static int i;
    static uint16_t UBXLengt;
    static uint8_t length1;
    static uint8_t UBXClass;
    static uint8_t UBXID;
    static uint8_t CheckSumA;
    static uint8_t CheckSumB;
    static uint8_t UBXChkA;
    static uint8_t UBXChkB;
    static uint16_t UBXPayloadLength;
    static uint8_t *buffer;

    switch ( state ) {

        case WaitStart1: 
            if ( c == 0xb5 ) { 
                CheckSumA = 0;
                CheckSumB = 0;
                state = WaitStart2;
            }
            break;

        case WaitStart2:
            if ( c == 0x62 ) {
                state = WaitClass;
            }
            break;

        case WaitClass:
            UBXClass = c;
            CheckSumA = CheckSumA + c;
            CheckSumB = CheckSumB + CheckSumA;
            state = WaitID;
            break;

        case WaitID:
            UBXID = c;
            CheckSumA = CheckSumA + c;
            CheckSumB = CheckSumB + CheckSumA;

            switch (UBXID) {
                case 7:
                    UBXPayloadLength = 92;
                    buffer = &NavPvt.UBXBuffer[0];
                    state = WaitLen1;
                    break;
                case 2:
                    UBXPayloadLength = 28;
                    buffer = &NavPosllh.UBXBuffer[0];
                    state = WaitLen1;
                    break;
                default:
#ifdef DEBUG
                    SerialUSB.print("no decoder for NAV-ID: ");
                    SerialUSB.println(UBXID);
#endif
                    state = WaitStart1;
                    break;
            }
            break;

        case WaitLen1:
            length1 = c;
            CheckSumA = CheckSumA + c;
            CheckSumB = CheckSumB + CheckSumA;
            state = WaitLen2;
            break;

        case WaitLen2:
            UBXLengt = (c << 8) | length1;
            CheckSumA = CheckSumA + c;
            CheckSumB = CheckSumB + CheckSumA;

            if ( UBXLengt == UBXPayloadLength) {
                state = WaitPayload;
                i = 0;
            } else {
                state = WaitStart1; // error, try again
#ifdef DEBUG
                SerialUSB.println("error UBXLengt");
#endif
                state = WaitStart1;
            }
            break;

        case  WaitPayload:
            if ( i < UBXLengt ) {
                buffer[i++] = c;
                CheckSumA = CheckSumA + c;
                CheckSumB = CheckSumB + CheckSumA;
            }  else {
                UBXChkA = c;
                state = WaitChk2;
            }
            break;

        case  WaitChk2:
            UBXChkB = c;
            if ( CheckSumA == UBXChkA && CheckSumB == UBXChkB ) {
                DecodeUBX(UBXClass, UBXID);
                digitalWrite( PIN_LED_13, LOW);
                blink = millis();
            }
            state = WaitStart1;
            break;

        default:
            state = WaitStart1;
            break;
    }
}

// -------------------- /UBX ---------------------------
#endif

void startWatchdog(void) {

    static bool initialized;

    if ( !initialized ) {

        REG_GCLK_GENDIV = GCLK_GENDIV_DIV (1) |             // Use the undivided 32.768kHz clock source
                          GCLK_GENDIV_ID (2);               // Select Generic Clock (GCLK) 2
        while ( GCLK->STATUS.bit.SYNCBUSY );                // Wait for synchronization

        REG_GCLK_GENCTRL = GCLK_GENCTRL_GENEN |             // Enable GCLK2
                           GCLK_GENCTRL_SRC_OSCULP32K |     // Set the clock source to the ultra low power oscillator (OSCULP32K)
                           GCLK_GENCTRL_ID (2);             // Select GCLK2
        while ( GCLK->STATUS.bit.SYNCBUSY );                // Wait for synchronization

        // Feed GCLK2 to WDT (Watchdog Timer)
        REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |             // Enable GCLK2 to the WDT
                           GCLK_CLKCTRL_GEN_GCLK2 |         // Select GCLK2
                           GCLK_CLKCTRL_ID_WDT;             // Feed the GCLK2 to the WDT
        while ( GCLK->STATUS.bit.SYNCBUSY );                // Wait for synchronization

        // Set up WDT
        REG_WDT_EWCTRL = WDT_EWCTRL_EWOFFSET_256;           // Set the Early Warning Interrupt Time Offset to 7.8125 ms
        REG_WDT_INTENSET = WDT_INTENSET_EW;                 // Enable the Early Warning interrupt
        REG_WDT_CONFIG = WDT_CONFIG_PER_1K;                 // Set the WDT reset timeout to 31.25 ms
        REG_WDT_CTRL = WDT_CTRL_ENABLE;                     // Enable the WDT in normal mode
        while (WDT->STATUS.bit.SYNCBUSY);                   // Await synchronization of registers between clock domains

        // Configure and enable WDT interrupt
        NVIC_SetPriority(WDT_IRQn, 0);                      // Top priority
        NVIC_EnableIRQ(WDT_IRQn);

        initialized = true;

    } else {

       REG_WDT_CTRL = WDT_CTRL_ENABLE;
       while(WDT->STATUS.bit.SYNCBUSY);

    }
}

void stopWatchdog(void) {
    REG_WDT_CTRL = 0x00;
    while(WDT->STATUS.bit.SYNCBUSY);
}

// Calculate altitude from Pressure & Sea level pressure
double getAltitude(double pressure, double zeroLevelPressure) {
    return (44330.0f * (1.0f - pow((double) pressure / (double) zeroLevelPressure, 0.1902949f)));
}

// Calculate sea level from Pressure given on specific altitude
double getSeaLevel(double pressure, double altitude) {
    return ((double) pressure / pow(1.0f - ((double) altitude / 44330.0f), 5.255f));
}

// resets referencePressure and set reset flag for GPS
void altiZero(void) {
    //getSeaLevel() calculates SealevelPressure from realpressure and real altitude MSL
#ifdef BARO
    referencePressureA = getSeaLevel(referencePressureA, - r_altitude0A);
#ifdef DUAL
    referencePressureB = getSeaLevel(referencePressureB, - r_altitude0B);
    // used for TAS, calc offset again when reset altitude to zero
    offsetPressureB = referencePressureA - referencePressureB;
#endif
#endif
    resetHome = true; // GPS Home
}

void WDT_Handler() {

#ifdef SERVO
    if ( cfg.fuse_WDTIME == 1) {
       cfg.fuse_WDTIME = 0;
       writeConfig(cfg);
    }
#endif

    wdTimeout = true;    // set here for short block and timely reset of WDT

    // indicate an error
    digitalWrite( PIN_LED_TXL, LOW);
    digitalWrite( PIN_LED_RXL, LOW);

    while ( WDT->STATUS.bit.SYNCBUSY );               // Wait for synchronization
    REG_WDT_INTFLAG = WDT_INTFLAG_EW;                 // Clear interrupt flag
    while ( WDT->STATUS.bit.SYNCBUSY );               // Wait for synchronization
}

#ifdef DUAL
double tAS( long realPressureA, long realPressureB ) {

    static double outTAS;

    // formula used
    // tas = 3.6 * ( sqrt( ( realPressureA - realPressureB ) * 2 / 1.2 ) );  // km/h

    double pDiff = double ( realPressureA - realPressureB - offsetPressureB );

    if ( pDiff < 0 ) {
        pDiff = 0;
    }

    double tas = 4.6476 * ( sqrt( ( pDiff ) ) );  // km/h

    // smooth it a bit
    outTAS = outTAS - 0.2 * (outTAS - tas);

//#ifdef DEBUG
    //SerialUSB.printf ("tas: %.2f km/h\n", outTAS);
//#endif

    return outTAS;
}
#endif

void setup () {

    // delay(10000);

#if defined (BARO) || defined (GPS)
    uint8_t i = 0;
#endif

    cfg = getConf();
    wdTimeout = !cfg.fuse_WDTIME;

#ifdef VOLT
    double gain_corr = ( cfg.high_MEASUR - cfg.lowr_MEASUR ) / ( cfg.high_VOLTAG - cfg.lowr_VOLTAG );
    GAIN_CORR = round ( 2048 / gain_corr );
    OFFSET_CORR = round ( (  cfg.lowr_MEASUR - gain_corr * cfg.lowr_VOLTAG ) * 4096 / 33 );

    if ( cfg.enab_CALIBR ) {
        ADC->OFFSETCORR.reg = ADC_OFFSETCORR_OFFSETCORR(OFFSET_CORR);
        ADC->GAINCORR.reg = ADC_GAINCORR_GAINCORR(GAIN_CORR);
        ADC->CTRLB.bit.CORREN = true;
    }
#endif

    // Attention! parameter priority added. Value will be send after every N times of completed sets
    // of all sensors are sent. Where N is priority.
    static JETISENSOR_CONST sensors[] = {
        //id,           name,            unit,               dataType, precision, priority
#ifdef BARO
        { ID_VARIOM,    "Vario",         "m/s",  JetiSensor::TYPE_14b, 2,         cfg.prio_VARIOM },
#ifdef DUAL
        { ID_VARIOB,    "VarioB",        "m/s",  JetiSensor::TYPE_14b, 2,         cfg.prio_VARIOB },
        { ID_TEKTAS,    "TAS",           "km/h", JetiSensor::TYPE_14b, 0,         cfg.prio_TEKTAS },
#endif
        { ID_ALTITU,    "AltRelat.",     "m",    JetiSensor::TYPE_14b, 1,         cfg.prio_ALTITU },
#endif
#ifdef GPS
        { ID_GPSLON,    "GPS Longitude", "",     JetiSensor::TYPE_GPS, 0,         cfg.prio_GPSLON },
        { ID_GPSLAT,    "GPS Latitude",  "",     JetiSensor::TYPE_GPS, 0,         cfg.prio_GPSLAT },
        { ID_GPSSPD,    "GPS Speed",     "m/s",  JetiSensor::TYPE_14b, 2,         cfg.prio_GPSSPD },
        { ID_GPSALT,    "GPS Altitude",  "m",    JetiSensor::TYPE_14b, 1,         cfg.prio_GPSALT },
        { ID_GPSTIM,    "GPS Time",      "",     JetiSensor::TYPE_DT,  0,         cfg.prio_GPSTIM },
        { ID_GPSVEH,    "GPS Vehicles",  "",     JetiSensor::TYPE_6b,  0,         cfg.prio_GPSSAT },
        { ID_GPSDIS,    "GPS Distance",  "m",    JetiSensor::TYPE_14b, 1,         cfg.prio_GPSDIS },
        { ID_GPSTRA,    "GPS Travel",    "m",    JetiSensor::TYPE_22b, 0,         cfg.prio_GPSTRA },
        { ID_GPSHAC,    "GPS hAccuracy", "m",    JetiSensor::TYPE_14b, 1,         cfg.prio_GPSDIS },
        { ID_GPSVAC,    "GPS vAccuracy", "m",    JetiSensor::TYPE_14b, 1,         cfg.prio_GPSDIS },
        { ID_GPSHEA,    "GPS Heading",   "deg",  JetiSensor::TYPE_14b, 1,         cfg.prio_GPSDIS },
#endif
#ifdef VOLT
        { ID_VOLTA1,    "Voltage1",       "V",    JetiSensor::TYPE_14b, 2,         cfg.prio_VOLTA1 },
        { ID_VOLTA2,    "Voltage2",       "V",    JetiSensor::TYPE_14b, 2,         cfg.prio_VOLTA2 },
#endif
        0                           // end of array
    };

    // enable sensors according to config
#ifdef BARO
    exBus.SetSensorActive( ID_VARIOM, cfg.enab_VARIOM != 0, sensors );
#ifdef DUAL
    exBus.SetSensorActive( ID_VARIOB, cfg.enab_VARIOB != 0, sensors );
    exBus.SetSensorActive( ID_TEKTAS, cfg.enab_TEKTAS != 0, sensors );
#endif
    exBus.SetSensorActive( ID_ALTITU, cfg.enab_ALTITU != 0, sensors );
#endif
#ifdef GPS
    exBus.SetSensorActive( ID_GPSLON, cfg.enab_GPSLON != 0, sensors );
    exBus.SetSensorActive( ID_GPSLAT, cfg.enab_GPSLAT != 0, sensors );
    exBus.SetSensorActive( ID_GPSSPD, cfg.enab_GPSSPD != 0, sensors );
    exBus.SetSensorActive( ID_GPSALT, cfg.enab_GPSALT != 0, sensors );
    exBus.SetSensorActive( ID_GPSTIM, cfg.enab_GPSTIM != 0, sensors );
    exBus.SetSensorActive( ID_GPSVEH, cfg.enab_GPSSAT != 0, sensors );
    exBus.SetSensorActive( ID_GPSDIS, cfg.enab_GPSDIS != 0, sensors );
    exBus.SetSensorActive( ID_GPSTRA, cfg.enab_GPSTRA != 0, sensors );
    exBus.SetSensorActive( ID_GPSHAC, cfg.enab_GPSHAC != 0, sensors );
    exBus.SetSensorActive( ID_GPSVAC, cfg.enab_GPSVAC != 0, sensors );
    exBus.SetSensorActive( ID_GPSHEA, cfg.enab_GPSHEA != 0, sensors );
#endif
#ifdef VOLT
    exBus.SetSensorActive( ID_VOLTA1, cfg.enab_VOLTA1 != 0, sensors );
    exBus.SetSensorActive( ID_VOLTA2, cfg.enab_VOLTA2 != 0, sensors );
#endif

    SerialUSB.begin(115200);

    //Wire.begin();
    //Wire.setClock(500000L);

#ifdef GPS
    if (!wdTimeout) {

        // Start GPS connection default baud rate
        Serial1.begin (9600);
	    while (!Serial1) {};

        for (i = 0; i < sizeof(prt_ubx_only_19200); i++) {
            Serial1.write(prt_ubx_only_19200[i]); 
        }

        delay(100);

        Serial1.begin (19200);
	    while (!Serial1) {};
        delay(100);

        // Set output UBX NAV-PVT on UART1
        for (i = 0; i < sizeof(ubx_nav_pvt); i++) {
            Serial1.write(ubx_nav_pvt[i]);
        }

        for (i = 0; i < sizeof(refresh_10hz); i++) {
            Serial1.write(refresh_10hz[i]);
        }

        for (i = 0; i < sizeof(enable_galileo); i++) {
            Serial1.write(enable_galileo[i]);
        }

    } // wdTimeout
#endif

#ifdef BARO
    if (!wdTimeout) {

        while (uint8_t ret = dpsA.begin(DPS_ADDRESS_A) != 0 ) {

            SerialUSB.printf("Init BaroA failed with: %d\n", ret);

#ifdef SERVO
            // after a chrash of baros it could be that the brownout detector triggers a reset
            // so start the watchdog here preventing a hang 
            startWatchdog();
#endif
            // if no baro hardware present LED will blink slow if watchdog is not running.
            pinMode(PIN_LED_13, OUTPUT);
            digitalWrite(PIN_LED_13, !digitalRead(PIN_LED_13) );
            delay (500);
        }

#ifdef SERVO
        stopWatchdog();
#endif

        int16_t temp_mr = 5;
        int16_t temp_osr = 0;
        int16_t prs_mr = 5;
        int16_t prs_osr = 3;
        int16_t resA = dpsA.startMeasureBothCont(temp_mr, temp_osr, prs_mr, prs_osr);
        if ( resA != 0 ) {
            while(1) {
                SerialUSB.printf("startMeasure dpsA failed with: %d\n", resA);
                delay(1000);
            }
        }

#ifdef DUAL
        while (uint8_t ret = dpsB.begin(DPS_ADDRESS_B) != 0) {

            SerialUSB.printf("Init BaroB failed with: %d\n",ret);

#ifdef SERVO
            // after a chrash of baros it could be that the brownout detector triggers a reset
            // so start the watchdog here preventing a hang 
            startWatchdog();
#endif

            // if no baro hardware present LED will blink slow if watchdog is not running.
            pinMode(PIN_LED_13, OUTPUT);
            digitalWrite(PIN_LED_13, !digitalRead(PIN_LED_13) );
            delay (500);
        }

#ifdef SERVO
        stopWatchdog();
#endif


        int16_t resB = dpsB.startMeasureBothCont(temp_mr, temp_osr, prs_mr, prs_osr);
        if ( resB != 0 ) {
            while (1) {
                SerialUSB.printf("startMeasure dpsB failed with: %d\n", resB);
                delay(1000);
            }
        }
#endif
        // calc default alfas from deltaT for time constants chosen and handle them separately for both sensors
        // deltaTA and deltaTB  were both measured initial for chosen MRs and OSRs for temperature and pressure sensors
        alfa_1A = deltaTA / ( T1 + deltaTA );
        alfa_2A = deltaTA / ( T2 + deltaTA );

        alfa_1B = deltaTB / ( T1 + deltaTB );
        alfa_2B = deltaTB / ( T2 + deltaTB );

        // calc default gain from time constants chosen 
        factor = 1000000 / (T2 - T1);

        i = 0;
        while (i < 100) {
            //SerialUSB.println(i);
            unsigned char pressureCountA = 1;
            unsigned char temperatureCountA = 1;
            dpsA.getContResults(temperatureA, temperatureCountA, pressureA, pressureCountA);

            if (pressureCountA ) {
                referencePressureA += pressureA[0];
                i++;
            }
        }
#ifdef DUAL
        i = 0;
        while (i < 100) {
            //SerialUSB.println(i);
            unsigned char pressureCountB = 1;
            unsigned char temperatureCountB = 1;
            dpsB.getContResults(temperatureB, temperatureCountB, pressureB, pressureCountB);

            if (pressureCountB ) {
                referencePressureB += pressureB[0];
                i++;
            }
        }
#endif
        referencePressureA = referencePressureA / 100;
#ifdef DUAL
        referencePressureB = referencePressureB / 100;
        // used for TAS
        offsetPressureB = referencePressureA - referencePressureB;
#endif

    } // wdTimeout
#endif //BARO

    exBus.SetDeviceId(0x77, 0x32); // 0x3276
    exBus.Start ("miniVario2", sensors, 0);

    // all build in LEDs
    pinMode(PIN_LED_13, OUTPUT);
    pinMode(PIN_LED_TXL, OUTPUT);
    pinMode(PIN_LED_RXL, OUTPUT);
    digitalWrite( PIN_LED_13, HIGH );
    if (!wdTimeout) {
        digitalWrite( PIN_LED_TXL, HIGH );
        digitalWrite( PIN_LED_RXL, HIGH );
    } else {
        digitalWrite( PIN_LED_TXL, LOW );
        digitalWrite( PIN_LED_RXL, LOW );
    }

    // for debugging with logic analyzer
    //pinMode(PIN_A0, OUTPUT);
    //digitalWrite(PIN_A0, LOW);

    analogReadResolution(12);

#ifdef SERVO
/*************************  setup servo timer  **********************************/

    // Enable and configure generic clock generator 4
    GCLK->GENCTRL.reg = GCLK_GENCTRL_IDC |          // Improve duty cycle
                        GCLK_GENCTRL_GENEN |        // Enable generic clock gen
                        GCLK_GENCTRL_SRC_DFLL48M |  // Select 48MHz as source
                        GCLK_GENCTRL_ID(4);         // Select GCLK4
    while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

    // Set clock divider of 3 to generic clock generator 4
    GCLK->GENDIV.reg = GCLK_GENDIV_DIV(3) |         // Divide 48 MHz by 3 to get 16 MHz
                       GCLK_GENDIV_ID(4);           // Apply to GCLK4 4
    while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

    // Enable GCLK4 and connect it to TCC0 and TCC1
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |        // Enable generic clock
                        GCLK_CLKCTRL_GEN_GCLK4 |    // Select GCLK4
                        GCLK_CLKCTRL_ID_TCC0_TCC1;  // Feed GCLK4 to TCC0/1
    while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

    // Set TCC0 ticks to 1 MHz
    //TCC0->CTRLA.reg |= TCC_CTRLA_PRESCALER(TCC_CTRLA_PRESCALER_DIV16_Val);
    TCC0->CTRLA.reg = TCC_CTRLA_PRESCALER_DIV16;

    // Use "Normal PWM" (single-slope PWM): count up to PER, match on CC[n]
    TCC0->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;         // Select NPWM as waveform
    while (TCC0->SYNCBUSY.bit.WAVE);                // Wait for synchronization

    // Set the period (the number to count to (TOP) before resetting timer)
    TCC0->PER.reg = 1000 * cfg.puls_TIMING - 1;     // period of pulse rate micro Seconds
    while (TCC0->SYNCBUSY.bit.PER);                 // Wait for synchronization

    // default routing
    // PA04 WO[0] XIAO Pin A1 -> Servo3 
    TCC0->CC[0].reg = 1500;                         // default puls length, never in use
    while (TCC0->SYNCBUSY.bit.CC0);

    // PA05 WO[1] XIAO Pin A9 -> Servo4
    TCC0->CC[1].reg = 1500;                         // default puls length, never in use
    while (TCC0->SYNCBUSY.bit.CC1);

    // PA10 WO[2] XIAO Pin A2 -> Servo2
    TCC0->CC[2].reg = 1500;                         // default puls length, never in use
    while (TCC0->SYNCBUSY.bit.CC2);

    // PA11 WO[3] XIAO Pin A3 -> Servo1
    TCC0->CC[3].reg = 1500;                         // default puls length, never in use
    while (TCC0->SYNCBUSY.bit.CC3);

    // Configure PA10, PA11, PA04 and PA05 to be output
    PORT->Group[PORTA].DIRSET.reg = PORT_PA04;      // Set pin as output
    PORT->Group[PORTA].OUTCLR.reg = PORT_PA04;      // Set pin to low

    PORT->Group[PORTA].DIRSET.reg = PORT_PA05;      // Set pin as output
    PORT->Group[PORTA].OUTCLR.reg = PORT_PA05;      // Set pin to low

    PORT->Group[PORTA].DIRSET.reg = PORT_PA10;      // Set pin as output
    PORT->Group[PORTA].OUTCLR.reg = PORT_PA10;      // Set pin to low

    PORT->Group[PORTA].DIRSET.reg = PORT_PA11;      // Set pin as output
    PORT->Group[PORTA].OUTCLR.reg = PORT_PA11;      // Set pin to low

    // Enable the port multiplexer for PA10 and PA11
    PORT->Group[PORTA].PINCFG[4].reg |= PORT_PINCFG_PMUXEN;
    PORT->Group[PORTA].PINCFG[5].reg |= PORT_PINCFG_PMUXEN;
    PORT->Group[PORTA].PINCFG[10].reg |= PORT_PINCFG_PMUXEN;
    PORT->Group[PORTA].PINCFG[11].reg |= PORT_PINCFG_PMUXEN;

    // F is TCC0/WO[2] for PA10 and TCC0/WO[3] for PA11
    // E is TCC0/WO[0] for PA04 and TCC0/WO[1] for PA05
    // see chapter 7.1
    // Odd port-num -> PMUXO
    // Even port-num -> PMUXE
    // For odd port-num last bit in PMUX index becomes shifted out which results in correct index
    PORT->Group[PORTA].PMUX[4>>1].reg |= PORT_PMUX_PMUXE_E;
    PORT->Group[PORTA].PMUX[5>>1].reg |= PORT_PMUX_PMUXO_E;
    PORT->Group[PORTA].PMUX[10>>1].reg |= PORT_PMUX_PMUXE_F;
    PORT->Group[PORTA].PMUX[11>>1].reg |= PORT_PMUX_PMUXO_F;

    // TCC0 to be enabled in loop()

/************************* end of setup servo timer *****************************/

#endif //SERVO

}

void loop () {

#ifdef SERVO
    if (!wdRun) {
        ++loopCount;
        if ( loopCount == 10) {
            startWatchdog();
            wdRun = true;
            //delay(40);
        }
    }
    //   else {
    //    ++loopCount;
    //    if (loopCount == 40000) {
    //        delay(10);
    //        SerialUSB.println(loopCount);
    //    }
    //}
#endif

#ifdef SERVO
    if (!WDT->STATUS.bit.SYNCBUSY) {            // let it synchronize while loop is running
        REG_WDT_CLEAR = WDT_CLEAR_CLEAR_KEY;    // Clear the watchdog timer
    }
#endif

#ifndef DEBUG
    if ( USB->DEVICE.DADD.reg &USB_DEVICE_DADD_ADDEN ) {    // if plugged in, fast check

#ifdef SERVO
        stopWatchdog();
#endif
        if ( SerialUSB ) { // if connected, slow check
            cliConf();
        }
    }
#endif

#ifdef VOLT
    uint32_t ar1 = analogRead(PIN_A8);
    if (adcStart) {
        avar1 = ar1;
        adcStart = false;
    }
    avar1 = avar1 - 0.01 * ( avar1 - (double) ar1); // avarage by exponential filter
    double voltage1 = (3.3 * avar1 ) / 409.6;     // assume 1:10 voltage divider

    uint32_t ar2 = analogRead(PIN_A0);
    if (adcStart) {
        avar2 = ar2;
        adcStart = false;
    }
    avar2 = avar2 - 0.01 * ( avar2 - (double) ar2); // avarage by exponential filter
    double voltage2 = (3.3 * avar2 ) / 409.6;     // assume 1:10 voltage divider
#endif

#ifdef SERVO
    if ( rcWatch + cfg.dlay_FAILSV * 1000 < millis() ) {

        TCC0->CC[0].reg = cfg.srv3_FAILSV;
        while (TCC0->SYNCBUSY.bit.CC0);

        TCC0->CC[1].reg = cfg.srv4_FAILSV;
        while (TCC0->SYNCBUSY.bit.CC1);

        TCC0->CC[2].reg = cfg.srv2_FAILSV;
        while (TCC0->SYNCBUSY.bit.CC2);

        TCC0->CC[3].reg = cfg.srv1_FAILSV;
        while (TCC0->SYNCBUSY.bit.CC3);

    }
#endif

    if ( exBus.HasNewChannelData() ) {

#ifdef SERVO
        if ( servoRun ) {
            rcWatch = millis();
        } else {
            servoRun = true;
            // Enable output (start PWM)
            TCC0->CTRLA.reg |= (TCC_CTRLA_ENABLE);
            while (TCC0->SYNCBUSY.bit.ENABLE);              // Wait for synchronization
        }

        TCC0->CC[0].reg = exBus.GetChannel(cfg.srv3_CHANNL);
        while (TCC0->SYNCBUSY.bit.CC0);

        TCC0->CC[1].reg = exBus.GetChannel(cfg.srv4_CHANNL);
        while (TCC0->SYNCBUSY.bit.CC1);

        TCC0->CC[2].reg = exBus.GetChannel(cfg.srv2_CHANNL);
        while (TCC0->SYNCBUSY.bit.CC2)

        TCC0->CC[3].reg = exBus.GetChannel(cfg.srv1_CHANNL);
        while (TCC0->SYNCBUSY.bit.CC3);
#endif

#ifdef BARO
        if (!wdTimeout) {
            // channel controls time constant
            channelValue = exBus.GetChannel(cfg.ctrl_CHANNL);

            // channelValue can be 0 sometimes if RC transmitter is started after the receiver
            if (channelValue > 0) {
                //char buf[30];
                //sprintf(buf, "chan-%d: %.4d", 12, channelValue);
                //SerialUSB.println(buf);

                // make time constants variable
                // channel at 100% and no trimm -> 1000 - 2000
                // check in servo monitor !
                t1 = T1 + T1 * ((double)channelValue / 1000 - 1 );
                t2 = T2 + T2 * ((double)channelValue / 1000 - 1 );

                // recalc alfas from deltaT for remote controlled time constants
                // and handle them separately for both sensors
                alfa_1A = deltaTA / ( t1 + deltaTA );
                alfa_2A = deltaTA / ( t2 + deltaTA );

                alfa_1B = deltaTB / ( t1 + deltaTB );
                alfa_2B = deltaTB / ( t2 + deltaTB );

                // calc gain from remote controlled time constants
                factor = 1000000 / (t2 - t1);

                //dtostrf(T1, 6, 0, buf);
                //SerialUSB.println(buf);
            }
        } // wdTimeout
#endif //BARO

        // reset if cfg.rset_CHANNL has a lower value (Edge trigger)
        channelValue = exBus.GetChannel(cfg.rset_CHANNL);

        if (channelValue < prevChannelValue - 300)
            altiZero();

        prevChannelValue = channelValue;

    } // has new channel data

#ifdef BARO
    if (!wdTimeout) {

        unsigned char pressureCountA = 1;
        unsigned char temperatureCountA = 1;

        dpsA.getContResults(temperatureA, temperatureCountA, pressureA, pressureCountA);

        if (pressureCountA) {

            realPressureA = pressureA[0];

            relativeAltitudeA = getAltitude (realPressureA, referencePressureA);

            r_altitude0A = r_altitude0A - alfa_1A * (r_altitude0A - relativeAltitudeA);

            r_altitudeA = r_altitudeA -  alfa_2A * (r_altitudeA - relativeAltitudeA);

            climb0A = (r_altitude0A - r_altitudeA) * factor;   // Factor is 1000000/dT ( 1/dT as seconds )

            // smoothing the climb value by another exponential filter
            // time constant of filter changes dynamically
            // greater speed of change means less filtering.
            // see "Nonlinear Exponential Filter"   
            dyn_alfaA = abs( (climbA - climb0A) / 0.4 );
            if ( dyn_alfaA >= 1 ) {
                dyn_alfaA = 1;
            }
            climbA = climbA - dyn_alfaA * ( climbA - climb0A );

#ifndef DUAL
            exBus.SetSensorValue (ID_VARIOM, round ((climbA) * 100), true);
#else
            exBus.SetSensorValue (ID_VARIOB, round ((climbA) * 100), true);
#endif
            exBus.SetSensorValue (ID_ALTITU, round ((r_altitude0A) * 10), true);
        }

#ifdef DUAL
        unsigned char pressureCountB = 1;
        unsigned char temperatureCountB = 1;
        dpsB.getContResults(temperatureB, temperatureCountB, pressureB, pressureCountB);

        if (pressureCountB) {

            realPressureB = pressureB[0];

            relativeAltitudeB = getAltitude (realPressureB, referencePressureB);

            r_altitude0B = r_altitude0B - alfa_1B * (r_altitude0B - relativeAltitudeB);

            r_altitudeB = r_altitudeB -  alfa_2B * (r_altitudeB - relativeAltitudeB);

            climb0B = (r_altitude0B - r_altitudeB) * factor;   // Factor is 1000000/dT ( 1/dT as seconds )

            // smoothing the climb value by another exponential filter
            // time constant of filter changes dynamically
            // greater speed of change means less filtering.
            // see "Nonlinear Exponential Filter"   
            dyn_alfaB = abs( (climbB - climb0B) / 0.6 );
            if ( dyn_alfaB >= 1 ) {
                dyn_alfaB = 1;
            }
            climbB = climbB - dyn_alfaB * ( climbB - climb0B );

            exBus.SetSensorValue (ID_VARIOM, round ((climbB) * 100), true);

            double TAS = tAS( realPressureA, realPressureB );
            exBus.SetSensorValue (ID_TEKTAS, round (TAS), true);
        }

#endif
    } // wdTimeout
#endif //BARO

#ifdef VOLT
        exBus.SetSensorValue (ID_VOLTA1, round ((voltage1) * 100 ), true);
        exBus.SetSensorValue (ID_VOLTA2, round ((voltage2) * 100 ), true);
#endif


#ifdef GPS
    if (!wdTimeout) {

        if (millis() - blink > 20) {
            digitalWrite( PIN_LED_13, HIGH);
        }

        while ( Serial1.available())
        {
            // calls decodeUBX() 
            parse_UBX_NAV(Serial1.read());

        }

    } // wdTimeout
#endif

    exBus.DoJetiExBus();

} // loop
