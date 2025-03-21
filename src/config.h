#ifndef CONFIG_H
#define CONFIG_H

#define GPS
#define DUAL
#define BARO
#define VOLT
#define SERVO

//#define DEBUG

#define CURSHOME "\033[H"               //cursor home
#define ERASEALL "\033[2J"              //erase screen
#define CURSBACK "\033[0G"              //cursor column 0
#define CURS_C10 "\033[10G"               //cursor culumn 8
#define ERASELINE "\033[0K"             //erase line
#define BRIGHTYELLOW_FG "\033[93m"      //set foreground
#define WHITE_FG "\033[97m"             //set foreground
#define GRAY_FG "\033[37m"              //set foreground
#define BLUE_BG "\033[44m"              //set background
#define CYAN_BG "\033[46m"              //set background
#define BLACK_BG "\033[40m"             //set background
#define CURSPOS "\033[%d;%dH"           //cursor pos y x (y does not work in Android UsbTerminal)

typedef struct {
    boolean valid;

    uint8_t prio_VARIOM;
    uint8_t prio_ALTITU;
    uint8_t prio_VOLTAG;  
    uint8_t prio_GPSLON;
    uint8_t prio_GPSLAT;
    uint8_t prio_GPSSPD;
    uint8_t prio_GPSALT;
    uint8_t prio_GPSTIM;
    uint8_t prio_GPSSAT;
    uint8_t prio_GPSDIS;
    uint8_t prio_GPSTRA;
    uint8_t prio_GPSHAC;
    uint8_t prio_GPSVAC;
    uint8_t prio_GPSHEA;

    bool enab_VARIOM;
    bool enab_ALTITU;
    bool enab_GPSLON;
    bool enab_GPSLAT;
    bool enab_GPSSPD;
    bool enab_GPSALT;
    bool enab_GPSTIM;
    bool enab_GPSSAT;
    bool enab_GPSDIS;
    bool enab_GPSTRA;
    bool enab_GPSHAC;
    bool enab_GPSVAC;
    bool enab_GPSHEA;
    bool enab_VOLTAG;

    uint8_t ctrl_CHANNL;
    uint8_t rset_CHANNL;

    uint16_t puls_TIMING;
    uint8_t srv1_CHANNL;
    uint8_t srv2_CHANNL;
    uint8_t srv3_CHANNL;
    uint8_t srv4_CHANNL;

    uint16_t srv1_FAILSV;
    uint16_t srv2_FAILSV;
    uint16_t srv3_FAILSV;
    uint16_t srv4_FAILSV;
    float dlay_FAILSV;

    bool fuse_WDTIME;

    bool enab_CALIBR;
    double high_MEASUR;
    double lowr_MEASUR;
    double high_VOLTAG;
    double lowr_VOLTAG;

} config_t;

void writeConfig(config_t); 
config_t getConf(void);
void printConfValue (const char*, int, int, int, int );
void showConf(config_t, config_t);
void cliConf (void);

#endif