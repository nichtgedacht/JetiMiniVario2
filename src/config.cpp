#include <FlashStorage.h>       //  build in lib for storing configuration
#include "config.h"

FlashStorage(flashStore, config_t);

config_t getConf() {

    config_t config = flashStore.read();

    if (config.valid == false) {
        // set default values
        config.prio_VARIOM = 0;
        config.prio_ALTITU = 4;

        config.prio_VOLTAG = 5;

        config.prio_GPSLON = 3;
        config.prio_GPSLAT = 3;
        config.prio_GPSSPD = 3;
        config.prio_GPSALT = 4;
        config.prio_GPSTIM = 0;
        config.prio_GPSSAT = 15;
        config.prio_GPSDIS = 3;
        config.prio_GPSTRA = 3;
        config.prio_GPSHAC = 15;
        config.prio_GPSVAC = 15;
        config.prio_GPSHEA = 3;

        config.enab_VARIOM = 1;
        config.enab_ALTITU = 1;

        config.enab_VOLTAG = 1;

        config.enab_GPSLON = 1;
        config.enab_GPSLAT = 1;
        config.enab_GPSSPD = 1;
        config.enab_GPSALT = 1;
        config.enab_GPSTIM = 0;
        config.enab_GPSSAT = 1;
        config.enab_GPSDIS = 1;
        config.enab_GPSTRA = 1;
        config.enab_GPSHAC = 1;
        config.enab_GPSVAC = 1;
        config.enab_GPSHEA = 1;

        config.ctrl_CHANNL = 11;

        config.rset_CHANNL = 10;

        config.fuse_WDTIME = 1;

        config.puls_TIMING = 10;

        config.srv1_CHANNL = 0;
        config.srv2_CHANNL = 1;
        config.srv3_CHANNL = 2;
        config.srv4_CHANNL = 3;

        config.srv1_FAILSV = 1500;
        config.srv2_FAILSV = 1500;
        config.srv3_FAILSV = 1500;
        config.srv4_FAILSV = 1500;
        config.dlay_FAILSV = 1.5;

        config.enab_CALIBR = 1;
        config.high_MEASUR = 31.04;
        config.lowr_MEASUR = 3.18;
        config.high_VOLTAG = 31.00;
        config.lowr_VOLTAG = 3.00;

        config.valid = true;

        flashStore.write(config);
    }

    return config;
}

void writeConfig(config_t config) {
    flashStore.write(config);
}

void printConfValue (const char* name, int value, int flashValue, int line, int column) {

    char string[50];
    if ( value != flashValue ) {
        sprintf(string, CURSPOS BRIGHTYELLOW_FG "%s %d(%d)" WHITE_FG, line, column, name, value, flashValue);
    } else {
        sprintf(string, CURSPOS "%s %d      ", line, column, name, value);
    }
    SerialUSB.print(string);
}

void printConfFloatValue (const char* name, double value, double flashValue, int line, int column) {

    char doubleBufferValue[10];
    char doubleBufferFlashValue[10];
    char string[40];

    dtostrf(value, 2, 2, doubleBufferValue);
    dtostrf(flashValue, 2, 2, doubleBufferFlashValue);

    if ( value != flashValue ) {
        sprintf(string, CURSPOS BRIGHTYELLOW_FG "%s %s(%s)" WHITE_FG, line, column, name, doubleBufferValue, doubleBufferFlashValue);
    } else {
        sprintf(string, CURSPOS "%s %s       ", line, column, name, doubleBufferValue); 
    }
    SerialUSB.print(string);
}

void showConf(config_t config, config_t flashConfig) {

    SerialUSB.print(BLUE_BG);
    SerialUSB.print(WHITE_FG);

#ifndef BARO
    SerialUSB.print(GRAY_FG);
#endif
    printConfValue("prio_VARIOM:", config.prio_VARIOM, flashConfig.prio_VARIOM, 14, 3);
    printConfValue("prio_ALTITU:", config.prio_ALTITU, flashConfig.prio_ALTITU, 14, 23);
    SerialUSB.print(WHITE_FG);
#ifndef VOLT
    SerialUSB.print(GRAY_FG);
#endif
    printConfValue("prio_VOLTAG:", config.prio_VOLTAG, flashConfig.prio_VOLTAG, 14, 43);
    SerialUSB.print(WHITE_FG);
#ifndef GPS
    SerialUSB.print(GRAY_FG);
#endif
    printConfValue("prio_GPSLON:", config.prio_GPSLON, flashConfig.prio_GPSLON, 14, 63);
    printConfValue("prio_GPSLAT:", config.prio_GPSLAT, flashConfig.prio_GPSLAT, 15, 3);
    printConfValue("prio_GPSSPD:", config.prio_GPSSPD, flashConfig.prio_GPSSPD, 15, 23);
    printConfValue("prio_GPSALT:", config.prio_GPSALT, flashConfig.prio_GPSALT, 15, 43 );
    printConfValue("prio_GPSTIM:", config.prio_GPSTIM, flashConfig.prio_GPSTIM, 15, 63);
    printConfValue("prio_GPSSAT:", config.prio_GPSSAT, flashConfig.prio_GPSSAT, 16, 3);
    printConfValue("prio_GPSDIS:", config.prio_GPSDIS, flashConfig.prio_GPSDIS, 16, 23);
    printConfValue("prio_GPSTRA:", config.prio_GPSTRA, flashConfig.prio_GPSTRA, 16, 43);
    printConfValue("prio_GPSHAC:", config.prio_GPSHAC, flashConfig.prio_GPSHAC, 16, 63);
    printConfValue("prio_GPSVAC:", config.prio_GPSVAC, flashConfig.prio_GPSVAC, 17, 3);
    printConfValue("prio_GPSHEA:", config.prio_GPSHEA, flashConfig.prio_GPSHEA, 17, 23);
    SerialUSB.print(WHITE_FG);
#ifndef BARO
    SerialUSB.print(GRAY_FG);
#endif
    printConfValue("enab_VARIOM:", config.enab_VARIOM, flashConfig.enab_VARIOM, 19, 3);
    printConfValue("enab_ALTITU:", config.enab_ALTITU, flashConfig.enab_ALTITU, 19, 23);
    SerialUSB.print(WHITE_FG);
#ifndef VOLT
    SerialUSB.print(GRAY_FG);
#endif
    printConfValue("enab_VOLTAG:", config.enab_VOLTAG, flashConfig.enab_VOLTAG, 19, 43);
    SerialUSB.print(WHITE_FG);
#ifndef GPS
    SerialUSB.print(GRAY_FG);
#endif
    printConfValue("enab_GPSLON:", config.enab_GPSLON, flashConfig.enab_GPSLON, 19, 63);
    printConfValue("enab_GPSLAT:", config.enab_GPSLAT, flashConfig.enab_GPSLAT, 20, 3);
    printConfValue("enab_GPSSPD:", config.enab_GPSSPD, flashConfig.enab_GPSSPD, 20, 23);
    printConfValue("enab_GPSALT:", config.enab_GPSALT, flashConfig.enab_GPSALT, 20, 43);
    printConfValue("enab_GPSTIM:", config.enab_GPSTIM, flashConfig.enab_GPSTIM, 20, 63);
    printConfValue("enab_GPSSAT:", config.enab_GPSSAT, flashConfig.enab_GPSSAT, 21, 3);
    printConfValue("enab_GPSDIS:", config.enab_GPSDIS, flashConfig.enab_GPSDIS, 21, 23);
    printConfValue("enab_GPSTRA:", config.enab_GPSTRA, flashConfig.enab_GPSTRA, 21, 43);
    printConfValue("enab_GPSHAC:", config.enab_GPSHAC, flashConfig.enab_GPSHAC, 21, 63);
    printConfValue("enab_GPSVAC:", config.enab_GPSVAC, flashConfig.enab_GPSVAC, 22, 3);
    printConfValue("enab_GPSHEA:", config.enab_GPSHEA, flashConfig.enab_GPSHEA, 22, 23);
    SerialUSB.print(WHITE_FG);
#ifndef BARO
    SerialUSB.print(GRAY_FG);
#endif
    printConfValue("ctrl_CHANNL:", config.ctrl_CHANNL+1, flashConfig.ctrl_CHANNL+1, 24, 3);
    SerialUSB.print(WHITE_FG);
#if !defined BARO && !defined GPS
    SerialUSB.print(GRAY_FG);
#endif
    printConfValue("rset_CHANNL:", config.rset_CHANNL+1, flashConfig.rset_CHANNL+1, 24, 23);
    SerialUSB.print(WHITE_FG);
#ifndef SERVO
    SerialUSB.print(GRAY_FG);
#endif
    printConfValue("puls_TIMING:", config.puls_TIMING, flashConfig.puls_TIMING, 24, 43);
    printConfValue("fuse_WDTIME:", config.fuse_WDTIME, flashConfig.fuse_WDTIME, 24, 63);
    printConfValue("srv1_CHANNL:", config.srv1_CHANNL+1, flashConfig.srv1_CHANNL+1, 26, 3);
    printConfValue("srv2_CHANNL:", config.srv2_CHANNL+1, flashConfig.srv2_CHANNL+1, 26, 23);
    printConfValue("srv3_CHANNL:", config.srv3_CHANNL+1, flashConfig.srv3_CHANNL+1, 26, 43);
    printConfValue("srv4_CHANNL:", config.srv4_CHANNL+1, flashConfig.srv4_CHANNL+1, 26, 63);
    printConfValue("srv1_FAILSV:", config.srv1_FAILSV, flashConfig.srv1_FAILSV, 28, 3);
    printConfValue("srv2_FAILSV:", config.srv2_FAILSV, flashConfig.srv2_FAILSV, 28, 30);
    printConfFloatValue("dlay_FAILSV:", config.dlay_FAILSV, flashConfig.dlay_FAILSV, 28, 57);
    printConfValue("srv3_FAILSV:", config.srv3_FAILSV, flashConfig.srv3_FAILSV, 29, 3);
    printConfValue("srv4_FAILSV:", config.srv4_FAILSV, flashConfig.srv4_FAILSV, 29, 30);
    SerialUSB.print(WHITE_FG);
#ifndef VOLT
    SerialUSB.print(GRAY_FG);
#endif
    printConfValue("enab_CALIBR:", config.enab_CALIBR, flashConfig.enab_CALIBR, 31, 3);
    printConfFloatValue("high_MEASUR:", config.high_MEASUR, flashConfig.high_MEASUR, 32, 3);
    printConfFloatValue("lowr_MEASUR:", config.lowr_MEASUR, flashConfig.lowr_MEASUR, 32, 30);
    printConfFloatValue("high_VOLTAG:", config.high_VOLTAG, flashConfig.high_VOLTAG, 33, 3);
    printConfFloatValue("lowr_VOLTAG:", config.lowr_VOLTAG, flashConfig.lowr_VOLTAG, 33, 30);
    SerialUSB.print(WHITE_FG);

    SerialUSB.print("\r\n\n" ERASELINE "  Input: " CURS_C10 BLACK_BG "                  " CURS_C10);
}

void cliConf (void) {

    uint8_t i;
    char c;
    char command[80];
    uint8_t pos = 0;
    char *cptr;
    char key[20];
    double value = 0;
    bool delete_unwanted = false;

    for(i=0; i<sizeof(command); i++) {
        command[i] = 0;
    }

    delay(100); //wait for other konsole output for example PlatformIO Serial Monitor messages

    SerialUSB.print(BLUE_BG);
    SerialUSB.print(WHITE_FG);
    SerialUSB.print(ERASEALL);

    config_t flashConfig = flashStore.read();
    config_t config = flashStore.read();

    SerialUSB.println();
    SerialUSB.println( "  Welcome to JetiMiniVario CLI\n" );
    SerialUSB.println( "  Usage: ");
    SerialUSB.println( "  \"<Keyword> <Value>\" stage change" );
    SerialUSB.println( "  \"exit\" exit without change" );
    SerialUSB.println( "  \"write\" write changes to config\n" );
    SerialUSB.println( "  prio_XXX values from 0 (highest) to 20, enab_XXX is 1 for enabled and 0 for");
    SerialUSB.println( "  disabled. ctrl_CHANNL rset_CHANNL and srvX_CHANNL are each channel-numbers." );
    SerialUSB.println( "  srvX_FAILSV are failsave positions 1000-2000ms, puls_TIMING is period of");
    SerialUSB.println( "  pulses 10-40ms. fuse_WDTIME is the watchdog fuse. dlay_FAILSV 0.2 - 10s.\n");

    SerialUSB.println( "  current config:\n" );
    //SerialUSB.println( "  \n" );

    showConf(config, flashConfig);

    while (1) {

        while (SerialUSB.available()) {

            c = SerialUSB.read();

            if ( ( (pos < 17) && (c >= 31 && c <= 126) ) ) {
                SerialUSB.write(c);
            }

            switch (c) {
                // some terminals send 0x0D 0x0A on <enter> others send 0x0D only as Putty on default config
                case '\n':
                case '\r':

                    pos = 0;

                    if (delete_unwanted) { //ESC was detected
                        command[pos] = 0;
                        for(i=0; i<sizeof(command); i++) {
                            command[i] = 0;
                        }
                        SerialUSB.print(CURS_C10 "                  " CURS_C10); //clear input field
                        delete_unwanted = false;
                        break;
                    }

                    // get one or two token from command
                    i = 0;
                    cptr = strtok(command, " ");
                    while ( cptr != NULL) {
                        i++;
                        if ( i == 1) {
                            strcpy(key, cptr);
                        } else if ( i == 2 ) {
                            value = strtod(cptr, NULL);
                        }
                        cptr = strtok(NULL, " =");
                    }

                    switch (i) {

                        case 1:

                            SerialUSB.print(CURS_C10); // cursor back

                            if (strncmp(command, "exit", 4) == 0 ) {
                                SerialUSB.print("exiting rebooting ... ");
                                delay(500);
                                NVIC_SystemReset();
                                break; //never reached
                            } else if ( strncmp(command, "write", 5) == 0 ) {
                                // write current values to config
                                flashStore.write(config);
                                SerialUSB.print("config written rebooting ...");
                                delay(500);
                                NVIC_SystemReset();
                                break; //never reached
                            } else { //wrong command
                                command[pos] = 0;
                                SerialUSB.print(CURS_C10 "                  " CURS_C10); //clear input field
                            }
                            break;

                        case 2:
                            // change values after checking them
                            if ( false ) {
#ifdef BARO
                            } else if ( strncmp(key, "prio_VARIOM", 11 ) == 0 ) {
                                if ( value <= 20 ) {
                                    config.prio_VARIOM = (uint8_t) value;
                                }
                            } else if ( strncmp(key, "prio_ALTITU", 11 ) == 0 ) {
                                if ( value <= 20 ) {
                                    config.prio_ALTITU = (uint8_t) value;
                                }
#endif
#ifdef VOLT
                            } else if ( strncmp(key, "prio_VOLTAG", 11 ) == 0 ) {
                                if ( value <= 20 ) { 
                                    config.prio_VOLTAG = (uint8_t) value;
                                }
#endif
#ifdef GPS
                            } else if ( strncmp(key, "prio_GPSLON", 11 ) == 0 ) {
                                if ( value <= 20 ) {
                                    config.prio_GPSLON = (uint8_t) value;
                                }
                            } else if ( strncmp(key, "prio_GPSLAT", 11 ) == 0 ) {
                                if ( value <= 20 ) {
                                    config.prio_GPSLAT = (uint8_t) value;
                                }
                            } else if ( strncmp(key, "prio_GPSSPD", 11 ) == 0 ) {
                                if ( value <= 20 ) {
                                    config.prio_GPSSPD = (uint8_t) value;
                                }
                            } else if ( strncmp(key, "prio_GPSALT", 11 ) == 0 ) {
                                if ( value <= 20 ) {
                                    config.prio_GPSALT = (uint8_t) value;
                                }
                            } else if ( strncmp(key, "prio_GPSTIM", 11 ) == 0 ) {
                                if ( value <= 20 ) {
                                    config.prio_GPSTIM = (uint8_t) value;
                                }
                            } else if ( strncmp(key, "prio_GPSSAT", 11 ) == 0 ) {
                                if ( value <= 20 ) {
                                    config.prio_GPSSAT = (uint8_t) value;
                                }
                            } else if ( strncmp(key, "prio_GPSDIS", 11 ) == 0 ) {
                                if ( value <= 20 ) {
                                    config.prio_GPSDIS = (uint8_t) value;
                                }
                            } else if ( strncmp(key, "prio_GPSTRA", 11 ) == 0 ) {
                                if ( value <= 20 ) {
                                    config.prio_GPSTRA = (uint8_t) value;
                                }
                            } else if ( strncmp(key, "prio_GPSHAC", 11 ) == 0 ) {
                                if ( value <= 20 ) {
                                    config.prio_GPSHAC = (uint8_t) value;
                                }
                            } else if ( strncmp(key, "prio_GPSVAC", 11 ) == 0 ) {
                                if ( value <= 20 ) {
                                    config.prio_GPSVAC = (uint8_t) value;
                                }
                            } else if ( strncmp(key, "prio_GPSHEA", 11 ) == 0 ) {
                                if ( value <= 20 ) {
                                    config.prio_GPSHEA = (uint8_t) value;
                                }
#endif
#ifdef BARO
                            } else if ( strncmp(key, "enab_VARIOM", 11 ) == 0 ) {
                                if ( value <= 1 ) {
                                    config.enab_VARIOM = (uint8_t) value;
                                }
                            } else if ( strncmp(key, "enab_ALTITU", 11 ) == 0 ) {
                                if ( value <= 1 ) {
                                    config.enab_ALTITU = (uint8_t) value;
                                }
#endif
#ifdef GPS
                            } else if ( strncmp(key, "enab_GPSLON", 11 ) == 0 ) {
                                if ( value <= 1 ) {
                                    config.enab_GPSLON = (uint8_t) value;
                                }
                            } else if ( strncmp(key, "enab_GPSLAT", 11 ) == 0 ) {
                                if ( value <= 1 ) {
                                    config.enab_GPSLAT = (uint8_t) value;
                                }
                            } else if ( strncmp(key, "enab_GPSSPD", 11 ) == 0 ) {
                                if ( value <= 1 ) {
                                    config.enab_GPSSPD = (uint8_t) value;
                                }
                            } else if ( strncmp(key, "enab_GPSALT", 11 ) == 0 ) {
                                if ( value <= 1 ) {
                                    config.enab_GPSALT = (uint8_t) value;
                                }
                            } else if ( strncmp(key, "enab_GPSTIM", 11 ) == 0 ) {
                                if ( value <= 1 ) {
                                    config.enab_GPSTIM = (uint8_t) value;
                                }
                            } else if ( strncmp(key, "enab_GPSSAT", 11 ) == 0 ) {
                                if ( value <= 1 ) {
                                    config.enab_GPSSAT = (uint8_t) value;
                                }
                            } else if ( strncmp(key, "enab_GPSDIS", 11 ) == 0 ) {
                                if ( value <= 1 ) {
                                    config.enab_GPSDIS = (uint8_t) value;
                                }
                            } else if ( strncmp(key, "enab_GPSTRA", 11 ) == 0 ) {
                                if ( value <= 1 ) {
                                    config.enab_GPSTRA = (uint8_t) value;
                                }
                            } else if ( strncmp(key, "enab_GPSHAC", 11 ) == 0 ) {
                                if ( value <= 1 ) {
                                    config.enab_GPSHAC = (uint8_t) value;
                                }
                            } else if ( strncmp(key, "enab_GPSVAC", 11 ) == 0 ) {
                                if ( value <= 1 ) {
                                    config.enab_GPSVAC = (uint8_t) value;
                                }
                            } else if ( strncmp(key, "enab_GPSHEA", 11 ) == 0 ) {
                                if ( value <= 1 ) {
                                    config.enab_GPSHEA = (uint8_t) value;
                                }
#endif
#ifdef VOLT
                            } else if ( strncmp(key, "enab_VOLTAG", 11 ) == 0 ) {
                                if ( value <= 1 ) {
                                    config.enab_VOLTAG = (uint8_t) value;
                                }
#endif
#ifdef BARO 
                            } else if ( strncmp(key, "ctrl_CHANNL", 11 ) == 0 ) {
                                if ( value < 16 ) {
                                    config.ctrl_CHANNL = (uint8_t) value - 1;
                                }
#endif
#if defined (GPS) || defined (BARO)
                            } else if ( strncmp(key, "rset_CHANNL", 11 ) == 0 ) {
                                if ( value < 16 ) {
                                    config.rset_CHANNL = (uint8_t) value - 1;
                                }
#endif
#ifdef SERVO
                            } else if ( strncmp(key, "fuse_WDTIME", 11 ) == 0 ) {
                                if ( value <= 1 ) {
                                    config.fuse_WDTIME = (uint8_t) value;
                                }
                            } else if ( strncmp(key, "puls_TIMING", 11 ) == 0 ) {
                                if ( value >= 10 || value <= 40 ) {
                                    config.puls_TIMING = (uint8_t) value;
                                }
                            } else if ( strncmp(key, "srv1_CHANNL", 11 ) == 0 ) {
                                if ( value <= 16 ) {
                                    config.srv1_CHANNL = (uint8_t) value - 1;
                                }
                            } else if ( strncmp(key, "srv2_CHANNL", 11 ) == 0 ) {
                                if ( value <= 16 ) {
                                    config.srv2_CHANNL = (uint8_t) value - 1;
                                }
                            } else if ( strncmp(key, "srv3_CHANNL", 11 ) == 0 ) {
                                if ( value <= 16 ) {
                                    config.srv3_CHANNL = (uint8_t) value - 1;
                                }
                            } else if ( strncmp(key, "srv4_CHANNL", 11 ) == 0 ) {
                                if ( value <= 16 ) {
                                    config.srv4_CHANNL = (uint8_t) value - 1;
                                }
                            } else if ( strncmp(key, "srv1_FAILSV", 11 ) == 0 ) {
                                if ( value >= 1000 && value <= 2000 ) {
                                    config.srv1_FAILSV = (uint16_t) value;
                                }
                            } else if ( strncmp(key, "srv2_FAILSV", 11 ) == 0 ) {
                                if ( value >= 1000 && value <= 2000 ) {
                                    config.srv2_FAILSV = (uint16_t) value;
                                }
                            } else if ( strncmp(key, "srv3_FAILSV", 11 ) == 0 ) {
                                if ( value >= 1000 && value <= 2000 ) {
                                    config.srv3_FAILSV = (uint16_t) value;
                                }
                            } else if ( strncmp(key, "srv4_FAILSV", 11 ) == 0 ) {
                                if ( value >= 1000 && value <= 2000 ) {
                                    config.srv4_FAILSV = (uint16_t) value;
                                }
                            } else if ( strncmp(key, "dlay_FAILSV", 11 ) == 0 ) {
                                if ( value <= 10 ) {
                                    config.dlay_FAILSV = value;
                                }
#endif
#ifdef VOLT
                            } else if ( strncmp(key, "enab_CALIBR", 11 ) == 0 ) {
                                if ( value <= 1 ) {
                                    config.enab_CALIBR = (uint8_t) value;
                                }
                            } else if ( strncmp(key, "high_MEASUR", 11 ) == 0 ) {
                                if ( value <= 33 ) {
                                    config.high_MEASUR = value;
                                }
                            } else if ( strncmp(key, "lowr_MEASUR", 11 ) == 0 ) {
                                if ( value <= 33 ) {
                                    config.lowr_MEASUR = value;
                                }
                            } else if ( strncmp(key, "high_VOLTAG", 11 ) == 0 ) {
                                if ( value <= 33 ) {
                                    config.high_VOLTAG = value;
                                }
                            } else if ( strncmp(key, "lowr_VOLTAG", 11 ) == 0 ) {
                                if ( value <= 33 ) {
                                    config.lowr_VOLTAG = value;
                                }
#endif
                            }
                            // resets input field also
                            showConf(config, flashConfig);
                    }

                    //clear command buffer after command was processed
                    for(i=0; i<sizeof(command); i++) {
                        command[i] = 0;
                    }
                    break;

                // accept both as backspace
                case '\b':
                case 0x7F:

                    if (delete_unwanted) { //ESC was detected
                        pos = 0;
                        command[pos] = 0;
                        for(i=0; i<sizeof(command); i++) {
                            command[i] = 0;
                        }
                        SerialUSB.print(CURS_C10 "                  " CURS_C10); //clear input field
                        delete_unwanted = false;
                        break;
                    }

                    if (pos > 0) {
                        command[--pos] = 0;
                        SerialUSB.print("\b \b");
                    }

                    break;

                case  0x1B:
                    delete_unwanted = true;

                default:
                    if (pos < 17) {
                        command[pos++] = c;
                    }
            }
        }
    }
}
