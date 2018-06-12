#ifndef _HELPERS_HPP_
#define _HELPERS_HPP_

#include <mbed.h>
#include "iniparser.h"
#include <config.hpp>

extern configStruct config;

void loadConfig(void){
    LocalFileSystem local("local");
    dictionary *dir = iniparser_load("/local/config.ini");

    // Read config for radio
    config.radioConfig.channel = iniparser_getint(dir, "radio:channel",101);
    config.radioConfig.txAddress = iniparser_getlongint(dir, "radio:txaddress",0x007DEADBEE);
    config.radioConfig.rxAddress = iniparser_getlongint(dir, "radio:rxaddress",0x007DEADBEE);
    config.radioConfig.transferSize = iniparser_getint(dir, "radio:transfersize",17);

    // Read remote prescalers
    config.controllerConfig.prescaler[0] = (float)iniparser_getdouble(dir, "controller:prescaler_roll",0);
    config.controllerConfig.prescaler[1] = (float)iniparser_getdouble(dir, "controller:prescaler_pitch",0);
    config.controllerConfig.prescaler[2] = (float)iniparser_getdouble(dir, "controller:prescaler_yaw",0);
    config.controllerConfig.imuPrescaler[0] = (float)iniparser_getdouble(dir, "controller:prescaler_roll_imu",0);
    config.controllerConfig.imuPrescaler[1] = (float)iniparser_getdouble(dir, "controller:prescaler_pitch_imu",0);
    config.controllerConfig.imuPrescaler[2] = (float)iniparser_getdouble(dir, "controller:prescaler_yaw_imu",0);


    // Read config for ACRO mode
    char * keysAcro[9];
    const char ** keysAcroPtr = (const char **) &keysAcro;
    keysAcroPtr = iniparser_getseckeys(dir, "acromode", keysAcroPtr);
    for (int i = 0; i<=2; i++){
        config.controllerConfig.acroModeConfig.Kp[i] = (float)iniparser_getdouble(dir, (keysAcro[i]),0);
    }
    for (int i = 0; i<=2; i++){
        config.controllerConfig.acroModeConfig.Ki[i] = (float)iniparser_getdouble(dir, (keysAcro[i+3]),0);
    }
    for (int i = 0; i<=2; i++){
        config.controllerConfig.acroModeConfig.Kd[i] = (float)iniparser_getdouble(dir, (keysAcro[i+6]),0);
    }

    // Read config for stabilizing mode
    char * keysStabilizing[9];
    const char ** keysStabilizingPtr = (const char **) &keysStabilizing;
    keysStabilizingPtr = iniparser_getseckeys(dir, "stabilizingmode", keysStabilizingPtr);
    for (int i = 0; i<=2; i++){
        config.controllerConfig.stabilizingModeConfig.Kp[i] = (float)iniparser_getdouble(dir, (keysStabilizing[i]),0);
    }
    for (int i = 0; i<=2; i++){
        config.controllerConfig.stabilizingModeConfig.Ki[i] = (float)iniparser_getdouble(dir, (keysStabilizing[i+3]),0);
    }
    for (int i = 0; i<=2; i++){
        config.controllerConfig.stabilizingModeConfig.Kd[i] = (float)iniparser_getdouble(dir, (keysStabilizing[i+6]),0);
    }

    // Read config for motor direction compensation
    char * keysSigns[12];
    const char ** keysSignsPtr = (const char **) &keysSigns;
    keysSignsPtr = iniparser_getseckeys(dir, "motordirections", keysSignsPtr);
    for (int i = 0; i<=3; i++){
        for (int j = 0; j<=2; j++){
            config.controllerConfig.signs[i][j] = iniparser_getint(dir, (keysSigns[3*i+j]),0);
        }
    }

    // Reading filter values
    config.imuconfig.itg3200.a = (float)iniparser_getdouble(dir, "itg3200:a",0);
    config.imuconfig.itg3200.b = (float)iniparser_getdouble(dir, "itg3200:b",0);
    config.imuconfig.itg3200.c = (float)iniparser_getdouble(dir, "itg3200:c",0);

    config.imuconfig.hmc5883l.a = (float)iniparser_getdouble(dir, "hmc5883l:a",0);
    config.imuconfig.hmc5883l.b = (float)iniparser_getdouble(dir, "hmc5883l:b",0);
    config.imuconfig.hmc5883l.c = (float)iniparser_getdouble(dir, "hmc5883l:c",0);

    config.imuconfig.adxl345.a = (float)iniparser_getdouble(dir, "adxl345:a",0);
    config.imuconfig.adxl345.b = (float)iniparser_getdouble(dir, "adxl345:b",0);
    config.imuconfig.adxl345.c = (float)iniparser_getdouble(dir, "adxl345:c",0);

    config.flightTickerFrequency = (uint16_t) iniparser_getint(dir, "misc:flighttickerfrequency",1000);
    config.gyroTickerFrequency = (uint16_t) iniparser_getint(dir, "misc:gyrofrequency",1000);
    config.angleTickerFrequency = (uint16_t) iniparser_getint(dir, "misc:anglefrequency",250);
    iniparser_freedict(dir);
}

#endif