#ifndef SFM3019_H
#define SFM3019_H
#include <stdint.h>

#define GAS0O2  0x3603
#define GAS1AIR 0x3608
#define GASMIX  0x3632

#define CHECKSUM_ON     1
#define CHECKSUM_OFF    0

#define GAS0_O2     0x03
#define GAS1_AIR    0x08
#define GAS_MIX     0x32




class SFM3019
{
public:
    SFM3019();
    int addTest(int a, int b);
    int initSensorI2C();
    int checkCRC(int data[2]);
    void startContMeas(int SFM3019id, uint8_t command);
    int scaleFactorAndOffset(int SFM3019id);
    double readFlow(int SFM3019id);
    int *readFlowTempSt(int SFM3019id, bool checkSumOn = CHECKSUM_OFF);
    int *readScaleOffset(int SFM3019id);
    int checkId(int SFM3019id);
};



#endif // SFM3019_H
