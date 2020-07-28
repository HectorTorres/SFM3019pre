#include "sfm3019.h"
#include <stdint.h>
#include "wiringPi.h"
#include "wiringPiI2C.h"
#include <stdio.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <errno.h>
#include <unistd.h>
#include <QProcess>
#include <QDebug>

SFM3019::SFM3019()
{

}

int SFM3019::addTest(int a, int b){
    return (a + b)+2;
}

int SFM3019::checkCRC(int data[2]){
    //uint8_t byteCtr; //Inicializamos la variable
    uint8_t calc_crc = 0xFF; //Inicializamos la varibale CRC, ojo revisar por que es común que inicialice en 0x00
    for (uint8_t byteCtr = 0; byteCtr < 2; ++byteCtr){
        calc_crc ^= (data[byteCtr]); //Se aaplica la función XOR de manera acumulada.
        for (int i = 8; i > 0; --i){
            if (calc_crc & 0x80) {
                calc_crc = uint8_t((calc_crc << 1) ^ 0x131);}
            else {
                calc_crc = uint8_t((calc_crc << 1)); }
        }
    }
    return calc_crc;
}


void SFM3019::startContMeas(int SFM3019id, uint8_t command){
    //definir las condiciones para command
    uint8_t StartCond[2];
    StartCond[0]=0x36;
    if(command == GAS1_AIR){
        StartCond[1]=0x08;
        write(SFM3019id,StartCond,2);
        QProcess::execute("sudo i2cset -y 1 0x2e 0x36 0x08");
    }
    else if(command == GAS0_O2){
        StartCond[1]=0x03;
        write(SFM3019id,StartCond,2);
        QProcess::execute("sudo i2cset -y 1 0x2e 0x36 0x08");
    }
    else if(command == GAS_MIX){
        StartCond[1]=0x32;
        write(SFM3019id,StartCond,2);
        QProcess::execute("sudo i2cset -y 1 0x2e 0x36 0x08");
    }
    else {
        qDebug() << "Comando erroneo";
    }
}

int SFM3019::scaleFactorAndOffset(int SFM3019id){

    uint8_t ReadScaleOffset[2];
    ReadScaleOffset[0]=0x36;
    ReadScaleOffset[1]=0x61;
    write(SFM3019id,ReadScaleOffset,2);
    QProcess::execute("sudo i2cset -y 1 0x2e 0x36 0x61");
    return 1;
}

int *SFM3019::readScaleOffset(int SFM3019id){
    static int sensirionData[3];
    uint8_t bufR[9];
    delayMicroseconds(20000);
    read(SFM3019id,bufR,9);
    int16_t scaleFactor = int16_t((bufR[0]<<8)+bufR[1]);
    int16_t offset = int16_t((bufR[3]<<8)+bufR[4]);
    int16_t flowUnit = int16_t((bufR[6]<<8)+bufR[7]);
    sensirionData[0] = scaleFactor;
    sensirionData[1] = offset;
    sensirionData[2] = flowUnit;
    return sensirionData;
}

double SFM3019::readFlow(int SFM3019id){
    uint8_t bufR[9];
    delayMicroseconds(200); // Revisar quitar de aqui cuando se hay validado.
    read(SFM3019id,bufR,2);
    int16_t flowData = int16_t((bufR[0]<<8)+bufR[1]);
    double flowMeas = (double(flowData)+24576.0)/170.0;
    return flowMeas;
}

int *SFM3019::readFlowTempSt(int SFM3019id, bool checkSumOn){
    static int sensirionData[3];
    //int *dataReadedPointer;
    //dataReadedPointer = readFlowTempSt(fn);
    //double FlowData = *(dataReadedPointer+0);
    //double FlowData = *(dataReadedPointer+1);
    uint8_t bufR[9];
    delayMicroseconds(20000);// Revisar quitar de aqui cuando se hay validado.
    read(SFM3019id,bufR,9);

    int16_t flowData = int16_t((bufR[0]<<8)+bufR[1]);
    int16_t tempData = int16_t((bufR[3]<<8)+bufR[4]);
    int16_t statusData = int16_t((bufR[6]<<8)+bufR[7]);

    sensirionData[0] = int(((double(flowData)+24576.0)/170.0)*1000);
    sensirionData[1] = ((tempData-0)/200)*1000;
    sensirionData[2] = statusData;

    if(checkSumOn){
        int dataCRC[2];
        dataCRC[0] = bufR[0];
        dataCRC[1] = bufR[1];
        int CRC = this->checkCRC(dataCRC);
        if(CRC == bufR[2]){
            return sensirionData;
        }
        else {
            return nullptr;
        }
    }
    else{
        return sensirionData;
    }
}

int SFM3019::initSensorI2C(void){
    return wiringPiI2CSetup(0x2e);
}


int SFM3019::checkId(int SFM3019id){
    uint8_t productId[2];
    productId[0]=0xE1;
    productId[1]=0x02;
    write(SFM3019id,productId,2);
    QProcess::execute("sudo i2cset -y 1 0x2e 0xe1 0x02");

    uint8_t bufR[2];
    delayMicroseconds(20000);
    read(SFM3019id,bufR,2);
    int16_t idFlow = int16_t((bufR[0]<<8)+bufR[1]);
    qDebug() << "-------------ID: " << idFlow;
    return 1;
}






