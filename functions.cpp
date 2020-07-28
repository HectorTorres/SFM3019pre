#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "wiringPiI2C.h"
#include "sfm3019.h"

#include <stdio.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <errno.h>
#include <unistd.h>

#include <QtDebug>
#include <QTimer>
#include <QDesktopWidget>
#include <QScreen>
#include <QMessageBox>
#include <QMetaEnum>
#include <math.h>

#define DEBUG_STATUS 1
#define DEBUG_STATUS_HIGH_SPEED 0

#define TIMER_DELAY 40
#define TIMER_PLOT 200
#define TIMER_SENSOR 40

#define RISE_TIME_COMPENSATOR 50

#define TRESHOLD_TA 7
#define PEEP_VAL 5
#define TRESHOLD_TD 15

#define PRESS_LIMIT_MAX 50

#define DEBOUNCE_TIME 500000

#define ValveExp 7

#define RG 2000
#define V0 1600
#define Vmax 26392
#define SLOPE_PRESSURE_SENSOR 0.26

#define AD_BASE 120

#define DATA 12
#define CLK 12
#define LATCH 12

#define REL1 9
#define REL2 11
#define REL3 5
#define REL4 10
#define REL5 6
#define REL6 13
#define REL7 19
#define REL8 1

#define RELE1 16
#define RELE2 20
#define RELE3 12
#define RELE4 21

#define AIR_PRESS_ALARM 4 //Por definir
#define O2_PRESS_ALARM 17 //Por definir



/**************************************************************************************************************************************/
//INICIO.
/**************************************************************************************************************************************/
/*------------------------------------------------------------------------------------------------------------------------------------*/
/* BOTÓN DE INICIO */
void MainWindow::on_pushButton_start_clicked()
{
    if(timerStatusFlag){
        valvesMainControl(0);
        valvesExControl(1);
        timeFromInit=0;
        controlTimer->stop();
        sensorTimer->stop();
        plotTimer->stop();
        ui->pushButton_start->setText("Inicio.");
        timerStatusFlag=false;
        timeFromStart = 0;
        if(DEBUG_STATUS){   qDebug() << "Timer Stops."; }
        digitalWrite(RELE4,LOW);
    }
    else {
            //validacion();
            evalVel();
            readedPressBottom = true;
            timeFRvTemporal = millis();
            timeMasterControl = millis();
            timeFromStart=millis();
            espontaneoPeriod = millis();
            inspirationTimeTop = false;  // REVISARRRRRRRRRRRRRRRRRRRRRR para evitar el error de el inicio y/o primer inspiración.
            vavleChange=true;
            controlTimer->start(TIMER_DELAY);
            sensorTimer->start(TIMER_SENSOR);
            plotTimer->start(TIMER_PLOT);
            ui->pushButton_start->setText("Paro.");
            timerStatusFlag=true;
            if(DEBUG_STATUS){   qDebug() << "Timer Starts."; }
            //valvesMainControl(0); // REVISARRRRRRRRRRRRRRRRRRRRRR para evitar el error de el inicio y/o primer inspiración.
        }
}

/**************************************************************************************************************************************/
//TEMPORAIZADOR DE SENSORES.
/**************************************************************************************************************************************/
/*------------------------------------------------------------------------------------------------------------------------------------*/
/* SENSOR TIMER FUNCTION
 * reads and checs the preassure sensor data */
void MainWindow::sensorTimerFunction(){

    if((millis() - tiempoTemporalS) >= (TIMER_DELAY+3)){
    qDebug() << "Tiempo sensor: " << millis() - tiempoTemporalS;
    }
    tiempoTemporalS = millis();

     readedPress=pressureRead();


     if(readedPress>maxReadedPress){
        maxReadedPress=readedPress;
     }

     pressProm[0] = pressProm[1]; pressProm[1] = pressProm[2]; pressProm[2] = pressProm[3]; pressProm[3] = pressProm[4]; pressProm[4] = pressProm[5];  pressProm[5] = readedPress;
     double readedPressProm = (pressProm[0]+pressProm[1]+pressProm[2]+pressProm[3]+pressProm[4]+pressProm[5])/6.0;
     //readedPress = readedPressProm;

     if(ui->tabWidget_sel->currentIndex()==0){
        if(readedPress >= setPIP*1.05){
            valvesMainControl(0); //Cerrar valvulas
            valvesValueControl--;
            qDebug() << "Cerrando valvulas. Nuevo valor: "  << valvesValueControl;
        }
     }

     //Control de PEEP por valvula.
     if((inspirationTimeTop==true) && (readedPress <= (setPEEP+offsetPEEP))){
         slopeFilterTime = millis();
         valvesExControl(0x00);
         PEEPOnControl = true;
     }

     if(PEEPOnControl){
         PEEPData.replace(indexPEEP,readedPress);
         indexPEEP++;
     }

/* //Borrar una vez validadas las alarmas.
     if(ui->tabWidget_sel->currentIndex()==0){
        if(readedPress >= maxPressLimit){
            //valvesMainControl(0x00);
            //valvesExControl(0x01);
            activateAlarm(1);
            qDebug() << "Alarma de activación alta.";
        }
     }*/
/*
     if(readedPress <= 1 && ((millis()-timeFromStart) >= 3000)){
        valvesMainControl(0x00);
        valvesExControl(0x01);
        activateAlarm(2);
        qDebug() << "Alarma de activación baja.";
     }*/
/*
    if((readedPress/readedPressTempD) >= 4 || (readedPress/readedPressTempD) <= 0.1 ){
        readedPress=readedPressTempD;
    }*/

    readedPressTempD = readedPress;

    //Pendiente--------------------------------------------------------
    if(indexPress<=4){
        pressSlope.replace(indexPress,readedPressProm);
    }
    else {
        if((millis()-slopeFilterTime)>=400){
            std::rotate(pressSlope.begin(),pressSlope.begin()+1,pressSlope.end());
            pressSlope.replace(4,readedPressProm);
        }
    }
   double slope=0;
   slope=std::accumulate(pressSlope.begin(),pressSlope.end(),slope);
   slope=atan(((pressSlope[0]-pressSlope[1])+(pressSlope[1]-pressSlope[2])+(pressSlope[2]-pressSlope[3])+(pressSlope[3]-pressSlope[4]))/5)*180/3.1415;
   //qDebug() << "Slope: " << slope;

   if((ui->radioButton_assit->isChecked() || ui->radioButton_esp->isChecked()) && (millis()-timeInPeak)>=(periodTime/5.0)){
        if(slope >= double(ui->horizontalSlider_sensibilidad->value())*6.1111+23.889){ // f001 Evaluación de el nuevo valor de la pendiente.
            qDebug() << "Slope angle inspiratio:" <<slope;
            timeMasterControl+=double(60.0/double(FRv))*1000;
            inspirationDetected = true;
        }
   }

    if(slopeIncrease == true && (millis()-timerSlopeIncrease) >= timeFRv){
        slopeIncrease = false;
    }

    if(!ui->radioButton_esp->isChecked()){
        if(ui->checkBox_alarmNoConnection->isChecked()){
            //qDebug() << "sLOPE: " << slope << "  slopeFlat: " << slopeFlat;
            if(slope<=0.2 && slope>=-0.2 && slopeFlat == false){
                qDebug() << "Deteccion de desconexión";
                timeFlatSlope = millis();
                slopeFlat=true;
        }

    //Evitar errores en alarma de desconexión
        if(((slope>=1) || (slope<=-1)) && (slopeFlat == true)){
            slopeFlat = false;
        }

   if(slope<=0.2 && slope>=-0.2 && slopeFlat == true && (millis()-timeFlatSlope)>6000){
       qDebug() << "Deteccion de desconexión confirmada";
       estadoDesconexion = " | Desconectado!";
       noConection = true;
       alarmOn = true;
       alarmText = " Deconexión. |";
       //ui->label_estado->setText(ui->label_estado->text() + " | DesconexiÓn.");
       timeFlatSlope = 0;
       //activateAlarm(9);
       slopeFlat=false;
    }

   //Re-activar la variable de control de PEEP offset cuando vuelva a haber presión.
   if(readedPress>=2.5){
       noConection = false;
   }

   if((millis()-timeFlatSlope)>10000){
       slopeFlat=false;
    }
    }
    }

    //Evitar errores en la lectura del sensor de flujo
   readedFlow =(flowRead(initI2C));
   if((readedFlow>=0) && (readedFlow<=2000)){
   }
   else {
       readedFlow = 0;
   }
   //qDebug() << readedFlow;

   flowProm[0] = flowProm[1]; flowProm[1] = flowProm[2]; flowProm[2] = flowProm[3]; flowProm[3] = flowProm[4]; flowProm[4] = flowProm[5];  flowProm[5] = readedFlow;
   double readedFlowProm = (flowProm[0]+flowProm[1]+flowProm[2]+flowProm[3]+flowProm[4]+flowProm[5])/6.0;
   readedFlow = readedFlowProm;

   if(flowDataReadStatus==false){
        volTemp=0;
    }

   if(readedFlow>=maxFlow){
    maxFlow = readedFlow;
   }

    readedVol = volRead(readedFlow);

    //Almacenar los datos en los vectores_______________________________________________
    pressData.replace(indexPress,readedPress);
    flowData.replace(indexPress,readedFlow);
    volData.replace(indexPress,readedVol);

    dataPress.replace(indexPress,QCPGraphData(indexPress, cos(2*3.1416*0.008*temporal*indexPress)*50+50));
    dataFlow.replace(indexPress,QCPGraphData(indexPress, cos(2*3.1416*0.009*temporal*indexPress)*1800+2000));
    dataVol.replace(indexPress,QCPGraphData(indexPress, cos(2*3.1416*0.006*temporal*indexPress)*50+50));

    indexPress++;
    if(indexPress>=251) {
        temporal++;
        indexPress=0;
        ui->customPlot->graph(0)->data()->clear();
        ui->customPlot->graph(1)->data()->clear();
        ui->customPlot->graph(2)->data()->clear();
    }

    //Alarma de presion_________________________________________________________
    ui->label_press_5->setText(QString::number(readedPressProm));
    if((readedPress >= maxPressLimit)  && (alarmOn == false) ){
        alarmOn = true;
        alarmText+= " Presión alta.|";
    }


}

/**************************************************************************************************************************************/
//TEMPORAIZADOR DE CONTROL PRINCIPAL.
/**************************************************************************************************************************************/
/*------------------------------------------------------------------------------------------------------------------------------------*/
void MainWindow::controlTimerFunction(){


        if((millis() - tiempoTemporal) >= (TIMER_DELAY+5)){
        qDebug() << "Tiempo control: " << millis() - tiempoTemporal;
        }
        tiempoTemporal = millis();

        periodTime = double(60.0/double(FRv))*1000;
        inspirationTime = periodTime/(ieRatioRef+1);

        //if(ui->radioButton_esp->isChecked())

        //Inspiracion mandatoria y asistida
        if(!ui->radioButton_esp->isChecked()){

        if(((millis()-timeMasterControl) >= uint32_t(inspirationTime)) && (inspirationTimeTop == false)){
            timeInPeak = millis();
            qDebug() << "Time up: " << timeInPeak;
            valvesMainControl(0x00);
            delayMicroseconds(10000);
            valvesExControl(0x01);
            inspirationTimeTop = true;
            flowDataReadStatus = false;

        // Sistema de control básico./////////////////////////////////////////////////////
         if(ui->checkBox_control->isChecked()){
             if(ui->tabWidget_sel->currentIndex()==0 || ((ui->tabWidget_sel->currentIndex()==2) && (pressControlActive==true))){

                 int errorPress = int((setPIP-readedPress)*Kp);
                 qDebug() << "Compensador: " << errorPress;
                 if(currentMaxPress <= setPEEP+2) {errorPress = 0;}
                 valvesValueControl+=errorPress; //Validar por conversion implicita
                 qDebug() << "Valvulas: " << valvesValueControl;
            }
             if(ui->tabWidget_sel->currentIndex()==1 || ((ui->tabWidget_sel->currentIndex()==2) && (volControlActive==true))){
                 if(readedVol<=setVOL){
                     valvesValueControl++;
                 }
                 else {
                     valvesValueControl--;
                 }

             }
          }

        ui->label_vti->setText(QString::number(readedVol));

        if(ui->tabWidget_sel->currentIndex()==0 || ((ui->tabWidget_sel->currentIndex()==2) && (pressControlActive==true))){
            pressError = ((readedPress-setPIP)/setPIP)*100.0;
            if(pressError > 5 || pressError < -5){
                alarmControl= true;
                //digitalWrite(RELE4,HIGH);
                ui->label_ajuste->setText("Ajustando.");
                ui->label_ajuste->setStyleSheet("color: rgb(170, 0, 0)");
                if(pressError > 2 || pressError < -2){
                    ui->checkBox_control->setChecked(true);
                }
            }
            if(pressError < 5 && pressError > -5){
                 alarmControl= false;
                ui->label_ajuste->setText("Estable.");
                ui->label_ajuste->setStyleSheet("color: rgb(0, 85, 0)");
                if(pressError < 2 && pressError > -2){
                    ui->checkBox_control->setChecked(false);
                }
            }
            currentMaxPress = readedPress;
            ui->label_press_5->setText(QString::number(currentMaxPress));
            qDebug() << "Presión máxima: " << readedPress << " Presion Objetivo: " << setPIP << " Porcentaje de diferencia: " << pressError << "%   Valvula: " << valvesValueControl;
            qDebug() << "Press: " << pressControlActive << " Vol: " << volControlActive;
        }
        if(ui->tabWidget_sel->currentIndex()==1 || ((ui->tabWidget_sel->currentIndex()==2) && (volControlActive==true))){
            volError = ((readedVol-setVOL)/setVOL)*100.0;
            if(volError > 5 || volError < -5){
                //digitalWrite(RELE4,HIGH);
                //ui->checkBox_control->setChecked(true);
                ui->label_ajuste->setText("Ajustando.");
                ui->label_ajuste->setStyleSheet("color: rgb(170, 0, 0)");
                if(volError > 2 || volError < -2){
                    ui->checkBox_control->setChecked(true);
                }
            }
            if(volError < 5 && volError > -5){
                //ui->checkBox_control->setChecked(false);
                ui->label_ajuste->setText("Estable.");
                ui->label_ajuste->setStyleSheet("color: rgb(0, 85, 0)");
                if(volError < 2 && volError > -2){
                    ui->checkBox_control->setChecked(false);
                }
            }
            qDebug() << "Volumen máxima: " << readedVol << " Vol Objetivo: " << setVOL << " Porcentaje de diferencia: " << volError << "%   Valvula: " << valvesValueControl;
            qDebug() << "Press: " << pressControlActive << " Vol: " << volControlActive;
        }
        }

/* //Borrar al validar las alarmas.
        if(millis()-timeInPeak >= 500 && ((alarmPressInputO2 == false) && (alarmPressInput == false))){
             // Desactivar audio
        }*/
/*
        if(ui->radioButton_esp->isChecked()){
            ui->tabWidget_sel->setTabEnabled(1,false);
        }
        else{
            ui->tabWidget_sel->setTabEnabled(1,true);
        }
*/
        //Fin de periodo////////////////////////////////////////////
        if(((millis()-timeMasterControl)>=periodTime) && (inspirationTimeTop == true)){
            ui->label_press_maxPress->setText(QString::number(maxReadedPress,'f',2));
            maxReadedPress = 0;
            slopeFilterTime = millis();
            qDebug() << "Period: " << millis();
            PEEPOnControl = false;

            if(indexPEEP >= 4){
            PEEPaverage = std::accumulate(PEEPData.begin()+3, PEEPData.begin()+indexPEEP, PEEPaverage);
            //qDebug() << PEEPData << " Index: " << indexPEEP;
            PEEPaverage = (PEEPaverage/(indexPEEP+1-3));
            PEEPaverage  = 1.124*PEEPaverage -1.0792;

            if((PEEPaverage >= setPEEP+0.1) || (PEEPaverage <= setPEEP+0.1)){
                offsetPEEP+=(setPEEP-PEEPaverage)*0.3;
            }

            if(noConection){ // Esto para evitar un sobrecontrol cuando e desconecte el pulmón. Codigo 654825
                offsetPEEP=0;
            }

            indexPEEP = 0;
            qDebug() << "PEEP Value: "<< PEEPaverage << " PEEP Diff: " << (setPEEP-PEEPaverage) << " PEEP Offset: " << offsetPEEP;
            ui->label_PEEP->setText(QString::number(PEEPaverage,'g',3));
            PEEPaverage = 0;
            }
            else{
                PEEPaverage = 0;
                indexPEEP = 0;
                offsetPEEP = -2;
            }

            cicleCounter++;
            ui->label_ciclos->setText(QString::number(cicleCounter));

            periodTimeRead = (millis()-timeMasterControl);

            //Alarmas///////////////////////////////////////////////
            /*----------------------------------------------------*/


            //Alarma de flujo________________________________________________
            //if(!inspirationTimeTop && (volControlActive== true)){
            if(volControlActive== true){

            double flowLm = ((setVOL/(inspirationTime/1000)))*0.06;
            ui->label_volSetmin->setText(QString::number(flowLm-flowLm*(flowError/100),'f',4));
            ui->label_volSetmax->setText(QString::number(flowLm+flowLm*(flowError/100),'f',4));
            ui->label_flowAlarm->setText(QString::number(maxFlow*0.06,'f',4)); //Conversión de ml/s a l/min

            if(((cicleCounter-alarmCurrenCiclosVol)>=4) && ((cicleCounter-alarmCurrenCiclosFR)>=6)){

            if((maxFlow*0.06 >= flowLm+flowLm*(flowError/100))  && (alarmOn == false)){
                alarmOn = true;
                alarmText+= " Vol/min alto.|";
            }
            if((maxFlow*0.06 <= flowLm-flowLm*(flowError/100))  && (alarmOn == false)){
                alarmOn = true;
                alarmText+= " Vol/min bajo.|";
            }
            }
            }
            maxFlow=0;

            /*----------------------------------------------------*/
            ui->label_fr_current->setText(QString::number((1000.0/double(millis()-timeMasterControl))*60.0,'f',1));
            ui->label_fr_current2->setText(QString::number((1000.0/double(millis()-timeMasterControl))*60.0,'f',1));

            //Alarma de fr____________________________
            frCurrent = (1000.0/double(millis()-timeMasterControl))*60.0;
            qDebug() << "FR: " << frCurrent << "  menor que: " << (1000.0/double(millis()-timeMasterControl))*60.0;
            //ui->label_fr_current2->setText(QString::number(frCurrent,'f',1));
            ui->label_frSetmax->setText(QString::number(FRv+FRv*(frError/100),'f',1));
            ui->label_frSetmin->setText(QString::number(FRv-FRv*(frError/100),'f',1));

            if((cicleCounter-alarmCurrenCiclosFR)>=4 && (cicleCounter-alarmCurrenCiclosVol)>=4){
                if((frCurrent >= FRv+FRv*(frError/100)) && (alarmOn == false)){
                    alarmOn = true;
                    alarmText+= " FR alto.|";
                }
                if((frCurrent <= FRv-FRv*(frError/100)) && (alarmOn == false)){
                    alarmOn = true;
                    alarmText+= " FR bajo.|";
                }
            }



            ui->label_press->setText("1:" + QString::number(ieRatioRef));
            timeMasterControl = millis();
            valvesExControl(0x00);
            delayMicroseconds(10000);
            valvesMainControl(uint8_t(valvesValueControl));
            inspirationTimeTop = false;
            flowDataReadStatus = true;
            }
        }
        
        //Inspiracion espontanea
        if(ui->radioButton_esp->isChecked()){

            if(inspirationDetected){
                espontaneoPeriod=millis();
                cicleCounter++;
                ui->label_ciclos->setText(QString::number(cicleCounter));
                
                valvesExControl(0x00);
                delayMicroseconds(10000);
                valvesMainControl(uint8_t(valvesValueControl));
                inspirationDetected = false;
                inspirationDetected2 = true;
                flowDataReadStatus = true;
            }           
            if((millis()-espontaneoPeriod) >= uint32_t(inspirationTime) && inspirationDetected2==true){
                flowDataReadStatus = false;
                valvesMainControl(0x00);
                delayMicroseconds(10000);
                valvesExControl(0x01);
                inspirationDetected2 = false;
                //ui->label_press_maxPress->setText(QString::number(readedPress,'f',2)); // f001 Impresion de presion maxima
                ui->label_fr_current->setText("NA");
                ui->label_press->setText("1:" + QString::number(ieRatioRef));
            }
            if((millis()-espontaneoPeriod)>=15000){
                AlarmOut();
                alarmText+="Apnea. |";
                alarmOn = true;
                inspirationDetected=true;
            }
        }

        //QString alarmPressInputState = "Estado: ";
        //qDebug() << "Pin AIR: " << digitalRead(AIR_PRESS_ALARM) << "  Pin O2: " <<  digitalRead(O2_PRESS_ALARM);
        //Alarmas de presión de entrada
        if(ui->checkBox_alarmAirPress->isChecked()){
            if(digitalRead(AIR_PRESS_ALARM)  && (alarmOn == false)){
                alarmOn = true;
                alarmText+= " P.Aire baja.|";
            }
        }
        if(ui->checkBox_alarmO2Press->isChecked()){
            if(digitalRead(O2_PRESS_ALARM)  && (alarmOn == false)){
               alarmOn = true;
               alarmText+= " P.O2 baja.|";
            }
        }

        if(alarmOn && ((millis()-alarmTimeStop)>=5000)){
            digitalWrite(RELE4,HIGH);
            //qDebug() << "ALARMA ACTIVADA!";
            //alarmOn=false;
            ui->label_estado->setStyleSheet("color: rgb(170, 0, 0)");
            ui->label_estado->setText("Estado: " + alarmText);
        }
/*
        if(alarmControl==false && (alarmPressInput==false && alarmPressInputO2==false)){
            digitalWrite(RELE4,LOW);
        }*/
}

/**************************************************************************************************************************************/
//LECTURA DE SENSORES.
/**************************************************************************************************************************************/
/*------------------------------------------------------------------------------------------------------------------------------------*/
/* SENSOR DE PRESIÓN */
double MainWindow::pressureRead(){
    uint16_t bitsA0 = uint16_t(analogRead(AD_BASE));
    //double kPaSensor = (double(bitsA0));
    double kPaSensor = 0.0033*(double(bitsA0))*1.08*0.96*1.07*0.86 - 7.6136+2.6-0.9+2.19;
    kPaSensor = (kPaSensor*1.055);
    kPaSensor = (kPaSensor*0.9483);

    kPaSensor = (kPaSensor*slopePressureAdj) + offsetPressureAdj;
    if(DEBUG_STATUS_HIGH_SPEED){   qDebug() << "Sensor de presión: " << kPaSensor << " mmH2O." ; }
    return kPaSensor;
}

/*------------------------------------------------------------------------------------------------------------------------------------*/
/* SENSOR DE FLUJO */
double MainWindow::flowRead(int I2Cfn){
    int flowRaw = wiringPiI2CReadReg16(I2Cfn,0x00);
    dataH = uint8_t(flowRaw);
    dataL = uint8_t(flowRaw>>8);
    dataFull = uint16_t(dataH<<8)+dataL;
    double flowSensor = (50.0*((double(dataFull)/16384.0)-0.1)/0.8)*16.67;
    flowSensor = flowSensor*(-0.0196647*log(flowSensor)+1.246406);
    flowSensor = (flowSensor*slopeFlowAdj)+offsetPressureAdj;
    //flowSensor = flowSensor*(ui->horizontalSlider_flowSlope->value())/100.0 + (ui->horizontalSlider_flowOffset->value()/20.0);
    //qDebug() << "I2C data: " << (dataFull) << " Flow: " << 50.0*((double(dataFull)/16384.0)-0.1)/0.8;
    //uint16_t bitsD23 = uint16_t(analogRead(AD_BASE+3));
    //double flowSensor = 0.00267694*(double(bitsD23))-3.07741234;
    //if(DEBUG_STATUS_HIGH_SPEED){   qDebug() << "Sensor de flujo: " << flowSensor << " ml/min." ; }
    return flowSensor;
}
/*------------------------------------------------------------------------------------------------------------------------------------*/
/* SENSOR DE VOLUMEN */
double MainWindow::volRead(double flowIn){
    double volTemp2 = (flowIn*(TIMER_SENSOR/1000.0))*1.15;
    volTemp = volTemp+volTemp2;
   // qDebug() << "Volumen: " << volTemp << " ml." << "   VolTemp2 " << volTemp2 << " Flag: " << flowDataReadStatus;
    return volTemp;
}
/*------------------------------------------------------------------------------------------------------------------------------------*/
/* SENSOR DE O2 */  //Max 7500 O2  // Max aire 1500
double MainWindow::o2Read(){
    uint16_t bitsA1 = uint16_t(analogRead(AD_BASE+3));
    double o2Sensor = double(bitsA1)*0.012625418 + 0.309364;
    //qDebug() << "Bits A1: " << bitsA1 << " | Concentración: " << o2Sensor;
    if(DEBUG_STATUS_HIGH_SPEED){   qDebug() << "Saturación de O2: " << o2Sensor << " %." ; }
    return o2Sensor;
}

/**************************************************************************************************************************************/
//REGISTRO DE TIEMPO ACTIVO DEL SISTEMA. Y ALARMA DE INOPERABLE
/**************************************************************************************************************************************/
/*------------------------------------------------------------------------------------------------------------------------------------*/
/* TEMPORIZADOR PARA LLEVAR REGISTRO */
void MainWindow::activeTimerFunction(){
timeActive = timeActive.addSecs(1);
ui->label_activeTime->setText(timeActive.toString("d HH:mm:ss"));
timeActiveToSave++;
if(timeActiveToSave>=10){
    timeActiveToSave=0;
    QFile file("/home/pi/TimeActive.txt");
    if(file.open(QFile::WriteOnly | QFile::Text)){
    }
    QTextStream dataToFile(&file);
    dataToFile << timeActive.toString("d HH:mm:ss");
    file.flush();
    file.close();
    }

if((readedPress <= -5.0) || (readedPress >= 101.0) || (readedFlow <= -5.0) || (readedPress >= 2000.0)){
    ventMainAlarmSystem = true;
    MainAlarmSystemTimer = millis();
        qDebug() << "Alarmá! sistema inoperable advertencia!.";
}

if(((readedPress <= -5.0) || (readedPress >= 101.0) || (readedFlow <= -5.0) || (readedPress >= 2000.0)) && (ventMainAlarmSystem == true) && ((millis() - MainAlarmSystemTimer)>=3000)){
    ventMainAlarmSystem = false;
    on_pushButton_start_clicked();
    QMessageBox::critical(this,"Error!.","Ventilador inoperable!.","Aceptar.");
}




}
/**************************************************************************************************************************************/
//CONTROL DE GRAFICAS DEL SISTEMA y ETAPA DE O2.
/**************************************************************************************************************************************/
/*------------------------------------------------------------------------------------------------------------------------------------*/
/* PLOT FUNCTION
 * Plots data, check the timer starts from this function to see
 * which is the timeout set */
void MainWindow::plotTimerFunction(){

    //qDebug() << "Tiempo plot : " << millis() - tiempoGraficas;
    tiempoGraficas = millis();

    plotData(ui->customPlot, pressAxisGraph, flowAxisGraph, volAxisGraph);
    //ui->customPlot->replot();

    qDebug() << "Tiempo plot preciso: " << millis() - tiempoGraficas;
    tiempoGraficas = millis();

    readedO2 = o2Read();

    O2Prom[0] = O2Prom[1]; O2Prom[1] = O2Prom[2]; O2Prom[2] = O2Prom[3]; O2Prom[3] = O2Prom[4]; O2Prom[4] = O2Prom[5]; O2Prom[5] = readedO2;
    double readedO2Prom = (O2Prom[0] + O2Prom[1] + O2Prom[2] + O2Prom[3] + O2Prom[4] + O2Prom[5])/6.0;
    readedO2 = readedO2Prom;

    ui->label_o2->setText(QString::number(readedO2,'f',1));
    ui->label_currentFio2->setText(QString::number(readedO2,'f',1));


    /*Alarma de FIO2 -------------------*/

    ui->label_fioSetmin->setText(QString::number(fioSetPoint-fioSetPoint*(fio2Error/100.0)));
    ui->label_fioSetmax->setText(QString::number(fioSetPoint+fioSetPoint*(fio2Error/100.0)));


    if(cicleCounter>=2){
    if((readedO2 > fioSetPoint+fioSetPoint*(fio2Error/100.0)) && (alarmOn == false)){
        alarmOn = true;
        alarmText = " FiO2 alto. |";
    }

    if((readedO2 < fioSetPoint-fioSetPoint*(fio2Error/100.0))  && (alarmOn == false)) {
        alarmOn = true;
        alarmText = " FiO2 bajo. |";
    }
    }

}
/*------------------------------------------------------------------------------------------------------------------------------------*/
/* CONFIGURACIÓN DEL PLOT*/
void MainWindow::plotSetup(QCustomPlot *customPlot){

    //ui->customPlot->setOpenGl(true,1);

    x.insert(0,251,0.0);
    for (int i=0; i<251; ++i)
    {
      x.replace(i,i*TIMER_SENSOR);
    }

    dataPress.insert(0,251,QCPGraphData(i, (i*i)/625));
    dataFlow.insert(0,251,QCPGraphData(i, (100-((i*i)/625))*40));
    dataVol.insert(0,251,QCPGraphData(i, 50));

    for (int i=0; i<251; ++i)
    {
      dataPress.replace(i,QCPGraphData(i, (i*i)/625));
      dataFlow.replace(i,QCPGraphData(i, (100-((i*i)/625))*40));
      dataVol.replace(i,QCPGraphData(i, 50));
    }

    customPlot->plotLayout()->clear();
    QCPLayoutGrid *subLayout = new QCPLayoutGrid;
    customPlot->plotLayout()->addElement(0, 0, subLayout);
    QCPAxisRect *flowAxisRect = new QCPAxisRect(customPlot);
    QCPAxisRect *volAxisRect = new QCPAxisRect(customPlot);
    QCPAxisRect *pressAxisRect = new QCPAxisRect(customPlot);

    subLayout->addElement(0, 0, flowAxisRect);
    subLayout->addElement(1, 0, volAxisRect);
    subLayout->addElement(2, 0, pressAxisRect);
    volAxisRect->setMinimumMargins(QMargins(0, 3, 0, 3));
    flowAxisRect->setMinimumMargins(QMargins(0, 3, 0, 3));
    pressAxisRect->setMinimumMargins(QMargins(0, 3, 0, 3));
    int width = 600;
    int height = 120;
    volAxisRect->setMaximumSize(width, height);
    volAxisRect->setMinimumSize(width, height);
    flowAxisRect->setMaximumSize(width, height);
    flowAxisRect->setMinimumSize(width, height);
    pressAxisRect->setMaximumSize(width, height);
    pressAxisRect->setMinimumSize(width, height);

    QList<QCPAxis*> allAxes;
    allAxes << pressAxisRect->axes() << flowAxisRect->axes() << volAxisRect->axes();
    foreach (QCPAxis *axis, allAxes)
    {
      axis->setLayer("axes");
      axis->grid()->setLayer("grid");
    }

    pressAxisRect->axis(QCPAxis::atLeft)->setVisible(true);
    flowAxisRect->axis(QCPAxis::atLeft)->setVisible(true);
    flowAxisRect->axis(QCPAxis::atBottom)->setVisible(false);
    volAxisRect->axis(QCPAxis::atLeft)->setVisible(true);
    volAxisRect->axis(QCPAxis::atBottom)->setVisible(false);

    pressAxisRect->axis(QCPAxis::atBottom)->setVisible(true);
    volAxisRect->axis(QCPAxis::atLeft)->setLabel("V [ml]");
    flowAxisRect->axis(QCPAxis::atLeft)->setLabel("F [ml/s]");
    pressAxisRect->axis(QCPAxis::atLeft)->setLabel("P [cmH20]");
    pressAxisRect->axis(QCPAxis::atBottom)->setLabel("T [s]");

    pressAxisGraph = customPlot->addGraph(pressAxisRect->axis(QCPAxis::atBottom), pressAxisRect->axis(QCPAxis::atLeft));
    flowAxisGraph = customPlot->addGraph(flowAxisRect->axis(QCPAxis::atBottom), flowAxisRect->axis(QCPAxis::atLeft));
    volAxisGraph = customPlot->addGraph(volAxisRect->axis(QCPAxis::atBottom), volAxisRect->axis(QCPAxis::atLeft));

    pressAxisGraph->data()->set(dataPress);
    pressAxisGraph->valueAxis()->setRange(-5, 100);
    pressAxisGraph->keyAxis()->setRange(0, 251);

    flowAxisGraph->data()->set(dataFlow);
    flowAxisGraph->valueAxis()->setRange(-1000, 5000);
    flowAxisGraph->keyAxis()->setRange(0, 251);

    volAxisGraph->data()->set(dataVol);
    volAxisGraph->valueAxis()->setRange(-5, 100);
    volAxisGraph->keyAxis()->setRange(0, 251);

   // customPlot->graph(0)->setBrush(QBrush(QColor(90, 90, 200))); // first graph will be filled with translucent blue
    //customPlot->graph(1)->setBrush(QBrush(QColor(90, 200, 90))); // first graph will be filled with translucent blue
   // customPlot->graph(2)->setBrush(QBrush(QColor(200, 90, 90))); // first graph will be filled with translucent blue

    //
    /*
    pen0.setWidth(2);
    pen1.setWidth(2);
    pen2.setWidth(2);
    */

    pen0.setColor(Qt::darkBlue);
    pen1.setColor(Qt::darkGreen);
    pen2.setColor(Qt::darkRed);

    /*
    pen0.setCosmetic(false);
    pen1.setCosmetic(false);
    pen2.setCosmetic(false);
*/
    customPlot->graph(0)->setPen(pen0);
    customPlot->graph(1)->setPen(pen1);
    customPlot->graph(2)->setPen(pen2);

/*
    customPlot->addGraph();
    customPlot->graph(0)->setPen(QPen(Qt::blue)); // line color blue for first graph
    customPlot->addGraph();
    customPlot->graph(1)->setPen(QPen(Qt::darkGreen)); // line color red for second graph
    customPlot->addGraph();
    customPlot->graph(2)->setPen(QPen(Qt::red)); // line color red for second graph

    customPlot->xAxis2->setVisible(true);
    customPlot->xAxis2->setTickLabels(false);
    customPlot->yAxis2->setVisible(true);
    customPlot->yAxis2->setTickLabels(false);
    customPlot->xAxis->setRange(0,251);
    customPlot->yAxis->setRange(-3,40);
    customPlot->yAxis2->setRange(-3,40);
    //customPlot->xAxis->setLabel("Tiempo [s]");
    //customPlot->yAxis->setLabel("Pr [cmH2O] / Fl [l/min] / Vol [ml]");

    QSharedPointer<QCPAxisTickerTime> timeTicker(new QCPAxisTickerTime);
    customPlot->xAxis->setTicker(timeTicker);

    customPlot->xAxis->setRange(0,251*TIMER_SENSOR);
    timeTicker->setTimeFormat("%z");

    QSharedPointer<QCPAxisTickerFixed> fixedTicker(new QCPAxisTickerFixed);
    customPlot->xAxis->setTicker(fixedTicker);
    fixedTicker->setTickStep(1000.0);
    fixedTicker->setScaleStrategy(QCPAxisTickerFixed::ssNone);

    QSharedPointer<QCPAxisTickerFixed> fixedTickerY(new QCPAxisTickerFixed);
    customPlot->yAxis->setTicker(fixedTickerY);
    fixedTickerY->setTickStep(3);
    fixedTickerY->setScaleStrategy(QCPAxisTickerFixed::ssNone);*/

/*

    QLinearGradient plotGradient;
    plotGradient.setStart(0, 0);
    plotGradient.setFinalStop(0, 350);
    plotGradient.setColorAt(0, QColor(255, 255, 255));
    plotGradient.setColorAt(1, QColor(230, 230, 230));
    customPlot->setBackground(plotGradient);
    QLinearGradient axisRectGradient;
    axisRectGradient.setStart(0, 0);
    axisRectGradient.setFinalStop(0, 350);
    axisRectGradient.setColorAt(0, QColor(255, 255, 255));
    axisRectGradient.setColorAt(1, QColor(230, 230, 230));
    customPlot->axisRect()->setBackground(axisRectGradient);

    customPlot->rescaleAxes();
*/


    //customPlot->xAxis->setTickLabelRotation(45);
/*
    customPlot->graph(0)->data()->clear();
    customPlot->graph(1)->data()->clear();
    customPlot->graph(2)->data()->clear();*/
}
/*------------------------------------------------------------------------------------------------------------------------------------*/
/* GRAFICAR DATOS */
void MainWindow::plotData(QCustomPlot *customPlot, QCPGraph *pressAxisGraph, QCPGraph *flowAxisGraph, QCPGraph *volAxisGraph)
{/*
  customPlot->graph(0)->setData(x, pressData);
  customPlot->graph(1)->setData(x, flowData);
  customPlot->graph(2)->setData(x, volData);
*/

  pressAxisGraph->data()->set(dataPress);
  flowAxisGraph->data()->set(dataFlow);
  volAxisGraph->data()->set(dataVol);
  ui->customPlot->replot();



  minPEEP = *std::min_element(pressData.begin(), pressData.end())+0.2;
  if(minPEEP <= 1){
      minPEEP = 5.2;
  }

  if(ui->tabWidget_sel->currentIndex()==0 ){

        //customPlot->graph(0)->setBrush(QBrush(QColor(0, 0, 255, 20))); // first graph will be filled with translucent blue
        //customPlot->graph(2)->setBrush(QBrush(QColor(0, 0, 0, 0))); // first graph will be filled with translucent blue
        //customPlot->yAxis->setRange(-1,setPIP+6);
        //customPlot->yAxis2->setRange(-1,setPIP+2);
        //QSharedPointer<QCPAxisTickerFixed> fixedTickerY(new QCPAxisTickerFixed);
        //customPlot->yAxis->setTicker(fixedTickerY);
        //fixedTickerY->setTickStep(2);
        //fixedTickerY->setScaleStrategy(QCPAxisTickerFixed::ssNone);
        //customPlot->yAxis->setLabel("Presion [cmH2O]");
  }

  if(ui->tabWidget_sel->currentIndex()==1 ){
      //customPlot->graph(2)->setBrush(QBrush(QColor(255, 0, 0, 20))); // first graph will be filled with translucent blue
      //customPlot->graph(0)->setBrush(QBrush(QColor(0, 0, 0, 0))); // first graph will be filled with translucent blue
      //customPlot->yAxis->setRange(-3,setVOL+100);
      //customPlot->yAxis2->setRange(-3,setVOL+100);
      //QSharedPointer<QCPAxisTickerFixed> fixedTickerY(new QCPAxisTickerFixed);
      //customPlot->yAxis->setTicker(fixedTickerY);
      //fixedTickerY->setTickStep(100);
      //fixedTickerY->setScaleStrategy(QCPAxisTickerFixed::ssNone);
      //customPlot->yAxis->setLabel("Flujo [ml/s] / Volumen [ml]");
  }
  //         ui->label_PEEP->setText(QString::number(minPEEP,'g',3));
}

/**************************************************************************************************************************************/
//ACTIVACIÓN DE ALARMAS.
/**************************************************************************************************************************************/
/*------------------------------------------------------------------------------------------------------------------------------------*/
/* ALARMAS */
void MainWindow::activateAlarm(uint16_t number){
    if(DEBUG_STATUS){   qDebug() << "Alarma activada!: " << number; }
    errorFrCounter=0;
    if(DEBUG_STATUS){   qDebug() << "Alarma activada!: " << errorFrCounter; }
    //on_pushButton_start_clicked();
    //evalVel();
    AlarmOut();
    digitalWrite(RELE4,HIGH);
    switch (number) {
    case 1: {
       // QMessageBox::critical(this,"Error!.","Presión alta!.","Aceptar.");
        break;
    }
    case 2: {
       // QMessageBox::critical(this,"Error!.","Presión baja!.","Aceptar.");
        break;
    }
    case 3: {
      //  QMessageBox::critical(this,"Error!.","Frecuencia respiratoria alta!.","Aceptar.");
        break;
    }
    case 4: {
       // QMessageBox::critical(this,"Error!.","Frecuencia respiratoria baja!.","Aceptar.");
        break;
    }
    case 5: {
       // QMessageBox::critical(this,"Error!.","O2 alto!.","Aceptar.");
        break;
    }
    case 6: {
       // QMessageBox::critical(this,"Error!.","O2 bajo!.","Aceptar.");
        break;
    }
    case 7: {
       // QMessageBox::critical(this,"Error!.","Volumen bajo!.","Aceptar.");
        break;
    }
    case 8: {
       // QMessageBox::critical(this,"Error!.","Volumen alto!.","Aceptar.");
        break;
    }
    case 9: {
        ui->label_estado->setText("Estado: Error! fuga o desconexion!");
        ui->label_estado->setStyleSheet("color: rgb(170, 0, 0)");
       // QMessageBox::critical(this,"Error!.","Fuga o desconexión!.","Aceptar.");
        break;
    }
    }
   /* if(((alarmPressInputO2 == false) && (alarmPressInput == false))){
    digitalWrite(RELE4,LOW);
    }*/
}

/*------------------------------------------------------------------------------------------------------------------------------------*/
/* ALARMA SONIDO */
void MainWindow::AlarmOut(){
    digitalWrite(RELE4,HIGH);
    delayMicroseconds(200000);
    digitalWrite(RELE4,LOW);
    delayMicroseconds(100000);

    digitalWrite(RELE4,HIGH);
    delayMicroseconds(100000);
    digitalWrite(RELE4,LOW);
    delayMicroseconds(100000);

    digitalWrite(RELE4,HIGH);
    delayMicroseconds(200000);
    digitalWrite(RELE4,LOW);
    delayMicroseconds(100000);
}

/**************************************************************************************************************************************/
//CONTROL DE VALVULAS
/**************************************************************************************************************************************/
/*------------------------------------------------------------------------------------------------------------------------------------*/
/* INALACIÓN */
void MainWindow::valvesMainControl(uint8_t inspirationValves ){
delayMicroseconds(100);
digitalWrite(REL1,(inspirationValves &  0x01));
digitalWrite(REL2,(inspirationValves &  0x02));
digitalWrite(REL4,(inspirationValves &  0x04));
digitalWrite(REL3,(inspirationValves &  0x08));
digitalWrite(REL5,(inspirationValves &  0x10));
digitalWrite(REL6,(inspirationValves &  0x20));
digitalWrite(REL7,(inspirationValves &  0x40));
digitalWrite(REL8,(inspirationValves &  0x80));
delayMicroseconds(5000);
}

/*------------------------------------------------------------------------------------------------------------------------------------*/
/* EXALACIÓN */
void MainWindow::valvesExControl(uint8_t exalationValves ){
delayMicroseconds(100);
digitalWrite(RELE1,(exalationValves &  0x01));
digitalWrite(RELE2,(exalationValves &  0x02));
digitalWrite(RELE3,(exalationValves &  0x04));
//digitalWrite(RELE4,(exalationValves &  0x08)); //456789
delayMicroseconds(5000);
}


/**************************************************************************************************************************************/
//RANGOS MAXIMOS PARA EL SISTEMA.
/**************************************************************************************************************************************/
/*------------------------------------------------------------------------------------------------------------------------------------*/
/* SUMA DE MAXIMO LIMITE DE VOLUMEN */
/*void MainWindow::on_pushButton_mor_maxPress_2_clicked()
{
    maxVolLimit+=50;
}*/
/*------------------------------------------------------------------------------------------------------------------------------------*/
/* RESTA DE MAXIMO LIMITE DE VOLUMEN */
/*void MainWindow::on_pushButton_min_maxPress_2_clicked()
{
    maxVolLimit-=50;
}*/
/*------------------------------------------------------------------------------------------------------------------------------------*/
/* RESTA DE MAXIMO LIMITE DE PRESION */
void MainWindow::on_pushButton_min_maxPress_clicked()
{
    maxPressLimit -= 1;
    ui->label_maxPressLimit->setText(QString::number(maxPressLimit));
}
/*------------------------------------------------------------------------------------------------------------------------------------*/
/* RESTA DE MAXIMO LIMITE DE PRESION */
void MainWindow::on_pushButton_mor_maxPress_clicked()
{
    maxPressLimit += 1;
    ui->label_maxPressLimit->setText(QString::number(maxPressLimit));
}







/**************************************************************************************************************************************/
//RANGOS DE CONTROL PARA EL SISTEMA.
/**************************************************************************************************************************************/
/*------------------------------------------------------------------------------------------------------------------------------------*/
/* DECREMENTO DE SETPOINT DE PEEP */
void MainWindow::on_pushButton_minPEEP_clicked()
{
    setPEEP-=0.5;
    if(setPEEP<=2) {setPEEP = 2;}
    ui->label_setPEEP->setText(QString::number(setPEEP,'f',1));
    evalVel();

}
/*------------------------------------------------------------------------------------------------------------------------------------*/
/* INCREMENTO DE SETPOINT DE PEEP */void MainWindow::on_pushButton_morePEEP_clicked()
{
    setPEEP+=0.5;
    if(pressControlActive){
    if(setPEEP>=(setPIP-3)) {setPEEP = setPIP-3;}
    }
    if(setPEEP >= 20){setPEEP = 20;}
    ui->label_setPEEP->setText(QString::number(setPEEP,'f',1));
    evalVel();

}
/*------------------------------------------------------------------------------------------------------------------------------------*/
/* INCREMENTO DE SETPOINT DE PIP */
void MainWindow::on_pushButton_morePIP_clicked()
{
    setPIP+=0.5;
    if(setPIP>=40) {setPIP  = 40;}
    ui->label_press_pip->setNum(setPIP);
    evalVel();
}
/*------------------------------------------------------------------------------------------------------------------------------------*/
/* DECREMENTO DE SETPOINT DE PIP */
void MainWindow::on_pushButton_minPIP_clicked()
{
    setPIP-=0.5;
    if(setPIP<=6) {setPIP  = 6;}
    if((setPIP-3)<=setPEEP){
        setPEEP=setPIP-3;
        ui->label_setPEEP->setText(QString::number(setPEEP,'f',1));
    }
    ui->label_press_pip->setNum(setPIP);
    evalVel();
}
/*------------------------------------------------------------------------------------------------------------------------------------*/
/* DECREMENTO DE SETPOINT DE VOLUMEN */
void MainWindow::on_pushButton_minVol_clicked()
{
    setVOL-=50;
    if(setVOL<=50) {setVOL = 50;}
    ui->label_press_volsetpoint->setNum(setVOL);
    alarmCurrenCiclosVol = cicleCounter;
    evalVel();
}
/*------------------------------------------------------------------------------------------------------------------------------------*/
/* INCREMENTO DE SETPOINT DE VOLUMEN */
void MainWindow::on_pushButton_moreVol_clicked()
{
    setVOL+=50;
    inspirationTimeForVol = ((double(60.0/double(FRv))*1000.0)/(ieRatioRef+1.0))/1000.0;
    //if(setVOL>=2050) {setVOL = 2050;}
    if(setVOL >= (1200.0*inspirationTimeForVol)) {
        setVOL = 1200.0*inspirationTimeForVol;
        setVOL = int((int(setVOL)-25)/50 * 50);
    }
    ui->label_press_volsetpoint->setNum(setVOL);
    alarmCurrenCiclosVol = cicleCounter;
    evalVel();
}
/*------------------------------------------------------------------------------------------------------------------------------------*/
/* INCREMENTO DE SETPOINT DE FR */
void MainWindow::on_pushButton_moreFR_clicked()
{
    FRv=FRv+1;

    if(ui->tabWidget_sel->currentIndex()==1){
        FRvMax=int(60.0/((setVOL/1200.0)*(ieRatioRef+1.0)));
        if(FRv>=FRvMax) {
            FRv = int(FRvMax);
        }
    }
    if(ui->tabWidget_sel->currentIndex()==0){
        if(FRv>=60) {
            FRv = 60;
        }
    }

    timeFRv = uint32_t(double(60.0/double(FRv))*1000.0);
    ui->label_fr->setNum(int(FRv));
    alarmCurrenCiclosFR = cicleCounter;
    evalVel();
}
/*------------------------------------------------------------------------------------------------------------------------------------*/
/* DECREMENTO DE SETPOINT DE FR */
void MainWindow::on_pushButton_minFR_clicked()
{
    FRv=FRv-1;
    if(FRv<=4) {FRv = 4;}
    timeFRv = uint32_t(double(60.0/double(FRv))*1000.0);
    ui->label_fr->setNum(int(FRv));
    alarmCurrenCiclosFR = cicleCounter;
    evalVel();
}
/*------------------------------------------------------------------------------------------------------------------------------------*/
/* INCREMENTO DE SETPOINT DE IE */
void MainWindow::on_pushButton_mor_ie_clicked()
{
    ieRatioRef = ieRatioRef+1;
    if(ieRatioRef>=5) {ieRatioRef = 5;}
    ui->label_ie_ratio->setText(QString::number(ieRatioRef,'f',0));
    on_pushButton_moreFR_clicked();
    evalVel();
}
/*------------------------------------------------------------------------------------------------------------------------------------*/
/* DECREMENTO DE SETPOINT DE IE */
void MainWindow::on_pushButton_min_ie_clicked()
{
    ieRatioRef = ieRatioRef - 1;
    if(ieRatioRef<=1) {ieRatioRef = 1;}
    ui->label_ie_ratio->setText(QString::number(ieRatioRef,'f',0));
    evalVel();
}



/**************************************************************************************************************************************/
//MISC.
/**************************************************************************************************************************************/
/*------------------------------------------------------------------------------------------------------------------------------------*/
/* FUNCION DUMMY DE VALIDACIÓN POR TIEMPOS QUITAR EL WHILE PARA PROBAR*/
void MainWindow::validacion(){
    uint8_t valTest = 50;
    while(true){

        valvesMainControl(valTest); //Inspiracion
        delayMicroseconds(1000000);
        valvesMainControl(0x00); //Inspiracion

        qDebug()<< "Presión maxima:" << pressureRead();
        valvesExControl(0x01); //Exalación
        delayMicroseconds(6000000);
        valvesExControl(0x00); //Exalación

        valTest = valTest+20;
        qDebug() << valTest;

        if(valTest>=190){
            valTest=30;
        }
    }
}
/*------------------------------------------------------------------------------------------------------------------------------------*/
/* DETECCIÓN DE FUGAS */
void MainWindow::on_pushButton_pressLeaks_clicked()
{
    valvesExControl(0);
    valvesMainControl(0x10);
    for(int i=0;i<=100;i++){
        pressLeaksData = pressureRead();
        ui->label_pressLeaks->setText(QString::number(pressLeaksData,'f',1));
        ui->progressBar_pressLeak->setValue(i);
        delayMicroseconds(1000);
    }
    ui->label_pressLeaksMax->setText(QString::number(pressLeaksData,'f',1));
    valvesMainControl(0);
    if(pressLeaksData<=5){
        QMessageBox::critical(this,"Error!.","No se detecto presión!.","Aceptar.");
    }
    else{

        for(int i=0;i<=100;i++){
            ui->label_pressLeaks->setText(QString::number(pressureRead(),'f',1));
            ui->progressBar_pressLeak->setValue(100-i);
            delayMicroseconds(1000);
        }
        ui->label_pressLeaksFin->setText(QString::number(pressureRead(),'f',1));
        ui->label_pressLeaksDiff->setText(QString::number(pressLeaksData-pressureRead(),'f',1));


        if(pressLeaksData-pressureRead()>=15){
            QMessageBox::critical(this,"Error!.","Fugas detectadas!.","Aceptar.");
        }
        else {
            QMessageBox::critical(this,"Ok!.","No se detectaron fugas!.","Aceptar.");
        }
    }
}
/*------------------------------------------------------------------------------------------------------------------------------------*/
/* AJUSTE DE CONFIGURACIÓN */
void MainWindow::on_pushButton_3_clicked()
{
    ui->tabWidget->setCurrentIndex(0);
}
/*------------------------------------------------------------------------------------------------------------------------------------*/
/* REINICIO DE CONTADOR */
void MainWindow::on_pushButton_resetC_clicked()
{
    cicleCounter = 0;
}
/*------------------------------------------------------------------------------------------------------------------------------------*/
/* IMPRESIÓN DE TIEMPO EN EL DEPURADOR */
void MainWindow::printTimer(QString info){
    timerMillis=millis();
    if(DEBUG_STATUS){   qDebug() << info << timerMillis;}
}
/*------------------------------------------------------------------------------------------------------------------------------------*/
/* TEST */
/*
void MainWindow::on_pushButton_4_clicked()
{
    qDebug() << "Presion: " << pressureRead();
    qDebug() << "O2: " << o2Read();
    qDebug() << "Flujo: " << flowRead(initI2C);
}*/
/*------------------------------------------------------------------------------------------------------------------------------------*/
/* CAMBIO DE TAB */
void MainWindow::on_pushButton_Conf_clicked()
{
    ui->tabWidget->setCurrentIndex(1);
}


void MainWindow::on_radioButton_name_clicked()
{
    if(ui->radioButton_name->isChecked()){
        ui->lineEdit_textName->setEnabled(true);
    }
    else{
        ui->lineEdit_textName->setEnabled(false);
    }
}

void MainWindow::on_radioButton_date_clicked()
{
    if(ui->radioButton_date->isChecked()){
        ui->lineEdit_textName->setEnabled(false);
    }
    else{
        ui->lineEdit_textName->setEnabled(true);
    }
}


void MainWindow::on_pushButton_10_clicked()
{
    testTimer->start(200);
}

void MainWindow::testTimerFunction(){

    qDebug() << "Presion: " << pressureRead() << "      Flujo: " << flowRead(initI2C) << "  O2: " << o2Read()
    << "P AIR: " << digitalRead(AIR_PRESS_ALARM) << "  P O2: " <<  digitalRead(O2_PRESS_ALARM);

}


void MainWindow::on_pushButton_stop_clicked()
{
    testTimer->stop();
}



void MainWindow::on_pushButton_2_clicked()
{
    uint8_t inalacion=0;
    uint8_t exalacion=0;
    if(ui->checkBox->isChecked()){inalacion+=0x01;}
    if(ui->checkBox_2->isChecked()){inalacion+=0x02;}
    if(ui->checkBox_3->isChecked()){inalacion+=0x04;}
    if(ui->checkBox_4->isChecked()){inalacion+=0x08;}
    if(ui->checkBox_5->isChecked()){inalacion+=0x10;}
    if(ui->checkBox_6->isChecked()){inalacion+=0x20;}
    if(ui->checkBox_7->isChecked()){inalacion+=0x40;}
    if(ui->checkBox_8->isChecked()){inalacion+=0x80;}

    if(ui->checkBox_E1->isChecked()){exalacion+=0x01;}
    if(ui->checkBox_E2->isChecked()){exalacion+=0x02;}
    if(ui->checkBox_E3->isChecked()){exalacion+=0x04;}
    if(ui->checkBox_E4->isChecked()){exalacion+=0x08;}


    qDebug() << "Inalacion: " << inalacion << " || Exalación: " << exalacion;

    valvesMainControl(inalacion);
    valvesExControl(exalacion);
}


void MainWindow::on_horizontalSlider_sensibilidad_sliderMoved(int position)
{
    ui->label_sensibilidad->setText(QString::number(position));
}

void MainWindow::evalVel(){
        if(ui->tabWidget_sel->currentIndex()==1){
            inspirationTime = (double(60.0/double(FRv))*1000.0)/(ieRatioRef+1.0);
            valvesValueControl=uint32_t(setVOL/((inspirationTime/1000.0)*5.0));
        }
        if(ui->tabWidget_sel->currentIndex()==0 || ((ui->tabWidget_sel->currentIndex()==2) && (pressControlActive == true))){

            if(ui->tabWidget_sel->currentIndex()==0){
                if(int(ieRatioRef) >= 3){
                    if(FRv >=9 && FRv < 12){
                        if(setPIP <8)                   {valvesValueControl = 8; }
                        if(setPIP>=8  && setPIP <11)    {valvesValueControl = 15; }
                        if(setPIP>=11 && setPIP <14)    {valvesValueControl = 23; }
                        if(setPIP>=14 && setPIP <17)    {valvesValueControl = 32; }
                        if(setPIP>=17 && setPIP <20)    {valvesValueControl = 45; }
                        if(setPIP>=20)                  {valvesValueControl = 53; }
                    }
                    if(FRv >=12 && FRv < 15){
                        if(setPIP <8)                   {valvesValueControl = 10; }
                        if(setPIP>=8  && setPIP <11)    {valvesValueControl = 16; }
                        if(setPIP>=11 && setPIP <14)    {valvesValueControl = 28; }
                        if(setPIP>=14 && setPIP <17)    {valvesValueControl = 44; }
                        if(setPIP>=17 && setPIP <20)    {valvesValueControl = 55; }
                        if(setPIP>=20)                  {valvesValueControl = 62; }
                    }
                    if(FRv >=15 && FRv < 18){
                        if(setPIP <8)                   {valvesValueControl = 15; }
                        if(setPIP>=8  && setPIP <11)    {valvesValueControl = 23; }
                        if(setPIP>=11 && setPIP <14)    {valvesValueControl = 36; }
                        if(setPIP>=14 && setPIP <17)    {valvesValueControl = 53; }
                        if(setPIP>=17 && setPIP <20)    {valvesValueControl = 63; }
                        if(setPIP>=20)                  {valvesValueControl = 75; }
                    }
                    if(FRv >=18 && FRv < 21){
                        if(setPIP <8)                   {valvesValueControl = 15; }
                        if(setPIP>=8  && setPIP <11)    {valvesValueControl = 29; }
                        if(setPIP>=11 && setPIP <14)    {valvesValueControl = 39; }
                        if(setPIP>=14 && setPIP <17)    {valvesValueControl = 59; }
                        if(setPIP>=17 && setPIP <20)    {valvesValueControl = 76; }
                        if(setPIP>=20)                  {valvesValueControl = 86; }
                    }
                }
                if(int(ieRatioRef) == 2){
                    if(FRv >=9 && FRv < 12){
                        if(setPIP <8)                   {valvesValueControl = 10; }
                        if(setPIP>=8  && setPIP <11)    {valvesValueControl = 12; }
                        if(setPIP>=11 && setPIP <14)    {valvesValueControl = 18; }
                        if(setPIP>=14 && setPIP <17)    {valvesValueControl = 24; }
                        if(setPIP>=17 && setPIP <20)    {valvesValueControl = 31; }
                        if(setPIP>=20)                  {valvesValueControl = 38; }
                    }
                        if(FRv >=12 && FRv < 15){
                        if(setPIP <8)                   {valvesValueControl = 11; }
                        if(setPIP>=8  && setPIP <11)    {valvesValueControl = 15; }
                        if(setPIP>=11 && setPIP <14)    {valvesValueControl = 28; }
                        if(setPIP>=14 && setPIP <17)    {valvesValueControl = 44; }
                        if(setPIP>=17 && setPIP <20)    {valvesValueControl = 53; }
                        if(setPIP>=20)                  {valvesValueControl = 62; }
                    }
                    if(FRv >=15 && FRv < 18){
                        if(setPIP <8)                   {valvesValueControl = 12; }
                        if(setPIP>=8  && setPIP <11)    {valvesValueControl = 22; }
                        if(setPIP>=11 && setPIP <14)    {valvesValueControl = 31; }
                        if(setPIP>=14 && setPIP <17)    {valvesValueControl = 47; }
                        if(setPIP>=17 && setPIP <20)    {valvesValueControl = 63; }
                        if(setPIP>=20)                  {valvesValueControl = 71; }
                    }
                    if(FRv >=18 && FRv < 21){
                        if(setPIP <8)                   {valvesValueControl = 14; }
                        if(setPIP>=8  && setPIP <11)    {valvesValueControl = 23; }
                        if(setPIP>=11 && setPIP <14)    {valvesValueControl = 38; }
                        if(setPIP>=14 && setPIP <17)    {valvesValueControl = 54; }
                        if(setPIP>=17 && setPIP <20)    {valvesValueControl = 70; }
                        if(setPIP>=20)                  {valvesValueControl = 78; }
                    }

                }
                if(setPIP>=20){
                    valvesValueControl+=(setPIP-20)*2;
                }
               //valvesValueControl = valvesValueControl*0.9;
            }

        if((setPEEP>=5) && (pressControlActive == true)){
             valvesValueControl-=(setPEEP-5.0)*3.0;
             qDebug() << "Nuevo valor de PEEP COMP: " << (setPEEP-5.0)*3.0;
        }
        }

        qDebug() << "Nuevo valor de velocidad: " << valvesValueControl;
}

void MainWindow::on_pushButton_11_clicked()
{
    inspirationDetected = true;
}

void MainWindow::on_pushButton_alarmTest_clicked()
{
    activateAlarm(1);
}


QString MainWindow::initSystem(){
    bool testInitSystem = true;
    QString initSystemString = "Analisis de sistema:\n";
    delayMicroseconds(200000);


    SFM3019 SFM3019sen;

    int SFM3019id = SFM3019sen.initSensorI2C();


    //(void)SFM3019sen.checkId(SFM3019id);



    SFM3019sen.startContMeas(SFM3019id,GAS1_AIR);
    int *SFMDataPointer;

    for(int i = 0; i<=10;i++){
    SFMDataPointer = SFM3019sen.readFlowTempSt(SFM3019id,CHECKSUM_ON);
    qDebug() <<  "Flow: " << *(SFMDataPointer+0)/1000.0 <<  " Temp: " << *(SFMDataPointer+1)/1000.0;
    delayMicroseconds(100000);
    }

    SFM3019sen.scaleFactorAndOffset(SFM3019id);
    for(int i = 0; i<=2;i++){
    SFMDataPointer = SFM3019sen.readScaleOffset(SFM3019id);
    qDebug() <<  "SF: " << *(SFMDataPointer+0) <<  " Of: " << *(SFMDataPointer+1) << " Un: " << *(SFMDataPointer+2);
    delayMicroseconds(100000);
    }

    SFM3019sen.startContMeas(SFM3019id,1);
    for(int i = 0; i<=10;i++){
    SFMDataPointer = SFM3019sen.readFlowTempSt(SFM3019id,CHECKSUM_ON);
    qDebug() <<  "Flow: " << *(SFMDataPointer+0)/1000.0 <<  " Temp: " << *(SFMDataPointer+1)/1000.0;
    delayMicroseconds(100000);
    }

    //(void)SFM3019sen.checkId(SFM3019id);

    /*
    qDebug() <<QProcess::execute("sudo i2cset -y 1 0x2e 0x36 0x61");
    initI2CSFM301 = wiringPiI2CSetup(0x2e);    delayMicroseconds(200000);

    uint8_t StartContAir[2];
    StartContAir[0]=0x36;
    StartContAir[1]=0x08;
    uint8_t ReadScaleOffset[2];
    ReadScaleOffset[0]=0x36;
    ReadScaleOffset[1]=0x61;
    uint8_t bufR[9];


    write(initI2CSFM301,ReadScaleOffset,2);

    read(initI2CSFM301,bufR,9);
    int16_t scaleFactor = (bufR[0]<<8)+bufR[1];
    int16_t offsetFlow = (bufR[3]<<8)+bufR[4];
    int16_t flowUnit = (bufR[6]<<8)+bufR[7];
    qDebug() << "SF: "<< scaleFactor << " OF: " << offsetFlow << " FU: " << flowUnit;

    uint8_t data[2];
    data[0]=bufR[0];
    data[1]=bufR[1];

    uint8_t byteCtr;
    uint8_t calc_crc = 0xFF;
    for (uint8_t byteCtr = 0; byteCtr < 2; ++byteCtr){
     calc_crc ^= (data[byteCtr]);
     for ( i = 8; i > 0; --i){
     if (calc_crc & 0x80) { calc_crc = (calc_crc << 1) ^ 0x131;}
     else { calc_crc = (calc_crc << 1); }
     }
    }
    qDebug() << "CRC Read: " << bufR[2] << "CRC Calc: " << calc_crc;


    qDebug() <<QProcess::execute("sudo i2cset -y 1 0x2e 0x36 0x08");

    write(initI2CSFM301,StartContAir,2);


    for(int rt=0;rt<=20;rt++){
    uint8_t data1[9];

    delayMicroseconds(20000);
    read(initI2CSFM301,bufR,9);

    int16_t flowData = (bufR[0]<<8)+bufR[1];
    int16_t tempData = (bufR[3]<<8)+bufR[4];
    int16_t statusData = (bufR[6]<<8)+bufR[7];
    //qDebug() << "Flow: "<< flowData << " Temp: " << tempData << " Status: " << statusData;

    double flowMeas = (double(flowData)+24576.0)/170.0;
    double tempMeas = (tempData-0)/200;
    qDebug() << "Flow real: " << flowMeas << " Temp real: " << tempMeas;

    delayMicroseconds(250000);
    }
*/




    initI2C = wiringPiI2CSetup(0x49); // Inicializar el sensor de flujo Honeywell
    qDebug() << "I2C init: " << initI2C;
    delayMicroseconds(100000);

    wiringPiI2CWrite(initI2C,0x01);
    delayMicroseconds(100000);
    int serial1 = wiringPiI2CReadReg16(initI2C,0x00);
    int serial2 = wiringPiI2CReadReg16(initI2C,0x00);

    qDebug() << "I2C serial number : " << serial1 << " " << serial2; //Verificando serial del sensor.
    wiringPiI2CWrite(initI2C,0x03);
    delayMicroseconds(100000);
    serial1 = wiringPiI2CReadReg16(initI2C,0x00);
    dataH = uint8_t(serial1);
    dataL = uint8_t(serial1>>8);
    dataFull = uint16_t(dataH<<8)+dataL;

    if(dataFull==52389){
        qDebug() << "I2C CheckSum del sensor: " << dataFull;
        testInitSystem = true;
        initSystemString+="I2C Flujo: OK!\n";
    }
    else{
        qDebug() << "Error no  se detecta sensor de flujo!";
        testInitSystem = false;
        initSystemString+="I2C Flujo: NOK!\n";
    }

    delayMicroseconds(100000);

    //----------------------------------------------------------------

    if(wiringPiSetupGpio()){
        qDebug() << "Error en perifericos.";
        testInitSystem = false;
        initSystemString+="Perifericos: NOK!\n";
    }
    else{
        qDebug() << "Inicializando perifericos.";
        testInitSystem = true;
        initSystemString+="Perifericos: OK!\n";
    }

    if(initFile()){
        qDebug() << "Documento inicializado.";
        initSystemString+="Memoria: OK!\n";
        testInitSystem = true;
    }
    else{
        qDebug() << "Error en memoria.";
        initSystemString+="Memoria: NOK!\n";
        testInitSystem = false;
    }

    pinMode(REL1,OUTPUT);
    pinMode(REL2,OUTPUT);
    pinMode(REL3,OUTPUT);
    pinMode(REL4,OUTPUT);
    pinMode(REL5,OUTPUT);
    pinMode(REL6,OUTPUT);
    pinMode(REL7,OUTPUT);
    pinMode(REL8,OUTPUT);

    pinMode(RELE1,OUTPUT);
    pinMode(RELE2,OUTPUT);
    pinMode(RELE3,OUTPUT);
    pinMode(RELE4,OUTPUT);

    //pullUpDnControl(AIR_PRESS_ALARM,PUD_UP);
    //pullUpDnControl(O2_PRESS_ALARM,PUD_UP);
    pinMode(AIR_PRESS_ALARM,INPUT);
    pinMode(O2_PRESS_ALARM,INPUT);
    //pullUpDnControl(AIR_PRESS_ALARM,PUD_UP);
    //pullUpDnControl(O2_PRESS_ALARM,PUD_UP);

    pinMode(ValveExp,OUTPUT);


    // Inicializando I2C / ADC 1115
    if(ads1115Setup(AD_BASE,0x48) < 0){
        qDebug() << "Error en el I2C con el ADC";
        testInitSystem = false;
        initSystemString+="I2C ADC: NOK!\n";
    }
    else{
        qDebug() << "ADC Inicializado.";
        testInitSystem = true;
        initSystemString+="I2C ADC: OK!\n";
    }

    digitalWrite(AD_BASE,2); // COnfigurando lla referencia de 2V.

    timerStatusFlag=false;

    sensorTimer->setTimerType(Qt::PreciseTimer);
    plotTimer->setTimerType(Qt::PreciseTimer);
    controlTimer->setTimerType(Qt::PreciseTimer);

    QObject::connect(sensorTimer, &QTimer::timeout, this,QOverload<>::of(&MainWindow::sensorTimerFunction));
    QObject::connect(plotTimer, &QTimer::timeout, this,QOverload<>::of(&MainWindow::plotTimerFunction));
    QObject::connect(controlTimer, &QTimer::timeout, this,QOverload<>::of(&MainWindow::controlTimerFunction));
    QObject::connect(testTimer, &QTimer::timeout, this,QOverload<>::of(&MainWindow::testTimerFunction));
    QObject::connect(activeTimer, &QTimer::timeout, this,QOverload<>::of(&MainWindow::activeTimerFunction));

    activeTimer->start(1000);

    pressData.insert(0,251,0.0);
    volData.insert(0,251,0.0);
    flowData.insert(0,251,0.0);
    PEEPData.insert(0,251,0.0);


    increaseVolTemp=0;
    volTemp=0;

    ui->label_press_pip->setNum(setPIP);
    ui->label_setPEEP->setNum(int(setPEEP));
    ui->label_fr->setNum(int(FRv));

    pressurePIP=false;
    pressure0=false;
    pressureMAX = false;

    ieRatioRef = 2;

    ui->label_ie_ratio->setText(QString::number(ieRatioRef,'f',0));
    ui->label_maxPressLimit->setText(QString::number(maxPressLimit));
    ui->label_errorFlow->setText(QString::number(flowError));
    ui->label_frError->setText(QString::number(frError));
    ui->label_o2setpoint_error->setText(QString::number(fio2Error));
    ui->label_o2setpoint->setText(QString::number(fioSetPoint ));


    getDateText();
    plotSetup(ui->customPlot);

    inspirationDetected2 = true;

    for(int i=0;i<=20;i++){
        readedO2 += o2Read();
    }

    readedO2 = readedO2/20;
    fioSetPoint = readedO2;
    ui->label_o2setpoint->setText(QString::number(readedO2,'g',2));
    ui->label_o2->setText(QString::number(readedO2,'g',2));
    ui->tabWidget->setCurrentIndex(0);
    ui->tabWidget_sel->setCurrentIndex(0);

    QFile file("/home/pi/TimeActive.txt");
    if(file.open(QFile::ReadOnly | QFile::Text)){
    }

    QTextStream in(&file);
    QString timeActiveRead = in.readAll();
    file.flush();
    file.close();

    timeActive = QDateTime::fromString(timeActiveRead,"d HH:mm:ss");
    ui->label_activeTime->setText(timeActive.toString("d HH:mm:ss"));
    valvesMainControl(0);
    valvesExControl(0);

    AlarmOut();

    double testPress = pressureRead();
    double testO2 = o2Read();
    qDebug() << "O2: " << testO2;

    if((testPress>=-1) && (testPress<=50)){
        testInitSystem = true;
        initSystemString+="Data de presión: OK!\n";
    }
    else{
        testInitSystem = false;
        initSystemString+="Data de presión: NOK!\n";
    }

    if((testO2>=0) && (testO2<=100)){
        testInitSystem = true;
        initSystemString+="Data de O2: OK!\n";
    }
    else{
        testInitSystem = false;
        initSystemString+="Data de O2: NOK!\n";
    }

    if(digitalRead(AIR_PRESS_ALARM)){
        testInitSystem = false;
        initSystemString+="Presión de O2: NOK!\n";
    }
    else{
        initSystemString+="Presión de O2: OK!\n";
    }

    if(digitalRead(O2_PRESS_ALARM)){
        testInitSystem = false;
        initSystemString+="Presión de Aire: NOK!\n";
    }
    else{
        initSystemString+="Presión de Aire: OK!\n";
    }


    initSystemString+="\n------------------------------------------------------\n\n\n\n\n";

    ui->tabWidget->setTabEnabled(3, false);
    ui->tabWidget->setTabEnabled(4, false);
    ui->tabWidget->setTabEnabled(5, false);

    ui->label_estado->setStyleSheet("color: rgb(0, 0, 0)");
    ui->label_estado->setText("Estado: ");

    readedO2 = fioSetPoint;

    return initSystemString;
}





void MainWindow::on_pushButton_alarmTest_3_clicked()
{
    digitalWrite(RELE4,LOW);
    alarmTimeStop = millis();
    alarmOn = false;
    alarmText = "";
    ui->label_estado->setStyleSheet("color: rgb(0, 0, 0)");
    ui->label_estado->setText("Estado: ");
}





void MainWindow::on_tabWidget_sel_currentChanged(int index)
{
    evalVel();
    if(index==0){
        pen0.setWidth(1);
        pen1.setWidth(1);
        pen2.setWidth(2);

        pressControlActive = true;
        volControlActive = false;
        alarmCurrenCiclosVol = cicleCounter;
        if((setPIP-3)<=setPEEP){
            setPEEP=setPIP-3;
            ui->label_setPEEP->setText(QString::number(setPEEP,'f',1));
        }
    }
    else if (index==1){
        pen0.setWidth(1);
        pen1.setWidth(2);
        pen2.setWidth(1);
        pressControlActive = false;
        volControlActive = true;
    }
    qDebug() << "Press: " << pressControlActive << " Vol: " << volControlActive;
}


void MainWindow::on_pushButton_alarmTest_2_clicked()
{
    testError=!testError;
}


void MainWindow::on_horizontalSlider_pressSlope_sliderMoved(int position)
{
    slopePressureAdj = ((double(position)-500.0)/200)+1;
    ui->label_slopePress->setText(QString::number(slopePressureAdj,'g',3));
}

void MainWindow::on_horizontalSlider_pressOffset_sliderMoved(int position)
{
    offsetPressureAdj = (double(position)/1000.0)*5.0;
    ui->label_offsetPress->setText(QString::number(offsetPressureAdj,'g',3));
}

void MainWindow::on_horizontalSlider_flowSlope_sliderMoved(int position)
{
    slopePressureAdj = (double(position)-500.0)/200.0;
    ui->label_slopeFlow->setText(QString::number(slopePressureAdj,'g',3));
}

void MainWindow::on_horizontalSlider_flowOffset_sliderMoved(int position)
{
    offsetFlowAdj = (double(position)/1000.0)*5.0;
    ui->label_offsetFlow->setText(QString::number(offsetFlowAdj,'g',3));
}

void MainWindow::on_pushButton_min_maxVol_clicked()
{
    flowError-=1;
    ui->label_errorFlow->setText(QString::number(flowError));
}

void MainWindow::on_pushButton_mor_maxVol_clicked()
{
    flowError+=1;
    ui->label_errorFlow->setText(QString::number(flowError));
}

void MainWindow::on_pushButton_min_fio2set_clicked()
{
    fioSetPoint-=1;
    ui->label_o2setpoint->setText(QString::number(fioSetPoint ));
}

void MainWindow::on_pushButton_mor_fio2set_clicked()
{
    fioSetPoint+=1;
    ui->label_o2setpoint->setText(QString::number(fioSetPoint ));
}

void MainWindow::on_pushButton_min_fio2setError_clicked()
{
    fio2Error-=1;
    ui->label_o2setpoint_error->setText(QString::number(fio2Error));
}

void MainWindow::on_pushButton_mor_fio2setError_clicked()
{
    fio2Error+=1;
    ui->label_o2setpoint_error->setText(QString::number(fio2Error));
}

void MainWindow::on_pushButton_min_frError_clicked()
{
    frError-=1;
    ui->label_frError->setText(QString::number(frError));
}

void MainWindow::on_pushButton_mor_frError_clicked()
{
    frError+=1;
    ui->label_frError->setText(QString::number(frError));
}
