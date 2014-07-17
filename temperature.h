#ifndef __TEMPERATURE_H
#define __TEMPERATURE_H

#ifdef __cplusplus
extern "C" {
#endif

#define EXTRUDER0  (0)
#define EXTRUDER1  (1)
#define HOTEDBED   (2)
#define COLDEDFAN0 (3)
#define COLDEDFAN1 (4)

bool tp_init(void);
void tp_close(void);

void manageHeater(unsigned long runTime);

double getExtruderTemp(int nExtruder);
double getHotedBedTemp(void);

double getExtruderTempRawValue(int nExtruder);
double getHotedBedTempRawValue(void);

void setTargetHotend(double target, int nExtruder);
void setTargetBed(double target);
double getExtruderTargetTemp(int nExtruder);
double getHotedBedTargetTemp(void);
double getExtruderPower(int nExtruder);
double getHotedBedPower(void);

bool isHeatingHotend(int nExtruder);
bool isHeatingBed();
bool isCoolingHotend(int nExtruder);
bool isCoolingBed();

void setFanPeriod(int fan, double period);
double constrain(double value, double min, double max);

void disable_heater();
bool PID_autotune(double temp, int extruder, int ncycles);

bool SelectThermistorTable(int nExtruder, int index);
bool CreateThermistorTable(int index, int num_temps, double min_temp, double max_temp, double rp,
                                                     double t1, double t2, double t3, 
                                                     double r1, double r2, double r3);

void setExtruderPGain(double p);
void setExtruderIGain(double i);
void setExtruderDGain(double d);
void setBedPGain(double p);
void setBedIGain(double i);
void setBedDGain(double d);
double scalePID_i(double i);
double unscalePID_i(double i);
double scalePID_d(double d);
double unscalePID_d(double d);
void updatePID();

#ifdef __cplusplus
}
#endif

#endif
