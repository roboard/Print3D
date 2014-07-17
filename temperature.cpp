#define __TEMPERATURE_LIB

#include <stdio.h>
#include <math.h>
#include "config.h"
#include "global_setting.h"
#include "temperature.h"
#include "mcex.h"
#include "io.h"
#define USE_COMMON
#include "common.h"
#include "communication.h"
#include "EEPROM.h"
#include "Arduino.h"

// Power Input and Voltage Limitator
#define RA                   (110000.0)
#define RB                   (10000.0)
#define STD_POWER_INPUT      (12.0)
static double power_input = STD_POWER_INPUT;
static double power_factor = 1.0;
#define POWER_INPUT(val)     (MAX_ADC_VOLTAGE*((double)val/(double)MAX_ADC_VAL)*(RA+RB)/RB)
#define POWER_FACTOR(input)  ((STD_POWER_INPUT/input)*(STD_POWER_INPUT/input))
#define POWER_LIMITATOR(val) ((val < power_factor) ? (val) : (power_factor))

typedef struct {
	int TempRaw;
	double Temp;
	double temp_iState_min;
	double temp_iState_max;
	double mintemp;
	double maxtemp;
	
	int table_length;
	double **table;
	
	int raw_temp_value;
	int current_temperature_raw;
	double current_temperature;
	double target_temperature;
	double target;
	bool pid_reset;
	double pTerm;
	double iTerm;
	double dTerm;
	double temp_iState;
	double temp_dState;
	double soft_pwm;
	
	double Kp;
	double Ki;
	double Kd;
	
} TemperatureControl;

TemperatureControl *tpc = NULL;
static double pid_dt = 0.0;

static bool temp_meas_ready = false;
static bool in_autotune_state = false;

#define EEPROM_EXTRUDER_P_GAIN    (0x00)
#define EEPROM_EXTRUDER_I_GAIN    (0x08)
#define EEPROM_EXTRUDER_D_GAIN    (0x10)
#define EEPROM_BED_P_GAIN         (0x18)
#define EEPROM_BED_I_GAIN         (0x20)
#define EEPROM_BED_D_GAIN         (0x28)
#define TEMP_TABLE_EEPROM_BASE    (0x30)
#define TEMP_TABLE_EEPROM_BIAS    (0x05)
#define TEMP_TABLE_EEPROM_OFFSET  (0x400) // 1024
#define MAX_TABLE_NUM (5)
#define MAX_LIST_NUM  (int)((TEMP_TABLE_EEPROM_OFFSET-4)/(sizeof(double)*2))
// static __inline__ void eeprom_write_double(int addr, double val)
// {
	// int i, size;
	// unsigned char tmp[8];
	
	// size = sizeof(double);
	// memcpy(tmp, &val, size);
	// for (i = 0; i < size; i++)
		// EEPROM.write(addr + i, tmp[i]);
// }

// static __inline__ double eeprom_read_double(int addr)
// {
	// int i, size;
	// double val;
	// unsigned char tmp[8];
	
	// size = sizeof(double);
	// for (i = 0; i < size; i++)
		// tmp[i] = EEPROM.read(addr + i);
	// memcpy(&val, tmp, size);
	
	// return val;
// }

static __inline__ double** read_thermistor_table(int index, int *length)
{
	int address, i, j, k, size;
	unsigned char tmp[8];
	double **table;
	
	address = TEMP_TABLE_EEPROM_BASE + TEMP_TABLE_EEPROM_BIAS + TEMP_TABLE_EEPROM_OFFSET*index;
	
	size = sizeof(int);
	for (i = 0; i < size; i++)
		tmp[i] = EEPROM.read(address++);
	memcpy(length, tmp, size);
	if (*length <= 0) return NULL;
	
	table = (double**)malloc(sizeof(double)*(*length)*2 + sizeof(double*)*(*length));
	
	size = sizeof(double);
	for (i = 0; i < *length; i++) {
		table[i] = (double*)(table + (*length)) + i*2;
		for (j = 0; j < 2; j++) {
			for (k = 0; k < size; k++)
				tmp[k] = EEPROM.read(address++);
			memcpy(&(table[i][j]), tmp, size);
		}
	}
	
	return table;
}

static __inline__ int get_table_index(int e)
{
	int ret = EEPROM.read(TEMP_TABLE_EEPROM_BASE + e);
	return (ret >= MAX_TABLE_NUM || ret < 0) ? -1 : ret;
}

static __inline__ void set_table_index(int e, int index)
{
	EEPROM.write(TEMP_TABLE_EEPROM_BASE + e, index);
}

bool tp_init(void)
{
	int i, size, vin_sample, pwm_num, index;
	unsigned long McUse, pow_adc;

	if (io_Init() == false) return false;

	tpc = (TemperatureControl*)malloc(sizeof(TemperatureControl)*3);
	if (tpc == NULL) {
		io_Close();
		return false;
	}
	memset(tpc, 0, sizeof(TemperatureControl)*3);
	//for(i = 0; i<16384; i++) {
	//	EEPROM.write(i,0);
	//}

	mc_setbaseaddr();
	for (i = 0, McUse = 0; i < 5; i++) {
		if (pins->mc_pwm[i] != -1)
		McUse |= (1 << pins->mc_pwm[i]);
	}

	i = 0;
	while (i < (sizeof(McUse)*8))
	{
		if (McUse & (1 << i)) mc_SetMode(i, MCMODE_PWM_SIFB);
		i++;
	}

	for (i = 0; i < 5; i++)
	{
		if (pins->mc_pwm[i] == -1 || pins->md_pwm[i] == -1) continue;
		mcpwm_SetOutMask(pins->mc_pwm[i], pins->md_pwm[i], MCPWM_HMASK_NONE + MCPWM_LMASK_NONE);
		mcpwm_SetOutPolarity(pins->mc_pwm[i], pins->md_pwm[i], MCPWM_HPOL_INVERSE + MCPWM_LPOL_INVERSE);
		mcpwm_SetDeadband(pins->mc_pwm[i], pins->md_pwm[i], 0L);
		mcpwm_ReloadOUT_Unsafe(pins->mc_pwm[i], pins->md_pwm[i], MCPWM_RELOAD_NOW);
		mcpwm_SetWaveform(pins->mc_pwm[i], pins->md_pwm[i], MCPWM_EDGE_A0I1);
		mcpwm_SetWidth(pins->mc_pwm[i], pins->md_pwm[i], tp->pwm_period-1, 0L);
		mcpwm_SetSamplCycle(pins->mc_pwm[i], pins->md_pwm[i], 0UL);
		mcpwm_Enable(pins->mc_pwm[i], pins->md_pwm[i]);
	}

	if (pins->mc_pwm[COLDEDFAN0] >= 0 && pins->md_pwm[COLDEDFAN0] >= 0) {
		mcpwm_SetWidth(pins->mc_pwm[COLDEDFAN0], pins->md_pwm[COLDEDFAN0], tp->pwm_period-1, tp->pwm_period-1);
		mcpwm_ReloadPWM(pins->mc_pwm[COLDEDFAN0], pins->md_pwm[COLDEDFAN0], MCPWM_RELOAD_NOW);
	}
	if (pins->mc_pwm[COLDEDFAN1] >= 0 && pins->md_pwm[COLDEDFAN1] >= 0) {
		mcpwm_SetWidth(pins->mc_pwm[COLDEDFAN1], pins->md_pwm[COLDEDFAN1], tp->pwm_period-1, tp->pwm_period-1);
		mcpwm_ReloadPWM(pins->mc_pwm[COLDEDFAN1], pins->md_pwm[COLDEDFAN1], MCPWM_RELOAD_NOW);
	}
	
	pid_dt = (double)tp->oversampler/1000.0;
	for (i = 0; i < tp->extruders; i++)
	{
		tpc[i].Kp = tp->default_Kp;//eeprom_read_double(EEPROM_EXTRUDER_P_GAIN);
		tpc[i].Ki = tp->default_Ki;//eeprom_read_double(EEPROM_EXTRUDER_I_GAIN);
		tpc[i].Kd = tp->default_Kd;//eeprom_read_double(EEPROM_EXTRUDER_D_GAIN);
		tpc[i].temp_iState_min = 0.0;
		tpc[i].temp_iState_max = tp->pid_integral_drive_max/(tpc[i].Ki*pid_dt);
		tpc[i].mintemp = tp->heater_mintemp[i];
		tpc[i].maxtemp = tp->heater_maxtemp[i];
		index = get_table_index(i);
		if (index < 0) index = 0;
		tpc[i].table = read_thermistor_table(index, &(tpc[i].table_length));
	}
	tpc[HOTEDBED].Kp = tp->default_bedKp;//eeprom_read_double(EEPROM_BED_P_GAIN);
	tpc[HOTEDBED].Ki = tp->default_bedKi;//eeprom_read_double(EEPROM_BED_I_GAIN);
	tpc[HOTEDBED].Kd = tp->default_bedKd;//eeprom_read_double(EEPROM_BED_D_GAIN);
	tpc[HOTEDBED].temp_iState_min = 0.0;
	tpc[HOTEDBED].mintemp = tp->bed_mintemp;
	tpc[HOTEDBED].maxtemp = tp->bed_maxtemp;
	tpc[HOTEDBED].temp_iState_max = tp->pid_integral_drive_max/(tpc[HOTEDBED].Ki*pid_dt);
	index = get_table_index(HOTEDBED);
	if (index < 0) index = 0;
	tpc[HOTEDBED].table = read_thermistor_table(index, &(tpc[HOTEDBED].table_length));

	// Power Input & Voltage Limitator 
	vin_sample = 16;
	for (i = 0, pow_adc = 0; i < vin_sample; i++)
		pow_adc += analogRead(pins->vin_ad);
	pow_adc /= vin_sample;
	power_input = POWER_INPUT(pow_adc);
	power_factor = POWER_FACTOR(power_input);
  
	return true;
}

void tp_close(void)
{
	int i, size;

	for (i = 0; i < 5; i++)
	{
		if (pins->mc_pwm[i] == -1 || pins->md_pwm[i] == -1) continue;
		mcpwm_SetOutMask(pins->mc_pwm[i], pins->md_pwm[i], MCPWM_HMASK_INACTIVE + MCPWM_LMASK_INACTIVE);
		mcpwm_ReloadOUT_Unsafe(pins->mc_pwm[i], pins->md_pwm[i], MCPWM_RELOAD_NOW);
		mcpwm_Disable(pins->mc_pwm[i], pins->md_pwm[i]);
	}
	if (tpc != NULL) {
		free(tpc);
		tpc = NULL;
	}
	io_Close();
}

static __inline__ double analog2temp(int adc, int e)
{
  int i;
  double temp;
  
  if (tpc[e].table_length <= 0) return 0;

  for (i = 0; i < tpc[e].table_length; i++)
    if (adc < tpc[e].table[i][0]) break;
	
  if (i == 0) return tpc[e].table[i][1];
  if (i == tpc[e].table_length) return tpc[e].table[i-1][1];
  
  temp = tpc[e].table[i-1][1] + ((double)adc-tpc[e].table[i-1][0])*
                                  (tpc[e].table[i][1]-tpc[e].table[i-1][1])/
								  (tpc[e].table[i][0]-tpc[e].table[i-1][0]);
  return temp;
}

static __inline__ double analog2tempBed(int adc)
{
	return analog2temp(adc, HOTEDBED);
}

static __inline__ void updateTemperaturesFromRawValues()
{
	int e;
	
	for (e = 0; e < tp->extruders; e++)
		tpc[e].current_temperature = analog2temp(tpc[e].current_temperature_raw, e);
	tpc[HOTEDBED].current_temperature = analog2tempBed(tpc[HOTEDBED].current_temperature_raw);
	
	temp_meas_ready = false;
}

void manageHeater(unsigned long runTime)
{
	static int temp_count = 0;
	static unsigned long pretime = 0UL;
	static unsigned long controlTime = 0UL;
	unsigned long nowtime;
	int e;
	double pid_input, pid_output, pid_error, dt;

	nowtime = timer_nowtime();
	if (nowtime - pretime < 1UL) return;
		pretime = nowtime;

	for (e = 0; e < tp->extruders; e++)
	{
		tpc[e].raw_temp_value += analogRead(pins->extruder_ad[e]);
	}
	tpc[HOTEDBED].raw_temp_value += analogRead(pins->hotedbed_ad);
	temp_count++;

	if (temp_count != tp->oversampler) return;

	for (e = 0; e < tp->extruders + 1; e++)
	{
		tpc[e].current_temperature_raw = tpc[e].raw_temp_value;
		tpc[e].raw_temp_value = 0;
	}
	tpc[HOTEDBED].current_temperature_raw = tpc[HOTEDBED].raw_temp_value;
	tpc[HOTEDBED].raw_temp_value = 0;
	temp_count = 0;

	temp_meas_ready = true;
	if (in_autotune_state == true) return;

	updateTemperaturesFromRawValues();
	
	for (e = 0; e < tp->extruders; e++)
	{
		if (tpc[e].target_temperature - tpc[e].current_temperature > 5.0) {
			tpc[e].target += tp->heater_velocity * ((double)(nowtime - controlTime)/1000.0);
			tpc[e].target = constrain(tpc[e].target, 0.0, tpc[e].target_temperature);
		}
		else
			tpc[e].target = tpc[e].target_temperature;
	}
	if (tpc[HOTEDBED].target_temperature - tpc[HOTEDBED].current_temperature > 5.0) {
		tpc[HOTEDBED].target += tp->heater_velocity * ((double)(nowtime - controlTime)/1000.0);
		tpc[HOTEDBED].target = constrain(tpc[HOTEDBED].target, 0.0, tpc[HOTEDBED].target_temperature);
	}
	else
		tpc[HOTEDBED].target = tpc[HOTEDBED].target_temperature;
	
	dt = (double)(nowtime - controlTime)/1000.0;
	controlTime = nowtime;
	
	for (e = 0; e < tp->extruders; e++)
	{
		pid_input = tpc[e].current_temperature;
		pid_error = tpc[e].target - pid_input;
		if(pid_error > tp->pid_functional_range)
		{
			pid_output = tp->bang_max;
			tpc[e].pid_reset = true;
		}
		else if(pid_error < -(tp->pid_functional_range) || tpc[e].target_temperature == 0)
		{
			pid_output = 0;
			tpc[e].pid_reset = true;
		}
		else {
			if(tpc[e].pid_reset == true)
			{
				tpc[e].temp_iState = 0.0;
				tpc[e].pid_reset = false;
			}
			tpc[e].pTerm = tpc[e].Kp * pid_error;
			tpc[e].temp_iState += pid_error;
			tpc[e].temp_iState = constrain(tpc[e].temp_iState, tpc[e].temp_iState_min, tpc[e].temp_iState_max);
			tpc[e].iTerm = (tpc[e].Ki*dt) * tpc[e].temp_iState;

			tpc[e].dTerm = ((tpc[e].Kd/dt) * (pid_input - tpc[e].temp_dState))*tp->K2 + (tp->K1 * tpc[e].dTerm);
			pid_output = constrain(tpc[e].pTerm + tpc[e].iTerm - tpc[e].dTerm, 0, tp->pid_max);
		}
		tpc[e].temp_dState = pid_input;
	
		if((tpc[e].current_temperature > tpc[e].mintemp) && (tpc[e].current_temperature < tpc[e].maxtemp)) 
		{
			tpc[e].soft_pwm = pid_output/tp->pid_max;
		}
		else
		{
			tpc[e].soft_pwm = 0;
		}

		tpc[e].soft_pwm = constrain(POWER_LIMITATOR(tpc[e].soft_pwm), 0.0, 1.0);
		
		if (pins->mc_pwm[e] >= 0 && pins->md_pwm[e] >= 0) {
			mcpwm_SetWidth(pins->mc_pwm[e], pins->md_pwm[e], tp->pwm_period-1, (unsigned long)(tp->pwm_period*tpc[e].soft_pwm));
			mcpwm_ReloadPWM(pins->mc_pwm[e], pins->md_pwm[e], MCPWM_RELOAD_PEREND);
		}
	}
  
	pid_input = tpc[HOTEDBED].current_temperature;
	pid_error = tpc[HOTEDBED].target - pid_input;
	tpc[HOTEDBED].pTerm = tpc[HOTEDBED].Kp * pid_error;
	tpc[HOTEDBED].temp_iState += pid_error;
	tpc[HOTEDBED].temp_iState = constrain(tpc[HOTEDBED].temp_iState, tpc[HOTEDBED].temp_iState_min, tpc[HOTEDBED].temp_iState_max);
	tpc[HOTEDBED].iTerm = (tpc[HOTEDBED].Ki*dt) * tpc[HOTEDBED].temp_iState;

	tpc[HOTEDBED].dTerm = ((tpc[HOTEDBED].Kd/dt) * (pid_input - tpc[HOTEDBED].temp_dState))*(tp->K2) + (tp->K1 * tpc[HOTEDBED].dTerm);
	tpc[HOTEDBED].temp_dState = pid_input;

	pid_output = constrain(tpc[HOTEDBED].pTerm + tpc[HOTEDBED].iTerm - tpc[HOTEDBED].dTerm, 0, tp->max_bed_power);

	if((tpc[HOTEDBED].current_temperature > tpc[HOTEDBED].mintemp) && (tpc[HOTEDBED].current_temperature < tpc[HOTEDBED].maxtemp)) 
	{
		tpc[HOTEDBED].soft_pwm = pid_output/tp->pid_max;
	}
	else
	{
		tpc[HOTEDBED].soft_pwm = 0;
	}
	tpc[HOTEDBED].soft_pwm = constrain(POWER_LIMITATOR(tpc[HOTEDBED].soft_pwm), 0.0, 1.0);
	if (pins->mc_pwm[HOTEDBED] >= 0 && pins->md_pwm[HOTEDBED] >= 0) {
		mcpwm_SetWidth(pins->mc_pwm[HOTEDBED], pins->md_pwm[HOTEDBED], tp->pwm_period-1, (unsigned long)(tp->pwm_period*tpc[HOTEDBED].soft_pwm));
		mcpwm_ReloadPWM(pins->mc_pwm[HOTEDBED], pins->md_pwm[HOTEDBED], MCPWM_RELOAD_PEREND);
	}
}

double getExtruderTemp(int e)
{
	return tpc[e].current_temperature;
}

double getHotedBedTemp(void)
{
	return getExtruderTemp(HOTEDBED);
}

double getExtruderTempRawValue(int e)
{
	return (double)(tpc[e].current_temperature_raw)/tp->oversampler;
}

double getHotedBedTempRawValue(void)
{
	return getExtruderTempRawValue(HOTEDBED);
}

void setTargetHotend(double target, int e)
{
	tpc[e].target_temperature = target;
	tpc[e].target = tpc[e].current_temperature;
}

void setTargetBed(double target)
{
	setTargetHotend(target, HOTEDBED);
}

double getExtruderTargetTemp(int e)
{
	return tpc[e].target_temperature;
}

double getHotedBedTargetTemp(void)
{
	return getExtruderTargetTemp(HOTEDBED);
}

double getExtruderPower(int e)
{
	return tpc[e].soft_pwm;
}

double getHotedBedPower(void)
{
	return getExtruderPower(HOTEDBED);
}

bool isHeatingHotend(int e)
{  
	return tpc[e].target_temperature > tpc[e].current_temperature;
}

bool isHeatingBed()
{
	return isHeatingHotend(HOTEDBED);
}

bool isCoolingHotend(int e)
{  
	return tpc[e].target_temperature < tpc[e].current_temperature;
}

bool isCoolingBed()
{
	return isCoolingHotend(HOTEDBED);
}

void setFanPeriod(int fan, double period)
{
	if (fan != COLDEDFAN0 && fan != COLDEDFAN1) return;
	
	period = constrain(period, 0, tp->fan_max);
	if (pins->mc_pwm[fan] >= 0 && pins->md_pwm[fan] >= 0) {
		mcpwm_SetWidth(pins->mc_pwm[fan], pins->md_pwm[fan], tp->pwm_period-1, (unsigned long)(tp->pwm_period*period/tp->fan_max));
		mcpwm_ReloadPWM(pins->mc_pwm[fan], pins->md_pwm[fan], MCPWM_RELOAD_NOW);
	}
}

void disable_heater()
{
	int e;
	
	for(e = 0; e < tp->extruders; e++) {
		setTargetHotend(0, e);
		
		tpc[e].soft_pwm = 0;
		if (pins->mc_pwm[e] >= 0 && pins->md_pwm[e] >= 0) {
			mcpwm_SetWidth(pins->mc_pwm[e], pins->md_pwm[e], tp->pwm_period-1, 0UL);
			mcpwm_ReloadPWM(pins->mc_pwm[e], pins->md_pwm[e], MCPWM_RELOAD_PEREND);
			while (mcpwm_ReadReloadPWM(pins->mc_pwm[e], pins->md_pwm[e]));
		}
	}

	setTargetBed(0);
	tpc[HOTEDBED].soft_pwm = 0;
	if (pins->mc_pwm[HOTEDBED] >= 0 && pins->md_pwm[HOTEDBED] >= 0) {
		mcpwm_SetWidth(pins->mc_pwm[HOTEDBED], pins->md_pwm[HOTEDBED], tp->pwm_period-1, 0UL);
		mcpwm_ReloadPWM(pins->mc_pwm[HOTEDBED], pins->md_pwm[HOTEDBED], MCPWM_RELOAD_PEREND);
		while (mcpwm_ReadReloadPWM(pins->mc_pwm[HOTEDBED], pins->md_pwm[HOTEDBED]));
	}
}

bool PID_autotune(double temp, int e, int ncycles)
{
	static int PID_autotune_state = 0;
	static double input, max, min;
	static int cycles;
	static bool heating;
	static unsigned long temp_millis, t1, t2;
	static long t_high, t_low, bias, d;
	double Ku, Tu, Kp, Ki, Kd;
	
	switch (PID_autotune_state)
	{
	case 0:
		input = 0.0;
		cycles = 0;
		heating = true;
		temp_millis = timer_nowtime();
		t1=temp_millis;
		t2=temp_millis;
		t_high = 0;
		t_low = 0;
		max = 0;
		min = 10000;
		if ((e >= tp->extruders) || 
		    (e < 0 && (pins->mc_pwm[HOTEDBED] < 0 || pins->md_pwm[HOTEDBED] < 0)))
		{
			COMM_WRITE("PID Autotune failed. Bad extruder number.\n");
			return true;
		}
		COMM_WRITE("PID Autotune start\n");
		disable_heater(); // switch off all heaters.
		
		if (e < 0)
		{
			tpc[HOTEDBED].soft_pwm = (double)(tp->max_bed_power)/(tp->max_bed_power);
			bias = d = (tp->max_bed_power)/2;
			tpc[HOTEDBED].soft_pwm = constrain(POWER_LIMITATOR(tpc[HOTEDBED].soft_pwm), 0.0, 1.0);
			if (pins->mc_pwm[HOTEDBED] >= 0 && pins->md_pwm[HOTEDBED] >= 0) {
				mcpwm_SetWidth(pins->mc_pwm[HOTEDBED], pins->md_pwm[HOTEDBED], tp->pwm_period-1, (unsigned long)(tp->pwm_period*tpc[HOTEDBED].soft_pwm));
				mcpwm_ReloadPWM(pins->mc_pwm[HOTEDBED], pins->md_pwm[HOTEDBED], MCPWM_RELOAD_PEREND);
			}
			while (mcpwm_ReadReloadPWM(pins->mc_pwm[HOTEDBED], pins->md_pwm[HOTEDBED]));
		}
		else
		{
			tpc[e].soft_pwm = (double)(tp->pid_max)/(tp->pid_max);
			bias = d = (tp->pid_max)/2;
			tpc[e].soft_pwm = constrain(POWER_LIMITATOR(tpc[e].soft_pwm), 0.0, 1.0);
			if (pins->mc_pwm[e] >= 0 && pins->md_pwm[e] >= 0) {
				mcpwm_SetWidth(pins->mc_pwm[e], pins->md_pwm[e], tp->pwm_period-1, (unsigned long)(tp->pwm_period*tpc[e].soft_pwm));
				mcpwm_ReloadPWM(pins->mc_pwm[e], pins->md_pwm[e], MCPWM_RELOAD_PEREND);
			}
			while (mcpwm_ReadReloadPWM(pins->mc_pwm[e], pins->md_pwm[e]));
		}
		PID_autotune_state = 1;
		in_autotune_state = true;
		break;
	case 1:
		if (temp_meas_ready == true) {
			updateTemperaturesFromRawValues();
			
			input = (e < 0) ? tpc[HOTEDBED].current_temperature:tpc[e].current_temperature;
			
			max = (max > input) ? (max) : (input);
			min = (min < input) ? (min) : (input);
			if(heating == true && input > temp) {
				if(timer_nowtime() - t2 > 5000) { 
					heating=false;
					if (e<0) {
						tpc[HOTEDBED].soft_pwm = (double)(bias - d)/tp->max_bed_power;
						tpc[HOTEDBED].soft_pwm = constrain(POWER_LIMITATOR(tpc[HOTEDBED].soft_pwm), 0.0, 1.0);
						if (pins->mc_pwm[HOTEDBED] >= 0 && pins->md_pwm[HOTEDBED] >= 0) {
							mcpwm_SetWidth(pins->mc_pwm[HOTEDBED], pins->md_pwm[HOTEDBED], tp->pwm_period-1, (unsigned long)(tp->pwm_period*tpc[HOTEDBED].soft_pwm));
							mcpwm_ReloadPWM(pins->mc_pwm[HOTEDBED], pins->md_pwm[HOTEDBED], MCPWM_RELOAD_PEREND);
						}
						while (mcpwm_ReadReloadPWM(pins->mc_pwm[HOTEDBED], pins->md_pwm[HOTEDBED]));
					}
					else {
						tpc[e].soft_pwm = (double)(bias - d)/(tp->pid_max);
						tpc[e].soft_pwm = constrain(POWER_LIMITATOR(tpc[e].soft_pwm), 0.0, 1.0);
						if (pins->mc_pwm[e] >= 0 && pins->md_pwm[e] >= 0) {
							mcpwm_SetWidth(pins->mc_pwm[e], pins->md_pwm[e], tp->pwm_period-1, (unsigned long)(tp->pwm_period*tpc[e].soft_pwm));
							mcpwm_ReloadPWM(pins->mc_pwm[e], pins->md_pwm[e], MCPWM_RELOAD_PEREND);
						}
						while (mcpwm_ReadReloadPWM(pins->mc_pwm[e], pins->md_pwm[e]));
					}
					t1=timer_nowtime();
					t_high=t1 - t2;
					max=temp;
				}
			}
			if(heating == false && input < temp) {
				if(timer_nowtime() - t1 > 5000) {
					heating=true;
					t2=timer_nowtime();
					t_low=t2 - t1;
					if(cycles > 0) {
						bias += (d*(t_high - t_low))/(t_low + t_high);
						bias = constrain(bias, 20 ,(e<0?(tp->max_bed_power):(tp->pid_max))-20);
						if(bias > (e<0?(tp->max_bed_power):(tp->pid_max))/2) d = (e<0?(tp->max_bed_power):(tp->pid_max)) - 1 - bias;
						else d = bias;

						COMM_WRITE(" bias: "); COMM_WRITE_LONG(bias);
						COMM_WRITE(" d: "); COMM_WRITE_LONG(d);
						COMM_WRITE(" min: "); COMM_WRITE_FLOAT(min);
						COMM_WRITE(" max: "); COMM_WRITE_FLOAT(max); COMM_WRITE("\n");
						if(cycles > 2) {
							Ku = (4.0*d)/(3.14159*(max-min)/2.0);
							Tu = ((double)(t_low + t_high)/1000.0);
							COMM_WRITE(" Ku: "); COMM_WRITE_FLOAT(Ku);
							COMM_WRITE(" Tu: "); COMM_WRITE_FLOAT(Tu); COMM_WRITE("\n");
							Kp = 0.6*Ku;
							Ki = 2*Kp/Tu;
							Kd = Kp*Tu/8;
							COMM_WRITE(" Classic PID \n");
							COMM_WRITE(" Kp: "); COMM_WRITE_FLOAT(Kp); COMM_WRITE("\n");
							COMM_WRITE(" Ki: "); COMM_WRITE_FLOAT(Ki); COMM_WRITE("\n");
							COMM_WRITE(" Kd: "); COMM_WRITE_FLOAT(Kd); COMM_WRITE("\n");
							/*
							Kp = 0.33*Ku;
							Ki = Kp/Tu;
							Kd = Kp*Tu/3;
							SERIAL_PROTOCOLLNPGM(" Some overshoot ");
							COMM_WRITE(" Kp: "); SERIAL_PROTOCOLLN(Kp);
							COMM_WRITE(" Ki: "); SERIAL_PROTOCOLLN(Ki);
							COMM_WRITE(" Kd: "); SERIAL_PROTOCOLLN(Kd);
							Kp = 0.2*Ku;
							Ki = 2*Kp/Tu;
							Kd = Kp*Tu/3;
							SERIAL_PROTOCOLLNPGM(" No overshoot ");
							COMM_WRITE(" Kp: "); SERIAL_PROTOCOLLN(Kp);
							COMM_WRITE(" Ki: "); SERIAL_PROTOCOLLN(Ki);
							COMM_WRITE(" Kd: "); SERIAL_PROTOCOLLN(Kd);
							*/
						}
					}
					if (e<0) {
						tpc[HOTEDBED].soft_pwm = (double)(bias + d)/tp->max_bed_power;
						tpc[HOTEDBED].soft_pwm = constrain(POWER_LIMITATOR(tpc[HOTEDBED].soft_pwm), 0.0, 1.0);
						if (pins->mc_pwm[HOTEDBED] >= 0 && pins->md_pwm[HOTEDBED] >= 0) {
							mcpwm_SetWidth(pins->mc_pwm[HOTEDBED], pins->md_pwm[HOTEDBED], tp->pwm_period-1, (unsigned long)(tp->pwm_period*tpc[HOTEDBED].soft_pwm));
							mcpwm_ReloadPWM(pins->mc_pwm[HOTEDBED], pins->md_pwm[HOTEDBED], MCPWM_RELOAD_PEREND);
						}
						while (mcpwm_ReadReloadPWM(pins->mc_pwm[HOTEDBED], pins->md_pwm[HOTEDBED]));
					}
					else {
						tpc[e].soft_pwm = (double)(bias + d)/(tp->pid_max);
						tpc[e].soft_pwm = constrain(POWER_LIMITATOR(tpc[e].soft_pwm), 0.0, 1.0);
						if (pins->mc_pwm[e] >= 0 && pins->md_pwm[e] >= 0) {
							mcpwm_SetWidth(pins->mc_pwm[e], pins->md_pwm[e], tp->pwm_period-1, (unsigned long)(tp->pwm_period*tpc[e].soft_pwm));
							mcpwm_ReloadPWM(pins->mc_pwm[e], pins->md_pwm[e], MCPWM_RELOAD_PEREND);
						}
						while (mcpwm_ReadReloadPWM(pins->mc_pwm[e], pins->md_pwm[e]));
					}
					cycles++;
					min=temp;
				}
			}
		}
		PID_autotune_state = 2;
		break;
	case 2:
		if(input > (temp + 30)) {
			COMM_WRITE("PID Autotune failed! Temperature too high\n");
			PID_autotune_state = 0;
			in_autotune_state = false;
			return true;
		}
		if(timer_nowtime() - temp_millis > 2000) {
			int p;
			if (e<0){
				p=tpc[HOTEDBED].soft_pwm;       
				COMM_WRITE("ok B:");
			}else{
				p=tpc[e].soft_pwm;       
				COMM_WRITE("ok T:");
			}
				
			COMM_WRITE_FLOAT(input);   
			COMM_WRITE(" @:");
			COMM_WRITE_LONG(p);  
			COMM_WRITE("\n");     

			temp_millis = timer_nowtime();
		}
		if(((timer_nowtime() - t1) + (timer_nowtime() - t2)) > (10L*60L*1000L*2L)) {
			COMM_WRITE("PID Autotune failed! timeout\n");
			PID_autotune_state = 0;
			in_autotune_state = false;
			return true;
		}
		if(cycles > ncycles) {
			COMM_WRITE("PID Autotune finished!");// Put the last Kp, Ki and Kd constants from above into Configuration.h\n");
			if (e < 0) {
				setBedPGain(Kp);
				setBedIGain(Ki);
				setBedDGain(Kd);
			} else {
				setExtruderPGain(Kp);
				setExtruderIGain(Ki);
				setExtruderDGain(Kd);
			}
			PID_autotune_state = 0;
			in_autotune_state = false;
			return true;
		}
		PID_autotune_state = 1;
		break;
	}

	return false;
}

bool SelectThermistorTable(int e, int index)
{
	int len;
	double **tmp;
	
	if (e >= tp->extruders) {
		COMM_WRITE("Error: The maximum number of extruder is ");
		COMM_WRITE_LONG(tp->extruders);
		COMM_WRITE(". (Please input -1 to ");
		COMM_WRITE_LONG(tp->extruders - 1);
		COMM_WRITE(")\n");
		return false;
	}
		
	if (index >= MAX_TABLE_NUM) {
		COMM_WRITE("Error: The maximum number of thermistor table is ");
		COMM_WRITE_LONG(MAX_TABLE_NUM);
		COMM_WRITE(". (Please input index 0 to ");
		COMM_WRITE_LONG(MAX_TABLE_NUM - 1);
		COMM_WRITE(")\n");
		return false;
	}
		
	tmp = read_thermistor_table(index, &len);
	
	if (tmp == NULL) {
		COMM_WRITE("Error: The index ");
		COMM_WRITE_LONG(index);
		COMM_WRITE(" thermistor table is empty.\n");
		return false;
	}
	
	if (e < 0) {
		set_table_index(HOTEDBED, index);
		if (tpc[HOTEDBED].table != NULL) free(tpc[HOTEDBED].table);
		tpc[HOTEDBED].table = tmp;
		tpc[HOTEDBED].table_length = len;
	} else {
		set_table_index(e, index);
		if (tpc[e].table != NULL) free(tpc[e].table);
		tpc[e].table = tmp;
		tpc[e].table_length = len;
	}
	
	COMM_WRITE("Success: Asign the index ");
	COMM_WRITE_LONG(index);
	COMM_WRITE(" thermistor table to ");
	if (e < 0) {
		COMM_WRITE("Bed\n");
	} else {
		COMM_WRITE("Extruder");
		COMM_WRITE_LONG(e);
		COMM_WRITE("\n");
	}
	
	return true;
}

bool CreateThermistorTable(int index, int num_temps, double min_temp, double max_temp, double rp,
                                                     double t1, double t2, double t3, 
                                                     double r1, double r2, double r3)
{
	double x, y, z, u, v, w, a1, a2, a3, c1, c2, c3;
	double cur, inc;
	double xx, yy, r, temp, adc;
	int address, i, size;
	unsigned char tmp[8];
		
	if (index >= MAX_TABLE_NUM) {
		COMM_WRITE("Error: The maximum number of thermistor table is ");
		COMM_WRITE_LONG(MAX_TABLE_NUM);
		COMM_WRITE(". (Please input index 0 to ");
		COMM_WRITE_LONG(MAX_TABLE_NUM - 1);
		COMM_WRITE(")\n");
		return false;
	}
	
	if (num_temps < 2 || num_temps > MAX_LIST_NUM) {
		COMM_WRITE("Error: The maximum number of table list is ");
		COMM_WRITE_LONG(MAX_LIST_NUM);
		COMM_WRITE(". (Please input 2 to ");
		COMM_WRITE_LONG(MAX_LIST_NUM);
		COMM_WRITE(")\n");
		return false;
	}
	
	if (max_temp < min_temp) {
		COMM_WRITE("Error: The maximum temperature ");
		COMM_WRITE_LONG(max_temp);
		COMM_WRITE("C is smaller than minimum temperature ");
		COMM_WRITE_LONG(min_temp);
		COMM_WRITE("C.\n");
		return false;
	}
	
	// caculate Steinhart-Hart Coefficients: c1, c2, c3
	t1 += 273.15;
	t2 += 273.15;
	t3 += 273.15;
	
	a1 = log(r1);
	a2 = log(r2);
	a3 = log(r3);
	
	x = 1.0/t1 - 1.0/t2;
	z = a1 - a2;
	y = a1 - a3;
	w = 1.0/t1 - 1.0/t3;
	v = pow(a1, 3) - pow(a2, 3);
	u = pow(a1, 3) - pow(a3, 3);
	
	c3 = (x - w*z/y)/(v - u*z/y);
	c2 = (x - c3*v)/z;
	c1 = 1.0/t1 - c2*a1 - c3*pow(a1,3);
	
	// create thermistor table
	inc = (min_temp - max_temp)/(num_temps - 1);
	
	address = TEMP_TABLE_EEPROM_BASE + TEMP_TABLE_EEPROM_BIAS + TEMP_TABLE_EEPROM_OFFSET*index;
	
	size = sizeof(int);
	memcpy(tmp, &num_temps, size);
	for (i = 0; i < size; i++)
		EEPROM.write(address++, tmp[i]);
		
	size = sizeof(double); 
	for (cur = max_temp; cur >= min_temp; cur += inc)
	{
		// caculate adc value on such temperature
		temp = cur + 273.15;
	
		yy = (c1 - 1.0/temp)/(2*c3);
		xx = sqrt(pow(c2/(3*c3), 3) + yy*yy);
		r = exp(pow(xx - yy, 1.0/3) - pow(xx + yy, 1.0/3));
		
		adc = (r/(rp + r))*MAX_ADC_VAL;
		adc = adc*tp->oversampler;
		
		memcpy(tmp, &adc, size);
		for (i = 0; i < size; i++)
			EEPROM.write(address++, tmp[i]);
		memcpy(tmp, &cur, size);
		for (i = 0; i < size; i++)
			EEPROM.write(address++, tmp[i]);
	}
	
	return true;
}

double scalePID_i(double i)
{
	return i*pid_dt;
}

double unscalePID_i(double i)
{
	return i/pid_dt;
}

double scalePID_d(double d)
{
    return d/pid_dt;
}

double unscalePID_d(double d)
{
	return d*pid_dt;
}

void updatePID()
{
	int e;
	
	for (e = 0; e < tp->extruders; e++) {
		tpc[e].temp_iState_max = tp->pid_integral_drive_max / (tpc[e].Ki*pid_dt);
	}
	tpc[HOTEDBED].temp_iState_max = tp->pid_integral_drive_max / (tpc[HOTEDBED].Ki*pid_dt);
}

void setExtruderPGain(double p)
{
	int e;
	
	for (e = 0; e < tp->extruders; e++) {
		tpc[e].Kp = p;
	}
	// eeprom_write_double(EEPROM_EXTRUDER_I_GAIN, p);
}

void setExtruderIGain(double i)
{
	int e;
	
	for (e = 0; e < tp->extruders; e++) {
		tpc[e].Ki = unscalePID_i(i);
	}
	// eeprom_write_double(EEPROM_EXTRUDER_I_GAIN, i);
}

void setExtruderDGain(double d)
{
	int e;
	
	for (e = 0; e < tp->extruders; e++) {
		tpc[e].Kd = unscalePID_d(d);
	}
	// eeprom_write_double(EEPROM_EXTRUDER_D_GAIN, d);
}

void setBedPGain(double p)
{
	tpc[HOTEDBED].Kp = p;
	// eeprom_write_double(EEPROM_BED_P_GAIN, tpc[HOTEDBED].Kp);
}

void setBedIGain(double i)
{
	tpc[HOTEDBED].Ki = unscalePID_i(i);
	// eeprom_write_double(EEPROM_BED_I_GAIN, tpc[HOTEDBED].Ki);
}

void setBedDGain(double d)
{
	tpc[HOTEDBED].Kd = unscalePID_d(d);
	// eeprom_write_double(EEPROM_BED_D_GAIN, tpc[HOTEDBED].Kd);
}

