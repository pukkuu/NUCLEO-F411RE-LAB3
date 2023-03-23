/*
 * user_function.c
 *
 *  Created on: Feb 22, 2023
 *      Author: 08809
 */

#include <motor_control.h>

// USER DEFINE VARIABLE BEGIN
extern TIM_HandleTypeDef htim1;
extern float PWM_Duty_Cycle;

MOTOR_Structure M1 = {
		&htim1,
		TIM_CHANNEL_1,
		{GPIOB, GPIO_PIN_10},
		{GPIOB, GPIO_PIN_4}
};

float error = 0;
float integral_error = 0;

float Kp = 2;
float Ki = 2;

float dt = 1 / 500.0;
// USER DEFINE VARIABLE BEGIN

// USER DEFINE FUNCTION BEGIN
void MotorSetDuty(MOTOR_Structure Mx, float DutyCycle)
{
	if (DutyCycle >= 0) {
		HAL_GPIO_WritePin(Mx.I1.Port, Mx.I1.Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(Mx.I2.Port, Mx.I2.Pin, GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(Mx.htim, Mx.TIM_CHANNEL, (uint16_t) DutyCycle * (__HAL_TIM_GET_AUTORELOAD(Mx.htim) + 1) / 100);
	} else {
		HAL_GPIO_WritePin(Mx.I1.Port, Mx.I1.Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Mx.I2.Port, Mx.I2.Pin, GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(Mx.htim, Mx.TIM_CHANNEL, (uint16_t) -1 * DutyCycle * (__HAL_TIM_GET_AUTORELOAD(Mx.htim) + 1) / 100);
	}
}

float MotorReadRPM(TIM_HandleTypeDef htimx, uint32_t *Buffer)
{
	int CurrentDMAPointer = (IC_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(htimx.hdma[1]) - 1 + IC_BUFFER_SIZE) % IC_BUFFER_SIZE;
	int PreviousDMAPointer = (CurrentDMAPointer - 1 + IC_BUFFER_SIZE) % IC_BUFFER_SIZE;
	return 1.0e6 * 60.0 / ((float) (Buffer[CurrentDMAPointer] - Buffer[PreviousDMAPointer]) * GEAR_RATIO * PULSE_PER_REVOLUTION);
}

void MotorControlRPM(MOTOR_Structure Mx, int SetRPM, float InRPM)
{
	static int Prev_SetRPM = 0;

	error = SetRPM - InRPM;

	if (SetRPM != Prev_SetRPM) integral_error = 0;
	if (!(PWM_Duty_Cycle >= 100 && (((error >= 0) && (integral_error >= 0)) || ((error < 0) && (integral_error < 0))))) integral_error += error * dt;

	PWM_Duty_Cycle = ((-0.0311 * pow(SetRPM, 2)) + (4.23 * SetRPM) + 2.5095) + (Kp * error) + (Ki * integral_error);

	if (PWM_Duty_Cycle > 100) PWM_Duty_Cycle = 100;
	else if (PWM_Duty_Cycle < 0) PWM_Duty_Cycle = 0;

	if (SetRPM == 0) PWM_Duty_Cycle = 0;

	MotorSetDuty(Mx, PWM_Duty_Cycle);
	Prev_SetRPM = SetRPM;
}

float ComputeLowpassConstant(uint16_t CutoffFreq, uint16_t SamplingFreq)
{
	return CutoffFreq / ((float) (CutoffFreq + SamplingFreq));
}
// USER DEFINE FUNCTION END
