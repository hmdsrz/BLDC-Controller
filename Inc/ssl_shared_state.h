#ifndef __ORBI_SHARED_STATE__
#define __ORBI_SHARED_STATE__

#include "stm32f7xx_hal.h"
/* #include "orbi_helper.h"
 */
//struct OrbiPacket{
//	uint8_t 				packet_id;
//	float 					velocity;
//	float						angle;
//	uint8_t					brake;
//	uint8_t					calibrate;
//	uint8_t					drift;
//	uint8_t					nitro;
//	
//	
//};
//struct OrbiSetting{
//	uint8_t 				packet_id;
//	struct RGB 					rgb;
//	struct CR_SETTING 	control;
//	
//};
struct SharedState
{
	//-- System Variables
	/* used for handling interuptions and other processor system calls.
	 * initialized in: main.c
	 * used in: main.c orbi_system.c
	 * [SYSTEM]
	 */
	
	ADC_HandleTypeDef* hadc;
	
	TIM_HandleTypeDef* htim_rgb;		
	TIM_HandleTypeDef* htim_mLoop;	
	TIM_HandleTypeDef* htim_motors;	
	TIM_HandleTypeDef* htim_log;		
	TIM_HandleTypeDef* htim_parser;
	
	UART_HandleTypeDef* huart_log;
	UART_HandleTypeDef* huart_ble;
	
	I2C_HandleTypeDef* hi2c_imu;
	
	DMA_TypeDef* 								dma_ble_rx;
	DMA_TypeDef* 								dma_ble_tx;
	DMA_Channel_TypeDef* 				dma_ble_ch_rx;
	DMA_Channel_TypeDef* 				dma_ble_ch_tx;
	
	//-- Physical Variables
	/* representing the phisical state of the robot including the robot 
	 * orientation in the space.
	 * provide in: orbi_imu_wrapper.c
	 * used in: orbi_controller.c
	 * [READ-ONLY : except for provider]
	 */
	 struct Point3D orientation;
	
	
	//-- Commands
	/* representing the command given by the user application.
	 * provide in: orbi_bluetooth_wrapper.c
	 * used in: orbi_controller.c
	 * [READ-ONLY : except for provider]
	 */
//	struct OrbiPacket packet;
//	struct OrbiSetting setting;
//	
//	uint8_t 				sent_setting_id;
	
	//-- Controllers Configuration
	/* representing PID controller configuration (KP, KI, KD, Lambda, MAX_INTEGRAL) for using 
	 * in controller.
	 * provide in: orbi_controller.c
	 * used in: orbi_controller.c
	 * [READ-WRITE : except for provider]
	 */
//	struct PIDConfiguration pid_forward_configuration;
//	struct PIDConfiguration pid_roll_configuration;
//	struct PIDConfiguration pid_yaw_configuration;
//	
//	struct PIDConfiguration pid_yaw_calibrate;
	
	//-- Controlling Variables
	/* representing the flag to know wheather the system is running or not.
	 * (set it to false in order to terminate the system)
	 * initialized in: main.c
	 * used in: main.c
	 * [WRITE-ONLY : except for main.c]
	 */
	char systemIsRunning;
	uint8_t user_disconnected;
	uint32_t global_counter;
	float temperature;
	uint8_t battery;
	float angle_offset;
	float last_offset;
	int accel_x;
	int accel_y;
	int accel_z;
	float linear_accel;
	float current_left,current_right;
	
};


#endif

