#include "ssl_shared_state.h"
#include <string.h>

struct SharedState shared_state;

//extern void loadPIDConfs(void);

//void shared_state_init(ADC_HandleTypeDef* hadc1,
//						TIM_HandleTypeDef* htim_rgb,
//						TIM_HandleTypeDef* htim_main_loop,
//						TIM_HandleTypeDef* htim_motors,
//						TIM_HandleTypeDef* htim_log,
//						TIM_HandleTypeDef* htim_parser,
//						UART_HandleTypeDef* huart_log, 
//						UART_HandleTypeDef* huart_ble,
//						I2C_HandleTypeDef* hi2c_imu)
//{
//	shared_state.hadc = hadc1;
//	
//	shared_state.htim_rgb = htim_rgb;
//	shared_state.htim_mLoop = htim_main_loop;
//	shared_state.htim_motors = htim_motors;
//	shared_state.htim_log = htim_log;
//	shared_state.htim_parser = htim_parser;
//	shared_state.huart_log = huart_log;
//	shared_state.huart_ble = huart_ble;
//	shared_state.hi2c_imu = hi2c_imu;
//	
//	shared_state.dma_ble_tx = huart_ble->hdmatx->DmaBaseAddress;
//	shared_state.dma_ble_rx = huart_ble->hdmarx->DmaBaseAddress;
//	
//	shared_state.dma_ble_ch_tx = huart_ble->hdmatx->Instance;
//	shared_state.dma_ble_ch_rx = huart_ble->hdmarx->Instance;
//	
//	memset(&shared_state.packet, 0, sizeof(struct OrbiPacket));
//	memset(&shared_state.setting, 0, sizeof(struct OrbiSetting));
//	shared_state.setting.packet_id = 255;
//	shared_state.sent_setting_id = 0;
//	
//	shared_state.systemIsRunning = true;
//	shared_state.user_disconnected = true;
//	shared_state.global_counter = 0;
//	
//	struct Point3D zeroPoint3D;
//	zeroPoint3D.x = zeroPoint3D.y = zeroPoint3D.z = 0;

//	shared_state.orientation = zeroPoint3D;
	
	
	shared_state.packet.velocity = 0;
	shared_state.packet.angle = 0;
	shared_state.packet.brake = 0;
	shared_state.global_counter = 0;
	shared_state.packet.calibrate = 0;
	shared_state.angle_offset = 0;
	shared_state.last_offset = 0;
	shared_state.battery = 0;
	shared_state.accel_x = 0;
	shared_state.accel_y = 0;
	shared_state.accel_z = 0;
	shared_state.linear_accel = 0;
	shared_state.current_left = 0;
	shared_state.current_right = 0;
	
	
	loadPIDConfs();
}
