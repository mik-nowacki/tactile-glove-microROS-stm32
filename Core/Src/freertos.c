/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "adc.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"
#include <math.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/float64_multi_array.h>
#include <std_msgs/msg/multi_array_dimension.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define SELECT_PINS_PORT GPIOE
#define MUX1 0
#define MUX2 1
#define MUX3 2
#define NUMBER_OF_MULTIPLEXERS 3
#define NUMBER_OF_SENSORS 19
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

const int MUX_SELECT_PINS[NUMBER_OF_MULTIPLEXERS][3] = {{M1_S0_Pin, M1_S1_Pin, M1_S2_Pin}, {M2_S0_Pin, M2_S1_Pin, M2_S2_Pin}, {M3_S0_Pin, M3_S1_Pin, M3_S2_Pin}};  // S0-S2 pins
const int MULTIPLEXER_SENSORS[NUMBER_OF_MULTIPLEXERS] = {6, 6 ,7};  // Number of sensors connected to each multiplexer
const int MAX_RESOLUTION_VALUE = 4095;  // 12-bit ADC resolution
const int RESISTOR_47 = 4700;  // 4.7 kOhm resistor for smaller sensors
const int RESISTOR_22 = 2200;  // 2.2 kOhm resistor for bigger sensors
const double VOLT_IN = 2.95;  // STM32 supply voltage

// Linearization of resitance functions
const double A_BIG_RES = 2.5610009396757896;
const double B_BIG_RES = 1.022786463208377;

const double A_SMALL_RES = 0.13601158321717574;
const double B_SMALL_RES = 1.7434005445070226;

std_msgs__msg__Float64MultiArray msg;  //  Message sent via micro-ROS

double voltageAOC = 0.0;
double current = 0.0;
double sensorResistance = 0.0;
double conductance = 0.0;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 3500 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);

void switchMuxPin(int mux, int pin)
{
	switch (pin){
	case 0:
		HAL_GPIO_WritePin(SELECT_PINS_PORT, MUX_SELECT_PINS[mux][0], GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SELECT_PINS_PORT, MUX_SELECT_PINS[mux][1], GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SELECT_PINS_PORT, MUX_SELECT_PINS[mux][2], GPIO_PIN_RESET);
		break;
	case 1:
		HAL_GPIO_WritePin(SELECT_PINS_PORT, MUX_SELECT_PINS[mux][0], GPIO_PIN_SET);
		HAL_GPIO_WritePin(SELECT_PINS_PORT, MUX_SELECT_PINS[mux][1], GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SELECT_PINS_PORT, MUX_SELECT_PINS[mux][2], GPIO_PIN_RESET);
		break;
	case 2:
		HAL_GPIO_WritePin(SELECT_PINS_PORT, MUX_SELECT_PINS[mux][0], GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SELECT_PINS_PORT, MUX_SELECT_PINS[mux][1], GPIO_PIN_SET);
		HAL_GPIO_WritePin(SELECT_PINS_PORT, MUX_SELECT_PINS[mux][2], GPIO_PIN_RESET);
		break;
	case 3:
		HAL_GPIO_WritePin(SELECT_PINS_PORT, MUX_SELECT_PINS[mux][0], GPIO_PIN_SET);
		HAL_GPIO_WritePin(SELECT_PINS_PORT, MUX_SELECT_PINS[mux][1], GPIO_PIN_SET);
		HAL_GPIO_WritePin(SELECT_PINS_PORT, MUX_SELECT_PINS[mux][2], GPIO_PIN_RESET);
		break;
	case 4:
		HAL_GPIO_WritePin(SELECT_PINS_PORT, MUX_SELECT_PINS[mux][0], GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SELECT_PINS_PORT, MUX_SELECT_PINS[mux][1], GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SELECT_PINS_PORT, MUX_SELECT_PINS[mux][2], GPIO_PIN_SET);
		break;
	case 5:
		HAL_GPIO_WritePin(SELECT_PINS_PORT, MUX_SELECT_PINS[mux][0], GPIO_PIN_SET);
		HAL_GPIO_WritePin(SELECT_PINS_PORT, MUX_SELECT_PINS[mux][1], GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SELECT_PINS_PORT, MUX_SELECT_PINS[mux][2], GPIO_PIN_SET);
		break;
	case 6:
		HAL_GPIO_WritePin(SELECT_PINS_PORT, MUX_SELECT_PINS[mux][0], GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SELECT_PINS_PORT, MUX_SELECT_PINS[mux][1], GPIO_PIN_SET);
		HAL_GPIO_WritePin(SELECT_PINS_PORT, MUX_SELECT_PINS[mux][2], GPIO_PIN_SET);
		break;
	case 7:
		HAL_GPIO_WritePin(SELECT_PINS_PORT, MUX_SELECT_PINS[mux][0], GPIO_PIN_SET);
		HAL_GPIO_WritePin(SELECT_PINS_PORT, MUX_SELECT_PINS[mux][1], GPIO_PIN_SET);
		HAL_GPIO_WritePin(SELECT_PINS_PORT, MUX_SELECT_PINS[mux][2], GPIO_PIN_SET);
		break;
	}

}


uint16_t readM1Analog()
{
	uint16_t analogRead = 0;

	HAL_ADC_Start(&hadc3);

	if(HAL_ADC_PollForConversion(&hadc3, 10) == HAL_OK)
	  {
	  analogRead = HAL_ADC_GetValue(&hadc3);
	  }

	HAL_ADC_Stop(&hadc3);

	return analogRead;
}


uint16_t readM2Analog()
{
	uint16_t analogRead = 0;

	HAL_ADC_Start(&hadc2);

	if(HAL_ADC_PollForConversion(&hadc2, 10) == HAL_OK)
	  {
		analogRead = HAL_ADC_GetValue(&hadc2);
	  }

	HAL_ADC_Stop(&hadc2);

	return analogRead;
}


uint16_t readM3Analog()
{
	uint16_t analogRead = 0;

	HAL_ADC_Start(&hadc1);

	if(HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
	  {
		analogRead = HAL_ADC_GetValue(&hadc1);
	  }

	HAL_ADC_Stop(&hadc1);

	return analogRead;
}


double calculateForce(uint16_t _analogRead, int _resistor)
{
//  convert analogRead to Volt
	voltageAOC = (double)_analogRead * (VOLT_IN / MAX_RESOLUTION_VALUE);  // 500mV max value
	current = voltageAOC / _resistor;
	sensorResistance = (VOLT_IN / current) - _resistor;
	conductance = 1000000 / sensorResistance;

	if (_resistor == RESISTOR_22) return ((A_BIG_RES * pow(conductance, B_BIG_RES)) / 1000) * 9.81;
	else return ((A_SMALL_RES * pow(conductance, B_SMALL_RES)) / 1000) * 9.81;
}


void readMultiplexerInput(int mux)
{
	for (int pin = 0; pin < MULTIPLEXER_SENSORS[mux]; pin++) // Starting from 1 because of the circuit board layout
	{
      switchMuxPin(mux, pin+1); // Y1 is the first pin used on the multiplexer
	  switch(mux)
	  {  // Read ADC pin corresponding to the given multiplexer
	  case MUX1:
		  msg.data.data[pin] = calculateForce(readM1Analog(), RESISTOR_47); break;
	  case MUX2:
		  msg.data.data[pin + MUX2 * MULTIPLEXER_SENSORS[MUX1]] = calculateForce(readM2Analog(), RESISTOR_47); break;
	  case MUX3:
		  msg.data.data[pin + MUX3 * MULTIPLEXER_SENSORS[MUX2]] = calculateForce(readM3Analog(), RESISTOR_22); break;
	  }
	}
}

void readGloveSignals()
{
	// Iterate through all multiplexers
	for (int selected_mux = MUX1; selected_mux < NUMBER_OF_MULTIPLEXERS; selected_mux++)
	{
		readMultiplexerInput(selected_mux);
	}
}

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */

  // micro-ROS configuration

  rmw_uros_set_custom_transport(
    true,
    (void *) &huart2,
    cubemx_transport_open,
    cubemx_transport_close,
    cubemx_transport_write,
    cubemx_transport_read);

  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
  freeRTOS_allocator.allocate = microros_allocate;
  freeRTOS_allocator.deallocate = microros_deallocate;
  freeRTOS_allocator.reallocate = microros_reallocate;
  freeRTOS_allocator.zero_allocate =  microros_zero_allocate;

  if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
      printf("Error on default allocators (line %d)\n", __LINE__);
  }

  // create init_options
  rcl_allocator_t allocator;
  allocator = rcl_get_default_allocator();
  rclc_support_t support;
  rclc_support_init(&support, 0, NULL, &allocator);

  // create node
  rcl_node_t node;
  rclc_node_init_default(&node, "microros_node", "", &support);

  // create publisher
  rcl_publisher_t publisher;
  rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
    "glove_data");

//  create empty buffer for sensor values
  double buffer[NUMBER_OF_SENSORS] = {};
  msg.data.data = buffer;
  msg.data.size = NUMBER_OF_SENSORS;
  msg.data.capacity = NUMBER_OF_SENSORS;

  std_msgs__msg__MultiArrayDimension dim[NUMBER_OF_SENSORS] = {};
  msg.layout.dim.data = dim;
  msg.layout.dim.size = 0;
  msg.layout.dim.capacity = NUMBER_OF_SENSORS;

  for(;;)
  {
    if (rcl_publish(&publisher, &msg, NULL) != RCL_RET_OK)
    {
      printf("Error publishing (line %d)\n", __LINE__);
    }

    readGloveSignals();

    osDelay(50);
  }
  /* USER CODE END 5 */
}
/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

