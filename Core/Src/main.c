#include "stm32f1xx_hal.h"

#define MPU6050_ADDR 0xD0

#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75

// Mang luu giá tri mã hex cua LED 7 doan tu 0 den 9
const uint8_t segmentValues[10] = {
    0x3F,  // 0
    0x06,  // 1
    0x5B,  // 2
    0x4F,  // 3
    0x66,  // 4
    0x6D,  // 5
    0x7D,  // 6
    0x07,  // 7
    0x7F,  // 8
    0x6F   // 9
};

I2C_HandleTypeDef hi2c1;
int16_t Accel_X_RAW, Accel_Y_RAW, Accel_Z_RAW;
float Ax=0, Ay=0, Az=0;
int buoc = 0;
float AVM=0;
float S[10];
int Dem_ms = 0; // Moi ms se duoc cong 1, reset khi dat 1s = 1000ms
int STATE = 1; // 1: Active, 0: Off
uint32_t time_active = 0; // Don vi: giay
	
void SystemClock_Config(void);
static void GPIO_Init(void);
static void I2C1_Init(void);

void PrintLed(uint32_t num);
void MPU6050_init(void); //Initialize the MPU 
void MPU6050_Read_Accel (float *Ax, float *Ay, float *Az); //Read MPU Accelerator
void Dem_Buoc_Setup(void);
void Dem_Buoc(int* buoc);

void Error_Handler(void);

int main(void)
{
  HAL_Init();

  SystemClock_Config();

  GPIO_Init();
  I2C1_Init();
	MPU6050_init();

  Dem_Buoc_Setup();
	
//	GPIOA->ODR |= 0xC00;
	
  while (1) {
		if(STATE==1) {
			Dem_Buoc(&buoc);
		}
		PrintLed(buoc);
  }
}

void SystemClock_Config(void) {
	// Cau hình thanh ghi RCC de khoi tao các bo dao dong
  RCC->CR |= RCC_CR_HSION; // Bat tín hieu dao dong RCC_HSI
  while((RCC->CR & RCC_CR_HSIRDY) == 0); // Cho cho tín hieu RCC_HSI san sang
  
  // Cau hình thanh ghi RCC de chon RCC_HSI làm nguon clock
  RCC->CFGR &= ~RCC_CFGR_SW; // Xóa các bit SW de chon nguon clock
  RCC->CFGR |= RCC_CFGR_SW_HSI; // Chon RCC_HSI làm nguon clock
  while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI); // Cho cho RCC_HSI duoc chon làm nguon clock chính
  
  // Cau hình thanh ghi RCC de cau hình tan so bus
  RCC->CFGR &= ~RCC_CFGR_HPRE; // Xóa các bit HPRE de cau hình tan so bus
  RCC->CFGR |= RCC_CFGR_HPRE_DIV1; // Thiet lap tan so bus chia 1
  RCC->CFGR &= ~RCC_CFGR_PPRE1; // Xóa các bit PPRE1 de cau hình tan so bus APB1
  RCC->CFGR |= RCC_CFGR_PPRE1_DIV1; // Thiet lap tan so bus APB1 chia 1
  RCC->CFGR &= ~RCC_CFGR_PPRE2; // Xóa các bit PPRE2 de cau hình tan so bus APB2
  RCC->CFGR |= RCC_CFGR_PPRE2_DIV1; // Thiet lap tan so bus APB2 chia 1
}

static void I2C1_Init(void) {
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
	
  /* GPIO Ports Clock Enable */
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN; // Enable Clock cho GPIO Port A
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN; // Enable Clock cho GPIO Port B

  /*Thiet lap chân PA8 PA9 thành input */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
	
	// Xóa các bit cua chân PA0-PA7
	GPIOA->CRL &= ~(GPIO_CRL_MODE0 | GPIO_CRL_CNF0 |
									GPIO_CRL_MODE1 | GPIO_CRL_CNF1 |
									GPIO_CRL_MODE2 | GPIO_CRL_CNF2 |
									GPIO_CRL_MODE3 | GPIO_CRL_CNF3 |
									GPIO_CRL_MODE4 | GPIO_CRL_CNF4 |
									GPIO_CRL_MODE5 | GPIO_CRL_CNF5 |
									GPIO_CRL_MODE6 | GPIO_CRL_CNF6 |
									GPIO_CRL_MODE7 | GPIO_CRL_CNF7);
	
	// Thiet lap chân PA0-PA7 thành che do output 50MHz
	GPIOA->CRL |= GPIO_CRL_MODE0_0 | GPIO_CRL_MODE1_0 |
								GPIO_CRL_MODE2_0 | GPIO_CRL_MODE3_0 |
								GPIO_CRL_MODE4_0 | GPIO_CRL_MODE5_0 |
								GPIO_CRL_MODE6_0 | GPIO_CRL_MODE7_0;
								
	// Xóa các bit cua chân PA10-PA11
	GPIOB->CRH &= ~(GPIO_CRH_MODE10 | GPIO_CRH_CNF10 |
									GPIO_CRH_MODE11 | GPIO_CRH_CNF11);
	
	// Thiet lap chân PA10-PA11 thành che do output 50MHz
	GPIOB->CRH |= GPIO_CRH_MODE10_0 | GPIO_CRH_MODE11_0;
								
	// Xóa các bit cua chân PB0-PB1-PB4-PB5
	GPIOB->CRL &= ~(GPIO_CRL_MODE0 | GPIO_CRL_CNF0 |
									GPIO_CRL_MODE1 | GPIO_CRL_CNF1 |
									GPIO_CRL_MODE5 | GPIO_CRL_CNF5 |
									GPIO_CRL_MODE4 | GPIO_CRL_CNF4);
									
	// Thiet lap chân PB3-PB6 thành che do output 50MHz
	GPIOB->CRL |= GPIO_CRL_MODE0_0 |GPIO_CRL_MODE1_0 |
								GPIO_CRL_MODE4_0 |GPIO_CRL_MODE5_0;	
}

void PrintLed(uint32_t number) {
	// Lay cac chu so hang nghin, hang tram, hang chuc và hang don vi
        uint8_t thousands = number / 1000;
        uint8_t hundreds = (number / 100) % 10;
        uint8_t tens = (number / 10) % 10;
        uint8_t units = number % 10;

        // Hien thi cac chu so lên LED 7 doan tuong ung
				GPIOA->ODR = (GPIOA->ODR & 0xFFFFFF00);
				GPIOB->ODR = (GPIOB->ODR & 0xFFFFFF00);
				GPIOB->ODR |= 1<<0;
        GPIOA->ODR |= ~segmentValues[thousands];
				HAL_Delay(2);
	
				GPIOA->ODR = (GPIOA->ODR & 0xFFFFFF00);
				GPIOB->ODR = (GPIOB->ODR & 0xFFFFFF00);
				GPIOB->ODR |= 1<<1;
        GPIOA->ODR |= ~segmentValues[hundreds];
				HAL_Delay(2);
	
				GPIOA->ODR = (GPIOA->ODR & 0xFFFFFF00);
				GPIOB->ODR = (GPIOB->ODR & 0xFFFFFF00);
				GPIOB->ODR |= 1<<4;
        GPIOA->ODR |= ~segmentValues[tens];
				HAL_Delay(2);
	
				GPIOA->ODR = (GPIOA->ODR & 0xFFFFFF00);
				GPIOB->ODR = (GPIOB->ODR & 0xFFFFFF00);
				GPIOB->ODR |= 1<<5;
        GPIOA->ODR |= ~segmentValues[units];
				HAL_Delay(2);
}

void MPU6050_init(void) {
	uint8_t check,data;
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1 , 1000);
	if (check == 104)
	{ 
		//Cai dat Power management register de kich hoat cam bien
		data = 0;
		HAL_I2C_Mem_Write(&hi2c1,MPU6050_ADDR, PWR_MGMT_1_REG, 1, &data, 1, 1000);
		//Cai SMPRT_DIV register de hoat dong voi toc do 1KHz
		data = 0x07;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &data, 1, 1000);
		//Cai che do full scale
		data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &data, 1, 1000);
		
		data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &data, 1, 1000);
	}
}

void MPU6050_Read_Accel (float* Ax, float* Ay, float* Az) {
	uint8_t Rec_Data[6];
	
	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);
	
	//Adding 2 BYTES into 16 bit integer 
	Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);
	
	*Ax = Accel_X_RAW;
	*Ay = Accel_Y_RAW;
	*Az = Accel_Z_RAW;
	
	if(Accel_X_RAW < 0) {
		*Ax = -Accel_X_RAW;
	} else {
		*Ax = Accel_X_RAW;
	}
	
	if(Accel_Y_RAW < 0)  {
		*Ay = -Accel_Y_RAW;
	} else {
		*Ay = Accel_Y_RAW;
	}
	
	if(Accel_Z_RAW < 0)  {
		*Az = -Accel_Z_RAW;
	} else {
		*Az = Accel_Z_RAW;
	}
}

void Dem_Buoc_Setup() {
	//Ghi 10 gia tri cho S[10]
	for(int i=0; i<10; i++) {
		MPU6050_Read_Accel(&Ax, &Ay, &Az);
		S[i] = (Ax + Ay + Az);
		AVM += S[i];
	}
	AVM = AVM/10;
}

void Dem_Buoc(int* buoc) {
	MPU6050_Read_Accel(&Ax, &Ay, &Az);
	
	//Cap nhat gia tri moi cho mang S
	for(int i=0; i<9; i++) {
		S[i] = S[i+1];
	}
	S[9] = (Ax + Ay + Az);
	
	//Tinh AVG moi
	for(int i=0; i<10; i++) {
		AVM += S[i];
	}
	AVM = AVM/10;
	
	//Dem buoc chan
	if(AVM > 37000) {
		if (S[9] > AVM && S[8] < AVM) {
			(*buoc)++;
		}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
 if (GPIO_Pin == GPIO_PIN_8) {
	 // Chuyen trang thai
	 if (STATE == 0) {
		 STATE = 1;
		 //Tat den vang
		 GPIOB->ODR = (GPIOB->ODR & 0xFFFFFBFF);
	 }else {
		 STATE = 0;
		 //Bat den vang PB10, Tat den xanh PB11
		 GPIOB->ODR = (GPIOB->ODR & 0xFFFFFBFF);
		 GPIOB->ODR |= 0x400;
		 GPIOB->ODR = (GPIOB->ODR & 0xFFFFF7FF);
	 }	 
 }
 else if (GPIO_Pin == GPIO_PIN_9) {
	 //Reset
	 buoc=0;
	 time_active=0;
 }
}

void SysTick_Handler(void) {
  uwTick += uwTickFreq;
	
	Dem_ms++;
	if(Dem_ms == 1000) {
		Dem_ms =0;
		time_active++;
		if (STATE == 1) {
			//Nhap nhay den xanh PB11
			if ((GPIOB->ODR & 0x800) != 0) {
				GPIOB->ODR = (GPIOB->ODR & 0xFFFFF7FF);
			}else {
				GPIOB->ODR = (GPIOB->ODR & 0xFFFFF7FF);
				GPIOB->ODR |= 0x800;
			}
		}
	}
}

void Error_Handler(void) {
  __disable_irq();
  while (1) {
		PrintLed(8888);
  }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {
}
#endif
