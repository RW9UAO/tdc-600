/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2023 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"
#include "stm32f1xx_it.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
}

/**
* @brief This function handles Memory management fault.
*/
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
  /* USER CODE BEGIN MemoryManagement_IRQn 1 */

  /* USER CODE END MemoryManagement_IRQn 1 */
}

/**
* @brief This function handles Prefetch fault, memory access fault.
*/
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
  /* USER CODE BEGIN BusFault_IRQn 1 */

  /* USER CODE END BusFault_IRQn 1 */
}

/**
* @brief This function handles Undefined instruction or illegal state.
*/
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
  /* USER CODE BEGIN UsageFault_IRQn 1 */

  /* USER CODE END UsageFault_IRQn 1 */
}

/**
* @brief This function handles System service call via SWI instruction.
*/
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
* @brief This function handles Debug monitor.
*/
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
* @brief This function handles Pendable request for system service.
*/
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/* USER CODE BEGIN 1 */
int line;
uint16_t SPIDATAtoFDC = 0x8080, SPIDATAfromFDC;
int spihavedata = 0;
extern uint8_t RQM, DIO, EXM, answer[10], answerdata[0x200];
extern int have_cmd, cmd, cmd_param_cnt, cmd_answer_cnt, data_answer_cnt, dataready, needdata;
extern int Headside, drive, cylinder, cylindernum, HeadAddr, record, number, EndOfTrack, GPL, DTL, wait_c, need_zero;
//extern uint8_t cmdbuf[20],cmdc;
int MSXtoFDC = 0, MSXtoFDCc = 0;
int format = 0;

#define LINE_START	15
#define LINE_END		LINE_START + 32

void SPITXRX(uint16_t data);

//=================================================================================================================================
void EXTI1_IRQHandler( void ){
	int i=0, p;
	if( EXTI->PR & GPIO_PIN_1 ){		
		__HAL_GPIO_EXTI_CLEAR_IT( GPIO_PIN_1 ); //clr int
		uint32_t synctime = TIM1->CNT;
		TIM1->CNT = 0xFFFF;
		if( synctime > 1000) line = 0;
		else line++;
		if( line > LINE_START && line < LINE_END ){
			// left space
			for(int g=0;g<100;g++)__NOP();
			p = line - LINE_START - 1;
			p = p * 16;
			GPIOB->BSRR = (uint32_t)GPIO_PIN_0 << 16u;		// mixer to OSD
#pragma push
#pragma O3
			do{
				SPI1->DR = answerdata[p];
				i++;
				p++;
				while (!(SPI1->SR & SPI_SR_TXE));				
			}while( i < 16 );
#pragma pop
			while (SPI1->SR & SPI_SR_BSY);
			GPIOB->BSRR = GPIO_PIN_0;		// mixer to MSX RGB
		}
	}
}
//=================================================================================================================================




void EXTI3_IRQHandler(void){

	if( EXTI->PR & GPIO_PIN_3 ){
		
				__HAL_GPIO_EXTI_CLEAR_IT( GPIO_PIN_3 ); //clr int

			HAL_GPIO_WritePin( GPIOA, GPIO_PIN_4, GPIO_PIN_RESET );		// SPI_CS
			SPI1->DR = SPIDATAtoFDC;
			while (SPI1->SR & SPI_SR_BSY);
			HAL_GPIO_WritePin( GPIOA, GPIO_PIN_4, GPIO_PIN_SET );		// SPI_CS
	
			uint32_t temp;
			SPIDATAfromFDC = SPI1->DR;		
			temp = SPI1->SR;
//		spihavedata++;

		
		
	if( (SPIDATAfromFDC & 0xFF00) == 0x8800 ){	// write LDOR
		SPIDATAtoFDC = 0x8080;
		have_cmd = 0;
		data_answer_cnt = 0;
		cmd_answer_cnt = 0;
		return;
	}else
	//------------------------------------------------------------------------------------------------
	if( (SPIDATAfromFDC & 0xFF00) == 0xF000 ){	// read DATA
		if( data_answer_cnt ){
				data_answer_cnt--;
				DIO = 0x40;
				if( data_answer_cnt == 0 ){
					EXM = 0;
					wait_c = 1;																									// on last byte of data we need send 0xC0 in MSR
				}
				SPIDATAtoFDC = 0x4000 | answerdata[ 511 - data_answer_cnt ];
			}else{
					if( wait_c == 1 ){
						wait_c = 2;
						SPIDATAtoFDC = 0x80C0;
						return;
					}
					if( cmd_answer_cnt ){
						cmd_answer_cnt--;
						SPIDATAtoFDC = 0x4000 | answer[ cmd_answer_cnt ];
						
						if( cmd_answer_cnt ){
							DIO = 0x40;
						}else{
							DIO = 0;
						}
					}else{
						SPIDATAtoFDC = 0x40AA;
					}
				}
		return;
	}else
	//------------------------------------------------------------------------------------------------
	if( (SPIDATAfromFDC & 0xFF00) == 0xD000 ){	// read MSR
		if( dataready ){
			dataready = 0;
			RQM = 0x80;
			if( cmd == 0x66 ){
				DIO = 0x40;
				EXM = 0x20;
				SPIDATAtoFDC = 0x4000 | answerdata[ 0 ]; data_answer_cnt--;
			}else
			if( cmd == 0x45 ){
				DIO = 0x40;
				wait_c = 0;
				cmd_answer_cnt--;																				
				SPIDATAtoFDC = 0x4000 | answer[ cmd_answer_cnt ];
				have_cmd = 0;																// bug
			}if( cmd == 0x4D ){
				DIO = 0x40;
				wait_c = 0;
				cmd_answer_cnt--;																				
				SPIDATAtoFDC = 0x4000 | answer[ cmd_answer_cnt ];
			}
		}else{
			if( wait_c == 2 ){
				wait_c = 0;
				cmd_answer_cnt--;																				
				SPIDATAtoFDC = 0x4000 | answer[ cmd_answer_cnt ];						
				return;
			}
			SPIDATAtoFDC = 0x8000 | RQM | DIO | EXM;
		}
		return;
	}else
	//------------------------------------------------------------------------------------------------
	if( (SPIDATAfromFDC & 0xFF00) == 0xE800 ){	// write DATA	

		if( MSXtoFDC ){
			answerdata[ MSXtoFDCc ] = SPIDATAfromFDC & 0xFF;
//			if( MSXtoFDCc < 512 )
			MSXtoFDCc++;
			if( MSXtoFDCc == 511 ){
					EXM = 0x00;
					DIO = 0x00;
					RQM = 0x00;																	// not ready, writing data
			}else
				if( MSXtoFDCc == 512 ){
					MSXtoFDC = 0;
					needdata = 2;																// write sector
					//cmd = 0x45;																	// bug
					//cmd_answer_cnt = 7;													// bug
					//have_cmd = 0;																// bug
				}else{
					SPIDATAtoFDC = 0x8000 | 0x80 | 0x20;
				}
				return;
		}

		if( have_cmd == 0 ){			
			cmd = SPIDATAfromFDC & 0xFF;
			data_answer_cnt = 0;
			cmd_answer_cnt = 0;
			DIO = 0x00;
//			if(cmdc<20){cmdbuf[cmdc++] = cmd;}//dbg
			switch( cmd ){
				case 0x03:// CMD - SPECIFY
					have_cmd = 1;
					cmd_param_cnt = 2;
					break;
				case 0x07:// CMD - RECALIBRATE
					have_cmd = 1;
					cmd_param_cnt = 1;
					break;
				case 0x08:// CMD - SENSE INTERRUPT STATUS
					cmd_answer_cnt = 2;
					// prepare data
					//if( drive == 0 )answer[1] = 0x20; else answer[1] = 0x08 | drive;		// ST0
					answer[1] = 0x20;
					answer[0] = cylinder;																								// PCN(PRESENT CYLINDER)
					SPITXRX( 0x4000 | answer[ 1 ] ); cmd_answer_cnt--;
					DIO = 0x40;
					SPIDATAtoFDC = 0x8000 | RQM | DIO | EXM;
					break;
				case 0x0F:// CMD - SEEK
					have_cmd = 1;
					cmd_param_cnt = 2;
					break;
				case 0x66:// CMD - READ DATA
					have_cmd = 1;
					cmd_param_cnt = 8;
					break;
				case 0x45:// CMD - WRITE DATA
					have_cmd = 1;
					cmd_param_cnt = 8;
					DIO = 0x00;
					//MSXtoFDC = 0; //bug
					//dataready = 0; //bug
					break;
				case 0x4D:	// CMD - FORMAT
					have_cmd = 1;
					cmd_param_cnt = 5;
					break;
				default:
					cmd = 0;
					have_cmd = 0;
					break;
			}
		}else{
			if( cmd_param_cnt ){
				cmd_param_cnt--;
				if( cmd_param_cnt == 0 ){
					have_cmd = 0;
				}
			
				switch( cmd ){
					case 0x03:// CMD - SPECIFY
						// parameters ignored
						break;
					case 0x07:// CMD - RECALIBRATE
						drive = SPIDATAfromFDC & 0xFF;
						break;
					case 0x0F:// CMD - SEEK
						if( cmd_param_cnt == 1 ){ drive = SPIDATAfromFDC & 0x03; Headside = (SPIDATAfromFDC & 0x04)>>2;}
						else cylinder = SPIDATAfromFDC & 0xFF;
						break;
					//-----------------------------------------------------------------------------------------------------------------------------
					case 0x66:// CMD - READ DATA
						if( cmd_param_cnt == 7 ) 			{ drive = SPIDATAfromFDC & 0x03; Headside = (SPIDATAfromFDC & 0x04)>>2;}	// HS, USx
						else if( cmd_param_cnt == 6 ) cylindernum = SPIDATAfromFDC & 0xFF;														// C - CYLINDER NUMBER
						else if( cmd_param_cnt == 5 ) HeadAddr 		= SPIDATAfromFDC & 0xFF;														// H - HEAD ADDRESS
						else if( cmd_param_cnt == 4 ) record 			= SPIDATAfromFDC & 0xFF;														// R - RECORD, stands for the sector number which will be read or written
						else if( cmd_param_cnt == 3 ) number 			= SPIDATAfromFDC & 0xFF;														// N - NUMBER
						else if( cmd_param_cnt == 2 ) EndOfTrack	= SPIDATAfromFDC & 0xFF;														// EOT
						else if( cmd_param_cnt == 1 ) {
							GPL					= SPIDATAfromFDC & 0xFF;
							SPIDATAtoFDC = 0x8000;
							RQM = 0x00;
						}else{// cmd_param_cnt == 0
							DTL	= SPIDATAfromFDC & 0xFF;																																// DATA LENGTH = 0xFF x 2(N) = 512
							
							SPITXRX( 0x8000 );							// Data Register NOT ready
							SPIDATAtoFDC = 0x8000;
							cmd_answer_cnt = 7;
							//if( drive == 0 )answer[6] = 0x20; else answer[6] = 0x08 | drive;	// ST0
							answer[6] = 0x20;																									// ST0
							answer[5] = 0x00; 																								// ST1
							answer[4] = 0x00;																									// ST2
							answer[3] = cylindernum;
							answer[2] = HeadAddr;
							answer[1] = record;
							answer[0] = number;

							EXM = 0x00;
							DIO = 0x00;
							RQM = 0x00;

							needdata = 1;
						}
						break;
					//-----------------------------------------------------------------------------------------------------------------------------
					case 0x45: // CMD - write data
						MSXtoFDC = 0; //bug
						if( cmd_param_cnt == 7 ) 			{ drive = SPIDATAfromFDC & 0x03; Headside = (SPIDATAfromFDC & 0x04)>>2;}	// HS, USx
						else if( cmd_param_cnt == 6 ) cylindernum = SPIDATAfromFDC & 0xFF;														// C - CYLINDER NUMBER
						else if( cmd_param_cnt == 5 ) HeadAddr 		= SPIDATAfromFDC & 0xFF;														// H - HEAD ADDRESS
						else if( cmd_param_cnt == 4 ) record 			= SPIDATAfromFDC & 0xFF;														// R - RECORD, stands for the sector number which will be read or written
						else if( cmd_param_cnt == 3 ) number 			= SPIDATAfromFDC & 0xFF;														// N - NUMBER
						else if( cmd_param_cnt == 2 ) EndOfTrack	= SPIDATAfromFDC & 0xFF;														// EOT
						else if( cmd_param_cnt == 1 ) {
							GPL					= SPIDATAfromFDC & 0xFF;
//							if( GPL == 0x45 ){
//								GPL = GPL;
								//cmd_param_cnt = 8;	// BUG!!!
//								return;
//							}
							SPIDATAtoFDC = 0x8000 | 0x80 | 0x20;
							EXM = 0x20;
							DIO = 0x00;
							RQM = 0x80;
						}else{// cmd_param_cnt == 0
							DTL	= SPIDATAfromFDC & 0xFF;																																// DATA LENGTH = 0xFF x 2(N) = 512
							//if( DTL == 0x45 ){
							//	cmd_param_cnt = 7;	// BUG!!!
							//	return;
							//}
							MSXtoFDC = 1;
							MSXtoFDCc = 0;

							cmd_answer_cnt = 7;
							//if( drive == 0 )answer[6] = 0x20; else answer[6] = 0x08 | drive;	// ST0
							answer[6] = 0x20;
							answer[5] = 0x00; 																								// ST1
							answer[4] = 0x00;																									// ST2
							answer[3] = cylindernum;
							answer[2] = HeadAddr;
							answer[1] = record;
							answer[0] = number;
							
							//if(cmdc<20){cmdbuf[cmdc++] = record;}//dbg
						}
						break;
					//-----------------------------------------------------------------------------------------------------------------------------
					case 0x4D:	// CMD - FORMAT
						if( cmd_param_cnt == 4 ) 			{ drive = SPIDATAfromFDC & 0x03; Headside = (SPIDATAfromFDC & 0x04)>>2;}	// HS, USx
						else if( cmd_param_cnt == 1 ){
							SPIDATAtoFDC = 0x8000 | 0x00 | 0x00;
							EXM = 0x00;
							DIO = 0x00;
							RQM = 0x00;
						}else if( cmd_param_cnt == 0 ){
							needdata = 3;
							cmd_answer_cnt = 7;
							//if( drive == 0 )answer[6] = 0x20; else answer[6] = 0x08 | drive;	// ST0
							answer[6] = 0x20;
							answer[5] = 0x00; 																								// ST1
							answer[4] = 0x00;																									// ST2
							answer[3] = 79;//cylindernum;
							answer[2] = 1;//HeadAddr;
							answer[1] = 9;//record;
							answer[0] = 255;//number;
						}
						break;
				}
			}
		}
	}
		
		
	}
}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
