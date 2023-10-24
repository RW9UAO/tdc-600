
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2023 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "fatfs.h"

/* USER CODE BEGIN Includes */
#include "sd.h"
#include "xconvert.h"
#include "oem6x8.h"
/* USER CODE END Includes */

//#define NO_SRAM_CHIP


/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;

TIM_HandleTypeDef htim1;
/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

volatile FRESULT res;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
uint8_t answerdata[0x200];
uint8_t RQM = 0x80, DIO = 0, EXM = 0, answer[10];
int have_cmd, cmd, cmd_param_cnt, cmd_answer_cnt, data_answer_cnt, dataready, needdata;
int Headside, drive, cylinder, cylindernum, HeadAddr, record, number, EndOfTrack, GPL, DTL, wait_c, need_zero;
extern uint16_t SPIDATAtoFDC, SPIDATAfromFDC;
extern int spihavedata, format;
static FIL handleA, handleB;

char fnames[64][13];		// 8.3 filenames, no long name support + 1 zero byte
const char FATFSerrors[][16]={"OK", "LOW LEV ERR", "Assertion", "NOT READY", "NO FILE", "NO PATH", "INVALID NAME", "DENIED", "EXIST", "INVALID OBJ", "WRITE PROT",
	"INVALID DRIVE","NOT ENABLED","NO FAT volume","MKFS ABORTED","TIMEOUT","LOCKED SHARE","NO BUFF","to many opened","INVALID PARAM"};

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
uint16_t SPITXRX(uint16_t data){
	uint32_t temp;

	HAL_GPIO_WritePin( GPIOA, GPIO_PIN_4, GPIO_PIN_RESET );		// SPI_CS
	SPI1->DR = data;
	while (SPI1->SR & SPI_SR_BSY);
	HAL_GPIO_WritePin( GPIOA, GPIO_PIN_4, GPIO_PIN_SET );		// SPI_CS
	
	SPIDATAfromFDC = SPI1->DR;		
	temp = SPI1->SR;
	return temp;
}

//=============================================================
void OSDEnable( void ){
// disable PA7 as GPIO
	//GPIOA->CRL &= ~( GPIO_CRL_CNF7 | GPIO_CRL_MODE7 ); // input analog
	// enable PB5 as SPI output
	GPIOB->CRL &= ~( GPIO_CRL_CNF5 | GPIO_CRL_MODE5 );
	GPIOB->CRL |= ( GPIO_CRL_CNF5_1 | GPIO_CRL_MODE5 );	// AF PP & hi speed
	// ENABLE: Remap     (NSS/PA15, SCK/PB3, MISO/PB4, MOSI/PB5)
	__HAL_AFIO_REMAP_SPI1_ENABLE();
	// set 8 bit SPI mode
	SPI1->CR1 &= ~SPI_CR1_DFF;
	// set SPI_BAUDRATEPRESCALER_16
	SPI1->CR1 &= ~SPI_CR1_BR_Msk;
	//SPI1->CR1 |= SPI_BAUDRATEPRESCALER_16;
	SPI1->CR1 |= SPI_BAUDRATEPRESCALER_8;
	// disable FDC
	HAL_NVIC_DisableIRQ(EXTI3_IRQn);
	// enable video sync
	HAL_NVIC_SetPriority( EXTI1_IRQn, 0, 0 );
  HAL_NVIC_EnableIRQ( EXTI1_IRQn );
}
//=============================================================
void OSDDisable( int FDCenable ){
	// DISABLE: No remap (NSS/PA4,  SCK/PA5, MISO/PA6, MOSI/PA7)
	__HAL_AFIO_REMAP_SPI1_DISABLE();
	// set 16 bit SPI mode
	SPI1->CR1 |= SPI_CR1_DFF;
	// set SPI_BAUDRATEPRESCALER_4
	SPI1->CR1 &= ~SPI_CR1_BR_Msk;
	SPI1->CR1 |= SPI_BAUDRATEPRESCALER_4;
	//SPI1->CR1 |= SPI_BAUDRATEPRESCALER_16;
	// disable video sync
  HAL_NVIC_DisableIRQ( EXTI1_IRQn );
	if( ! FDCenable )return;
	// enable FDC
	HAL_NVIC_EnableIRQ(EXTI3_IRQn);
}//=============================================================
void OSDText( char *textA, char *textB, char *textC, char *textD ){
	int i, k = 0;
	char c;
	int pos, j;

	for(int i = 0; i < 512; i++){
		answerdata[i] = 0xFF;	
	}
	// top and bottom lines a clean
	for( k = 0; k < 4; k++){
		i = 0;
		do{
			switch( k ){
				case 0: c = textA[i]; break;
				case 1: c = textB[i]; break;
				case 2: c = textC[i]; break;
				case 3: c = textD[i]; break;
			}
			if( c > '%' ){
				if( c >= 'a') c -= 0x20;
				c = c - 0x30 + 11;
				pos = c * 8;
				for( j = (0 + k * 8); j < ( 8 + k * 8); j++ ){
					answerdata[ j * 16 + i ] = ~oem6x8[ pos + j - (0 + k * 8) ];
				}
			}
			i++;
		}while( c != 0 && i < 16 );
	}
/*
	i = 0;
	do{
		c = textA[i];
		if( c > '%' ){
			if( c >= 'a') c -= 0x20;
			c = c - 0x30 + 11;
			pos = c * 8;
			for( j = 2; j < 10; j++ ){
				answerdata[ j * 16 + i ] = ~oem6x8[ pos + j-2 ];
			}
		}
		i++;
	}while( c != 0 && i < 16 );
	i = 0;
	do{
		c = textB[i];
		if( c > '%' ){
			if( c >= 'a') c -= 0x20;
			c = c - 0x30 + 11;
			pos = c * 8;
			for( j = 11; j < 19; j++ ){
				answerdata[ j * 16 + i ] = ~oem6x8[ pos + j-11 ];
			}
		}
		i++;
	}while( c != 0 && i < 16 );
*/
}
//=============================================================
void ROMload( int filesize ){
	uint8_t buff[32];
	uint32_t addr = 0, temp;
	int block, b;
	int bytesread;
	
	addr = 0;
			HAL_GPIO_WritePin( GPIOA, GPIO_PIN_1, GPIO_PIN_SET );		// RAM_nWR
			HAL_GPIO_WritePin( GPIOA, GPIO_PIN_2, GPIO_PIN_SET );		// RAM_LOAD activate
			HAL_GPIO_WritePin( GPIOA, GPIO_PIN_4, GPIO_PIN_SET );		// SPI_CS
			
			for( block = 0; block < filesize/32; block++ ){
				// read block
				//res = f_read(&handleA, buff, sizeof(buff), (void *)&bytesread);
				res = f_read(&handleA, buff, 32, (UINT  *)&bytesread);
				if( res != FR_OK){
					int s = s;
				}			
				// send to spartan
				for( b = 0; b < 32; b++){

					HAL_GPIO_WritePin( GPIOA, GPIO_PIN_4, GPIO_PIN_RESET );		// SPI_CS

					// send addr
					temp = addr;
					SPI1->DR = temp >> 8;
/*
					// PATCH
					
					// assume disk unchanged for 2 seconds
					// motor off timer = 4 seconds
					if( addr == 0x753d - 0x4000 || addr == 0x7548 - 0x4000  || addr == 0x7e81 - 0x4000 ){
						buff[b] = 0x05;				// timer value
					}
					// wait for normal termination of seek/calibrate
					if( addr == 0x788E - 0x4000 ){
						buff[b] = 0x01;				// try
					}
*/

					// send data
					temp = addr << 8;
					temp = temp & 0xFF00;
					temp |= buff[b];


					while (!(SPI1->SR & SPI_SR_TXE));
					SPI1->DR = temp;
					while (SPI1->SR & SPI_SR_BSY);
					
					//__NOP();__NOP();__NOP();__NOP();
					HAL_GPIO_WritePin( GPIOA, GPIO_PIN_4, GPIO_PIN_SET );		// SPI_CS
					__NOP();__NOP();__NOP();__NOP();
					HAL_GPIO_WritePin( GPIOA, GPIO_PIN_1, GPIO_PIN_RESET );			// RAM_nWR		
					__NOP();//__NOP();__NOP();__NOP();
					HAL_GPIO_WritePin( GPIOA, GPIO_PIN_1, GPIO_PIN_SET );			// RAM_nWR
					
					addr++;
				}

			}
			HAL_GPIO_WritePin( GPIOA, GPIO_PIN_2, GPIO_PIN_RESET );		// RAM_LOAD remove
}
//=============================================================

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	int i;
	int bytesread;
	int position;
	char newfilename[11];
	FILINFO FileInfo;
	int dismount = 0;
	int filesize;
	int FilesInDir = 0, CurrentDisk = 0, DiskChange = 0;
	char textA[16], textB[16], textC[16], textD[16];
	char driveA[13], driveB[13];
	
	uint32_t time;
	int MenuDisk = 1;	// 0 - A, 1 - B
	int MenuType = 0; // 0 - DSK emu, 1 - ROM loader
	int MenuPos = 0;
	int MapperType = 0;
	
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
	
	MX_TIM1_Init();
	HAL_TIM_Base_Start( &htim1 );

// Disable all interrupts
	__HAL_SPI_DISABLE_IT( &hspi1, SPI_IT_RXNE | SPI_IT_TXE | SPI_IT_ERR );
	__HAL_SPI_ENABLE( &hspi1 );


	HAL_GPIO_WritePin( GPIOA, GPIO_PIN_2, GPIO_PIN_RESET );		// RAM_LOAD remove

		// 0x20xx - enable mapper mode
		//		00 - ASCII8
		//		01 - ASCII16
		//		10 - konami
		//		11 - konami SCC


//	if( MenuType == 0 ){		// floppy emu	
//		SPITXRX(0x0000 );	// no mapper
//		SPITXRX(0x0000 );	// no mapper
//		SPITXRX(0x0000 );	// no mapper
//		SPITXRX(0x0000 );	// no mapper
//		SPITXRX(0x0000 );	// no mapper
		//SPITXRX(0x0000 );	// no mapper
//	}else{
//		SPITXRX(0x2002 );SPITXRX(0x2002 );SPITXRX(0x2002 );SPITXRX(0x2002 );SPITXRX(0x2002 );SPITXRX(0x2002 );
		//SPITXRX(0x0000 );SPITXRX(0x0000 );SPITXRX(0x0000 );SPITXRX(0x0000 );SPITXRX(0x0000 );SPITXRX(0x0000 );
//	}

	
	//SPITXRX( 0x8000  | 0x80 | 0x20 );							// Data Register
	//SPITXRX( 0x8000 );							// Data Register NOT ready


 
				OSDEnable();
				sprintf( textA, "  %s", __DATE__ );
				OSDText( "  TDC-600", "", "  VERSION", textA );
				HAL_Delay( 3000 );
				OSDDisable( 0 );	

reboot:
	disk_initialize(USERFatFS.drv);
	res = f_mount( &USERFatFS, (TCHAR const*)USERPath, 0 );
	if( res != FR_OK ){
				OSDEnable();
				sprintf( textB, "%d", res );
				sprintf( textA, "%s", &FATFSerrors[res][0] );
				OSDText( "MOUNT", "DISK ERROR", textB, textA );
				HAL_Delay( 5000 );
				OSDDisable( 0 );	
		while(1);
	}
	
	
	// ---------------------------------------------
	// read file list
	DIR dj;         // Directory object
	FilesInDir = 0;
	if( MenuType == 0 ){		// floppy emu	
		res = f_findfirst(&dj, &FileInfo, "DSK", "*.DSK");
	}else{
		res = f_findfirst(&dj, &FileInfo, "ROM", "*.ROM");
	}
	if( res != FR_OK ){
					OSDEnable();
					sprintf( textB, "%d", res );
					sprintf( textA, "%s", &FATFSerrors[res][0] );
					OSDText( "findfirst", "DISK ERROR", textB, textA );
					HAL_Delay( 5000 );
					OSDDisable( 0 );	
	}	
	while( res == FR_OK && FileInfo.fname[0] ){				// Repeat while an item is found
			if( strlen( FileInfo.fname ) < 16 ){
				strcpy( &fnames[ FilesInDir++ ][0], FileInfo.fname );
			}
			res = f_findnext(&dj, &FileInfo);               // Search for next item
			if( FilesInDir == 64 )break;
	}
	FilesInDir--;
	f_closedir(&dj);

	if( FilesInDir == 63 || FilesInDir == 0 ){
			OSDEnable();
			if(FilesInDir == 63) 	sprintf( textA, "IMG > 64" );
			else 									sprintf( textA, "no IMG found" );
					OSDText( textA, "", "", "" );
					HAL_Delay( 5000 );
					OSDDisable( 0 );	
			while(1);
	}	
	
	if( MenuType == 0 ){		// floppy emu
		// ---------------------------------------------	
		// read BIOS to SRAM chip
		if( res == FR_OK ){
#ifndef NO_SRAM_CHIP
			res = f_open(&handleA, "TDC600.rom", FA_READ);
			//res = f_open(&handleA, "ROM\\testram.rom", FA_READ);
			//res = f_open(&handleA, "ROM\\XBasic.rom", FA_READ);
			filesize = handleA.fsize;
		if( res != FR_OK ){
					OSDEnable();
					sprintf( textB, "%d", res );
					sprintf( textA, "%s", &FATFSerrors[res][0] );
					OSDText( "TDC600.rom", "DISK ERROR", textB, textA );
					HAL_Delay( 5000 );
					OSDDisable( 0 );	
			while(1);
		}		
			if( res == FR_OK){
				ROMload( filesize );
				res = f_close( &handleA );
			}
#endif
		}

		
		// open default files
		sprintf( textA, "DSK\\%s", "MSXDOS.dsk" );
		res = f_open(&handleA, textA, FA_READ | FA_WRITE );
		if( res != FR_OK ){
					OSDEnable();
					sprintf( textB, "%d", res );
					sprintf( textA, "%s", &FATFSerrors[res][0] );
					OSDText( "A>", "DISK ERROR", textB, textA );
					HAL_Delay( 5000 );
					OSDDisable( 0 );	
		}	
		strcpy(driveA, "MSXDOS.dsk");

		sprintf( textA, "DSK\\%s", &fnames[ CurrentDisk ][0] );
		res = f_open(&handleB, textA, FA_READ | FA_WRITE);
		if( res != FR_OK ){
					OSDEnable();
					sprintf( textB, "%d", res );
					sprintf( textA, "%s", &FATFSerrors[res][0] );
					OSDText( "B>", "DISK ERROR", textB, textA );
					HAL_Delay( 5000 );
					OSDDisable( 0 );	
		}		

		SPITXRX(0x0000 );	// no mapper
		// enable FDC_CS
		HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(EXTI3_IRQn);
	}	
	
	/* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		//---------------------------------------------------------------
		// keys
		if( (GPIOC->IDR & GPIO_PIN_14) == (uint32_t)GPIO_PIN_RESET ){											// enter
			while( (GPIOC->IDR & GPIO_PIN_14) == (uint32_t)GPIO_PIN_RESET );			
			if( MenuType == 0 ){//-------
				if( MenuDisk == 0 ){
					MenuDisk = 1;
					sprintf( textA, "B>" );
				}else{ 
					MenuDisk = 0;
					sprintf( textA, "A>" );
				}
				SPITXRX( 0x8000 );							// Data Register NOT ready
				OSDText( "DRIVE CHANGE", textA, "", "" );
				OSDEnable();
				for( i = 0; i < 200; i++){
					HAL_Delay( 10 );
					if( (GPIOC->IDR & GPIO_PIN_15) == (uint32_t)GPIO_PIN_RESET )break;
					if( (GPIOC->IDR & GPIO_PIN_13) == (uint32_t)GPIO_PIN_RESET )break;
				}
				OSDDisable( 1 );
			}else{//-------
				MapperType++;
				if( MapperType == 4 ) MapperType = 1;
				sprintf( textB, "  %s", "NONE" );
				sprintf( textC, "  %s", "KONAMI" );
				sprintf( textD, "  %s", "KONAMI SCC" );
				switch( MapperType ){
						case 1:textB[0] = '>';break;
						case 2:textC[0] = '>';break;
						case 3:textD[0] = '>';break;
				}
				OSDText( "MAPPER:", textB, textC, textD );
				OSDEnable();
				for( i = 0; i < 200; i++){
					HAL_Delay( 10 );																													// debounce
					if( (GPIOC->IDR & GPIO_PIN_14) == (uint32_t)GPIO_PIN_RESET )break;
				}
				OSDDisable( 1 );
				switch( MapperType ){
						case 1:SPITXRX(0x0000 );HAL_Delay( 10 );SPITXRX(0x0000 );break;// no mapper. for 16kb games
						case 2:SPITXRX(0x0000 );HAL_Delay( 10 );SPITXRX(0x2002 );break;// konami
						case 3:SPITXRX(0x0000 );HAL_Delay( 10 );SPITXRX(0x2003 );break;// konamiSCC
				}
			}
		}
		//--------------------------
		if( (GPIOC->IDR & GPIO_PIN_13) == (uint32_t)GPIO_PIN_RESET ){										// down
			i = 0;
			while( (GPIOC->IDR & GPIO_PIN_13) == (uint32_t)GPIO_PIN_RESET ){				
				if( (GPIOC->IDR & GPIO_PIN_15) == (uint32_t)GPIO_PIN_RESET ){								// both up & down pressed
					i = 1;
				}
			};
			if( i == 1 ){
				OSDEnable();
				if( MenuType ){
					MenuType = 0;
					OSDText( "TDC-600 MODE", "", "", "" );
				}else{
					MenuType = 1;
					OSDText( "ROM", "MODE", "", "" );
					HAL_NVIC_DisableIRQ(EXTI3_IRQn);
				}
				HAL_Delay( 3000 );
				OSDDisable( 1 );

				f_close( &handleA );
				f_close( &handleB );		
				f_mount( 0, (TCHAR const*)USERPath, 0 );
				goto reboot;
			}else{
				if( CurrentDisk )CurrentDisk--;
				DiskChange = 1;
				HAL_Delay( 10 );
			}
		}
		//--------------------------
		if( (GPIOC->IDR & GPIO_PIN_15) == (uint32_t)GPIO_PIN_RESET ){										// up
			i = 0;
			while( (GPIOC->IDR & GPIO_PIN_15) == (uint32_t)GPIO_PIN_RESET ){
				if( (GPIOC->IDR & GPIO_PIN_13) == (uint32_t)GPIO_PIN_RESET ){								// both up & down pressed
					i = 1;
				}
			};
			if( i == 1 ){
				OSDEnable();
				if( MenuType ){
					MenuType = 0;
					OSDText( "TDC-600 MODE", "", "", "" );
				}else{
					MenuType = 1;
					OSDText( "ROM", "MODE", "", "" );
					HAL_NVIC_DisableIRQ(EXTI3_IRQn);
				}
				HAL_Delay( 3000 );
				OSDDisable( 1 );

				f_close( &handleA );
				f_close( &handleB );		
				f_mount( 0, (TCHAR const*)USERPath, 0 );
				goto reboot;
			}else{
				if( CurrentDisk < FilesInDir )CurrentDisk++;
				DiskChange = 1;
				HAL_Delay( 10 );
			}
		}
		//--------------------------
		if( DiskChange == 1 ){
			DiskChange = 2;
			if( MenuType == 0 ){
				SPITXRX( 0x8000 );							// Data Register NOT ready
			}
			if( (FilesInDir - CurrentDisk) < 3 ) 
					MenuPos = 3 - (FilesInDir - CurrentDisk);
			else MenuPos = 0;
			sprintf( textA, "  %s", &fnames[ CurrentDisk + 0 - MenuPos ][0] );
			sprintf( textB, "  %s", &fnames[ CurrentDisk + 1 - MenuPos ][0] );
			sprintf( textC, "  %s", &fnames[ CurrentDisk + 2 - MenuPos ][0] );
			sprintf( textD, "  %s", &fnames[ CurrentDisk + 3 - MenuPos ][0] );
			switch( MenuPos ){
					case 0:textA[0] = '>';break;
					case 1:textB[0] = '>';break;
					case 2:textC[0] = '>';break;
					case 3:textD[0] = '>';break;
			}
			OSDText( textA, textB, textC, textD );
			OSDEnable();
			time = HAL_GetTick() + 2000;
		}
		//--------------------------
		if( DiskChange == 2 && HAL_GetTick() > time ){
			DiskChange = 0;
			if( MenuType == 0 ){
			// check for duplicate open
				if( (MenuDisk == 0 && strcmp(driveB, &fnames[ CurrentDisk ][0]) == 0) ||
						(MenuDisk != 0 && strcmp(driveA, &fnames[ CurrentDisk ][0]) == 0) ){
		
					OSDText( "DUPLICATE", "DISK NOT", "ALLOWED", "" );
					HAL_Delay( 5000 );
				}else{
					if( MenuDisk == 0 )			res = f_close( &handleA );
					else										res = f_close( &handleB );
					if( res == FR_OK ){/*
					if( MenuDisk == 0 )		res = f_open(&handleA, &fnames[ CurrentDisk ][0], FA_READ | FA_WRITE);
					else									res = f_open(&handleB, &fnames[ CurrentDisk ][0], FA_READ | FA_WRITE);
					if( res != FR_OK ){
						sprintf( textB, "%d", res );
						OSDText( "OPEN", &fnames[ CurrentDisk ][0], "DISK ERROR", textB );
						HAL_Delay( 2000 );
					}
	*/			}else	{
						sprintf( textB, "%d", res );
						sprintf( textA, "%s", &FATFSerrors[res][0] );
						OSDText( "CLOSE", "DISK ERROR", textB, textA );
						HAL_Delay( 5000 );
					}
					sprintf( textA, "DSK\\%s", &fnames[ CurrentDisk ][0] );
					if( MenuDisk == 0 )		res = f_open(&handleA, textA, FA_READ | FA_WRITE);
					else									res = f_open(&handleB, textA, FA_READ | FA_WRITE);
					if( res != FR_OK ){
						sprintf( textB, "%d", res );
						sprintf( textA, "%s", &FATFSerrors[res][0] );
						OSDText( "OPEN", &fnames[ CurrentDisk ][0], "DISK ERROR", textA );
						HAL_Delay( 5000 );
					}else{
						if( MenuDisk == 0 )		strcpy(driveA, &fnames[ CurrentDisk ][0] );
						else									strcpy(driveB, &fnames[ CurrentDisk ][0] );
					}
				}
				OSDDisable( 1 );
			}else{//--------------------------------------------------------------------------------
				sprintf( textA, "ROM\\%s", &fnames[ CurrentDisk ][0] );
				res = f_open(&handleA, textA, FA_READ );
				if( res != FR_OK ){
						sprintf( textB, "%d", res );
						sprintf( textA, "%s", &FATFSerrors[res][0] );
						OSDText( "OPEN", &fnames[ CurrentDisk ][0], "DISK ERROR", textA );
						HAL_Delay( 5000 );
				}else{
					OSDDisable( 0 );
					ROMload( handleA.fsize );
					OSDEnable();
					res = f_close( &handleA );
						sprintf( textB, "%d", (int)handleA.fsize );
						OSDText( "load ok", &fnames[ CurrentDisk ][0], textB, "" );				
						HAL_Delay( 2000 );
				}
				OSDDisable( 0 );
				HAL_NVIC_DisableIRQ(EXTI3_IRQn);
				//SPITXRX(0x0000 );//SPITXRX(0x0000 );SPITXRX(0x0000 );SPITXRX(0x0000 );SPITXRX(0x0000 );SPITXRX(0x0000 );
				//SPITXRX(0x2002 );SPITXRX(0x2002 );SPITXRX(0x2002 );SPITXRX(0x2002 );SPITXRX(0x2002 );SPITXRX(0x2002 );

				switch( MapperType ){
						case 1:SPITXRX(0x0000 );HAL_Delay( 10 );SPITXRX(0x0000 );break;// no mapper. for 16kb games
						case 2:SPITXRX(0x0000 );HAL_Delay( 10 );SPITXRX(0x2002 );break;// konami
						case 3:SPITXRX(0x0000 );HAL_Delay( 10 );SPITXRX(0x2003 );break;// konamiSCC
				}
			}


			
		}
		//---------------------------------------------------------------
		// READ
		if( needdata == 1 ) {
			needdata = 0;
			position =  (((cylindernum * 2 + HeadAddr) * 9 ) + (record - 1) ) * 512;
			if( drive == 0 ){
				res = f_lseek(&handleA, position );
				res += f_read(&handleA, answerdata, sizeof(answerdata), (void *)&bytesread);
			}else{
				res = f_lseek(&handleB, position );
				res += f_read(&handleB, answerdata, sizeof(answerdata), (void *)&bytesread);
			}
			data_answer_cnt = 512;
			if( res != FR_OK )answer[5] = 0x20; 																							// ST1, data error

			if( res != FR_OK ){
				SPITXRX( 0x8000 );							// Data Register NOT ready
				OSDEnable();
				sprintf( textB, "%d", res );
				sprintf( textA, "%s", &FATFSerrors[res][0] );
				OSDText( "READ", "DISK ERROR", textB, textA );
				HAL_Delay( 5000 );
				OSDDisable( 1 );
			}

			dataready = 1;
		}else
		//---------------------------------------------------------------
		// WRITE
		if( needdata == 2 ) {
			needdata = 0;
			position =  (((cylindernum * 2 + HeadAddr) * 9 ) + (record - 1) ) * 512;
			if( drive == 0 ){
				res = f_lseek( &handleA, position );
				if(res == FR_OK) res = f_write( &handleA, answerdata, 512, (void *)&bytesread);
//				if(res == FR_OK) f_sync( &handleA );
			}else{
				sprintf( textA, "seek" );
				res = f_lseek( &handleB, position );
				if(res == FR_OK){
					sprintf( textA, "write" );
					res = f_write( &handleB, answerdata, 512, (void *)&bytesread);
				}
//				if(res == FR_OK && bytesread == 512 ){ 
//					sprintf( textA, "sync" );
//					res = f_sync( &handleB );
//				}
			}
			if( res != FR_OK )answer[5] = 0x20; 																							// ST1, data error

			if( res != FR_OK ){
				SPITXRX( 0x8000 );							// Data Register NOT ready
				OSDEnable();
				sprintf( textB, "%d %d %d %d", res, cylindernum, HeadAddr, (record - 1) );
				sprintf( textC, "%s", &FATFSerrors[res][0] );
				OSDText( textA, "DISK ERROR", textB, textC );
				HAL_Delay( 5000 );
				OSDDisable( 1 );
			}

			dataready = 1;
		}
		//---------------------------------------------------------------
		// FORMAT
		if( needdata == 3 ) {
			if( format == 2 ){
				needdata = 0;
				dataready = 3;
			}else{
				// close current file
				if( drive == 0 ){
					res = f_close( &handleA );
				}else{
					res = f_close( &handleB );
				}
				// make new file name
				i = 0;
				do{
					sprintf(newfilename, "DISK%02i.DSK", i);
					res = f_stat(newfilename, &FileInfo);
					if( res != FR_OK ){
						break;
					}
					i++;
				}while( i < 100 );
				if( res != FR_OK ){
					if( drive == 0 ){
						res = f_open(&handleA, newfilename, FA_READ | FA_WRITE | FA_CREATE_NEW );
					}else{
						res = f_open(&handleB, newfilename, FA_READ | FA_WRITE | FA_CREATE_NEW );
					}				
					if(res == FR_OK ){
/*						for(i = 0; i < 512; i++)answerdata[i] = 0xE5;										// sector fill byte
						for(i = 0; i < 1440; i++){
							if( drive == 0 ){
								res = f_write(&handleA, answerdata, 512, (void *)&bytesread);
							}else{
								res = f_write(&handleB, answerdata, 512, (void *)&bytesread);
							}
							if( res != FR_OK ){
								answer[5] = 0x20;
								break;
							}
						}*/
						format = 2;
					}else{
						// error open new file
						answer[5] = 0x20; 																							// ST1, data error
					}
				}else{
					// ooooops all names busy
					answer[5] = 0x20; 																							// ST1, data error
				}
				needdata = 0;
				dataready = 3;
			}
		}
  
	
	
		if( dismount == 0xABCD ){
			f_close( &handleA );
			f_close( &handleB );		
			f_mount( 0, (TCHAR const*)USERPath, 0 );
		}

  }
//==========================================================================================================================
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
	//hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	//hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;				// CPOL
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	//hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;						// CPHA
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
	//hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
	//hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  HAL_SPI_Init(&hspi1);

	//SPI1->CR1 |= SPI_CR1_CPOL | SPI_CR1_CPHA;
}

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  //hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  HAL_SPI_Init(&hspi2);

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // buttons
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	// SPI2_CS
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Pin = GPIO_PIN_12;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	// V_EN
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	HAL_GPIO_WritePin( GPIOB, GPIO_PIN_0, GPIO_PIN_SET );		// mixer to MSX RGB

	// RAM_nWR, RAM_LOAD, SPI_CS
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_4;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	//nFDC_CS
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	EXTI->IMR |= (1<<3);

	// video sync PB1
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	EXTI->IMR |= (1<<1);


  //GPIO_InitStruct.Pin = GPIO_PIN_5;
  //GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	//GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	//HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	//HAL_GPIO_WritePin( GPIOB, GPIO_PIN_5, GPIO_PIN_SET );
}

static void MX_TIM1_Init(void){

	 // Peripheral clock enable
   __HAL_RCC_TIM1_CLK_ENABLE();

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 63;																						// for 64MHz
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xFFFF;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_Base_Init(&htim1);
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig);
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
