/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "main.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "math.h"
#include "fatfs.h"
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
} GPIO_PortPin;

#define VOLUP (GPIO_PortPin){GPIOB, GPIO_PIN_5}
#define VOLDOWN (GPIO_PortPin){GPIOC, GPIO_PIN_2}
#define PAUSE_START (GPIO_PortPin){GPIOB, GPIO_PIN_7}
#define NEXT (GPIO_PortPin){GPIOC, GPIO_PIN_1}
#define PREV (GPIO_PortPin){GPIOB, GPIO_PIN_6}

#define VOLTEST (GPIO_PortPin){GPIOC, GPIO_PIN_0}

uint8_t MUSIC_NUM = 5; // 曲数
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac_ch1;

I2C_HandleTypeDef hi2c2;
DMA_HandleTypeDef hdma_i2c2_tx;


SD_HandleTypeDef hsd;
DMA_HandleTypeDef hdma_sdio;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
void display_default_screen(int,int, int);
uint32_t readADC(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define SAMPLE_RATE 44100
#define TABLE_SIZE  2048
#define BUF_SIZE 2048


// FATFS 構造体
FATFS fs;
FRESULT res;

int8_t music_volume = 64; // 0-64
uint8_t is_playing = 0;
uint8_t is_finished_playing = 0;
int music_idx = 0;



typedef struct {
    char riff_id[4];     // "RIFF"
    uint32_t riff_size;
    char wave_id[4];     // "WAVE"
    char fmt_id[4];      // "fmt "
    uint32_t fmt_size;   // 16 for PCM
    uint16_t audio_format; // 1 = PCM
    uint16_t num_channels;
    uint32_t sample_rate;
    uint32_t byte_rate;
    uint16_t block_align;
    uint16_t bits_per_sample;
    char data_id[4];     // "data"
    uint32_t data_size;
} WAV_Header;

WAV_Header wavHeader;
FIL file;
// WAVファイルを開いてヘッダを読む
int WAV_Open(const char *filename) {
    unsigned int br;

    res = f_open(&file, filename, FA_READ);
    if (res != FR_OK) {
        printf("File open error: %d\r\n",res);
        return -1;
    }

    // ヘッダ読み込み
    printf("sizeof(WAV_Header)=%d\r\n", sizeof(WAV_Header));
    res = f_read(&file, &wavHeader, sizeof(WAV_Header), &br);
    if (res != FR_OK || br != sizeof(WAV_Header)) {
        printf("Header read error\r\n");
        f_close(&file);
        return -1;
    }

    if (strncmp(wavHeader.riff_id, "RIFF", 4) != 0 ||
        strncmp(wavHeader.wave_id, "WAVE", 4) != 0) {
        printf("Not a WAV file\r\n");
        f_close(&file);
        return -1;
    }

    printf("Channels: %d, SampleRate: %lu, Bits: %d\r\n",
           wavHeader.num_channels,
           wavHeader.sample_rate,
           wavHeader.bits_per_sample);

    return 0;
}

// WAVデータを順次 buf に読み込む
int WAV_ReadChunk(uint8_t* buf) {
    unsigned int br;
    res = f_read(&file, buf, BUF_SIZE*2, &br);
    if (res != FR_OK) {
        printf("Read error: %d\r\n", res);
        return -1;
    }
    if (br == 0) {
        // EOF
        printf("End of file reached\r\n");
        return 1;
    }
    return 0; // 正常
}

void Convert16to12(uint8_t *src, uint16_t *dst, uint32_t len_bytes) 
{
    int16_t *s16 = (int16_t*)src;
    uint32_t samples = len_bytes / 2;
    for (uint32_t i = 0; i < samples; i++) {
        dst[i] = (uint16_t)(((s16[i]+32768) * music_volume / 64 / 16/4)); // 16bit→12bit
        //dst[i] = (uint16_t)(((s16[i]* music_volume / 1024)) + 2048); // 16bit→12bit
    }
}

void dac_dma_init(void) {
    // DAC init
    hdac.Instance = DAC;
    HAL_DAC_Init(&hdac);

    DAC_ChannelConfTypeDef sConfig = {0};
    sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;    // Timer6 trigger
    sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
    HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1);

    // Timer6 for sample rate
    htim6.Instance = TIM6;
    htim6.Init.Prescaler = 0;
    htim6.Init.Period    = 170;
    HAL_TIM_Base_Init(&htim6);

    TIM_MasterConfigTypeDef sMasterConfig = {0};
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig);
}

#define DOUBLE_BUFFER 0
#if DOUBLE_BUFFER
uint16_t bufA[BUF_SIZE];
uint16_t bufB[BUF_SIZE];
static volatile int current_buf = 0; // 0: bufA, 1: bufB
uint8_t sd_read_buf[BUF_SIZE * 2]; // 16bitデータ用
void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef* hdac) {
  uint8_t res;
  if (current_buf == 0) {
    // bufA再生完了 → bufBを再生開始
    HAL_DAC_Start_DMA(hdac, DAC_CHANNEL_1,
                      (uint32_t*)bufB, BUF_SIZE, DAC_ALIGN_12B_R);
    current_buf = 1;
    // ここで bufA にSDから次のデータを読み込み
    res = WAV_ReadChunk(sd_read_buf);
    Convert16to12(sd_read_buf, bufA, BUF_SIZE * 2);
  } else {
    // bufB再生完了 → bufAを再生開始
    HAL_DAC_Start_DMA(hdac, DAC_CHANNEL_1,
                      (uint32_t*)bufA, BUF_SIZE, DAC_ALIGN_12B_R);
    current_buf = 0;
    // ここで bufB にSDから次のデータを読み込み
    res = WAV_ReadChunk(sd_read_buf);
    Convert16to12(sd_read_buf, bufB, BUF_SIZE * 2);
  }
  if(res) {
    // エラーまたはEOF
    //printf("WAV_ReadChunk error or EOF\r\n");
    // 再生停止
    HAL_DAC_Stop_DMA(hdac, DAC_CHANNEL_1);
    // WAVファイルを閉じる
    f_close(&file);
    is_finished_playing = 1;
    is_playing = 0;
  }
    
  //printf("HAL_DAC_ConvCpltCallbackCh1\r\n");
}
#else // CIRCULAR BUFFER

#define DAC_BUF_SIZE 4096
uint16_t dac_buffer[DAC_BUF_SIZE];  // 出力バッファ

#define WAV_CHUNK_SIZE (DAC_BUF_SIZE / 2)
uint8_t wav_temp_buf[WAV_CHUNK_SIZE * 2]; // 16bit PCM用一時バッファ

void FillWave(uint32_t len, uint32_t offset) {
    uint8_t res;
    // WAVデータを一時バッファに読み込む
    res = WAV_ReadChunk(wav_temp_buf);
    // 16bit PCM → 12bit DAC用データに変換
    Convert16to12(wav_temp_buf, &dac_buffer[offset], len * 2);
    if(res) {
      // エラーまたはEOF
      HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
      f_close(&file);
      is_finished_playing = 1;
      is_playing = 0;
    }
}

void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef* hdac)
{
  FillWave(DAC_BUF_SIZE/2, 0);
  //printf("Half\r\n");
}

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef* hdac)
{
    if (HAL_DAC_Start_DMA(hdac, DAC_CHANNEL_1,
                      (uint32_t*)dac_buffer, DAC_BUF_SIZE,
                      DAC_ALIGN_12B_R) != HAL_OK) {
      printf("DAC Start DMA Error\r\n");
    }
    FillWave(DAC_BUF_SIZE/2, DAC_BUF_SIZE/2);
    //printf("Full\r\n");
}
#endif

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
  #if 1 //Initializations
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C2_Init();   // I2C初期化
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_DAC_Init();
  MX_TIM6_Init();

  __HAL_RCC_TIM6_CLK_ENABLE();

  uint8_t switch_in = 0;
  uint8_t last_switch_in = 0;
  uint8_t rising_edge_switches = 0;
  uint8_t falling_edge_switches = 0;
  unsigned int vol_last_debounce_time = 0;
  unsigned int batt_last_time = 0;
  uint8_t display_isupdated = 0;
  uint8_t music_idx_isupdated = 0;
  


  // SSD1306 初期化
  //HAL_Delay(3000);
  printf("Initing SSD1306...\n");
  ssd1306_Init();
  printf("ssd1306 Inited\n");
  HAL_Delay(100);

  printf("Initing SDIO,FATFS...\n");
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  printf("SDIO,FATFS Inited\n");
    

  // 画面をクリア
  display_default_screen(music_idx+1, music_volume, 15);

  #if 1     //------------------------!! SDカードテスト ------------------------
  #define MAX_FILES 20
  char fileList[MAX_FILES][13];  // ファイル名格納用
  uint16_t fileCount = 0;
#if 1
  if(HAL_SD_Init(&hsd) != HAL_OK){
      printf("SD Init failed, Err=0x%08lX\r\n", hsd.ErrorCode);
      while(1);
  }
  //printf("SD card detected!\r\n");
  //HAL_Delay(1000);
  //printf("Checking SD card state...\r\n");

  HAL_SD_CardStateTypeDef state;
  state = HAL_SD_GetCardState(&hsd);

  if (state == HAL_SD_CARD_TRANSFER) {
      printf("Card is ready for transfer!\r\n");
  } else {
      printf("Card not ready, state=%ld\r\n", state);
  }
#endif

  if (HAL_SD_ConfigWideBusOperation(&hsd, SDIO_BUS_WIDE_4B) != HAL_OK) {
      printf("SD ConfigWideBusOperation failed, Err=0x%08lX\r\n", hsd.ErrorCode);
      Error_Handler();
  }
  //printf("SD bus width set to 4 bits.\r\n");


  // マウント
  res = f_mount(&fs, "", 1);
  if(res != FR_OK){
      printf("f_mount failed: %d\r\n", res);
      while(1);
  }
  // ディレクトリ構造用構造体
  DIR dir;
  FILINFO fno;

  // ルートディレクトリオープン
  res = f_opendir(&dir, "/");
  if(res != FR_OK){
      printf("f_opendir failed: %d\r\n", res);
      while(1);
  }

  printf("Files in root:\r\n");
  MUSIC_NUM = 0;
  for(;;){
      res = f_readdir(&dir, &fno);
      if(res != FR_OK || fno.fname[0] == 0) break; // すべて読み終わった
      if(!(fno.fattrib & AM_DIR)){ // ディレクトリは無視
          printf("  %s\r\n", fno.fname);
          strncpy(fileList[fileCount], fno.fname, 12);
          fileList[fileCount][12] = '\0';
          MUSIC_NUM++;
          if (fileCount >= MAX_FILES) break;
      }
  }
  MUSIC_NUM = 32;

  printf("Done\r\n");
  #endif

  printf("Initializing DAC and Timer...\r\n");
  dac_dma_init();

  printf("DAC and Timer initialized.\r\n");

  if(HAL_TIM_Base_Start(&htim6) != HAL_OK) {
    printf("Timer start Error\r\n");
  }
  printf("Timer started.\r\n");
  
  #endif //Initializations
  
  #if DOUBLE_BUFFER
  current_buf = 0;
  #endif
  is_playing = 0;
  is_finished_playing = 1;
  uint32_t adcVal = readADC();
  int16_t vol_meter = (adcVal - 1850) / 9;

  HAL_Delay(100);



  while (1) {
    last_switch_in = switch_in;
    switch_in = 0;
    display_isupdated = 0;
    music_idx_isupdated = 0;
    rising_edge_switches = 0;
    falling_edge_switches = 0;

    if(!HAL_GPIO_ReadPin(PAUSE_START.port, PAUSE_START.pin)) {
      switch_in |= 1<<2;
    }
    if(!HAL_GPIO_ReadPin(VOLUP.port, VOLUP.pin)) {
      switch_in |= 1<<0;
    }
    if(!HAL_GPIO_ReadPin(VOLDOWN.port, VOLDOWN.pin)) {
      switch_in |= 1<<1;
    }
    if(!HAL_GPIO_ReadPin(NEXT.port, NEXT.pin)) {
      switch_in |= 1<<3;
    }
    if(!HAL_GPIO_ReadPin(PREV.port, PREV.pin)) {
      switch_in |= 1<<4;
    }

    if(switch_in & ( (1<<0) | (1<<1) )) {
      // VOLUP,VOLDOWNのどちらかが押されている
      switch_in &= ~((1<<3) | (1<<4)); // NEXT,PREVは無視
    }

    for(int i=0; i<5; i++) {
      if(switch_in & (1<<i)) {
        if(!(last_switch_in & (1<<i))){   //このフレームで押された
          rising_edge_switches |= (1<<i);
        }
      }else{
        if(last_switch_in & (1<<i)) {    //このフレームで離された
          falling_edge_switches |= (1<<i);
        }
      }
    }
    
    if(rising_edge_switches & (1<<0)) {  // VOLUP
      printf("VOLUP\r\n");
      display_isupdated = 1;
      music_volume = (music_volume + 1);
      if(music_volume > 64) music_volume = 64;
      vol_last_debounce_time = HAL_GetTick();
    }else if(rising_edge_switches & (1<<1)) {  // VOLDOWN
      printf("VOLDOWN\r\n");
      display_isupdated = 1;
      music_volume = (music_volume - 1);
      if(music_volume < 0) music_volume = 0;
      vol_last_debounce_time = HAL_GetTick();
    }else if(rising_edge_switches & (1<<2)) {  // PAUSE/START
      printf("PAUSE/START\r\n");
      if(is_playing) {
        // 再生中 → 停止
        HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
        is_playing = 0;
        printf("music paused\r\n");
      } else {
        // 停止中 → 再生
        if(is_finished_playing) {
          // 最初の再生または再生終了後の再生
          char filename[13];
          sprintf(filename, "%d.wav", music_idx+1);
          WAV_Open(filename);
          #if DOUBLE_BUFFER
          current_buf = 0;
          printf("current_buf reseted\r\n");
          // 最初の2バッファ分を読み込み済み
          res = WAV_ReadChunk(sd_read_buf);
          Convert16to12(sd_read_buf, bufA, BUF_SIZE * 2);
          res = WAV_ReadChunk(sd_read_buf);
          Convert16to12(sd_read_buf, bufB, BUF_SIZE * 2);

          printf("DAC started\r\n");
          if (HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1,
                            (uint32_t*)bufA, BUF_SIZE,
                            DAC_ALIGN_12B_R) != HAL_OK) {
            printf("DAC Start DMA Error\r\n");
          }
          #else // CIRCULAR BUFFER
          // 最初の2バッファ分を読み込み
          FillWave(DAC_BUF_SIZE/2, 0);
          FillWave(DAC_BUF_SIZE/2, DAC_BUF_SIZE/2);
          printf("DAC started\r\n");
          if (HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1,
                            (uint32_t*)dac_buffer, DAC_BUF_SIZE,
                            DAC_ALIGN_12B_R) != HAL_OK) {
            printf("DAC Start DMA Error\r\n");
          }


          #endif
          is_finished_playing = 0;
        }else{
          // 再生途中で停止 → 再開
          #if DOUBLE_BUFFER
          HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1,
                            (uint32_t*)(current_buf==0 ? bufA : bufB),
                            BUF_SIZE, DAC_ALIGN_12B_R);
          #else // CIRCULAR BUFFER
          HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1,
                            (uint32_t*)dac_buffer, DAC_BUF_SIZE,
                            DAC_ALIGN_12B_R);
          #endif
        }
        
        is_playing = 1;
      }
      display_isupdated = 1;
    }else if(rising_edge_switches & (1<<3)) {  // NEXT
      printf("NEXT\r\n");
      display_isupdated = 1;
      music_idx = (music_idx + 1) % MUSIC_NUM;
      music_idx_isupdated = 1;
    }else if(rising_edge_switches & (1<<4)) {  // PREV
      printf("PREV\r\n");
      display_isupdated = 1;
      music_idx = (music_idx - 1 + MUSIC_NUM) % MUSIC_NUM;
      music_idx_isupdated = 1;
    }

    if(music_idx_isupdated) {
      if(is_playing) {
        // 再生中 → 再生曲変更
        HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
        f_close(&file);
        is_playing = 0;
        is_finished_playing = 1;
        printf("music stopped for changing track\r\n");
      }
      char filename[13];
      sprintf(filename, "%d.wav", music_idx+1);
      printf("Changing track to %s\r\n", filename);
      WAV_Open(filename);
      #if DOUBLE_BUFFER
      current_buf = 0;
      printf("current_buf reseted\r\n");
      // 最初の2バッファ分を読み込み
      res = WAV_ReadChunk(sd_read_buf);
      Convert16to12(sd_read_buf, bufA, BUF_SIZE * 2);
      res = WAV_ReadChunk(sd_read_buf);
      Convert16to12(sd_read_buf, bufB, BUF_SIZE * 2);
      is_finished_playing = 0;
      // 再生開始
      printf("DAC started\r\n");
      if (HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1,
                        (uint32_t*)bufA, BUF_SIZE,
                        DAC_ALIGN_12B_R) != HAL_OK) {
        printf("DAC Start DMA Error\r\n");
      }
      #else // CIRCULAR BUFFER
      // 最初の2バッファ分を読み込み
      FillWave(DAC_BUF_SIZE/2, 0);
      FillWave( DAC_BUF_SIZE/2, DAC_BUF_SIZE/2);
      is_finished_playing = 0;
      // 再生開始
      printf("DAC started\r\n");
      if (HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1,
                        (uint32_t*)dac_buffer, DAC_BUF_SIZE,
                        DAC_ALIGN_12B_R) != HAL_OK) {
        printf("DAC Start DMA Error\r\n");
      }
      #endif
      is_playing = 1;
    }

    if(HAL_GetTick() - vol_last_debounce_time > 400) {
      if(switch_in & (1<<0)){        // VOLUP
        vol_last_debounce_time = HAL_GetTick()-350;
        display_isupdated = 1;
        music_volume = (music_volume + 1);
        if(music_volume > 63) music_volume = 63;
      }else if(switch_in & (1<<1)){  // VOLDOWN
        vol_last_debounce_time = HAL_GetTick()-350;
        display_isupdated = 1;
        music_volume = (music_volume - 1);
        if(music_volume < 0) music_volume = 0;
      }
    }

    if(HAL_GetTick() - batt_last_time > 500) {
      adcVal = readADC();
      vol_meter*=0.8;
      vol_meter+= (adcVal - 1850) / 9/5;
      batt_last_time = HAL_GetTick();
      display_isupdated = 1;
    }
    
    //printf("switch_in: %d\r\n", switch_in);
    if(display_isupdated) {
      //uint16_t voltage = (3300 * adcVal) / 4095; // Vref=3.3Vの場合

      display_default_screen(music_idx+1, music_volume, vol_meter);
    }

  }
}

uint32_t readADC(void)
{
    uint32_t val = 0;
    
    // 変換開始
    HAL_ADC_Start(&hadc1);
    
    // 変換完了待ち
    if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK)
    {
        val = HAL_ADC_GetValue(&hadc1);  // 12bitなら0〜4095
    }
    
    // 停止
    HAL_ADC_Stop(&hadc1);
    
    return val;
}

void display_default_screen(int midx, int vol, int batt_voltage) {
  char a[3] = "00";
  a[0] = '0'+(midx/10);
  a[1] = '0'+(midx%10);
  if(!disp_ready) return;

  ssd1306_Fill(Black);
  // 番号
  ssd1306_SetCursor(32, 16);
  ssd1306_WriteString(a, Font_32x48, White);

  //電池残量
  ssd1306_FillRectangle(0,0,31,11,White);
  ssd1306_FillRectangle(30,1,batt_voltage,10,Black);
  ssd1306_FillRectangle(28,0,31,2,Black);
  ssd1306_FillRectangle(28,11,31,9,Black);
  ssd1306_Line(31,2,28,2,White);
  ssd1306_Line(28,0,28,2,White);
  ssd1306_Line(31,9,28,9,White);
  ssd1306_Line(28,9,27,11,White);

  // 音量
  ssd1306_Line(127, 63, 127, 0, White);
  //ssd1306_Line(127, 64, 127-8, 0, White);
  ssd1306_Line(127-16, 0, 127, 0, White);
  for(int i=0; i<vol; i++) {
    ssd1306_Line(126, 63-i, 126-(i/4), 63-i, White);
  }



  #if 1
  //ssd1306_SetCursor(32,0);
  //char buf[8];
  //sprintf(buf, "%d", batt_voltage);   // buf = "2417"
  //ssd1306_WriteString(buf, Font_7x10, White);
  #endif

  ssd1306_UpdateScreen();
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}


/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */

  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;

  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  //__HAL_LINKDMA(&hdac, DMA_Handle1, hdma_dac_ch1);
  /* USER CODE END DAC_Init 2 */

}
/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 500000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */
  __HAL_LINKDMA(&hi2c2, hdmatx, hdma_i2c2_tx);

  HAL_NVIC_SetPriority(I2C2_EV_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(I2C2_EV_IRQn);

  HAL_NVIC_SetPriority(I2C2_ER_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(I2C2_ER_IRQn);

  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);


  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 5;
  #if 0
  if (HAL_SD_Init(&hsd) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_SD_ConfigWideBusOperation(&hsd, SDIO_BUS_WIDE_4B) != HAL_OK)
  {
    Error_Handler();
  }
  #endif
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

}


/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
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
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA2_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel3_IRQn);
  #if 1
  /* DMA2_Channel4_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel4_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel4_5_IRQn);

  hdma_dac_ch1.Instance = DMA2_Channel3;
  hdma_dac_ch1.Init.Direction           = DMA_MEMORY_TO_PERIPH;
  hdma_dac_ch1.Init.PeriphInc           = DMA_PINC_DISABLE;
  hdma_dac_ch1.Init.MemInc              = DMA_MINC_ENABLE;
  hdma_dac_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD; // DACは12bit → HALFWORD
  hdma_dac_ch1.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
  hdma_dac_ch1.Init.Mode                = DMA_CIRCULAR;            // 連続再生用
  hdma_dac_ch1.Init.Priority            = DMA_PRIORITY_HIGH;
  HAL_DMA_Init(&hdma_dac_ch1);

  #endif
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pins : PC1 PC2 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  
  /*Configure GPIO pins : PB5 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  #if 0
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  #endif

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}


/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 170;      //48000Hz
  //htim6.Init.Period = 180;      //44100Hz
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/* USER CODE BEGIN 4 */
int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart1,(uint8_t *)ptr,len,10);
  return len;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
