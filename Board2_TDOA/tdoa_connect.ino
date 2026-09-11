#include <Arduino.h>
#include <math.h>
#include "driver/i2s.h"
#include "arduinoFFT.h"
#include "driver/rtc_io.h"

// 웨이크업용 택트 스위치 핀 (A5)
#define BUTTON_PIN 12

// ===== 하드웨어 핀 정의 (I2S, 긱블 나노 기준) =====
#define I2S0_WS   D2
#define I2S0_SCK  D3
#define I2S0_SD   D4

#define I2S1_WS   D6
#define I2S1_SCK  D7
#define I2S1_SD   D5

// ===== UART 통신 핀 정의 (보드 간 연결, 긱블 나노 맞춤형) =====
#define UART_TX   TX
#define UART_RX   RX

// 긱블 나노 S3 빌트인 LED 핀
#define LED_PIN   48

// ===== 오디오 및 TDOA 상수 =====
#define FS                  16000.0f  
#define SAMPLES_PER_READ    1024      
#define DISTANCE_MICS_M     0.25f     
#define SOUND_SPEED_MPS     343.0f    
// 💡 만약 너무 작은 소리에도 반응해서 각도가 튄다면 이 값을 키우고(예: 1.0e-5f), 
// 소리를 잘 못 잡으면 이 값을 줄여주세요(예: 1.0e-6f).
#define ENERGY_THRESH       5.0e-6f   
#define CORR_QUALITY_THRESH 1.8f      
#define FILTER_SIZE         8         
#define USE_BANDPASS        true
#define MIN_FREQ            200.0f
#define MAX_FREQ            5000.0f

int32_t raw_buf_i2s0[SAMPLES_PER_READ * 2]; 
int32_t raw_buf_i2s1[SAMPLES_PER_READ * 2]; 

float buf_E[SAMPLES_PER_READ], buf_W[SAMPLES_PER_READ]; 
float buf_S[SAMPLES_PER_READ], buf_N[SAMPLES_PER_READ]; 

float vReal1[SAMPLES_PER_READ], vImag1[SAMPLES_PER_READ];
float vReal2[SAMPLES_PER_READ], vImag2[SAMPLES_PER_READ];

ArduinoFFT FFT1 = ArduinoFFT(vReal1, vImag1, SAMPLES_PER_READ, FS);
ArduinoFFT FFT2 = ArduinoFFT(vReal2, vImag2, SAMPLES_PER_READ, FS);

float x_delay_history[FILTER_SIZE] = {0};
float y_delay_history[FILTER_SIZE] = {0};
int filter_idx = 0;

float calculate_energy(const float* buffer, int len) {
  double sum = 0.0;
  for (int i = 0; i < len; ++i) sum += (double)buffer[i] * buffer[i];
  return (float)(sum / len);
}

int compute_gcc_phat_delay(float* sig1, float* sig2) {
  for (int i = 0; i < SAMPLES_PER_READ; i++) {
    vReal1[i] = sig1[i]; vImag1[i] = 0;
    vReal2[i] = sig2[i]; vImag2[i] = 0;
  }
  FFT1.windowing(FFT_WIN_TYP_HANN, FFT_FORWARD);
  FFT1.compute(FFT_FORWARD);
  FFT2.windowing(FFT_WIN_TYP_HANN, FFT_FORWARD);
  FFT2.compute(FFT_FORWARD);

  if (USE_BANDPASS) {
    float freq_res = FS / SAMPLES_PER_READ;
    int min_bin = (int)(MIN_FREQ / freq_res);
    int max_bin = (int)(MAX_FREQ / freq_res);
    for (int i = 1; i < (SAMPLES_PER_READ / 2); i++) {
      if (i < min_bin || i > max_bin) {
        vReal1[i] = 0; vImag1[i] = 0; vReal2[i] = 0; vImag2[i] = 0;
        vReal1[SAMPLES_PER_READ - i] = 0; vImag1[SAMPLES_PER_READ - i] = 0;
        vReal2[SAMPLES_PER_READ - i] = 0; vImag2[SAMPLES_PER_READ - i] = 0;
      }
    }
  }

  for (int i = 0; i < SAMPLES_PER_READ; i++) {
    float r = vReal1[i] * vReal2[i] + vImag1[i] * vImag2[i];
    float im = vImag1[i] * vReal2[i] - vReal1[i] * vImag2[i];
    float mag = sqrtf(r * r + im * im);
    if (mag > 1.0e-9f) {
      vReal1[i] = r / mag; vImag1[i] = im / mag;
    } else {
      vReal1[i] = 0; vImag1[i] = 0;
    }
  }
  FFT1.compute(FFT_REVERSE);

  float max_corr = -1000.0f;
  int delay_index = 0;
  double corr_sum = 0;

  for (int i = 0; i < SAMPLES_PER_READ; i++) {
    corr_sum += vReal1[i];
    if (vReal1[i] > max_corr) { max_corr = vReal1[i]; delay_index = i; }
  }

  float avg_corr = (float)(corr_sum / SAMPLES_PER_READ);
  if (max_corr < avg_corr * CORR_QUALITY_THRESH) return 9999; 

  if (delay_index >= SAMPLES_PER_READ / 2) delay_index -= SAMPLES_PER_READ;
  return delay_index;
}

void setup() {
  Serial.begin(115200); // 디버깅용 PC 연결
  Serial1.begin(115200, SERIAL_8N1, UART_RX, UART_TX); // 보드 간 통신용 설정
  
  // LED 설정 및 켜기 (깨어있음을 표시)
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  
  // 버튼 핀 설정
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  
  // 웨이크업 시 누르고 있던 버튼에서 손을 뗄 때까지 대기 
  // (루프 진입 후 곧바로 다시 딥슬립에 빠지는 현상 방지)
  while(digitalRead(BUTTON_PIN) == LOW) { delay(10); }
  delay(50);

  i2s_config_t i2s_cfg = {};
  i2s_cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX);
  i2s_cfg.sample_rate = (uint32_t)FS;
  i2s_cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT;
  i2s_cfg.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
  i2s_cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  i2s_cfg.intr_alloc_flags = 0;
  i2s_cfg.dma_buf_count = 8;
  i2s_cfg.dma_buf_len = SAMPLES_PER_READ;
  i2s_cfg.use_apll = false;

  i2s_pin_config_t pins0 = { .bck_io_num = I2S0_SCK, .ws_io_num = I2S0_WS, .data_out_num = I2S_PIN_NO_CHANGE, .data_in_num = I2S0_SD };
  i2s_driver_install(I2S_NUM_0, &i2s_cfg, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pins0);
  i2s_zero_dma_buffer(I2S_NUM_0);

  i2s_pin_config_t pins1 = { .bck_io_num = I2S1_SCK, .ws_io_num = I2S1_WS, .data_out_num = I2S_PIN_NO_CHANGE, .data_in_num = I2S1_SD };
  i2s_driver_install(I2S_NUM_1, &i2s_cfg, 0, NULL);
  i2s_set_pin(I2S_NUM_1, &pins1);
  i2s_zero_dma_buffer(I2S_NUM_1);

  Serial.println("보드 1: TDOA 방향 추정 시작...");
}

void loop() {
  // 물리적 버튼을 눌렀을 때 딥슬립 진입 (main.cpp와 동일한 기능)
  if (digitalRead(BUTTON_PIN) == LOW) {
    delay(50); // 디바운스
    if (digitalRead(BUTTON_PIN) == LOW) {
      Serial.println("🌙 물리적 버튼 입력으로 딥슬립에 진입합니다...");
      digitalWrite(LED_PIN, LOW); // 수면 상태 표시 (LED OFF)
      
      i2s_driver_uninstall(I2S_NUM_0);
      i2s_driver_uninstall(I2S_NUM_1);
      
      while(digitalRead(BUTTON_PIN) == LOW) { delay(10); } // 버튼에서 손을 뗄 때까지 대기
      delay(50);
      
      rtc_gpio_pullup_en((gpio_num_t)BUTTON_PIN);
      rtc_gpio_pulldown_dis((gpio_num_t)BUTTON_PIN);
      esp_sleep_enable_ext0_wakeup((gpio_num_t)BUTTON_PIN, 0);
      esp_deep_sleep_start();
    }
  }
  // 💡 메인 보드(main.cpp)로부터 UART(Serial1) 수신 대기 (딥슬립 동기화)
  if (Serial1.available() > 0) {
    String inputData = Serial1.readStringUntil('\n');
    inputData.trim();
    if (inputData == "SLEEP") {
      Serial.println("🌙 메인 보드의 요청으로 딥슬립에 진입합니다...");
      
      // LED 끄기 (수면 상태 표시)
      digitalWrite(LED_PIN, LOW);

      // 전력 소모를 최소화하기 위해 I2S 마이크 드라이버 해제
      i2s_driver_uninstall(I2S_NUM_0);
      i2s_driver_uninstall(I2S_NUM_1);
      
      // A5(GPIO 12) 핀으로 깨어나기 설정 (LOW 신호 감지 시 기상)
      // 메인 보드와 마찬가지로 플로팅 현상 방지를 위해 내부 풀업 켜기
      rtc_gpio_pullup_en((gpio_num_t)BUTTON_PIN);
      rtc_gpio_pulldown_dis((gpio_num_t)BUTTON_PIN);
      esp_sleep_enable_ext0_wakeup((gpio_num_t)BUTTON_PIN, 0);
      
      // 딥슬립 진입
      esp_deep_sleep_start();
    }
  }

  size_t bytes_read0 = 0, bytes_read1 = 0;
  i2s_read(I2S_NUM_0, raw_buf_i2s0, sizeof(raw_buf_i2s0), &bytes_read0, portMAX_DELAY);
  i2s_read(I2S_NUM_1, raw_buf_i2s1, sizeof(raw_buf_i2s1), &bytes_read1, portMAX_DELAY);

  for (int i = 0; i < SAMPLES_PER_READ; ++i) {
    // I2S 데이터 구조 (ESP32): [0] = Left 채널(GND), [1] = Right 채널(3.3V)
    // I2S0: 동쪽(East) = Right [1], 서쪽(West) = Left [0]
    buf_E[i] = (float)(raw_buf_i2s0[2 * i + 1] >> 8) / 8388608.0f; 
    buf_W[i] = (float)(raw_buf_i2s0[2 * i + 0] >> 8) / 8388608.0f; 
    // I2S1: 북쪽(North) = Right [1], 남쪽(South) = Left [0]
    buf_N[i] = (float)(raw_buf_i2s1[2 * i + 1] >> 8) / 8388608.0f; 
    buf_S[i] = (float)(raw_buf_i2s1[2 * i + 0] >> 8) / 8388608.0f; 
  }

  float eE = calculate_energy(buf_E, SAMPLES_PER_READ);
  float eW = calculate_energy(buf_W, SAMPLES_PER_READ);
  float eS = calculate_energy(buf_S, SAMPLES_PER_READ);
  float eN = calculate_energy(buf_N, SAMPLES_PER_READ);

  // 설정된 임계치(ENERGY_THRESH)보다 소리가 작으면 무시 (노이즈 필터링)
  if (eE < ENERGY_THRESH && eW < ENERGY_THRESH && eS < ENERGY_THRESH && eN < ENERGY_THRESH) return;

  // 수학적 오류 수정: GCC-PHAT의 IFFT 위상 특성상 순서를(W, E)로 넣어야 양수 딜레이가 올바른 방향을 가리킴
  int delay_x = compute_gcc_phat_delay(buf_W, buf_E); // 올바른 위상을 위한 W->E 크로스 스펙트럼
  int delay_y = compute_gcc_phat_delay(buf_S, buf_N); // 올바른 위상을 위한 S->N 크로스 스펙트럼

  int max_delay = (int)ceilf((DISTANCE_MICS_M / SOUND_SPEED_MPS) * FS);
  if (abs(delay_x) > max_delay + 2 || abs(delay_y) > max_delay + 2) return;

  x_delay_history[filter_idx] = (float)delay_x;
  y_delay_history[filter_idx] = (float)delay_y;
  filter_idx = (filter_idx + 1) % FILTER_SIZE;

  float avg_x = 0.0f, avg_y = 0.0f;
  for (int i = 0; i < FILTER_SIZE; i++) { avg_x += x_delay_history[i]; avg_y += y_delay_history[i]; }
  avg_x /= FILTER_SIZE; avg_y /= FILTER_SIZE;

  // 2차원 방위각(Azimuth Angle) 계산 (atan2)
  float angle_rad = atan2f(avg_y, avg_x);
  float math_deg = angle_rad * (180.0f / M_PI);

  // 나침반 방위각으로 변환 (북쪽: 0°, 동쪽: 90°, 남쪽: 180°, 서쪽: 270°)
  float angle_deg = 90.0f - math_deg;
  
  // 마이크 물리적 배치 보정: 배열이 시계방향(오른쪽)으로 45도 회전됨
  angle_deg += 45.0f;

  if (angle_deg < 0.0f) {
    angle_deg += 360.0f; // 0 ~ 359도 범위로 보정
  } else if (angle_deg >= 360.0f) {
    angle_deg -= 360.0f;
  }

  // 2번 보드(TinyML 보드)로 계산된 각도를 전송
  Serial1.println(angle_deg);
  
  // 시리얼 모니터 확인용 출력
  Serial.print("전송된 각도: "); Serial.println(angle_deg);
}