#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "math.h"
#include "string.h"
#include "stdlib.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define DAC_PORT GPIOB   // Cổng GPIO xuất tín hiệu DAC0808

uint8_t sin_wave[256];      // Bảng tra cứu (LUT) sóng sin
uint8_t triangle_wave[256]; // Bảng tra cứu sóng tam giác
uint8_t square_wave[256];   // Bảng tra cứu sóng vuông

volatile uint8_t wave_index = 0;  // Chỉ số đọc LUT
volatile uint8_t *current_wave;   // Con trỏ trỏ đến dạng sóng hiện tại
volatile uint16_t wave_frequency = 2; // Tần số mặc định

void SystemClock_Config(void);
static void MX_GPIO_Init(void);

void init_GPIO(void);
void init_TIM2(void);
void update_TIM2(void);
void generate_waveforms(void);

/**
 * @brief Khởi tạo GPIOB làm output để xuất dữ liệu cho DAC0808
 */
void init_GPIO(void) {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIOB->MODER |= 0x5555; // PB0-PB7 là Output
}

/**
 * @brief Khởi tạo Timer2 để điều khiển tần số phát sóng
 */
void init_TIM2(void) {
    __HAL_RCC_TIM2_CLK_ENABLE();
    TIM2->PSC = (SystemCoreClock / (3* wave_frequency * 256)) - 1;
    TIM2->ARR = 3; //
    TIM2->DIER |= TIM_DIER_UIE; // Cho phép ngắt cập nhật
    TIM2->CR1 |= TIM_CR1_CEN;   // Kích hoạt Timer
    NVIC_EnableIRQ(TIM2_IRQn);  // Bật ngắt NVIC
}

/**
 * @brief Cập nhật Timer2 khi thay đổi tần số
 */
void update_TIM2(void) {
    TIM2->CR1 &= ~TIM_CR1_CEN; // Dừng Timer trước khi cập nhật
    TIM2->PSC = (SystemCoreClock / (3 * wave_frequency * 256)) - 1;
    TIM2->EGR |= TIM_EGR_UG; // Kích hoạt cập nhật ngay lập tức
    TIM2->CR1 |= TIM_CR1_CEN; // Chạy lại Timer
}

/**
 * @brief Tạo bảng tra cứu dạng sóng
 */
void generate_waveforms(void) {
    for (int i = 0; i < 256; i++) {
        sin_wave[i] = (uint8_t)(128 + 127 * sin(2.0 * M_PI * i / 256.0));
        triangle_wave[i] = i;
        square_wave[i] = (i < 128) ? 255 : 0;
    }
}

/**
 * @brief Ngắt Timer2 - Xuất dữ liệu ra DAC0808
 */
void TIM2_IRQHandler(void) {
    if (TIM2->SR & TIM_SR_UIF) {
        TIM2->SR &= ~TIM_SR_UIF; // Xóa cờ ngắt
        DAC_PORT->ODR = current_wave[wave_index]; // Xuất dữ liệu
        wave_index++;
        if (wave_index >= 256) wave_index = 0;
    }
}

/**
 * @brief Xử lý dữ liệu nhận từ USB CDC
 */
void CDC_Receive_Callback(uint8_t* Buf, uint32_t *Len)
{
	TIM2->CR1 &= ~TIM_CR1_CEN;
	 if (Buf == NULL || *Len == 0 || *Len >= 20) return; // Bảo vệ bộ nhớ

	char command[20] = {0}; // Khởi tạo chuỗi rỗng
	strncpy(command, (char*)Buf, *Len); // Sao chép dữ liệu từ Buf vào command
	command[*Len] = '\0'; // Đảm bảo chuỗi kết thúc hợp lệ

    char wave[3];
    int freq = 0;

    if (sscanf(command, "%2sF%d", wave, &freq) == 2) {
        if (strcmp(wave, "W1") == 0) {
            current_wave = sin_wave;
        } else if (strcmp(wave, "W2") == 0) {
            current_wave = triangle_wave;
        } else if (strcmp(wave, "W3") == 0) {
            current_wave = square_wave;
        } else {
            return; // Không có dạng sóng hợp lệ
        }

        if (freq > 0 && freq <= 10000) { // Giới hạn tần số
            wave_frequency = freq;
            update_TIM2();
        }
    }
}

/**
 * @brief Chương trình chính
 */
int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USB_DEVICE_Init(); // Khởi tạo USB CDC
    init_GPIO();
    init_TIM2();
    generate_waveforms();
    current_wave = sin_wave; // Mặc định dùng sóng sin
    HAL_Delay(10000);
    while (1) {

    }
}

/**
 * @brief Cấu hình Clock
 */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 15;
    RCC_OscInitStruct.PLL.PLLN = 144;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ = 5;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        while(1);
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        while(1);
    }
}

/**
 * @brief Khởi tạo GPIO
 */
static void MX_GPIO_Init(void) {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIOB->MODER |= 0x5555; // PB0-PB7 là output
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
    // Vòng lặp vô hạn để dừng chương trình khi có lỗi
  }
}

