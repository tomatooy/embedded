
// =================================================
// * Recitation 6: Final Recitation - LCD Screen *
// =================================================

// TODOs:
// [1] Recap on Polling, Interrupts, Debouncing, Multithreading, etc.
// [2] LCD Screen - Reading the datasheet and writing the code to display text and shapes on to the Screen!  
// [3] Extra --> Graphs on Screen!, Semaphores, Introduction to DSP!
// [4] Embedded Challenge Tips

// Introduction to DSP and Filters
#include "mbed.h"
#include "arm_math.h"
#include "drivers/LCD_DISCO_F429ZI.h"
#define WINDOW_SIZE 10 // Example window size, adjust as needed

// Define Regs & Configurations --> Gyroscope's settings
#define CTRL_REG1 0x20
#define CTRL_REG1_CONFIG 0b01'10'1'1'1'1
#define CTRL_REG4 0x23 // Second configure to set the DPS // page 33
#define CTRL_REG4_CONFIG 0b0'0'01'0'00'0
#define CTRL_REG3 0x22 // page 32
#define CTRL_REG3_CONFIG 0b0'0'0'0'1'000

#define MAX_GESTURE_SAMPLES 50 //---------------------------
#define MATCH_THRESHOLD 10.0f  // 根据需要调整阈值

enum State {
    RECORD_GESTURE,
    LOCKED,
    UNLOCKED
};

// 全局变量
volatile State currentState = LOCKED;



#define OUT_X_L 0x28

#define SPI_FLAG 1
#define DATA_READY_FLAG 2

#define SCALING_FACTOR (17.5f * 0.017453292519943295769236907684886f / 1000.0f)

#define FILTER_COEFFICIENT 0.1f // Adjust this value as needed


#define BUFFER_SIZE 256 // Adjust the buffer size as needed
#define FFT_LENGTH BUFFER_SIZE // FFT长度等于缓冲区大小
#define SAMPLE_RATE 100 // 假设采样率为100Hz
#define USER_BUTTON PA_0

DigitalOut ledGreen(LED1);  // 绿灯
DigitalOut ledRed(LED2);    // 红灯
InterruptIn userButton(USER_BUTTON);  // 用户按钮



// EventFlags object declaration
EventFlags flags;

// spi callback function
void spi_cb(int event) {
    flags.set(SPI_FLAG);
}


// data ready callback function
void data_cb() {
    flags.set(DATA_READY_FLAG);
}

    // Initialize the LCD screen
    LCD_DISCO_F429ZI lcd;


// Circular buffer to store gyro values
struct GyroData {
    float x, y, z;
};

GyroData recordedGesture[MAX_GESTURE_SAMPLES];
volatile int gestureDataIndex = 0;

GyroData currentGesture[MAX_GESTURE_SAMPLES];
volatile int currentDataIndex = 0;

GyroData gyroBuffer[BUFFER_SIZE];
int bufferHead = 0;
int bufferTail = 0;

void buttonPressed() {
    // 去抖动：忽略200ms内的重复按键
    // static Timer debounceTimer;
    // static bool timerStarted = false;
    // if (!timerStarted) {
    //     debounceTimer.start();
    //     timerStarted = true;
    // } else {
    //     if (debounceTimer.read_ms() < 200) {
    //         // 忽略200ms内的按键
    //         return;
    //     }
    //     debounceTimer.reset();
    // }

    // 根据当前状态切换状态
    if (currentState == LOCKED) 
    { 
        if (gestureDataIndex == 0) {
            currentState = RECORD_GESTURE;
            // 开始录制手势
            lcd.Clear(LCD_COLOR_BLACK);
            lcd.SetTextColor(LCD_COLOR_WHITE);
            lcd.SetBackColor(LCD_COLOR_BLACK);
            lcd.SetFont(&Font12);
            lcd.DisplayStringAt(0, LINE(8), (uint8_t *)"Recording Gesture...", CENTER_MODE);
        } 
        } else if (currentState == RECORD_GESTURE) {
            currentState = LOCKED;
            lcd.Clear(LCD_COLOR_BLACK);
            lcd.SetTextColor(LCD_COLOR_WHITE);
            lcd.DisplayStringAt(0, LINE(8), (uint8_t *)"System Locked1", CENTER_MODE);
            ledGreen = 0;
            ledRed = 1;  // 红灯亮起，表示系统已锁定

            // 尝试解锁
            // currentDataIndex = 0;
            // lcd.Clear(LCD_COLOR_BLACK);
            // lcd.SetTextColor(LCD_COLOR_WHITE);
            // lcd.DisplayStringAt(0, LINE(8), (uint8_t *)"Unlocked", CENTER_MODE);
        }
}

float computeGestureDifference(GyroData* gesture1, GyroData* gesture2, int length) {
    float sumSquaredDiff = 0.0f;
    for (int i = 0; i < length; i++) {
        float dx = gesture1[i].x - gesture2[i].x;
        float dy = gesture1[i].y - gesture2[i].y;
        float dz = gesture1[i].z - gesture2[i].z;
        sumSquaredDiff += dx * dx + dy * dy + dz * dz;
    }
    return sumSquaredDiff;
}

void storeGyroData(float gx, float gy, float gz) {
    gyroBuffer[bufferHead].x = gx;
    gyroBuffer[bufferHead].y = gy;
    gyroBuffer[bufferHead].z = gz;
    bufferHead = (bufferHead + 1) % BUFFER_SIZE;

    // If the buffer becomes full, overwrite the oldest data
    if (bufferHead == bufferTail) {
        bufferTail = (bufferTail + 1) % BUFFER_SIZE;
    }
}

void performFFT(float* data, float* fftOutput) {
    arm_cfft_radix4_instance_f32 fft_instance;
    arm_cfft_radix4_init_f32(&fft_instance, FFT_LENGTH, 0, 1);

    arm_cfft_radix4_f32(&fft_instance, data);

    // 计算FFT输出的幅值
    arm_cmplx_mag_f32(data, fftOutput, FFT_LENGTH);
}

float calculateFrequency(float* fftOutput, int fftLength, float sampleRate) {
    float maxMagnitude = 0.0f;
    int maxFreqIndex = 0;

    // 找到最大幅值及其对应的频率索引
    for (int i = 0; i < fftLength / 2; i++) {
        if (fftOutput[i] > maxMagnitude) {
            maxMagnitude = fftOutput[i];
            maxFreqIndex = i;
        }
    }

    float frequency = (float)maxFreqIndex * sampleRate / (float)fftLength;
    return frequency;
}

int main() {
    //spi initialization
    SPI spi(PF_9, PF_8, PF_7, PC_1, use_gpio_ssel);
    uint8_t write_buf[32], read_buf[32];

    //interrupt initialization
    InterruptIn int2(PA_2, PullDown);
    int2.rise(&data_cb);
    
    //spi format and frequency
    spi.format(8, 3);
    spi.frequency(1'000'000);

    // Write to control registers --> spi transfer
    write_buf[0] = CTRL_REG1;
    write_buf[1] = CTRL_REG1_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, spi_cb);
    flags.wait_all(SPI_FLAG);

    write_buf[0] = CTRL_REG4;
    write_buf[1] = CTRL_REG4_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, spi_cb);
    flags.wait_all(SPI_FLAG);

    write_buf[0] = CTRL_REG3;
    write_buf[1] = CTRL_REG3_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, spi_cb);
    flags.wait_all(SPI_FLAG);

    write_buf[1] = 0xFF;

    //(polling for\setting) data ready flag
    if (!(flags.get() & DATA_READY_FLAG) && (int2.read() == 1)) {
        flags.set(DATA_READY_FLAG);
    }


    // Example 2: LPF definitions
    float filtered_gx = 0.0f, filtered_gy = 0.0f, filtered_gz = 0.0f;

    // Example 3: HPF definitions
    float high_pass_gx = 0.0f, high_pass_gy = 0.0f, high_pass_gz = 0.0f;

    // Initialize the LCD screen
    LCD_DISCO_F429ZI lcd;
    lcd.Clear(LCD_COLOR_BLACK);
    lcd.SetTextColor(LCD_COLOR_WHITE);
    lcd.SetBackColor(LCD_COLOR_BLACK);
    lcd.SetFont(&Font12);
    lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"System Lock", CENTER_MODE);
    
    
    // FFT输入和输出缓冲区
    float fftInput[FFT_LENGTH * 2];
    float fftOutput[FFT_LENGTH];

   
    
        // 初始化用户按钮
    userButton.rise(&buttonPressed);  // 当按钮被按下时调用 buttonPressed

    // 初始化 LED
    ledGreen = 0;  // 关闭
    ledRed = 0;    // 关闭


    int16_t raw_gx, raw_gy, raw_gz;
    float gx, gy, gz;

    while (1) {
         //等待数据准备好
        flags.wait_all(DATA_READY_FLAG);
        flags.clear(DATA_READY_FLAG);
        

        // 读取陀螺仪数据
        write_buf[0] = OUT_X_L | 0x80 | 0x40;
        spi.transfer(write_buf, 7, read_buf, 7, spi_cb);
        flags.wait_all(SPI_FLAG);
        

        // 处理原始数据
        raw_gx = (((uint16_t)read_buf[2]) << 8) | read_buf[1];
        raw_gy = (((uint16_t)read_buf[4]) << 8) | read_buf[3];
        raw_gz = (((uint16_t)read_buf[6]) << 8) | read_buf[5];

        gx = raw_gx * SCALING_FACTOR;
        gy = raw_gy * SCALING_FACTOR;
        gz = raw_gz * SCALING_FACTOR;

        switch (currentState) {
            case RECORD_GESTURE:
                // 录制手势
                if (gestureDataIndex < MAX_GESTURE_SAMPLES) {
                    recordedGesture[gestureDataIndex].x = gx;
                    recordedGesture[gestureDataIndex].y = gy;
                    recordedGesture[gestureDataIndex].z = gz;
                    gestureDataIndex++;

                    char indexBuffer[30];
                    sprintf(indexBuffer, "Recording... %d", gestureDataIndex);
                    lcd.SetTextColor(LCD_COLOR_WHITE);
                    lcd.SetBackColor(LCD_COLOR_BLACK);
                    lcd.DisplayStringAt(0, LINE(7), (uint8_t *)indexBuffer, CENTER_MODE);
                    
                } else {
                    // 手势数据缓冲区已满，进入锁定状态
                    currentState = LOCKED;
                    lcd.Clear(LCD_COLOR_BLACK);
                    lcd.SetTextColor(LCD_COLOR_WHITE);
                    lcd.DisplayStringAt(0, LINE(7), (uint8_t *)"System Locked", CENTER_MODE);
                    ledGreen = 0;
                    ledRed = 1;  // 红灯亮起，表示系统已锁定
                }
                break;

                case LOCKED:
                    char indexBuffer1[30];
                    sprintf(indexBuffer1, "Current %d", currentDataIndex);
                    lcd.SetTextColor(LCD_COLOR_WHITE);
                    lcd.SetBackColor(LCD_COLOR_BLACK);
                    lcd.DisplayStringAt(0, LINE(10), (uint8_t *)indexBuffer1, CENTER_MODE);

                    char indexBuffer[30];
                    sprintf(indexBuffer, "Gesture %d", gestureDataIndex);
                    lcd.SetTextColor(LCD_COLOR_WHITE);
                    lcd.SetBackColor(LCD_COLOR_BLACK);
                    lcd.DisplayStringAt(0, LINE(11), (uint8_t *)indexBuffer, CENTER_MODE);

                // 锁定状态，等待用户解锁
                    if (gestureDataIndex != 0){
                        if (currentDataIndex < gestureDataIndex) {
                            // 收集当前手势数据
                            currentGesture[currentDataIndex].x = gx;
                            currentGesture[currentDataIndex].y = gy;
                            currentGesture[currentDataIndex].z = gz;
                            currentDataIndex++;

                            char indexBuffer1[30];
                            sprintf(indexBuffer1, "Recording... %d", currentDataIndex);
                            lcd.SetTextColor(LCD_COLOR_WHITE);
                            lcd.SetBackColor(LCD_COLOR_BLACK);
                            lcd.DisplayStringAt(0, LINE(7), (uint8_t *)indexBuffer1, CENTER_MODE);

                            } 
                            else {
                                // 手势数据收集完成，进行匹配
                                float difference = computeGestureDifference(recordedGesture, currentGesture, gestureDataIndex);
                                if (difference < MATCH_THRESHOLD) {
                                    // 手势匹配成功，解锁系统
                                    currentState = UNLOCKED;
                                    ledGreen = 1;
                                    ledRed = 0;
                                    lcd.Clear(LCD_COLOR_BLACK);
                                    lcd.SetTextColor(LCD_COLOR_GREEN);
                                    lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"System Unlocked!", CENTER_MODE);
                                } else {
                                    // 手势匹配失败，保持锁定
                                    ledGreen = 0;
                                    ledRed = 1;
                                    lcd.Clear(LCD_COLOR_BLACK);
                                    lcd.SetTextColor(LCD_COLOR_RED);
                                    lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"Unlock Failed", CENTER_MODE);
                                }
                                // 重置 currentDataIndex
                                currentDataIndex = 0;
                            }
                    }
                break;

                }
        // int16_t raw_gx, raw_gy, raw_gz;
        // float gx, gy, gz;

        // flags.wait_all(DATA_READY_FLAG);
        // write_buf[0] = OUT_X_L | 0x80 | 0x40;

        // spi.transfer(write_buf, 7, read_buf, 7, spi_cb);
        // flags.wait_all(SPI_FLAG);

        // // Process raw data
        // raw_gx = (((uint16_t)read_buf[2]) << 8) | ((uint16_t)read_buf[1]);
        // raw_gy = (((uint16_t)read_buf[4]) << 8) | ((uint16_t)read_buf[3]);
        // raw_gz = (((uint16_t)read_buf[6]) << 8) | ((uint16_t)read_buf[5]);

        // gx = ((float)raw_gx) * SCALING_FACTOR;
        // gy = ((float)raw_gy) * SCALING_FACTOR;
        // gz = ((float)raw_gz) * SCALING_FACTOR;



        // // Example 2: Apply Simple low-pass filter
        // filtered_gx = FILTER_COEFFICIENT * gx + (1 - FILTER_COEFFICIENT) * filtered_gx;
        // filtered_gy = FILTER_COEFFICIENT * gy + (1 - FILTER_COEFFICIENT) * filtered_gy;
        // filtered_gz = FILTER_COEFFICIENT * gz + (1 - FILTER_COEFFICIENT) * filtered_gz;

        // high_pass_gx = gx - filtered_gx;
        // high_pass_gy = gy - filtered_gy;
        // high_pass_gz = gz - filtered_gz;

        // printf(">x_axis_high:%4.5f\n", high_pass_gx);
        // printf(">y_axis_high:%4.5f\n", high_pass_gy);
        // printf(">z_axis_high:%4.5f\n", high_pass_gz);

        // // Store the gyro values in the circular buffer
        // storeGyroData(gx, gy, gz);

        // // Clear the LCD screen
        // lcd.Clear(LCD_COLOR_BLACK);


        // // Print the gyro values on the LCD screen
        // lcd.SetTextColor(LCD_COLOR_WHITE);
        // lcd.SetBackColor(LCD_COLOR_BLACK);
        // lcd.SetFont(&Font16);

        // sprintf((char *)gyroBuffer, "X: %.2f", gx);
        // lcd.DisplayStringAt(0, LINE(0), (uint8_t *)gyroBuffer, CENTER_MODE);

        // sprintf((char *)gyroBuffer, "Y: %.2f", gy);
        // lcd.DisplayStringAt(0, LINE(2), (uint8_t *)gyroBuffer, CENTER_MODE);

        // sprintf((char *)gyroBuffer, "Z: %.2f", gz);
        // lcd.DisplayStringAt(0, LINE(4), (uint8_t *)gyroBuffer, CENTER_MODE);




        // // 对陀螺仪数据执行FFT
        // for (int i = 0; i < FFT_LENGTH; i++) {
        //     int bufferIndex = (bufferTail + i) % BUFFER_SIZE;
        //     fftInput[i * 2] = gyroBuffer[bufferIndex].x;
        //     fftInput[i * 2 + 1] = 0.0f; // 实数输入数据的虚部为0
        // }

        // performFFT(fftInput, fftOutput);

        //  // 计算当前检测到的频率
        // float currentFrequency = calculateFrequency(fftOutput, FFT_LENGTH, SAMPLE_RATE);

        // // 在LCD屏幕上打印频率信息
        // lcd.SetFont(&Font12);

        // char freqBuffer[50];
        // sprintf(freqBuffer, "Current Freq: %.2f Hz", currentFrequency);
        // lcd.DisplayStringAt(0, LINE(7), (uint8_t *)freqBuffer, CENTER_MODE);

        // // 检查当前频率是否在期望的范围内
        // if (currentFrequency >= 3 && currentFrequency <= 6) {
        //     lcd.SetTextColor(LCD_COLOR_GREEN);
        //     lcd.DisplayStringAt(0, LINE(11), (uint8_t *)"Parkinsonian Tremor Detected", CENTER_MODE);
        // } else {
        //     lcd.SetTextColor(LCD_COLOR_RED);
        //     lcd.DisplayStringAt(0, LINE(12), (uint8_t *)"No0 Parkinsonian Tremor", CENTER_MODE);
        // }

        thread_sleep_for(100);

    }
}





// #include "mbed.h"
// #include "arm_math.h"
// #include "drivers/LCD_DISCO_F429ZI.h"

// SPI and IMU registers
// Convert mdps to rad/s

// // Event flags
// #define SPI_FLAG 1
// #define DATA_READY_FLAG 2

// // IMU data sample size
// #define SAMPLE_SIZE 100

// // Tolerance level for gesture matching
// #define TOLERANCE_THRESHOLD 5000.0f // Adjust based on testing

// // Pins for buttons and LED
// #define RECORD_BUTTON_PIN PA_0   // PA0
// #define ENTER_BUTTON_PIN PB_1           // Connect an external button to PB1
// #define LED_PIN PG_13                    // PD13

// // Global objects
// SPI spi(PB_5, PB_4, PB_3, PE_3, use_gpio_ssel); // MOSI, MISO, SCK, CS
// DigitalOut led(LED_PIN);
// InterruptIn record_button(RECORD_BUTTON_PIN);
// InterruptIn enter_button(ENTER_BUTTON_PIN);
// EventFlags flags;

// // Buffers for IMU data
// int16_t recorded_sequence[SAMPLE_SIZE][3];  // [X, Y, Z]
// int16_t input_sequence[SAMPLE_SIZE][3];

// // SPI callback function
// void spi_cb(int event) {
//     flags.set(SPI_FLAG);
// }

// DigitalOut cs(PE_3);
// // IMU initialization
// void imu_init() {
//     // 配置 CTRL_REG1
//     cs = 0; // 开始通信
//     spi.write(CTRL_REG1);          // 发送寄存器地址
//     spi.write(CTRL_REG1_CONFIG);   // 发送配置值
//     cs = 1; // 结束通信

//     // 配置 CTRL_REG4
//     cs = 0;
//     spi.write(CTRL_REG4);
//     spi.write(CTRL_REG4_CONFIG);
//     cs = 1;
// }

// // Read IMU data function
// void read_imu_data(int16_t* data) {
//     uint8_t write_buf[7];
//     uint8_t read_buf[7];

//     write_buf[0] = OUT_X_L | 0x80 | 0x40; // Read command with auto-increment

//     spi.transfer(write_buf, 7, read_buf, 7, spi_cb);
//     flags.wait_all(SPI_FLAG);

//     // Combine low and high bytes for each axis
//     data[0] = (int16_t)(read_buf[1] | (read_buf[2] << 8)); // X-axis
//     data[1] = (int16_t)(read_buf[3] | (read_buf[4] << 8)); // Y-axis
//     data[2] = (int16_t)(read_buf[5] | (read_buf[6] << 8)); // Z-axis
// }

// // Gesture recording function
// void record_gesture() {
//     printf("Recording gesture...\n");
//     for (int i = 0; i < SAMPLE_SIZE; ++i) {
//         read_imu_data(recorded_sequence[i]);
//         thread_sleep_for(10);  // Adjust sampling rate as needed
//     }
//     printf("Gesture recorded.\n");
// }

// // Gesture input function
// bool input_gesture() {
//     printf("Input your gesture...\n");
//     for (int i = 0; i < SAMPLE_SIZE; ++i) {
//         read_imu_data(input_sequence[i]);
//         thread_sleep_for(10);  // Adjust sampling rate as needed
//     }
//     printf("Gesture input completed.\n");

//     // Compare the input sequence with the recorded sequence
//     float difference = 0.0f;
//     for (int i = 0; i < SAMPLE_SIZE; ++i) {
//         for (int axis = 0; axis < 3; ++axis) {
//             float recorded = (float)recorded_sequence[i][axis];
//             float input = (float)input_sequence[i][axis];
//             difference += fabs(recorded - input);
//         }
//     }
//     // Total difference
//     printf("Total Difference: %f\n", difference);

//     return (difference < TOLERANCE_THRESHOLD);
// }

// // Record button interrupt handler
// void on_record_button_pressed() {
//     record_gesture();
// }

// // Enter button interrupt handler
// void on_enter_button_pressed() {
//     if (input_gesture()) {
//         printf("Unlock successful!\n");
//         led = 1;  // Turn on LED
//     } else {
//         printf("Unlock failed.\n");
//         led = 0;  // Turn off LED
//     }
// }

// int main() {
//     // Initialize hardware
//     spi.format(8, 3);
//     spi.frequency(1'000'000);
//     imu_init();
//     led = 0;  // Ensure LED is off initially

//     // // Attach button interrupts
//     record_button.rise(&on_record_button_pressed);
//     enter_button.rise(&on_enter_button_pressed);

//     printf("System initialized. Ready for input.\n");
//     LCD_DISCO_F429ZI lcd;
//     lcd.Clear(LCD_COLOR_BLACK);


//     // Print the gyro values on the LCD screen
//     lcd.SetTextColor(LCD_COLOR_WHITE);
//     lcd.SetBackColor(LCD_COLOR_BLACK);
//     lcd.SetFont(&Font12);
//     lcd.DisplayStringAt(0, LINE(11), (uint8_t *)"System initialized66. ", CENTER_MODE);
//     lcd.DisplayStringAt(0, LINE(12), (uint8_t *)"Ready for input. ", CENTER_MODE);

//     // // Main loop
//     while (true) {
//         // Idle loop
//         thread_sleep_for(1000);
//     }
// }

// #include "mbed.h"
// #include "arm_math.h"
// #include "drivers/LCD_DISCO_F429ZI.h"
// #include "platform/mbed_thread.h"
// #include <cfloat> 
// #include "stm32f4xx_hal.h"

// #define CTRL_REG1 0x20
// #define CTRL_REG1_CONFIG 0x0F // Normal mode, all axes enabled, ODR 95 Hz
// #define CTRL_REG4 0x23
// #define CTRL_REG4_CONFIG 0x30 // 2000 dps full scale
// #define OUT_X_L 0x28
// #define SENSITIVITY 70.0f // mdps/LSB
// #define SCALING_FACTOR (SENSITIVITY / 1000.0f * 0.0174533f) 
// #define CTRL_REG3 0x22 // page 32
// #define CTRL_REG3_CONFIG 0b0'0'0'0'1'000

// // 定义窗口大小和其他宏
// #define WINDOW_SIZE 10
// #define BUFFER_SIZE 256
// #define FFT_LENGTH BUFFER_SIZE
// #define SAMPLE_RATE 100
// #define TRAJECTORY_SIZE 1000
// #define MSE_THRESHOLD 500.0f // 调整匹配的阈值
// #define SCALING_FACTOR (17.5f * 0.017453292519943295769236907684886f / 1000.0f)
// #define FILTER_COEFFICIENT 0.1f

// // 事件标志
// #define SPI_FLAG 1
// #define DATA_READY_FLAG 2

// #define USER_BUTTON PA_0   // PA0


// // 声明事件标志对象
// EventFlags flags;

// #define BUFFER_SIZE 256
// #define SAMPLE_RATE 100
// #define GESTURE_LENGTH 100

// // 按键和LED引脚定义
// InterruptIn button(USER_BUTTON);
// DigitalOut greenLED(LED1);
// DigitalOut redLED(LED2);

// enum State {
//     IDLE,
//     RECORDING,
//     DETECTING
// };

// // 全局变量
// volatile State currentState = IDLE;

// // LCD屏幕
// LCD_DISCO_F429ZI lcd;
// #define MAX_GESTURE_SAMPLES 1000
// #define MATCH_THRESHOLD 1000.0f  // 根据需要调整阈值

// void data_cb() {
//     flags.set(DATA_READY_FLAG);
// }

// void spi_cb(int event) {
//     flags.set(SPI_FLAG);
// }

// struct GyroData {
//     float x, y, z;
// };

// // 手势数据缓冲区
// GyroData recordedGesture[MAX_GESTURE_SAMPLES];
// volatile int gestureDataIndex = 0;

// GyroData currentGesture[MAX_GESTURE_SAMPLES];
// volatile int currentDataIndex = 0;

// void buttonPressed() {
//     // 去抖动：忽略200ms内的重复按键
//     static Timer debounceTimer;
//     static bool timerStarted = false;
//     if (!timerStarted) {
//         debounceTimer.start();
//         timerStarted = true;
//     } else {
//         if (debounceTimer.read_ms() < 200) {
//             // 忽略200ms内的按键
//             return;
//         }
//         debounceTimer.reset();
//     }

//     // 根据当前状态切换状态
//     if (currentState == IDLE) {
//         currentState = RECORDING;
//         gestureDataIndex = 0;
//         // 显示“Recording...”在LCD上
//         lcd.Clear(LCD_COLOR_BLACK);
//         lcd.SetTextColor(LCD_COLOR_WHITE);
//         lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"Recording...", CENTER_MODE);

//         // 关闭LED
//         greenLED = 0;
//         redLED = 0;
//     } else if (currentState == RECORDING) {
//         currentState = DETECTING;
//         // 显示“Gesture Saved”在LCD上
//         lcd.Clear(LCD_COLOR_BLACK);
//         lcd.SetTextColor(LCD_COLOR_WHITE);
//         lcd.SetBackColor(LCD_COLOR_BLACK);
//         lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"Gesture Saved", CENTER_MODE);

//         // 初始化检测变量
//         currentDataIndex = 0;
//         // 关闭LED
//         greenLED = 0;
//         redLED = 0;
//     } 
// }

// // 手势比较函数
// float computeGestureDifference(GyroData* gesture1, GyroData* gesture2, int length) {
//     float sumSquaredDiff = 0.0f;
//     for (int i = 0; i < length; i++) {
//         float dx = gesture1[i].x - gesture2[i].x;
//         float dy = gesture1[i].y - gesture2[i].y;
//         float dz = gesture1[i].z - gesture2[i].z;
//         sumSquaredDiff += dx * dx + dy * dy + dz * dz;
//     }
//     return sumSquaredDiff;
// }



// int main() {
//     // SPI初始化
//     SPI spi(PF_9, PF_8, PF_7, PC_1, use_gpio_ssel);
//     uint8_t write_buf[32], read_buf[32];

//     // 中断初始化
//     InterruptIn int2(PA_2, PullDown);
//     int2.rise(&data_cb);

//     // SPI格式和频率
//     spi.format(8, 3);
//     spi.frequency(1'000'000);

//     // 写入控制寄存器
//     write_buf[0] = CTRL_REG1;
//     write_buf[1] = CTRL_REG1_CONFIG;
//     spi.transfer(write_buf, 2, read_buf, 2, spi_cb);
//     flags.wait_all(SPI_FLAG);

//     write_buf[0] = CTRL_REG4;
//     write_buf[1] = CTRL_REG4_CONFIG;
//     spi.transfer(write_buf, 2, read_buf, 2, spi_cb);
//     flags.wait_all(SPI_FLAG);

//     write_buf[0] = CTRL_REG3;
//     write_buf[1] = CTRL_REG3_CONFIG;
//     spi.transfer(write_buf, 2, read_buf, 2, spi_cb);
//     flags.wait_all(SPI_FLAG);

//     write_buf[1] = 0xFF;

//     // 初始化用户按钮
//     button.fall(&buttonPressed);  // 当按钮被按下时调用 buttonPressed

//     // 初始化LED
//     greenLED = 0;  // 关闭
//     redLED = 0;    // 关闭

//     // 初始化LCD
//     lcd.Clear(LCD_COLOR_BLACK);
//     lcd.SetTextColor(LCD_COLOR_WHITE);
//     lcd.SetBackColor(LCD_COLOR_BLACK);
//     lcd.SetFont(&Font12);
//     lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"Press button to record gesture", CENTER_MODE);

//     // 主循环变量
//     int16_t raw_gx, raw_gy, raw_gz;
//     float gx, gy, gz;

//     while (1) {
//         // 等待数据准备好
//         flags.wait_all(DATA_READY_FLAG);
//         flags.clear(DATA_READY_FLAG);

//         // 读取陀螺仪数据
//         write_buf[0] = OUT_X_L | 0x80 | 0x40;
//         spi.transfer(write_buf, 7, read_buf, 7, spi_cb);
//         flags.wait_all(SPI_FLAG);

//         // 处理原始数据
//         raw_gx = (((uint16_t)read_buf[2]) << 8) | ((uint16_t)read_buf[1]);
//         raw_gy = (((uint16_t)read_buf[4]) << 8) | ((uint16_t)read_buf[3]);
//         raw_gz = (((uint16_t)read_buf[6]) << 8) | ((uint16_t)read_buf[5]);

//         gx = ((float)raw_gx) * SCALING_FACTOR;
//         gy = ((float)raw_gy) * SCALING_FACTOR;
//         gz = ((float)raw_gz) * SCALING_FACTOR;

//         switch (currentState) {
//             case IDLE:
//                 // 空闲状态，不执行任何操作
//                 break;
//             case RECORDING:
//                 // 存储陀螺仪数据
//                 if (gestureDataIndex < MAX_GESTURE_SAMPLES) {
//                     recordedGesture[gestureDataIndex].x = gx;
//                     recordedGesture[gestureDataIndex].y = gy;
//                     recordedGesture[gestureDataIndex].z = gz;
//                     gestureDataIndex++;
//                     lcd.Clear(LCD_COLOR_BLACK);
//                     lcd.SetTextColor(LCD_COLOR_WHITE);
//                     lcd.SetFont(&Font12);
//                     lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"Recording..", CENTER_MODE);
//                 } else {
//                     // 手势数据缓冲区已满，切换到检测状态
//                     currentState = DETECTING;
//                     // 显示“Gesture Saved”在LCD上
//                     lcd.Clear(LCD_COLOR_BLACK);
//                     lcd.SetTextColor(LCD_COLOR_WHITE);
//                     lcd.SetFont(&Font12);
//                     lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"Gesture Saved", CENTER_MODE);

//                     // 初始化检测变量
//                     currentDataIndex = 0;
//                     // 关闭LED
//                     greenLED = 0;
//                     redLED = 0;
//                 }
//                 break;
//             case DETECTING:
//                 // 收集当前数据
//                 if (currentDataIndex < gestureDataIndex) {
//                     currentGesture[currentDataIndex].x = gx;
//                     currentGesture[currentDataIndex].y = gy;
//                     currentGesture[currentDataIndex].z = gz;
//                     currentDataIndex++;
//                 } else {
//                     // 已收集足够的数据，比较手势
//                     float difference = computeGestureDifference(recordedGesture, currentGesture, gestureDataIndex);
//                     if (difference < MATCH_THRESHOLD) {
//                         // 手势匹配
//                         greenLED = 1;
//                         redLED = 0;
//                         lcd.Clear(LCD_COLOR_BLACK);
//                         lcd.SetTextColor(LCD_COLOR_GREEN);
//                         lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"Gesture Matched!", CENTER_MODE);
//                     } else {
//                         // 手势不匹配
//                         greenLED = 0;
//                         redLED = 1;
//                         lcd.Clear(LCD_COLOR_BLACK);
//                         lcd.SetTextColor(LCD_COLOR_RED);
//                         lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"Gesture Not Matched", CENTER_MODE);
//                     }
//                     // 重置currentDataIndex以收集新数据
//                     currentDataIndex = 0;
//                 }
//                 break;
//             default:
//                 break;
//         }

//         // 小的延迟
//         thread_sleep_for(10);
//     }
// }