
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

#define MAX_GESTURE_SAMPLES 10 //---------------------------
#define MATCH_THRESHOLD 10.0f  // 根据需要调整阈值

enum State
{
    ENTRY,
    RECORD_GESTURE_SETUP,
    LOCKED,
    RECORD_GESTURE_UNLOCK,
    UNLOCKED,
};

// 全局变量
volatile State currentState = ENTRY;

#define OUT_X_L 0x28

#define SPI_FLAG 1
#define DATA_READY_FLAG 2

#define SCALING_FACTOR (17.5f * 0.017453292519943295769236907684886f / 1000.0f)

#define FILTER_COEFFICIENT 0.1f // Adjust this value as needed

#define BUFFER_SIZE 256        // Adjust the buffer size as needed
#define FFT_LENGTH BUFFER_SIZE // FFT长度等于缓冲区大小
#define SAMPLE_RATE 100        // 假设采样率为100Hz
#define USER_BUTTON PA_0

DigitalOut ledGreen(LED1);           // 绿灯
DigitalOut ledRed(LED2);             // 红灯
InterruptIn userButton(USER_BUTTON); // 用户按钮

// EventFlags object declaration
EventFlags flags;

// spi callback function
void spi_cb(int event)
{
    flags.set(SPI_FLAG);
}

// data ready callback function
void data_cb()
{
    flags.set(DATA_READY_FLAG);
}

// Initialize the LCD screen
LCD_DISCO_F429ZI lcd;

// Circular buffer to store gyro values
struct GyroData
{
    float x, y, z;
};

GyroData recordedGesture[MAX_GESTURE_SAMPLES];
volatile int gestureDataIndex = 0;

GyroData currentGesture[MAX_GESTURE_SAMPLES];
volatile int currentDataIndex = 0;

GyroData gyroBuffer[BUFFER_SIZE];
int bufferHead = 0;
int bufferTail = 0;

void buttonPressed()
{
    switch (currentState) {
        case ENTRY:
            currentState = RECORD_GESTURE_SETUP;
            lcd.Clear(LCD_COLOR_BLACK);
            lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"Recording Setup Gesture", CENTER_MODE);
            gestureDataIndex = 0;
            break;

        case LOCKED:
            currentState = RECORD_GESTURE_UNLOCK;
            lcd.Clear(LCD_COLOR_BLACK);
            lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"Recording Unlock Gesture", CENTER_MODE);
            currentDataIndex = 0;
            break;

        case UNLOCKED:
            // Do nothing or reset to ENTRY
            break;

        default:
            break;
    }

}

float computeGestureDifference(GyroData *gesture1, GyroData *gesture2, int length)
{
    float sumSquaredDiff = 0.0f;
    for (int i = 0; i < length; i++)
    {
        float dx = gesture1[i].x - gesture2[i].x;
        float dy = gesture1[i].y - gesture2[i].y;
        float dz = gesture1[i].z - gesture2[i].z;
        sumSquaredDiff += dx * dx + dy * dy + dz * dz;
    }
    return sumSquaredDiff;
}

void storeGyroData(float gx, float gy, float gz)
{
    gyroBuffer[bufferHead].x = gx;
    gyroBuffer[bufferHead].y = gy;
    gyroBuffer[bufferHead].z = gz;
    bufferHead = (bufferHead + 1) % BUFFER_SIZE;

    // If the buffer becomes full, overwrite the oldest data
    if (bufferHead == bufferTail)
    {
        bufferTail = (bufferTail + 1) % BUFFER_SIZE;
    }
}

void performFFT(float *data, float *fftOutput)
{
    arm_cfft_radix4_instance_f32 fft_instance;
    arm_cfft_radix4_init_f32(&fft_instance, FFT_LENGTH, 0, 1);

    arm_cfft_radix4_f32(&fft_instance, data);

    // 计算FFT输出的幅值
    arm_cmplx_mag_f32(data, fftOutput, FFT_LENGTH);
}

float calculateFrequency(float *fftOutput, int fftLength, float sampleRate)
{
    float maxMagnitude = 0.0f;
    int maxFreqIndex = 0;

    // 找到最大幅值及其对应的频率索引
    for (int i = 0; i < fftLength / 2; i++)
    {
        if (fftOutput[i] > maxMagnitude)
        {
            maxMagnitude = fftOutput[i];
            maxFreqIndex = i;
        }
    }

    float frequency = (float)maxFreqIndex * sampleRate / (float)fftLength;
    return frequency;
}

void recordGesture(float gx, float gy, float gz) {
    switch (currentState) {
        case RECORD_GESTURE_SETUP:
            if (gestureDataIndex < MAX_GESTURE_SAMPLES) {
                recordedGesture[gestureDataIndex++] = {gx, gy, gz};

                // Optionally display progress on LCD
                char progress[30];
                sprintf(progress, "Setup Gesture: %d/%d", gestureDataIndex, MAX_GESTURE_SAMPLES);
                lcd.ClearStringLine(LINE(5));
                lcd.DisplayStringAt(0, LINE(5), (uint8_t *)progress, CENTER_MODE);

                if (gestureDataIndex == MAX_GESTURE_SAMPLES) {
                    currentState = LOCKED;
                    lcd.Clear(LCD_COLOR_BLACK);
                    lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"System Locked", CENTER_MODE);
                    ledGreen = 0;
                    ledRed = 1;
                }
            }
            break;

        case RECORD_GESTURE_UNLOCK:
            if (currentDataIndex < MAX_GESTURE_SAMPLES) {
                currentGesture[currentDataIndex++] = {gx, gy, gz};

                // Optionally display progress on LCD
                char progress[30];
                sprintf(progress, "Unlock Gesture: %d/%d", currentDataIndex, MAX_GESTURE_SAMPLES);
                lcd.ClearStringLine(LINE(5));
                lcd.DisplayStringAt(0, LINE(5), (uint8_t *)progress, CENTER_MODE);

                if (currentDataIndex == MAX_GESTURE_SAMPLES) {
                    // Compare the recorded gesture with the current gesture
                    float difference = computeGestureDifference(recordedGesture, currentGesture, MAX_GESTURE_SAMPLES);
                    if (difference < MATCH_THRESHOLD) {
                        currentState = UNLOCKED;
                        lcd.Clear(LCD_COLOR_BLACK);
                        lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"System Unlocked!", CENTER_MODE);
                        ledGreen = 1;
                        ledRed = 0;
                    } else {
                        currentState = LOCKED;
                        lcd.Clear(LCD_COLOR_BLACK);
                        lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"Unlock Failed", CENTER_MODE);
                        ledGreen = 0;
                        ledRed = 1;
                    }
                }
            }
            break;

        default:
            // Do nothing for other states
            break;
    }
}

void displayGyroData(float gx, float gy, float gz) {
    // Clear the current line and move the cursor to the start
    printf("\033[2K\r"); // \033[2K clears the line, \r moves the cursor to the beginning
    printf("Gyroscope: X=%.2f, Y=%.2f, Z=%.2f", gx, gy, gz);
    fflush(stdout); // Ensure immediate output
}
int main()
{
    // spi initialization
    SPI spi(PF_9, PF_8, PF_7, PC_1, use_gpio_ssel);
    uint8_t write_buf[32], read_buf[32];

    // interrupt initialization
    InterruptIn int2(PA_2, PullDown);
    int2.rise(&data_cb);

    // spi format and frequency
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
    if (!(flags.get() & DATA_READY_FLAG) && (int2.read() == 1))
    {
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
    userButton.rise(&buttonPressed); // 当按钮被按下时调用 buttonPressed

    // 初始化 LED
    ledGreen = 0; // 关闭
    ledRed = 0;   // 关闭

    int16_t raw_gx, raw_gy, raw_gz;
    float gx, gy, gz;

    while (1)
    {
        // 等待数据准备好
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
        displayGyroData(gx,gy,gz);
        if (currentState == RECORD_GESTURE_SETUP || currentState == RECORD_GESTURE_UNLOCK) {
            recordGesture(gx, gy, gz);
        }

        thread_sleep_for(100);
    }
}

