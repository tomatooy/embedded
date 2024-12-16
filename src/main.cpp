//Team member
//Zongxin Ouyang(zo2065)
//Yiming Wang(yw8065)
//Ruixin Wang (rw3495)

#include "mbed.h"
#include "arm_math.h"
#include "drivers/LCD_DISCO_F429ZI.h"
#include <vector>
#include <cmath>
#include <cfloat>

#define CTRL_REG1 0x20
#define CTRL_REG1_CONFIG 0b01'10'1'1'1'1
#define CTRL_REG4 0x23
#define CTRL_REG4_CONFIG 0b0'0'01'0'00'0
#define CTRL_REG3 0x22
#define CTRL_REG3_CONFIG 0b0'0'0'0'1'000

#define MAX_GESTURE_SAMPLES 30
#define MATCH_THRESHOLD 20.0f

enum State {
    ENTRY,
    RECORD_GESTURE_SETUP,
    LOCKED,
    RECORD_GESTURE_UNLOCK,
    UNLOCK_FAILED,
    UNLOCKED
};

volatile State currentState = ENTRY;

#define OUT_X_L 0x28
#define SPI_FLAG 1
#define DATA_READY_FLAG 2

// Scaling factor from raw units to rad/s approximately
#define SCALING_FACTOR (17.5f * 0.0174532925199432957692236907684886f / 1000.0f)
#define USER_BUTTON PA_0

DigitalOut ledGreen(LED1);
DigitalOut ledRed(LED2);
InterruptIn userButton(USER_BUTTON);
LCD_DISCO_F429ZI lcd;
EventFlags flags;

struct GyroData {
    float x, y, z;
};

// Global offsets (computed during calibration)
float data_offset_x = 0.0f;
float data_offset_y = 0.0f;
float data_offset_z = 0.0f;

GyroData recordedGesture[MAX_GESTURE_SAMPLES];
int gestureDataIndex = 0;

GyroData currentGesture[MAX_GESTURE_SAMPLES];
int currentDataIndex = 0;

// Define the window size for the moving average
#define WINDOW_SIZE 5
float window_gx[WINDOW_SIZE] = {0}, window_gy[WINDOW_SIZE] = {0}, window_gz[WINDOW_SIZE] = {0};
int window_index = 0;

// High-pass filter parameters
static float alpha = 0.95f; // filter coefficient
static float prev_x_in = 0.0f, prev_y_in = 0.0f, prev_z_in = 0.0f;
static float prev_x_out = 0.0f, prev_y_out = 0.0f, prev_z_out = 0.0f;

void updateDisplayAndLEDs(State st);
void buttonPressed();
void spi_cb(int event);
void data_cb();

GyroData subtract_offset(const GyroData &raw_data, float offsetX, float offsetY, float offsetZ) {
    GyroData data;
    data.x = raw_data.x - offsetX;
    data.y = raw_data.y - offsetY;
    data.z = raw_data.z - offsetZ;
    return data;
}

// Function to calibrate the gyro offsets
void gyro_calibrate(SPI &spi, DigitalOut &cs) {
    int samples = 100;
    float sumX = 0.0f, sumY = 0.0f, sumZ = 0.0f;

    for (int i = 0; i < samples; i++) {
        uint8_t write_buf[7] = { OUT_X_L | 0x80 | 0x40 };
        uint8_t read_buf[7] = {0};
        cs = 0;
        for (int j = 0; j < 7; j++) {
            read_buf[j] = spi.write(write_buf[j]);
        }
        cs = 1;

        int16_t raw_gx_int = (((uint16_t)read_buf[2]) << 8) | read_buf[1];
        int16_t raw_gy_int = (((uint16_t)read_buf[4]) << 8) | read_buf[3];
        int16_t raw_gz_int = (((uint16_t)read_buf[6]) << 8) | read_buf[5];

        float gx = raw_gx_int * SCALING_FACTOR;
        float gy = raw_gy_int * SCALING_FACTOR;
        float gz = raw_gz_int * SCALING_FACTOR;

        sumX += gx;
        sumY += gy;
        sumZ += gz;

        thread_sleep_for(10);
    }

    data_offset_x = sumX / samples;
    data_offset_y = sumY / samples;
    data_offset_z = sumZ / samples;
}

std::vector<std::vector<float>> convertToVector(const GyroData* gesture, int length) {
    std::vector<std::vector<float>> vec;
    vec.reserve(length);
    for (int i = 0; i < length; i++) {
        vec.push_back({gesture[i].x, gesture[i].y, gesture[i].z});
    }
    return vec;
}

float dtw_distance(const std::vector<std::vector<float>> &seq1, const std::vector<std::vector<float>> &seq2) {
    int len1 = (int)seq1.size();
    int len2 = (int)seq2.size();

    std::vector<std::vector<float>> dtw(len1 + 1, std::vector<float>(len2 + 1, FLT_MAX));
    dtw[0][0] = 0.0f;

    for (int i = 1; i <= len1; ++i) {
        for (int j = 1; j <= len2; ++j) {
            float cost = 0;
            for (int k = 0; k < 3; ++k) {
                cost += fabsf(seq1[i - 1][k] - seq2[j - 1][k]);
            }
            float minPrev = dtw[i - 1][j];
            if (dtw[i][j - 1] < minPrev) minPrev = dtw[i][j - 1];
            if (dtw[i - 1][j - 1] < minPrev) minPrev = dtw[i - 1][j - 1];
            dtw[i][j] = cost + minPrev;
        }
    }
    return dtw[len1][len2];
}

void updateDisplayAndLEDs(State st) {
    lcd.Clear(LCD_COLOR_BLACK);
    switch (st) {
        case ENTRY:
            lcd.SetTextColor(LCD_COLOR_WHITE);
            lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"Press USER to record gesture", CENTER_MODE);
            ledGreen = 0;
            ledRed = 0;
            break;

        case RECORD_GESTURE_SETUP:
            lcd.SetTextColor(LCD_COLOR_YELLOW);
            lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"Recording Lock Gesture...", CENTER_MODE);
            ledGreen = 0;
            ledRed = 0;
            break;

        case LOCKED:
            lcd.SetTextColor(LCD_COLOR_WHITE);
            lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"System Locked", CENTER_MODE);
            lcd.DisplayStringAt(0, LINE(7), (uint8_t *)"Press USER to Unlock", CENTER_MODE);
            ledGreen = 0;
            ledRed = 1;
            break;

        case RECORD_GESTURE_UNLOCK:
            lcd.SetTextColor(LCD_COLOR_YELLOW);
            lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"Recording Unlock Gesture...", CENTER_MODE);
            ledGreen = 0;
            ledRed = 0;
            break;

        case UNLOCK_FAILED:
            lcd.SetTextColor(LCD_COLOR_RED);
            lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"Unlock Failed", CENTER_MODE);
            lcd.DisplayStringAt(0, LINE(7), (uint8_t *)"Press USER to Try Again", CENTER_MODE);
            ledGreen = 0;
            ledRed = 1;
            break;

        case UNLOCKED:
            lcd.SetTextColor(LCD_COLOR_GREEN);
            lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"System Unlocked!", CENTER_MODE);
            ledGreen = 1;
            ledRed = 0;
            break;
    }
}

void buttonPressed() {
    switch (currentState) {
        case ENTRY:
            gestureDataIndex = 0;
            currentState = RECORD_GESTURE_SETUP;
            updateDisplayAndLEDs(currentState);
            break;

        case LOCKED:
            currentDataIndex = 0;
            currentState = RECORD_GESTURE_UNLOCK;
            updateDisplayAndLEDs(currentState);
            break;

        case UNLOCK_FAILED:
            currentDataIndex = 0;
            currentState = RECORD_GESTURE_UNLOCK;
            updateDisplayAndLEDs(currentState);
            break;

        case UNLOCKED:
            // Optionally revert to LOCKED or do nothing
            break;

        case RECORD_GESTURE_SETUP:
        case RECORD_GESTURE_UNLOCK:
            // No direct transition on button press
            break;
    }
}

void spi_cb(int event) {
    flags.set(SPI_FLAG);
}

void data_cb() {
    flags.set(DATA_READY_FLAG);
}

int main() {
    SPI spi(PF_9, PF_8, PF_7, PC_1, use_gpio_ssel);
    uint8_t write_buf[32], read_buf[32];
    DigitalOut cs(PC_1, 1);

    InterruptIn int2(PA_2, PullDown);
    int2.rise(&data_cb);

    spi.format(8, 3);
    spi.frequency(1'000'000);

    // Configure gyro registers
    write_buf[0] = CTRL_REG1;
    write_buf[1] = CTRL_REG1_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, &spi_cb);
    flags.wait_all(SPI_FLAG);

    write_buf[0] = CTRL_REG4;
    write_buf[1] = CTRL_REG4_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, &spi_cb);
    flags.wait_all(SPI_FLAG);

    write_buf[0] = CTRL_REG3;
    write_buf[1] = CTRL_REG3_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, &spi_cb);
    flags.wait_all(SPI_FLAG);

    write_buf[1] = 0xFF;
    if (!(flags.get() & DATA_READY_FLAG) && (int2.read() == 1)) {
        flags.set(DATA_READY_FLAG);
    }

    // Initialize LCD and LEDs
    lcd.Clear(LCD_COLOR_BLACK);
    lcd.SetTextColor(LCD_COLOR_WHITE);
    lcd.SetBackColor(LCD_COLOR_BLACK);
    lcd.SetFont(&Font12);

    currentState = ENTRY;
    updateDisplayAndLEDs(currentState);

    userButton.rise(&buttonPressed);

    // Calibrate gyro offsets
    gyro_calibrate(spi, cs);

    while (1) {
        flags.wait_all(DATA_READY_FLAG);
        flags.clear(DATA_READY_FLAG);

        // Read raw gyro data
        write_buf[0] = OUT_X_L | 0x80 | 0x40;
        spi.transfer(write_buf, 7, read_buf, 7, spi_cb);
        flags.wait_all(SPI_FLAG);

        int16_t raw_gx_int = (((uint16_t)read_buf[2]) << 8) | read_buf[1];
        int16_t raw_gy_int = (((uint16_t)read_buf[4]) << 8) | read_buf[3];
        int16_t raw_gz_int = (((uint16_t)read_buf[6]) << 8) | read_buf[5];

        float gx = raw_gx_int * SCALING_FACTOR;
        float gy = raw_gy_int * SCALING_FACTOR;
        float gz = raw_gz_int * SCALING_FACTOR;

        // Subtract the calibrated offsets
        GyroData data = subtract_offset({gx, gy, gz}, data_offset_x, data_offset_y, data_offset_z);

        // Apply high-pass filter
        float x_in = data.x;
        float y_in = data.y;
        float z_in = data.z;

        float x_out = alpha * (prev_x_out + x_in - prev_x_in);
        float y_out = alpha * (prev_y_out + y_in - prev_y_in);
        float z_out = alpha * (prev_z_out + z_in - prev_z_in);

        // Update previous values
        prev_x_in = x_in; prev_x_out = x_out;
        prev_y_in = y_in; prev_y_out = y_out;
        prev_z_in = z_in; prev_z_out = z_out;

        // Replace data with the filtered values
        data.x = x_out;
        data.y = y_out;
        data.z = z_out;

        // Apply moving average filtering
        window_gx[window_index] = data.x;
        window_gy[window_index] = data.y;
        window_gz[window_index] = data.z;

        float avg_gx = 0.0f, avg_gy = 0.0f, avg_gz = 0.0f;
        for (int i = 0; i < WINDOW_SIZE; i++) {
            avg_gx += window_gx[i];
            avg_gy += window_gy[i];
            avg_gz += window_gz[i];
        }
        avg_gx /= WINDOW_SIZE;
        avg_gy /= WINDOW_SIZE;
        avg_gz /= WINDOW_SIZE;

        window_index = (window_index + 1) % WINDOW_SIZE;

        float use_x = avg_gx;
        float use_y = avg_gy;
        float use_z = avg_gz;

        // Record gestures based on current state
        switch (currentState) {
            case RECORD_GESTURE_SETUP:
                if (gestureDataIndex < MAX_GESTURE_SAMPLES) {
                    recordedGesture[gestureDataIndex].x = use_x;
                    recordedGesture[gestureDataIndex].y = use_y;
                    recordedGesture[gestureDataIndex].z = use_z;
                    gestureDataIndex++;
                    printf("[%f, %f, %f],\n", gx, gy, gz);

                    char indexBuffer[30];
                    sprintf(indexBuffer, "Recording... %d", gestureDataIndex);
                    lcd.SetTextColor(LCD_COLOR_WHITE);
                    lcd.SetBackColor(LCD_COLOR_BLACK);
                    lcd.DisplayStringAt(0, LINE(7), (uint8_t *)indexBuffer, CENTER_MODE);

                    if (gestureDataIndex == MAX_GESTURE_SAMPLES) {
                        currentState = LOCKED;
                        updateDisplayAndLEDs(currentState);
                    }
                }
                break;

            case RECORD_GESTURE_UNLOCK:
                if (currentDataIndex < gestureDataIndex && gestureDataIndex > 0) {
                    currentGesture[currentDataIndex].x = use_x;
                    currentGesture[currentDataIndex].y = use_y;
                    currentGesture[currentDataIndex].z = use_z;
                    currentDataIndex++;
                    printf("[%f, %f, %f],\n", gx, gy, gz);

                    char indexBuffer1[30];
                    sprintf(indexBuffer1, "Recording... %d", currentDataIndex);
                    lcd.SetTextColor(LCD_COLOR_WHITE);
                    lcd.SetBackColor(LCD_COLOR_BLACK);
                    lcd.DisplayStringAt(0, LINE(7), (uint8_t *)indexBuffer1, CENTER_MODE);

                    if (currentDataIndex == gestureDataIndex) {
                        std::vector<std::vector<float>> recordedVec = convertToVector(recordedGesture, gestureDataIndex);
                        std::vector<std::vector<float>> currentVec = convertToVector(currentGesture, currentDataIndex);

                        float difference = dtw_distance(recordedVec, currentVec);
                        printf("DTW distance: %f\n", difference);
                        for (int i = 0; i < MAX_GESTURE_SAMPLES; i++){
                            printf("[%f, %f, %f],\n", recordedGesture[i].x, recordedGesture[i].y, recordedGesture[i].z);
                        }
                        for (int i = 0; i < MAX_GESTURE_SAMPLES; i++){
                            printf("[%f, %f, %f],\n", currentGesture[i].x, currentGesture[i].y, currentGesture[i].z);
                        }
                        if (difference < MATCH_THRESHOLD) {
                            currentState = UNLOCKED;
                        } else {
                            currentState = UNLOCK_FAILED;
                            currentDataIndex = 0;
                        }
                        updateDisplayAndLEDs(currentState);
                    }
                }
                break;

            case LOCKED:
            case ENTRY:
            case UNLOCK_FAILED:
            case UNLOCKED:
                // No action required in these states
                break;
        }

        thread_sleep_for(100);
    }
}
