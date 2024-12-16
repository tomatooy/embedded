#include "mbed.h"
#include "arm_math.h"
#include "drivers/LCD_DISCO_F429ZI.h"

#define CTRL_REG1 0x20
#define CTRL_REG1_CONFIG 0b01'10'1'1'1'1
#define CTRL_REG4 0x23
#define CTRL_REG4_CONFIG 0b0'0'01'0'00'0
#define CTRL_REG3 0x22
#define CTRL_REG3_CONFIG 0b0'0'0'0'1'000

#define MAX_GESTURE_SAMPLES 30
#define MATCH_THRESHOLD 15.0f

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

// Gesture data structure
struct GyroData {
    float x, y, z;
};

GyroData recordedGesture[MAX_GESTURE_SAMPLES];
int gestureDataIndex = 0;

GyroData currentGesture[MAX_GESTURE_SAMPLES];
int currentDataIndex = 0;

// Define the window size for the moving average
#define WINDOW_SIZE 5
float window_gx[WINDOW_SIZE] = {0}, window_gy[WINDOW_SIZE] = {0}, window_gz[WINDOW_SIZE] = {0};
int window_index = 0;

// DTW distance computation
inline float pointDistance(const GyroData &a, const GyroData &b) {
    float dx = a.x - b.x;
    float dy = a.y - b.y;
    float dz = a.z - b.z;
    return sqrtf(dx*dx + dy*dy + dz*dz);
}

float computeGestureDifferenceDTW(GyroData* gesture1, int len1, GyroData* gesture2, int len2) {
    static float dtw[MAX_GESTURE_SAMPLES+1][MAX_GESTURE_SAMPLES+1];

    for (int i = 0; i <= len1; i++) {
        for (int j = 0; j <= len2; j++) {
            dtw[i][j] = INFINITY;
        }
    }
    dtw[0][0] = 0.0f;

    for (int i = 1; i <= len1; i++) {
        for (int j = 1; j <= len2; j++) {
            float cost = pointDistance(gesture1[i-1], gesture2[j-1]);
            float minPrev = dtw[i-1][j];
            if (dtw[i][j-1] < minPrev) minPrev = dtw[i][j-1];
            if (dtw[i-1][j-1] < minPrev) minPrev = dtw[i-1][j-1];
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
            // If desired, handle re-lock or do nothing here
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

    int16_t raw_gx_int, raw_gy_int, raw_gz_int;
    float gx, gy, gz;

    while (1) {
        flags.wait_all(DATA_READY_FLAG);
        flags.clear(DATA_READY_FLAG);

        write_buf[0] = OUT_X_L | 0x80 | 0x40;
        spi.transfer(write_buf, 7, read_buf, 7, spi_cb);
        flags.wait_all(SPI_FLAG);

        raw_gx_int = (((uint16_t)read_buf[2]) << 8) | read_buf[1];
        raw_gy_int = (((uint16_t)read_buf[4]) << 8) | read_buf[3];
        raw_gz_int = (((uint16_t)read_buf[6]) << 8) | read_buf[5];

        gx = raw_gx_int * SCALING_FACTOR;
        gy = raw_gy_int * SCALING_FACTOR;
        gz = raw_gz_int * SCALING_FACTOR;

        // Moving Average Filter directly
        window_gx[window_index] = gx;
        window_gy[window_index] = gy;
        window_gz[window_index] = gz;

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

        // Use the averaged values
        float use_x = avg_gx;
        float use_y = avg_gy;
        float use_z = avg_gz;

        switch (currentState) {
            case RECORD_GESTURE_SETUP:
                if (gestureDataIndex < MAX_GESTURE_SAMPLES) {
                    recordedGesture[gestureDataIndex].x = use_x;
                    recordedGesture[gestureDataIndex].y = use_y;
                    recordedGesture[gestureDataIndex].z = use_z;
                    gestureDataIndex++;

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

                    char indexBuffer1[30];
                    sprintf(indexBuffer1, "Recording... %d", currentDataIndex);
                    lcd.SetTextColor(LCD_COLOR_WHITE);
                    lcd.SetBackColor(LCD_COLOR_BLACK);
                    lcd.DisplayStringAt(0, LINE(7), (uint8_t *)indexBuffer1, CENTER_MODE);

                    if (currentDataIndex == gestureDataIndex) {
                        float difference = computeGestureDifferenceDTW(recordedGesture, gestureDataIndex, currentGesture, currentDataIndex);
                        printf("DTW distance: %f\n", difference);
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
                // No action required
                break;
        }

        thread_sleep_for(100);
    }
}
