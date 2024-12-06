
// =================================================
// * Recitation 6: Final Recitation - LCD Screen *
// =================================================

// TODOs:
// [1] Recap on Polling, Interrupts, Debouncing, Multithreading, etc.
// [2] LCD Screen - Reading the datasheet and writing the code to display text and shapes on to the Screen!
// [3] Extra --> Graphs on Screen!, Semaphores, Introduction to DSP!
// [4] Embedded Challenge Tips

#include "mbed.h"
#include "arm_math.h"
#include "drivers/LCD_DISCO_F429ZI.h"

#define WINDOW_SIZE 10 
#define CTRL_REG1 0x20
#define CTRL_REG1_CONFIG 0b01'10'1'1'1'1
#define CTRL_REG4 0x23
#define CTRL_REG4_CONFIG 0b0'0'01'0'00'0
#define CTRL_REG3 0x22
#define CTRL_REG3_CONFIG 0b0'0'0'0'1'000

#define MAX_GESTURE_SAMPLES 30
#define MATCH_THRESHOLD 10.0f

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

#define SCALING_FACTOR (17.5f * 0.017453292519943295769236907684886f / 1000.0f)
#define FILTER_COEFFICIENT 0.1f // Adjust as needed

#define BUFFER_SIZE 256 
#define FFT_LENGTH BUFFER_SIZE 
#define SAMPLE_RATE 100
#define USER_BUTTON PA_0

DigitalOut ledGreen(LED1);  
DigitalOut ledRed(LED2);    
InterruptIn userButton(USER_BUTTON);  
LCD_DISCO_F429ZI lcd;

EventFlags flags;

struct GyroData {
    float x, y, z;
};

GyroData recordedGesture[MAX_GESTURE_SAMPLES];
int gestureDataIndex = 0;

GyroData currentGesture[MAX_GESTURE_SAMPLES];
int currentDataIndex = 0;

void spi_cb(int event) {
    flags.set(SPI_FLAG);
}

void data_cb() {
    flags.set(DATA_READY_FLAG);
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
    // State transitions based on button press
    switch (currentState) {
        case ENTRY:
            // Start recording lock gesture
            gestureDataIndex = 0;
            currentState = RECORD_GESTURE_SETUP;
            updateDisplayAndLEDs(currentState);
            break;

        case LOCKED:
            // Start recording unlock gesture
            currentDataIndex = 0;
            currentState = RECORD_GESTURE_UNLOCK;
            updateDisplayAndLEDs(currentState);
            break;

        case UNLOCK_FAILED:
            // Try unlock gesture again
            currentDataIndex = 0;
            currentState = RECORD_GESTURE_UNLOCK;
            updateDisplayAndLEDs(currentState);
            break;

        case UNLOCKED:
            // Optional: If you want to allow re-lock, or do nothing.
            // For now, do nothing or reset if desired.
            break;

        case RECORD_GESTURE_SETUP:
        case RECORD_GESTURE_UNLOCK:
            // These states do not transition on button press directly
            break; 
    }
}

int main() {
    // SPI initialization
    SPI spi(PF_9, PF_8, PF_7, PC_1, use_gpio_ssel);
    uint8_t write_buf[32], read_buf[32];

    InterruptIn int2(PA_2, PullDown);
    int2.rise(&data_cb);

    spi.format(8, 3);
    spi.frequency(1'000'000);

    // Write to control registers
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
    if (!(flags.get() & DATA_READY_FLAG) && (int2.read() == 1)) {
        flags.set(DATA_READY_FLAG);
    }

    // Initialize LCD and LEDs
    lcd.Clear(LCD_COLOR_BLACK);
    lcd.SetTextColor(LCD_COLOR_WHITE);
    lcd.SetBackColor(LCD_COLOR_BLACK);
    lcd.SetFont(&Font12);

    // Initial State
    currentState = ENTRY;
    updateDisplayAndLEDs(currentState);

    userButton.rise(&buttonPressed);

    int16_t raw_gx, raw_gy, raw_gz;
    float gx, gy, gz;

    while (1) {
        flags.wait_all(DATA_READY_FLAG);
        flags.clear(DATA_READY_FLAG);

        write_buf[0] = OUT_X_L | 0x80 | 0x40;
        spi.transfer(write_buf, 7, read_buf, 7, spi_cb);
        flags.wait_all(SPI_FLAG);

        raw_gx = (((uint16_t)read_buf[2]) << 8) | read_buf[1];
        raw_gy = (((uint16_t)read_buf[4]) << 8) | read_buf[3];
        raw_gz = (((uint16_t)read_buf[6]) << 8) | read_buf[5];

        gx = raw_gx * SCALING_FACTOR;
        gy = raw_gy * SCALING_FACTOR;
        gz = raw_gz * SCALING_FACTOR;

        switch (currentState) {
            case RECORD_GESTURE_SETUP:
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

                    // Once we have captured enough samples, transition to LOCKED state
                    if (gestureDataIndex == MAX_GESTURE_SAMPLES) {
                        currentState = LOCKED;
                        updateDisplayAndLEDs(currentState);
                    }
                }
                break;

            case RECORD_GESTURE_UNLOCK:
                if (currentDataIndex < gestureDataIndex && gestureDataIndex > 0) {
                    currentGesture[currentDataIndex].x = gx;
                    currentGesture[currentDataIndex].y = gy;
                    currentGesture[currentDataIndex].z = gz;
                    currentDataIndex++;

                    char indexBuffer1[30];
                    sprintf(indexBuffer1, "Recording... %d", currentDataIndex);
                    lcd.SetTextColor(LCD_COLOR_WHITE);
                    lcd.SetBackColor(LCD_COLOR_BLACK);
                    lcd.DisplayStringAt(0, LINE(7), (uint8_t *)indexBuffer1, CENTER_MODE);

                    if (currentDataIndex == gestureDataIndex) {
                        // Compare gestures
                        float difference = computeGestureDifference(recordedGesture, currentGesture, gestureDataIndex);
                        if (difference < MATCH_THRESHOLD) {
                            currentState = UNLOCKED;
                        } else {
                            currentState = UNLOCK_FAILED;
                        }
                        updateDisplayAndLEDs(currentState);
                    }

                }
                break;

            case LOCKED:
            case ENTRY:
            case UNLOCK_FAILED:
            case UNLOCKED:
                // Do nothing special, wait for button press
                break;
        }
        thread_sleep_for(100);
    }
}