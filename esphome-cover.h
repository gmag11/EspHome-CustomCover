#include "esphome.h"
#ifdef ESP32
#include <esp_log.h>
#endif

#include <functional>

constexpr auto COVER_TAG = "Esphome cover";
constexpr auto MOTOR_UP_PIN = 19;
constexpr auto MOTOR_DOWN_PIN = 21;
// constexpr auto UP_BUTTON_PIN = 5;
// constexpr auto DOWN_BUTTON_PIN = 4;
constexpr auto ROLLING_TIME = 5000;
constexpr auto NOTIF_PERIOD_RATIO = 5;
constexpr auto KEEP_ALIVE_PERIOD_RATIO = 4;
constexpr auto ON_STATE_DEFAULT = HIGH;

struct blindControlerHw_t {
    int motorUpPin = MOTOR_UP_PIN;
    int motorDownPin = MOTOR_DOWN_PIN;
    // uint8_t upButton = UP_BUTTON_PIN;
    // uint8_t downButton = DOWN_BUTTON_PIN;
    clock_t fullTravellingTime = ROLLING_TIME;
    clock_t notifPeriod = fullTravellingTime / NOTIF_PERIOD_RATIO;
    clock_t keepAlivePeriod = fullTravellingTime * KEEP_ALIVE_PERIOD_RATIO;
    int ON_STATE = ON_STATE_DEFAULT;
};

typedef enum {
    rollingUp = 1,
    rollingDown = 2,
    stopped = 4,
    error = 0
} blindState_t;

const uint8_t pos_len_lut[] = { 0,  0,  0,  0,  0,  0,  0,  1,  1,  1, // 0  -  9
                                 1,  2,  2,  2,  2,  3,  3,  4,  4,  4, // 10 - 19
                                 5,  5,  6,  6,  7,  8,  8,  9,  9, 10, // 20 - 29
                                11, 12, 13, 13, 14, 15, 16, 17, 17, 18, // 30 - 39
                                19, 20, 21, 22, 23, 24, 25, 26, 27, 28, // 40 - 49
                                29, 30, 32, 33, 34, 35, 36, 38, 39, 40, // 50 - 59
                                41, 43, 44, 45, 46, 48, 49, 51, 52, 53, // 60 - 69
                                55, 56, 58, 59, 60, 62, 63, 65, 66, 67, // 70 - 79
                                69, 71, 72, 73, 75, 77, 78, 80, 81, 83, // 80 - 89
                                84, 86, 87, 89, 90, 92, 94, 95, 97, 98, // 90 - 99
                                100 };									// 100

class EsphomeCover : public Component, public Cover, public CustomAPIDevice {
protected:
    blindControlerHw_t config;
    //int positionRequest;
    int currentPosition;
    //int currentAnglePosition;
    int originalPosition;
    int targetPosition;
    //int OFF_STATE;
    blindState_t blindState = stopped;
    time_t travellingTime;
    time_t blindStartedMoving;
    bool movingUp = false;
    bool movingDown = false;

    void configurePins () {
        pinMode (config.motorUpPin, OUTPUT);
        pinMode (config.motorDownPin, OUTPUT);
    }

    float angleToPosition (int angle) {
        if (angle >= 0 && angle <= 100) {
            return pos_len_lut[angle];
        }
        return -1;
    }
    
    int positionToAngle (int position) {
        int pos = position;
        
        if (pos < 0) {
            return -1;
        }
        if (pos == 0) {
            return 0;
        }
        if (pos >= 100) {
            return 100;
        }
        for (int i = 0; i < sizeof (pos_len_lut); i++) {
            if (pos_len_lut[i] >= pos) {
                return i + 1;
            }
        }
        return -1;
    }
    
    void configureFullRollup () {
        ESP_LOGD (COVER_TAG, "Configure full roll up");
        targetPosition = 100;
        travellingTime = config.fullTravellingTime * 1.1;
        current_operation = COVER_OPERATION_OPENING;
        blindState = rollingUp;
        ESP_LOGD (COVER_TAG, "--- STATE: Rolling up");
    }

    void configureFullRolldown () {
        ESP_LOGD (COVER_TAG, "Configure full roll down");
        targetPosition = 0;
        travellingTime = config.fullTravellingTime * 1.1;
        current_operation = COVER_OPERATION_CLOSING;
        blindState = rollingDown;
        ESP_LOGD (COVER_TAG, "--- STATE: Rolling down");
    }

    bool process_goto_position (float pos_requested) {
        targetPosition = angleToPosition (pos_requested * 100);
        ESP_LOGD (COVER_TAG, "process_goto_position. Requested pos: %d, Requested angle: %d", (int)(pos_requested * 100), targetPosition);
        if (targetPosition <= 0) {
            ESP_LOGD (COVER_TAG, "Full Rolldown");
            configureFullRolldown ();
        }
        if (targetPosition >= 100) {
            ESP_LOGD (COVER_TAG, "Full Rollup");
            configureFullRollup ();
        }
        bool result = gotoTargetPosition ();
        return result;
    }

    const char* stateToStr (int state) {
        switch (state) {
        case rollingUp:
            return "Rolling up";
        case rollingDown:
            return "Rolling down";
        case stopped:
            return "Stopped";
        default:
            return "Error";
        }
    }

    void processBlindEvent (blindState_t state, int8_t pos) {
        ESP_LOGD (COVER_TAG, "State: %s. Position %d", stateToStr (state), pos);

        this->position = (float)pos / 100.0;
        
        switch (state) {
        case rollingUp:
            this->current_operation = COVER_OPERATION_OPENING;
            break;
        case rollingDown:
            this->current_operation = COVER_OPERATION_CLOSING;
            break;
        default:
            this->current_operation = COVER_OPERATION_IDLE;
        }

        ESP_LOGI (COVER_TAG, "Publish state. Operation: %d, Position %f", this->current_operation, this->position);
        this->publish_state ();
    }
    
    bool gotoTargetPosition () {
        //currentPosition = position * 100;
        ESP_LOGI (COVER_TAG, "Go to position %d. Current = %d", targetPosition, currentPosition);

        if (targetPosition <= 0) {
            ESP_LOGD (COVER_TAG, "Force gotoPosition full roll down");
            targetPosition = 0;
        } else if (targetPosition >= 100) {
            ESP_LOGD (COVER_TAG, "Force gotoPosition full roll up");
            targetPosition = 100;
        } 
        stop ();
        ESP_LOGD (COVER_TAG, "Current position: %d, Position requested: %d", currentPosition, targetPosition);
        if (targetPosition > currentPosition) {
            blindState = rollingUp;
            ESP_LOGD (COVER_TAG, "--- STATE: Rolling up from %d to  %d", positionToAngle (currentPosition), positionToAngle (targetPosition));
            if (targetPosition < 100) {
                travellingTime = movementToTime (targetPosition - currentPosition);
            } else {
                configureFullRollup ();
            }
        } else if (targetPosition < currentPosition) {
            blindState = rollingDown;
            ESP_LOGD (COVER_TAG, "--- STATE: Rolling down from %d to %d", positionToAngle (currentPosition), positionToAngle (targetPosition));
            if (targetPosition > 0) {
                travellingTime = movementToTime (currentPosition - targetPosition);
            } else {
                configureFullRolldown ();
            }
        } else {
            ESP_LOGD (COVER_TAG, "Requested = Current position");
            blindState = stopped;
            processBlindEvent (blindState, positionToAngle (currentPosition));
        }
        return true;
    }

    time_t movementToTime (int8_t movement) {
        clock_t calculatedTime = movement * config.fullTravellingTime / 100;
        ESP_LOGD (COVER_TAG, "config.fullTravellingTime = %lu", config.fullTravellingTime);
        ESP_LOGD (COVER_TAG, "Calculated time: %lu", calculatedTime);

        if (calculatedTime >= config.fullTravellingTime) {
            calculatedTime = config.fullTravellingTime * 1.1;
        }
        ESP_LOGD (COVER_TAG, "Desired movement: %d. Calculated time: %lu", movement, calculatedTime);
        return calculatedTime;
    }

    int8_t timeToPos (time_t movementTime) {
        return movementTime * 100 / config.fullTravellingTime;
    }

    void sendPosition () {
        static clock_t lastShowedPos;
        switch (blindState) {
        case rollingUp:
        case rollingDown:
            if (millis () - lastShowedPos > config.notifPeriod) {
                lastShowedPos = millis ();
                ESP_LOGD (COVER_TAG, "Position: %d", currentPosition);
                processBlindEvent (blindState, positionToAngle (currentPosition));
            }
            break;
        case stopped:
            if (millis () - lastShowedPos > config.keepAlivePeriod) {
                lastShowedPos = millis ();
                ESP_LOGD (COVER_TAG, "Position: %d", currentPosition);
                processBlindEvent (blindState, positionToAngle (currentPosition));
            }
            break;
        case error:
            if (millis () - lastShowedPos > config.keepAlivePeriod) {
                lastShowedPos = millis ();
                ESP_LOGD (COVER_TAG, "Blind in error status");
                ESP_LOGD (COVER_TAG, "Position: %d", currentPosition);
                processBlindEvent (blindState, positionToAngle (currentPosition));
            }
            break;
        }
    }

    void stop () {
        digitalWrite (config.motorUpPin, !config.ON_STATE);
        digitalWrite (config.motorDownPin, !config.ON_STATE);
        movingDown = false;
        movingUp = false;
    }

    void rollup () {
        time_t timeMoving;

        if (!movingUp) {
            ESP_LOGI (COVER_TAG, "Started roll up. Position Request %d. Original position %d", targetPosition, currentPosition);
            movingDown = false;
            movingUp = true;
            blindStartedMoving = millis ();
            originalPosition = currentPosition;
            //targetPosition = positionRequest;
            digitalWrite (config.motorDownPin, !config.ON_STATE);
            digitalWrite (config.motorUpPin, config.ON_STATE);
        }
        timeMoving = millis () - blindStartedMoving;
        if (currentPosition != -1 && currentPosition < 100) {
            currentPosition = originalPosition + timeToPos (timeMoving);
            if (currentPosition > 100) {
                currentPosition = 100;
            }
            //currentAnglePosition = positionToAngle (currentPosition);
        }
        if (travellingTime > 0 && timeMoving > travellingTime) {
            ESP_LOGI (COVER_TAG, "Stopped roll up");
            if (targetPosition == 100) {
                currentPosition = 100; // Calibrate position
            }
            blindState = stopped;
            ESP_LOGI (COVER_TAG, "--- STATE: Stopped");
            processBlindEvent (blindState, positionToAngle (currentPosition));
        } else {
            if (timeMoving > config.fullTravellingTime * 1.1) {
                blindState = stopped;
                currentPosition = 100;
                ESP_LOGI (COVER_TAG, "--- STATE: Stopped");
                processBlindEvent (blindState, positionToAngle (currentPosition));
            }
        }
    }

    void rolldown () {
        time_t timeMoving;

        if (!movingDown) {
            ESP_LOGI (COVER_TAG, "Started roll down. Position Request %d. Original position %d", targetPosition, currentPosition);
            movingUp = false;
            movingDown = true;
            blindStartedMoving = millis ();
            originalPosition = currentPosition;
            // targetPosition = positionRequest;
            digitalWrite (config.motorUpPin, !config.ON_STATE);
            digitalWrite (config.motorDownPin, config.ON_STATE);
        }
        timeMoving = millis () - blindStartedMoving;
        if (currentPosition != -1 && currentPosition > 0) {
            currentPosition = originalPosition - timeToPos (timeMoving);
            if (currentPosition < 0) {
                currentPosition = 0;
            }
        }
        if (travellingTime > 0 && timeMoving > travellingTime) {
            ESP_LOGI (COVER_TAG, "Stopped roll down");
            if (targetPosition == 0) {
                currentPosition = 0; // Calibrate position
            }
            blindState = stopped;
            ESP_LOGI (COVER_TAG, "--- STATE: Stopped");
            processBlindEvent (blindState, positionToAngle (currentPosition));
        } else {
            if (timeMoving > config.fullTravellingTime * 1.1) {
                blindState = stopped;
                currentPosition = 0;
                ESP_LOGI (COVER_TAG, "--- STATE: Stopped");
                processBlindEvent (blindState, positionToAngle (currentPosition));
            }
        }

    }

public:

    
    void setup () override {
        ESP_LOGI (COVER_TAG, "Called setup()");
        config.fullTravellingTime = fullTravellingTime_config->value () * 1000;
        config.motorDownPin = downMotorPin->value ();
        config.motorUpPin = upMotorPin->value ();

        // OFF_STATE = !config.ON_STATE;
        ESP_LOGI (COVER_TAG, "==== Blind Controller Configuration ====");
        ESP_LOGI (COVER_TAG, "Up Relay pin: %d", config.motorUpPin);
        ESP_LOGI (COVER_TAG, "Down Relay pin: %d", config.motorDownPin);
        // DEBUG_INFO ("Up Button pin: %d", config.upButton);
        // DEBUG_INFO ("Down Button pin: %d", config.downButton);
        ESP_LOGI (COVER_TAG, "Full travelling time: %lu ms", config.fullTravellingTime);
        ESP_LOGI (COVER_TAG, "Notification period time: %lu ms", config.notifPeriod);
        ESP_LOGI (COVER_TAG, "Keep Alive period time: %lu ms", config.keepAlivePeriod);
        ESP_LOGI (COVER_TAG, "On Relay state: %s", config.ON_STATE ? "HIGH" : "LOW");

        configurePins ();

        register_service (&EsphomeCover::on_calibrate, "calibration", { "action" });

        currentPosition = angleToPosition (this->position * 100);
        ESP_LOGI (COVER_TAG, "Position %f", this->position);
    }

    void execute_key_sequence (const int* sequence, int size) {
        const int space = 250;
        int i;
        ESP_LOGD (COVER_TAG, "execute_calibration_sequence. Size: %d", size);
        for (i = 0; i < (size - 1); i++) {
            ESP_LOGD (COVER_TAG, "%d: %d", i, sequence[i]);
            digitalWrite (sequence[i], config.ON_STATE);
            delay (space);
            digitalWrite (sequence[i], !config.ON_STATE);
            delay (space);
        }
        ESP_LOGD (COVER_TAG, "%d: %d Last step", i, sequence[i]);
        digitalWrite (sequence[i], config.ON_STATE);
        delay (2500);
        ESP_LOGD (COVER_TAG, "Sequence end");
        digitalWrite (sequence[i], !config.ON_STATE);
        delay (space);
    }

    void on_calibrate (int action) {
        ESP_LOGD (COVER_TAG, "Calibration called. Parameter %d", action);
        const int up = config.motorUpPin;
        const int down = config.motorDownPin;
        const int cancel_calibration[] = { up, up, down, up, up, down };
        const int up_calibration[] = { up, up, up};
        const int down_calibration[] = { down ,down ,down };

        switch (action) {
        case 0:
            ESP_LOGI (COVER_TAG, "Reset calibration");
            execute_key_sequence (cancel_calibration, sizeof (cancel_calibration) / sizeof (int));
            break;
        case 1:
            ESP_LOGI (COVER_TAG, "Up calibration");
            execute_key_sequence (up_calibration, sizeof (up_calibration) / sizeof (int));
            break;
        case 2:
            ESP_LOGI (COVER_TAG, "Down calibration");
            execute_key_sequence (down_calibration, sizeof (down_calibration) / sizeof (int));
            break;
        }
    }
    
    void loop () override {
        switch (blindState) {
        case stopped:
            stop ();
            break;
        case rollingUp:
            rollup ();
            break;
        case rollingDown:
            rolldown ();
            break;
        default:
            break;
        }

        sendPosition ();
    }
    
    CoverTraits get_traits () override {
        auto traits = CoverTraits ();
        traits.set_is_assumed_state (false);
        traits.set_supports_position (true);
        traits.set_supports_tilt (false);
        return traits;
    }

    void control (const CoverCall& call) override {
      // This will be called every time the user requests a state change.
        if (call.get_position ().has_value ()) {
            float pos = *call.get_position ();
            ESP_LOGI (COVER_TAG, "Called goto position %f", pos);
            bool error = process_goto_position (pos);
        }
        if (call.get_stop ()) {
          // User requested cover stop
            ESP_LOGI (COVER_TAG, "Called stop");
            blindState = stopped;
            processBlindEvent (blindState, positionToAngle (currentPosition));
        }
    }
};

