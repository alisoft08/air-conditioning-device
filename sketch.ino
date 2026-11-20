/**
 * @file sketch.ino
 * @brief EcoFlow MicroGrid Node M1 prototype.
 * @author Alison Arrieta
 *
 * Event-driven application based on Modest IoT Nano Framework.
 * Telemetry is recorded as JSON in the serial console.
 */

#include <Arduino.h>
#include <ArduinoJson.h>
#include <ModestIoT.h>
#include <HX711.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>

namespace Config {
    constexpr char DEVELOPER_NAME[] = "Alison Arrieta";
    constexpr char COMPANY[] = "EcoFlow";
    constexpr char WEBSITE[] = "https://www.ecoflow.com/global";
    constexpr char DEVICE[] = "EcoFlow MicroGrid Node M1";

    constexpr int TANK_TRIGGER = 18;
    constexpr int TANK_ECHO = 19;
    constexpr int PERSON_TRIGGER = 26;
    constexpr int PERSON_ECHO = 27;
    constexpr int SCALE_DATA = 4;
    constexpr int SCALE_CLOCK = 5;
    constexpr int SERVO_PIN = 13;
    constexpr int SDA_PIN = 21;
    constexpr int SCL_PIN = 22;

    constexpr uint8_t LCD_ADDRESS = 0x27;
    constexpr int LCD_COLUMNS = 16;
    constexpr int LCD_ROWS = 2;
    constexpr int LINE_BUFFER_SIZE = LCD_COLUMNS + 1;

    constexpr int FULL_DISTANCE_CM = 5;
    constexpr int EMPTY_DISTANCE_CM = 35;
    constexpr int TANK_CAPACITY_ML = 3000;
    constexpr int PERSON_LIMIT_CM = 15;
    constexpr int START_WEIGHT_G = 200;
    constexpr int STOP_WEIGHT_G = 800;
    constexpr int MAX_WEIGHT_G = 1000;
    constexpr int PERCENT_MAX = 100;
    constexpr int INVALID = -1;
    constexpr int CLOSED_ANGLE = 0;

    constexpr uint32_t SAMPLE_MS = 200;
    constexpr uint32_t ENGINE_TICK_MS = 50;
    constexpr uint32_t SCALE_TIMEOUT_MS = 1500;
    constexpr unsigned long LOOP_DELAY_MS = 60000;
    constexpr unsigned long SERIAL_BAUD = 115200;
    constexpr uint8_t QUEUE_SIZE = 8;

    constexpr unsigned int TRIGGER_LOW_US = 2;
    constexpr unsigned int TRIGGER_HIGH_US = 10;
    constexpr unsigned long ECHO_TIMEOUT_US = 30000;
    constexpr float MICROSECONDS_PER_CM = 58.0f;
    constexpr float ROUNDING = 0.5f;

    // Wokwi 5 kg load cell: 2100 raw units represent 5000 grams.
    constexpr float SCALE_FACTOR = 2100.0f / 5000.0f;

    constexpr char LEVEL_FORMAT[] = "Tank:%3d%% %s";
    constexpr char WEIGHT_FORMAT[] = "Weight:%5dg";
    constexpr char PAD_FORMAT[] = "%-*s";
    constexpr char EMPTY_TEXT[] = "Tank: Empty";
    constexpr char UNKNOWN_LEVEL[] = "Tank: --";
    constexpr char UNKNOWN_WEIGHT[] = "Weight: --";

    constexpr const char* MODE_NAMES[] = {
        "Starting", "Ready", "Running", "Empty",
        "TargetReached", "SensorFault", "Overload"
    };

    constexpr const char* MODE_LABELS[] = {
        "INIT", "WAIT", "RUN", "EMPTY", "FULL", "ERROR", "OVER"
    };
}

namespace Keys {
    constexpr char DEVICE[] = "device";
    constexpr char TIME[] = "uptime_ms";
    constexpr char SOURCE[] = "source_pin";
    constexpr char MODE[] = "mode";
    constexpr char PREVIOUS[] = "previous_mode";
    constexpr char LEVEL[] = "tank_percent";
    constexpr char VOLUME[] = "tank_ml";
    constexpr char WEIGHT[] = "weight_g";
    constexpr char DISTANCE[] = "person_cm";
    constexpr char NEAR[] = "person_near";
    constexpr char OPEN[] = "gate_open";
    constexpr char PENDING[] = "cycle_pending";
    constexpr char STARTED[] = "cycles_started";
    constexpr char COMPLETED[] = "cycles_completed";
    constexpr char PROXIMITY[] = "proximity_entries";
    constexpr char TRANSITIONS[] = "mode_transitions";
}

/** @brief Device operating modes. */
enum class Mode : uint8_t {
    Starting, Ready, Running, Empty,
    TargetReached, SensorFault, Overload
};

/** @brief HC-SR04 distance sensor integrated with the framework. */
class DistanceInput : public Sensor {
private:
    int echoPin;
    int distance = Config::INVALID;

protected:
    /** @brief Reads the echo pulse and emits a measurement event. */
    void processEvent(Event& event) override {
        if (event.identifier != MEASURE_DATA_REQUESTED_EVENT_IDENTIFIER) {
            event.identifier = Config::INVALID;
            return;
        }

        digitalWrite(pin, LOW);
        delayMicroseconds(Config::TRIGGER_LOW_US);
        digitalWrite(pin, HIGH);
        delayMicroseconds(Config::TRIGGER_HIGH_US);
        digitalWrite(pin, LOW);

        const unsigned long duration =
            pulseIn(echoPin, HIGH, Config::ECHO_TIMEOUT_US);

        distance = duration == 0
            ? Config::INVALID
            : static_cast<int>(
                duration / Config::MICROSECONDS_PER_CM + Config::ROUNDING
            );

        hasRead = true;
        event = Event(DATA_READ_EVENT_IDENTIFIER, pin);
    }

    uint32_t getDefaultSamplingIntervalInMilliseconds() const override {
        return Config::SAMPLE_MS;
    }

    uint32_t getMinimalSamplingIntervalInMilliseconds() const override {
        return Config::SAMPLE_MS;
    }

public:
    bool hasRead = false; ///< Indicates that a measurement was attempted.

    /** @brief Configures the ultrasonic sensor pins. */
    DistanceInput(int trigger, int echo, EventHandler* parent)
        : Sensor(trigger, parent), echoPin(echo) {
        pinMode(pin, OUTPUT);
        pinMode(echoPin, INPUT);
        digitalWrite(pin, LOW);
    }

    /** @brief Returns centimeters, or -1 when no echo was received. */
    int getDistance() const {
        return distance;
    }
};

/** @brief HX711 weight sensor integrated with the framework. */
class WeightInput : public Sensor {
private:
    HX711 scale;
    int grams = Config::INVALID;
    unsigned long lastRead = 0;

protected:
    /** @brief Reads an available conversion and reports missing data. */
    void processEvent(Event& event) override {
        if (event.identifier != MEASURE_DATA_REQUESTED_EVENT_IDENTIFIER) {
            event.identifier = Config::INVALID;
            return;
        }

        if (scale.is_ready()) {
            const float measured = scale.get_units(1);
            grams = measured < 0.0f
                ? Config::INVALID
                : static_cast<int>(measured + Config::ROUNDING);
            lastRead = millis();
        }
        else if (millis() - lastRead >= Config::SCALE_TIMEOUT_MS) {
            grams = Config::INVALID;
        }
        else {
            event.identifier = Config::INVALID;
            return;
        }

        hasRead = true;
        event = Event(DATA_READ_EVENT_IDENTIFIER, pin);
    }

    uint32_t getDefaultSamplingIntervalInMilliseconds() const override {
        return Config::SAMPLE_MS;
    }

    uint32_t getMinimalSamplingIntervalInMilliseconds() const override {
        return Config::SAMPLE_MS;
    }

public:
    bool hasRead = false; ///< Indicates that data or a timeout was received.

    /** @brief Applies the calibration for Wokwi's 5 kg load cell. */
    WeightInput(int dataPin, int clockPin, EventHandler* parent)
        : Sensor(dataPin, parent) {
        scale.begin(dataPin, clockPin);
        scale.set_scale(Config::SCALE_FACTOR);
        scale.set_offset(0);
        lastRead = millis();
    }

    /** @brief Returns grams, or -1 when the sensor is unavailable. */
    int getGrams() const {
        return grams;
    }
};

/** @brief Stores device state and serializes operating metrics. */
class MicroGridTelemetry : public TelemetryPackage {
public:
    int level = Config::INVALID;
    int volume = Config::INVALID;
    int weight = Config::INVALID;
    int distance = Config::INVALID;
    int sourcePin = Config::INVALID;

    bool near = false;
    bool open = false;
    bool pending = false;

    unsigned long started = 0;
    unsigned long completed = 0;
    unsigned long proximityEntries = 0;
    unsigned long transitions = 0;

    Mode mode = Mode::Starting;
    Mode previousMode = Mode::Starting;

    /** @brief Serializes readings, operating modes and counters. */
    void serialize(JsonDocument& doc) const override {
        doc[Keys::DEVICE] = Config::DEVICE;
        doc[Keys::TIME] = millis();
        doc[Keys::SOURCE] = sourcePin;
        doc[Keys::MODE] = Config::MODE_NAMES[static_cast<uint8_t>(mode)];
        doc[Keys::PREVIOUS] =
            Config::MODE_NAMES[static_cast<uint8_t>(previousMode)];

        doc[Keys::LEVEL] = level;
        doc[Keys::VOLUME] = volume;
        doc[Keys::WEIGHT] = weight;
        doc[Keys::DISTANCE] = distance;
        doc[Keys::NEAR] = near;
        doc[Keys::OPEN] = open;
        doc[Keys::PENDING] = pending;
        doc[Keys::STARTED] = started;
        doc[Keys::COMPLETED] = completed;
        doc[Keys::PROXIMITY] = proximityEntries;
        doc[Keys::TRANSITIONS] = transitions;
    }
};

/** @brief Coordinates measurements, operating rules and outputs. */
class MicroGridDevice : public Device {
private:
    DistanceInput tank;
    DistanceInput person;
    WeightInput scale;
    ServoMotor gate;
    LiquidCrystal_I2C lcd;
    MicroGridTelemetry data;

    /** @brief Updates the operating mode and servo position. */
    void control() {
        if (!tank.hasRead || !person.hasRead || !scale.hasRead) {
            data.mode = Mode::Starting;
        }
        else if (data.level < 0 || data.weight < 0 || data.distance < 0) {
            data.mode = Mode::SensorFault;
        }
        else if (data.level == 0) {
            // Keep the cycle pending while the empty tank is refilled.
            data.mode = Mode::Empty;
        }
        else if (data.weight > Config::MAX_WEIGHT_G) {
            data.pending = false;
            data.mode = Mode::Overload;
        }
        else if (data.weight >= Config::STOP_WEIGHT_G) {
            if (data.pending) {
                data.completed++;
            }
            data.pending = false;
            data.mode = Mode::TargetReached;
        }
        else {
            if (!data.pending &&
                data.weight < Config::START_WEIGHT_G &&
                data.near) {
                data.pending = true;
                data.started++;
            }

            data.mode = data.pending ? Mode::Running : Mode::Ready;
        }

        const bool shouldOpen = data.mode == Mode::Running;

        if (shouldOpen != data.open) {
            gate.handle(
                shouldOpen
                    ? ServoMotor::LOCK_TO_MAXIMUM_LIMIT_COMMAND
                    : ServoMotor::LOCK_TO_MINIMUM_LIMIT_COMMAND
            );
            data.open = shouldOpen;
        }
    }

    /** @brief Writes a padded LCD line to remove previous characters. */
    void writeLine(uint8_t row, const char* text) {
        char padded[Config::LINE_BUFFER_SIZE];
        snprintf(
            padded, sizeof(padded),
            Config::PAD_FORMAT, Config::LCD_COLUMNS, text
        );
        lcd.setCursor(0, row);
        lcd.print(padded);
    }

    /** @brief Displays tank percentage and module weight. */
    void updateDisplay() {
        char line[Config::LINE_BUFFER_SIZE];

        if (data.level < 0) {
            writeLine(0, Config::UNKNOWN_LEVEL);
        }
        else if (data.level == 0) {
            writeLine(0, Config::EMPTY_TEXT);
        }
        else {
            snprintf(
                line, sizeof(line), Config::LEVEL_FORMAT,
                data.level,
                Config::MODE_LABELS[static_cast<uint8_t>(data.mode)]
            );
            writeLine(0, line);
        }

        if (data.weight < 0) {
            writeLine(1, Config::UNKNOWN_WEIGHT);
        }
        else {
            snprintf(
                line, sizeof(line),
                Config::WEIGHT_FORMAT, data.weight
            );
            writeLine(1, line);
        }
    }

public:
    /** @brief Creates sensors and outputs in their initial safe state. */
    MicroGridDevice()
        : Device(Config::ENGINE_TICK_MS, 0),
          tank(Config::TANK_TRIGGER, Config::TANK_ECHO, this),
          person(Config::PERSON_TRIGGER, Config::PERSON_ECHO, this),
          scale(Config::SCALE_DATA, Config::SCALE_CLOCK, this),
          gate(Config::SERVO_PIN, Config::CLOSED_ANGLE, this),
          lcd(Config::LCD_ADDRESS, Config::LCD_COLUMNS, Config::LCD_ROWS) {
        lcd.init();
        lcd.backlight();
        updateDisplay();
    }

    /** @brief Stops the workers before destroying the device members. */
    ~MicroGridDevice() override {
        stopAsynchronousEngine();
    }

    /** @brief Registers all sensors and starts the framework. */
    void begin() {
        Sensor* sensors[] = { &tank, &person, &scale };

        for (Sensor* sensor : sensors) {
            if (!appendSensorToScheduler(
                    sensor, Sensor::MEASURE_DATA_REQUESTED_EVENT_IDENTIFIER)) {
                Serial.println(F("Sensor registration failed."));
                writeLine(0, "Setup error");
                return;
            }
        }

        initializeAsynchronousEngine(Config::QUEUE_SIZE);
    }

    /** @brief Processes measurements and records meaningful changes. */
    void on(Event event) override {
        if (event.identifier != Sensor::DATA_READ_EVENT_IDENTIFIER) {
            return;
        }

        const int oldLevel = data.level;
        const int oldWeight = data.weight;
        const int oldDistance = data.distance;
        const bool wasNear = data.near;
        const Mode oldMode = data.mode;

        const int tankDistance = tank.getDistance();
        data.weight = scale.getGrams();
        data.distance = person.getDistance();
        data.near =
            data.distance >= 0 && data.distance <= Config::PERSON_LIMIT_CM;

        if (tankDistance < 0) {
            data.level = Config::INVALID;
            data.volume = Config::INVALID;
        }
        else {
            const int bounded = constrain(
                tankDistance,
                Config::FULL_DISTANCE_CM,
                Config::EMPTY_DISTANCE_CM
            );

            const int remaining = Config::EMPTY_DISTANCE_CM - bounded;
            const int height =
                Config::EMPTY_DISTANCE_CM - Config::FULL_DISTANCE_CM;

            data.level = remaining * Config::PERCENT_MAX / height;
            data.volume = remaining * Config::TANK_CAPACITY_ML / height;
        }

        if (data.near && !wasNear) {
            data.proximityEntries++;
        }

        control();

        if (data.mode != oldMode) {
            data.transitions++;
        }

        const bool changed =
            data.level != oldLevel ||
            data.weight != oldWeight ||
            data.distance != oldDistance ||
            data.mode != oldMode;

        if (changed) {
            updateDisplay();

            data.sourcePin = event.sourceId;
            data.previousMode = oldMode;

            JsonDocument document;
            data.serialize(document);
            serializeJson(document, Serial);
            Serial.println();
        }
    }
};

static MicroGridDevice* application = nullptr;

/** @brief Identifies the developer and starts the application. */
void setup() {
    Serial.begin(Config::SERIAL_BAUD);
    Wire.begin(Config::SDA_PIN, Config::SCL_PIN);

    Serial.println(Config::COMPANY);
    Serial.println(Config::WEBSITE);
    Serial.println(Config::DEVICE);
    Serial.print(F("Developer: "));
    Serial.println(Config::DEVELOPER_NAME);

    application = new MicroGridDevice();
    application->begin();
}

/** @brief Leaves execution to the framework background workers. */
void loop() {
    vTaskDelay(pdMS_TO_TICKS(Config::LOOP_DELAY_MS));
}