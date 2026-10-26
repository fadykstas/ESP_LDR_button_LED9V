#include <Arduino.h>

// ============================================================
// GPIO CONFIGURATION
// ============================================================

// Аналоговий вхід: LDR + резистор 10 кОм до GND.
// При освітленні опір LDR зменшується, тому ADC зростає.
constexpr uint8_t LDR_PIN = 5;

// Кнопка між GPIO4 та GND.
// Використовуємо внутрішній pull-up:
// HIGH = відпущена, LOW = натиснута.
constexpr uint8_t BUTTON_PIN = 4;

// Вихід на базу BC547B через резистор 4.7 кОм.
// HIGH = транзистор відкритий, LED світиться.
// LOW  = транзистор закритий, LED вимкнений.
constexpr uint8_t LED_PIN = 15;

// ============================================================
// TIMING CONFIGURATION
// ============================================================

// Час, протягом якого стан кнопки повинен залишатися
// стабільним, перш ніж ми підтвердимо натискання.
constexpr uint32_t DEBOUNCE_MS = 40;

// Інтервал між вимірюваннями освітленості.
constexpr uint32_t SENSOR_INTERVAL_MS = 100;

// Інтервал виведення діагностики в Serial Monitor.
constexpr uint32_t LOG_INTERVAL_MS = 1000;


// ============================================================
// ADC CONFIGURATION
// ============================================================

// ADC ESP32-S3: 12 біт, значення 0...4095.
constexpr uint8_t ADC_BITS = 12;

// Пороги для автоматичного режиму.
//
// ADC < 1800: темно, LED вмикається.
// ADC > 2200: світло, LED вимикається.
// Між порогами стан LED не змінюється.
constexpr uint16_t THRESHOLD_DARK = 1800;
constexpr uint16_t THRESHOLD_LIGHT = 2200;

// Кількість вимірювань у фільтрі Moving Average.
// Степінь двійки зручний для цілочисельних операцій.
constexpr size_t FILTER_SIZE = 8;


// ============================================================
// OPERATING MODES
// ============================================================

// enum class для режимів роботи.
enum class LedMode : uint8_t {
    OFF,
    ON,
    AUTO
};

// Початковий режим після запуску ESP32.
LedMode currentMode = LedMode::OFF;

// ============================================================
// BUTTON STATE
// ============================================================

// Останнє фізичне значення GPIO.
bool lastRawButtonState = HIGH;

// Підтверджений стан кнопки після debounce.
bool stableButtonState = HIGH;

// Час останньої зміни фізичного стану кнопки.
uint32_t lastDebounceTime = 0;


// ============================================================
// LIGHT SENSOR STATE
// ============================================================

// Кільцевий буфер останніх вимірювань ADC.
uint16_t readings[FILTER_SIZE] = {};

// Поточна позиція для нового вимірювання.
size_t readingIndex = 0;

// Сума всіх елементів буфера.
// Дозволяє не підсумовувати весь масив щоразу.
uint32_t readingsSum = 0;

// Кількість валідних вимірювань.
// Потрібна, щоб на початку не усереднювати з нулями.
size_t readingsCount = 0;

// Останнє сире значення ADC.
uint16_t adcRaw = 0;

// Усереднене значення ADC.
uint16_t adcFiltered = 0;


// ============================================================
// LED STATE
// ============================================================

// Поточний стан зовнішньої LED-лампи.
bool ledState = false;


// ============================================================
// TIMERS
// ============================================================

uint32_t lastSensorTime = 0;
uint32_t lastLogTime = 0;


// ============================================================
// LED DRIVER
// ============================================================

// Єдина функція, яка безпосередньо змінює GPIO18.
// Таким чином контролер не працює з GPIO напряму.
void setLed(bool enabled)
{
    // Якщо стан не змінився, повторний запис не потрібен.
    if (ledState == enabled) {
        return;
    }

    ledState = enabled;

    // BC547B працює як low-side switch.
    digitalWrite(LED_PIN, enabled ? HIGH : LOW);

    Serial.printf(
        "[LED] %s\n",
        enabled ? "ON" : "OFF"
    );
}


// ============================================================
// MODE CONTROLLER
// ============================================================

// Перетворення режиму на текст для Serial Monitor.
const char* modeToString(LedMode mode)
{
    switch (mode) {
        case LedMode::OFF:
            return "OFF";

        case LedMode::ON:
            return "ON";

        case LedMode::AUTO:
            return "AUTO";
    }

    return "UNKNOWN";
}


// Викликається один раз для кожного підтвердженого натискання кнопки.
void nextMode()
{
    switch (currentMode) {
        case LedMode::OFF:
            currentMode = LedMode::ON;
            break;

        case LedMode::ON:
            currentMode = LedMode::AUTO;
            break;

        case LedMode::AUTO:
            currentMode = LedMode::OFF;
            break;
    }

    Serial.printf(
        "[MODE] Changed to %s\n",
        modeToString(currentMode)
    );
}


// ============================================================
// BUTTON DRIVER WITH DEBOUNCE
// ============================================================

// Викликаємо в кожній ітерації loop().
//
// Алгоритм:
// 1. Зчитуємо фізичний стан кнопки.
// 2. Якщо він змінився, запускаємо таймер debounce.
// 3. Якщо стан стабільний DEBOUNCE_MS, приймаємо його.
// 4. Перемикаємо режим лише на подію натискання, а не під час утримування кнопки.
void updateButton(uint32_t now)
{
    const bool rawState = digitalRead(BUTTON_PIN);

    // Виявили зміну фізичного сигналу.
    if (rawState != lastRawButtonState) {
        lastDebounceTime = now;
        lastRawButtonState = rawState;
    }

    // Перевіряємо, чи минув час стабілізації.
    if (now - lastDebounceTime < DEBOUNCE_MS) {
        return;
    }

    // Якщо підтверджений стан уже такий самий, нової події немає.
    if (rawState == stableButtonState) {
        return;
    }

    // Підтверджуємо новий стан.
    stableButtonState = rawState;

    // INPUT_PULLUP: LOW означає натискання. Реагуємо лише на перехід HIGH -> LOW.
    if (stableButtonState == LOW) {
        Serial.println("[BUTTON] Pressed");

        nextMode();
    }
}


// ============================================================
// ADC MOVING AVERAGE FILTER
// ============================================================

// Додаємо нове вимірювання до кільцевого буфера.
void addAdcReading(uint16_t value)
{
    // Якщо буфер уже заповнений, віднімаємо старе значення від суми перед його заміною.
    if (readingsCount == FILTER_SIZE) {
        readingsSum -= readings[readingIndex];
    } else {
        ++readingsCount;
    }

    // Записуємо нове значення.
    readings[readingIndex] = value;

    // Додаємо його до загальної суми.
    readingsSum += value;

    // Переходимо до наступної позиції.
    readingIndex = (readingIndex + 1) % FILTER_SIZE;

    // Обчислюємо середнє значення.
    adcFiltered = static_cast<uint16_t>(
        readingsSum / readingsCount
    );
}


// ============================================================
// LIGHT SENSOR DRIVER
// ============================================================

// Періодично зчитуємо LDR
void updateLightSensor(uint32_t now)
{
    if (now - lastSensorTime < SENSOR_INTERVAL_MS) {
        return;
    }

    lastSensorTime = now;

    // Отримуємо сире значення ADC.
    adcRaw = analogRead(LDR_PIN);

    // Оновлюємо ковзне середнє.
    addAdcReading(adcRaw);
}

// ============================================================
// AUTO MODE WITH HYSTERESIS
// ============================================================

// У режимі AUTO керуємо LED за освітленістю.
// Але між двома порогами зберігаємо попередній стан.
void updateAutoMode()
{
    if (readingsCount == 0) {
        return;
    }

    // Темно -> вмикаємо лампу.
    if (adcFiltered < THRESHOLD_DARK) {
        setLed(true);
    }
    // Світло -> вимикаємо лампу.
    else if (adcFiltered > THRESHOLD_LIGHT) {
        setLed(false);
    }

    // Якщо ADC між порогами, нічого не робимо.
}


// ============================================================
// MAIN CONTROLLER
// ============================================================

// Контролер вирішує, чи потрібно ввімкнути LED,
// залежно від поточного режиму.
void updateController()
{
    switch (currentMode) {
        case LedMode::OFF:
            setLed(false);
            break;

        case LedMode::ON:
            setLed(true);
            break;

        case LedMode::AUTO:
            updateAutoMode();
            break;
    }
}


// ============================================================
// SERIAL DIAGNOSTICS
// ============================================================

void updateDiagnostics(uint32_t now)
{
    if (now - lastLogTime < LOG_INTERVAL_MS) {
        return;
    }

    lastLogTime = now;

    Serial.printf(
        "[STATUS] Uptime=%lu ms | Mode=%s | "
        "ADC RAW=%u | ADC AVG=%u | LED=%s\n",
        static_cast<unsigned long>(now),
        modeToString(currentMode),
        static_cast<unsigned>(adcRaw),
        static_cast<unsigned>(adcFiltered),
        ledState ? "ON" : "OFF"
    );
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(115200);

    // Налаштовуємо вихід транзисторного ключа.
    // Спочатку встановлюємо LOW, щоб уникнути
    // небажаного ввімкнення LED під час старту.
    digitalWrite(LED_PIN, LOW);
    pinMode(LED_PIN, OUTPUT);

    // Кнопка з внутрішнім pull-up.
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    // LDR підключений до аналогового входу.
    pinMode(LDR_PIN, INPUT);

    // Роздільна здатність ADC: 12 біт.
    analogReadResolution(ADC_BITS);

    // Підлаштування для вимірювання широкого діапазону
    // напруг у межах допустимого входу ESP32-S3.
    analogSetPinAttenuation(LDR_PIN, ADC_11db);

    // Зчитуємо початковий стан кнопки,
    // щоб не отримати помилкове натискання.
    lastRawButtonState = digitalRead(BUTTON_PIN);
    stableButtonState = lastRawButtonState;

    // Заповнюємо фільтр першим реальним вимірюванням.
    adcRaw = analogRead(LDR_PIN);
    addAdcReading(adcRaw);

    Serial.printf(
        "Initial mode: %s\n",
        modeToString(currentMode)
    );

    Serial.println("System initialized.");
}


// ============================================================
// MAIN LOOP
// ============================================================
void loop()
{
    // Один час для всіх неблокувальних таймерів.
    const uint32_t now = millis();

    // 1. Обробляємо кнопку.
    updateButton(now);

    // 2. Зчитуємо й фільтруємо ADC.
    updateLightSensor(now);

    // 3. Керуємо LED відповідно до режиму.
    updateController();

    // 4. Виводимо діагностику.
    updateDiagnostics(now);
}
