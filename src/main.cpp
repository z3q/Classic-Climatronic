// #define DEBUG_PID  // Закомментировать для финального релиза (отключит UART и отладку)

#include <msp430.h>
#include <stdint.h>
#include <Arduino.h>
#include <TM1637TinyDisplay.h>

#ifdef DEBUG_PID
#include <SoftwareSerial.h>                       // Для эмуляции UART на GPIO
#define DEBUG_TXD 3                               // Пин для TX (P1.1)
#define DEBUG_RXD 5                               // Пин для RX (не используется)
SoftwareSerial debugSerial(DEBUG_RXD, DEBUG_TXD); // Инициализация софтового UART
#endif

// Коэффициенты ПИД-регулятора (фиксированная точка Q8.8)
#define KP 0x0060 // пропорциональнай коэффициент
#define KD 0x0080 // дифференциальный коэффицинет
#define KI 0x00d0 // интегральный коэффициент (использовать в расчете как (KI * integral) >> 16) 0,0003/сек точность Q16.16

// Display connection pins (Digital Pins)
#define CLK 14
#define DIO 15

// Пин для датчика DS18B20 (P2.5)
#define DS18B20_PIN_DIR P2DIR
#define DS18B20_PIN_OUT P2OUT
#define DS18B20_PIN_IN P2IN
#define DS18B20_PIN BIT5

// Пин для ШИМ нагревателя (P1.2 - TA0.1)
#define HEATER_PIN_SEL P1SEL
#define HEATER_PIN_DIR P1DIR
#define HEATER_PIN BIT2

// Пин для аналогового входа (P1.4 - A4)
#define SETPOINT_ADC_IN INCH_4

#define MIN_VALID_TEMP -55     // -55.0°C (минимальная возможная температура для DS18B20)
#define MAX_VALID_TEMP 80      // 80.0°C (максимальная возможная температура)
#define TEMP_READ_ERROR 0x2000 // Значение при ошибке чтения

#define SETPOINT_MIN_Q6 1024  // (16.0 * 64)  = 1024 (16.0°C в Q10.6) минимальная уставка // 22*64 = 1472 для отладки в жару
#define SETPOINT_RANGE_Q6 600 // (9.375 * 64)  = 600 (9.375°C в Q10.6) диапазон уставки

#define ADC_DEADZONE_LOW 100  // Нижняя граница "мертвой зоны" АЦП
#define ADC_DEADZONE_HIGH 923 // Верхняя граница "мертвой зоны" АЦП (1023 - 100)
#define ADC_WORKZONE 823      // Ширина рабочей зоны АЦП = 923-100

// Ограничения ШИМ
#define PWM_MIN 0
#define PWM_MAX 255

// Интервалы (в циклах таймера)
#define PWM_FREQ 47                 // Частота ШИМ (Гц) 12000/PWM_MAX
#define PD_UPDATE_INTERVAL PWM_FREQ // Обновление ПИ каждую 1 сек (в периодах ШИМ)
#define TEMP_MEASURE_INTERVAL 30    // Измерение температуры каждые 30 сек (в PD_UPDATE_INTERVAL)

// Глобальные переменные
volatile uint16_t pwmCounter = 0;
volatile uint16_t updateCounter = 0;
volatile uint8_t measureFlag = 0;
volatile uint8_t updateFlag = 0;
uint16_t adcValue = 0; // значние АЦП
int16_t setpoint = 0;  // уставка
int16_t temperature = 0;
int32_t integral = 0; // Накопленная интегральная сумма (Q16.16)
int16_t lastError = 0;
uint16_t pwmValue = 0;
uint16_t lastADC = 0;                     // последнее значение АЦП - нужно для детектирования изменения уставки
int16_t lastTemperature = 0;              // последнее значение температуры
int32_t d_term = 0;                       // Дифференциальная составляющая
int32_t filtered_d_term = 0;              // Отфильтрованное значение
const int32_t INTEGRAL_MIN = -2147450879; // минимальное безопасное значение интеграла
const int32_t INTEGRAL_MAX = 2147450879;  // максимальное безопасное значение интеграла

TM1637TinyDisplay display(CLK, DIO); // 4-разрядный 7-сегментный дисплей с точками

// Прототипы функций
void initClock();
void initGPIO();
void initPWM();
void initADC();
uint16_t readADC();
int16_t readDS18B20();
void oneWireReset();
void oneWireWrite(uint8_t data);
uint8_t oneWireRead();
void showLevel(uint8_t level, uint8_t pos);
#ifdef DEBUG_PID
void sendPIDDebug(int16_t error, int32_t p_term, int32_t i_term, int16_t d_term, int32_t output, uint16_t pwm);
#endif

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD; // Остановить watchdog

    initClock();
    initGPIO();
    initPWM();
    initADC();
    display.clear();
    lastTemperature = readDS18B20(); // для правильной работы ПИД сразу после запуска
    __enable_interrupt();

    // Первое измерение температуры сразу
    measureFlag = 1;

    while (1)
    {
        int32_t output = 0;
#ifdef DEBUG_PID
        int32_t raw_output = 0;
#endif
        int16_t error = 0;         // Ошибка
        int32_t integral_term = 0; // Интегральная составляющая
        int32_t p_term = 0;        // Пропорциональная составляющая

        if (measureFlag)
        {
            __disable_interrupt();
            measureFlag = 0;
            temperature = readDS18B20();
            __enable_interrupt();

            if (temperature != TEMP_READ_ERROR)
            {
                // Расчет производной ошибки (dError/dt). уставка считается константой => расчёт по изменению температуры
                int16_t dError = lastTemperature - temperature;
                lastTemperature = temperature;
                d_term = (int32_t)KD * (int32_t)dError; // Дифференциальная составляющая
                filtered_d_term = (filtered_d_term + d_term) >> 1;
                display.showNumber((int)(temperature >> 6), false, 2, 2);
            }
            else
            {
                display.showString("Er", 2, 2); // показать ошибку
            }
            display.setBrightness(BRIGHT_1);
        }

        if (updateFlag)
        {
            __disable_interrupt();
            updateFlag = 0;
            __enable_interrupt();
            // Чтение уставки (0-1023 -> 160-250, фиксированная точка 10.6)
            adcValue = (adcValue + readADC()) >> 1; // Безопасно, так как значение АЦП 10-битное

            // Если поменяли уставку, увеличить яркость и отложить измерение температуры, совмещённое с понижением яркости
            if (abs((int)adcValue - (int)lastADC) > 40)
            {
                display.setBrightness(BRIGHT_HIGH);
                __disable_interrupt();
                updateCounter = 0; // Отложить измерение на 30 секунд, чтобы показать уставку с максимальной яркостью
                __enable_interrupt();
                lastADC = adcValue; // Запомнить уставку для следующего сравнения
            }

            // Проверка крайних положений задающего органа
            if (adcValue <= ADC_DEADZONE_LOW)
            {
                output = PWM_MIN;               // 0%
                display.showString("LO", 2, 0); // Показать Low
            }
            else if (adcValue >= ADC_DEADZONE_HIGH)
            {
                output = PWM_MAX;               // 100%
                display.showString("HI", 2, 0); // Показать High
            }
            else
            {
                // Корректировка adcValue с учетом "мертвой зоны"
                uint16_t adjustedValue = adcValue - ADC_DEADZONE_LOW;
                uint32_t scaledValue = (uint32_t)adjustedValue * SETPOINT_RANGE_Q6;

                // Расчет уставки с масштабированием на новый диапазон АЦП (100-923 → 0-823) // 16.0-25.3°C
                setpoint = SETPOINT_MIN_Q6 + (scaledValue + (ADC_WORKZONE >> 1)) / ADC_WORKZONE;
                display.showNumber((int)(setpoint >> 6), false, 2, 0); // Показать значение уставки

                // Установка ШИМ
                if (temperature == TEMP_READ_ERROR)
                {                           // Действия при ошибке датчика
                    output = adcValue >> 2; // прямое управление ШИМ
                }
                else
                {

                    // Расчет ошибки (фиксированная точка 10.6)
                    error = setpoint - temperature;

                    // Интегральная составляющая (с насыщением)
                    integral += error;
                    if (integral > INTEGRAL_MAX)
                        integral = INTEGRAL_MAX; // Ограничение
                    if (integral < INTEGRAL_MIN)
                        integral = INTEGRAL_MIN;

                    integral_term = (KI * integral) >> 16; // Интегральная составляющая
                    p_term = (int32_t)KP * (int32_t)error; // Пропорциональная составляющая

                    // Расчет выхода (Q16.16)
                    output = p_term + filtered_d_term + integral_term;
                    // display.showNumber((p_term),  false, 4, 0);

#ifdef DEBUG_PID
                    raw_output = output;
#endif

                    // Масштабирование и ограничение выхода, предотвращение насыщения интеграла
                    output >>= 6;

                    // отладка
                    // display.showNumberHex((output), 0, false, 4, 0);

                    if (output < PWM_MIN)
                    {
                        output = PWM_MIN;
                        if (error < 0)
                            integral -= error; // Уменьшаем интеграл при отрицательной ошибке
                    }
                    if (output > PWM_MAX)
                    {
                        output = PWM_MAX;
                        if (error > 0)
                            integral -= error; // Уменьшаем интеграл при положительной ошибке
                    }
                }
            }

            pwmValue = (uint16_t)output;
            uint8_t zerosegments[1] = {0};
            display.setSegments(zerosegments, 1, 2);
            showLevel(pwmValue, 3);
            // display.showNumberHex(pwmValue, 0, false, 2, 0);    // Показать значение ШИМ
            TA0CCR1 = pwmValue; // Записать регистр ШИМ

#ifdef DEBUG_PID
            sendPIDDebug(error, p_term, integral_term, d_term, raw_output, pwmValue); // Отправка отладочной информации
#endif
        }
        LPM3;
    }
}

// Инициализация тактирования
void initClock()
{
    BCSCTL1 = CALBC1_1MHZ;
    DCOCTL = CALDCO_1MHZ;
    BCSCTL3 |= LFXT1S_2; // ACLK = VLO (~12 кГц)
    BCSCTL2 |= DIVS_3;   // SMCLK = DCO/8 = 125 кГц
}

// Инициализация GPIO
void initGPIO()
{
    DS18B20_PIN_DIR &= ~DS18B20_PIN;
    DS18B20_PIN_OUT |= DS18B20_PIN;
    HEATER_PIN_DIR |= HEATER_PIN;
    HEATER_PIN_SEL |= HEATER_PIN;
}

// Инициализация ШИМ и таймера
void initPWM()
{
    TA0CCR0 = PWM_MAX;                // (125000 / PWM_FREQ) - 1; // Период ШИМ (47 Гц)
    TA0CCTL1 = OUTMOD_7;              // Режим Reset/Set
    TA0CCR1 = 0;                      // Начальный КЗИ 0%
    TA0CCTL0 = CCIE;                  // Разрешить прерывания по CCR0
    TA0CTL = TASSEL_1 + MC_1 + TACLR; // ACLK, счет вверх
}

// Инициализация АЦП
void initADC()
{
    ADC10CTL0 = ADC10SHT_2 + ADC10ON;
    ADC10CTL1 = SETPOINT_ADC_IN + ADC10SSEL_3;
    ADC10AE0 |= BIT4;
}

// Обработчик прерывания таймера
#pragma vector = TIMER0_A0_VECTOR
__interrupt void Timer0_A0_ISR(void)
{
    pwmCounter++;

    // Обновление ПД-регулятора каждую секунду (1000 циклов при 1 кГц)
    if (pwmCounter >= PD_UPDATE_INTERVAL)
    {
        pwmCounter = 0;
        updateFlag = 1;
        updateCounter++;

        // Измерение температуры каждые 30 сек
        if (updateCounter >= TEMP_MEASURE_INTERVAL)
        {
            updateCounter = 0;
            measureFlag = 1;
        }
    }
    LPM3_EXIT;
}

// Чтение АЦП
uint16_t readADC()
{
    ADC10CTL0 |= ENC + ADC10SC;
    while (ADC10CTL1 & ADC10BUSY)
        ;
    return ADC10MEM;
}

// Функции работы с DS18B20
void oneWireReset()
{
    DS18B20_PIN_DIR |= DS18B20_PIN;
    DS18B20_PIN_OUT &= ~DS18B20_PIN;
    __delay_cycles(480);
    DS18B20_PIN_DIR &= ~DS18B20_PIN;
    __delay_cycles(70);
    while (DS18B20_PIN_IN & DS18B20_PIN)
        ;
    __delay_cycles(410);
}

void oneWireWrite(uint8_t data)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        DS18B20_PIN_DIR |= DS18B20_PIN;
        DS18B20_PIN_OUT &= ~DS18B20_PIN;
        __delay_cycles(2);
        if (data & 0x01)
            DS18B20_PIN_DIR &= ~DS18B20_PIN;
        __delay_cycles(60);
        DS18B20_PIN_DIR &= ~DS18B20_PIN;
        data >>= 1;
    }
}

uint8_t oneWireRead()
{
    uint8_t data = 0;
    for (uint8_t i = 0; i < 8; i++)
    {
        DS18B20_PIN_DIR |= DS18B20_PIN;
        DS18B20_PIN_OUT &= ~DS18B20_PIN;
        __delay_cycles(2);
        DS18B20_PIN_DIR &= ~DS18B20_PIN;
        __delay_cycles(8);
        if (DS18B20_PIN_IN & DS18B20_PIN)
            data |= 0x01 << i;
        __delay_cycles(50);
    }
    return data;
}

int16_t readDS18B20()
{
    uint16_t temp = TEMP_READ_ERROR;
    uint8_t absence = 1; // отсутствие датчика
    uint32_t timeout = 0;
    const uint32_t CONVERSION_TIMEOUT_CYCLES = 750000; // Для 1MHz ~750ms

    // 1. Reset и проверка присутствия
    DS18B20_PIN_DIR |= DS18B20_PIN;
    DS18B20_PIN_OUT &= ~DS18B20_PIN;
    __delay_cycles(480); // Reset pulse (минимум 480 мкс)
    DS18B20_PIN_DIR &= ~DS18B20_PIN;
    __delay_cycles(70); // Ожидание presence pulse (15-60 мкс)
    absence = (DS18B20_PIN_IN & DS18B20_PIN);
    __delay_cycles(410); // Завершение тайминга reset

    if (absence)
    { // No sensor connected
        return TEMP_READ_ERROR;
    }

    // 2. Запуск преобразования
    oneWireWrite(0xCC); // Skip ROM
    oneWireWrite(0x44); // Convert T

    // 3. Ожидание завершения с таймаутом (~750ms)
    while (timeout++ < CONVERSION_TIMEOUT_CYCLES)
    {
        __delay_cycles(1000); // Проверяем каждые 1ms

        oneWireReset();
        oneWireWrite(0xCC);
        oneWireWrite(0xBE); // Читаем scratchpad
        uint8_t status = oneWireRead();
        if (status & 0x01)
            break; // Преобразование завершено
    }

    if (timeout >= CONVERSION_TIMEOUT_CYCLES)
    { // Timeout
        return TEMP_READ_ERROR;
    }

    // 4. Чтение результата
    oneWireReset();
    oneWireWrite(0xCC);
    oneWireWrite(0xBE);

    uint8_t lsb = oneWireRead();
    uint8_t msb = oneWireRead();
    int16_t raw_temp = (int16_t)(msb << 8 | lsb);

    int16_t integerPart = raw_temp >> 4;                            // Знаковая целая часть
    uint8_t fractionalPart = raw_temp & 0x0F;                       // Дробная часть
    int16_t converted_temp = integerPart * 64 + fractionalPart * 4; // Q6

    if (converted_temp < MIN_VALID_TEMP * 64 || converted_temp > MAX_VALID_TEMP * 64)
    {
        return TEMP_READ_ERROR;
    }
    return converted_temp; // Возвращаем знаковое число
}

void showLevel(uint8_t level, uint8_t pos)
{
    uint8_t digits[1] = {0};
    // Must fit within 4 bars
    int bars = (int)(((level * 4) / 256) + 1);
    if (level == 0)
        bars = 0;
    switch (bars)
    {
    case 1:
        digits[0] = 0b10000000;
        break;
    case 2:
        digits[0] = 0b10001000;
        break;
    case 3:
        digits[0] = 0b11001000;
        break;
    case 4:
        digits[0] = 0b11001001;
        break;
    default: // Keep at zero
        break;
    }

    display.setSegments(digits, 1, pos);
}

#ifdef DEBUG_PID
// Заголовок CSV
void initPIDDebug()
{
    debugSerial.begin(9600);
    debugSerial.println("Error,P-Term,I-Term,D-Term,Output,PWM"); // CSV header
}

// Функция вывода данных в CSV формате
void sendPIDDebug(int16_t error, int32_t p_term,
                  int32_t i_term, int16_t d_term,
                  int32_t output, uint16_t pwm)
{
    debugSerial.print(error);
    debugSerial.print(',');
    debugSerial.print(p_term);
    debugSerial.print(',');
    debugSerial.print(i_term);
    debugSerial.print(',');
    debugSerial.print(d_term);
    debugSerial.print(',');
    debugSerial.print(output);
    debugSerial.print(',');
    debugSerial.println(pwm);
}
#endif