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
#define KP 0x0060 // 2.0 = 0x0200
#define KD 0x0080 // 5.0 = 0x0500
#define KI 0x00a0 // 0.0003 * 65536 ≈ 20 (использовать в расчете как (KI * integral) >> 16) 0,0003/сек точность Q16.16

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

// размер фильтра аналогового входа
#define ADC_FILTER_SIZE 3

#define MIN_VALID_TEMP -55     // -55.0°C (минимальная возможная температура для DS18B20)
#define MAX_VALID_TEMP 80      // 80.0°C (максимальная возможная температура)
#define TEMP_READ_ERROR 0x2000 // Значение при ошибке чтения

#define SETPOINT_MIN_Q6 1472  // (16.0 * 64)  = 1024 (16.0°C в Q10.6) минимальная уставка // 22*64 = 1472 для отладки в жару
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
volatile uint16_t adcBuffer[ADC_FILTER_SIZE]; // буфер значений АЦП
volatile uint8_t adcIndex = 0;                // счётчик измерений
volatile int16_t setpoint = 0;                // уставка
volatile int16_t temperature = 0;
volatile int32_t integral = 0; // Накопленная интегральная сумма (Q16.16)
volatile int16_t lastError = 0;
volatile uint16_t pwmValue = 0;
volatile uint16_t pwmCounter = 0;
volatile uint16_t updateCounter = 0;
volatile uint8_t measureFlag = 0;
volatile uint8_t updateFlag = 0;
volatile uint16_t lastADC = 0;        // последнее значение АЦП - нужно для детектирования изменения уставки
volatile int16_t lastTemperature = 0; // последнее значение температуры
int32_t d_term = 0;                   // Дифференциальная составляющая

TM1637TinyDisplay display(CLK, DIO);

// Прототипы функций
void initClock();
void initGPIO();
void initPWM();
void initADC();
uint16_t readADC();
uint16_t readFilteredADC();
int16_t readDS18B20();
void oneWireReset();
void oneWireWrite(uint8_t data);
uint8_t oneWireRead();
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
            measureFlag = 0;
            temperature = readDS18B20();
            // display.clear();
            if (temperature != TEMP_READ_ERROR)
            {
                // Расчет производной ошибки (dError/dt). уставка считается константой => расчёт по изменению температуры
                int16_t dError = lastTemperature - temperature;
                lastTemperature = temperature;
                d_term = KD * dError; // Дифференциальная составляющая
                display.setBrightness(BRIGHT_2);
                display.showNumber((int)(temperature >> 6), false, 2, 2);
            }
        }

        if (updateFlag)
        {
            updateFlag = 0;

            // Чтение уставки (0-1023 -> 160-250, фиксированная точка 10.6)
            uint16_t adcValue = readFilteredADC();

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

                // Расчет setpoint с масштабированием на новый диапазон АЦП (100-923 → 0-823) // 16.0-25.3°C
                setpoint = SETPOINT_MIN_Q6 + (scaledValue + (ADC_WORKZONE >> 1)) / ADC_WORKZONE;
                display.showNumber((int)(setpoint >> 6), false, 2, 0); // Показать значение уставки

                // setpoint = SETPOINT_MIN_Q6 + (uint32_t)(adjustedValue * SETPOINT_RANGE_Q6) / (ADC_DEADZONE_HIGH - ADC_DEADZONE_LOW);

                // Расчет ошибки (фиксированная точка 10.6)
                error = setpoint - temperature;

                // Интегральная составляющая (с насыщением)
                integral += error;
                if (integral > 52428800)
                    integral = 52428800; // Ограничение
                if (integral < -52428800)
                    integral = -52428800;

                integral_term = (KI * integral) >> 16; // Интегральная составляющая
                p_term = KP * error;                   // Пропорциональная составляющая

                // Расчет выхода (Q16.16)
                output = p_term + d_term + integral_term;
                display.showNumberHex((uint16_t)(output >> 6), 0, false, 2, 2);
                // display.showNumberHex((uint16_t)(output),0, false, 4, 0);
#ifdef DEBUG_PID
                raw_output = output;
#endif

                // Масштабирование и ограничение выхода
                output >>= 6;
                if (output < PWM_MIN)
                    output = PWM_MIN;
                if (output > PWM_MAX)
                    output = PWM_MAX;
                lastError = error;
            }

            // Установка ШИМ
            if (temperature == TEMP_READ_ERROR)
            {                                   // Действия при ошибке датчика
                pwmValue = adcValue >> 2;       // прямое управление ШИМ
                display.showString("Er", 2, 2); // показать ошибку
            }
            else
            {
                pwmValue = (uint16_t)output;
                if (abs((int)adcValue - (int)lastADC) > 40)
                { // Если поменяли уставку, показать новое значение
                    display.setBrightness(BRIGHT_HIGH);
                    updateCounter = 0;  // Отложить измерение на 30 секунд, чтобы показать уставку
                    lastADC = adcValue; // Запомнить уставку для следующего сравнения
                }
            }
            // display.showNumberHex(pwmValue, 0, false, 2, 0);    // Показать значение ШИМ
            TA0CCR1 = pwmValue; // Записать регистр ШИМ

#ifdef DEBUG_PID
            sendPIDDebug(error, p_term, integral_term, d_term, raw_output, pwmValue); // Отправка отладочной информации
#endif
        }
        LPM0;
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
    TA0CCR0 = PWM_MAX;                // (125000 / PWM_FREQ) - 1; // Период ШИМ (488 Гц)
    TA0CCTL1 = OUTMOD_7;              // Режим Reset/Set
    TA0CCR1 = 0;                      // Начальная скважность 0%
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
    LPM0_EXIT;
}

// Чтение АЦП
uint16_t readADC()
{
    ADC10CTL0 |= ENC + ADC10SC;
    while (ADC10CTL1 & ADC10BUSY)
        ;
    return ADC10MEM;
}

uint16_t readFilteredADC()
{
    adcBuffer[adcIndex] = readADC();
    adcIndex = (adcIndex + 1) % ADC_FILTER_SIZE;
    uint32_t sum = 0;
    for (uint8_t i = 0; i < ADC_FILTER_SIZE; i++)
        sum += adcBuffer[i];
    return sum / ADC_FILTER_SIZE;
}

// Функции работы с DS18B20 (остаются без изменений)
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
        if (oneWireRead())
            break; // Бит 0 = 1 -> преобразование завершено
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
                                                                    /*
                                                                        int16_t converted_temp;
                                                                        if (raw_temp & 0x8000)
                                                                        {                             // Отрицательная температура
                                                                            raw_temp = ~raw_temp + 1; // Дополнение до двух
                                                                            converted_temp = -((raw_temp >> 4) * 64 + ((raw_temp & 0x0F) * 4));
                                                                        }
                                                                        else
                                                                        {
                                                                            converted_temp = (raw_temp >> 4) * 64 + ((raw_temp & 0x0F) * 4);
                                                                        }
                                                                    */
    if (converted_temp < MIN_VALID_TEMP * 64 || converted_temp > MAX_VALID_TEMP * 64)
    {
        return TEMP_READ_ERROR;
    }
    return converted_temp; // Возвращаем знаковое число
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