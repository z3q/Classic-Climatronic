/*
    PID thermostat for automotive climate control
    Copyright (C) 2025 z3q (Kirill A. Vorontsov)

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

/* MSP430G2452 pinout
        ┌───────┐
DVCC  1 │●      │ 20 DVSS   (3.3V Power | GND)
P1.0  2 │       │ 19 XIN    (Unused | Unused)
P1.1  3 │       │ 18 XOUT   (DEBUG_TX when DEBUG_PID | Unused)
P1.2  4 │       │ 17 TEST   (Heater PWM output | SBW Programming)
P1.3  5 │       │ 16 RST    (Unused | Reset)
P1.4  6 │       │ 15 P1.7   (Setpoint Analog Input | TM1637 DIO)
P1.5  7 │       │ 14 P1.6   (DEBUG_RX when DEBUG_PID | TM1637 CLK)
P2.0  8 │       │ 13 P2.5   (Unused | DS18B20 Temperature Sensor)
P2.1  9 │       │ 12 P2.4   (Unused | Unused)
P2.2 10 │       │ 11 P2.3   (Unused | Unused)
        └───────┘

Pin Functions:
1.  DVCC    - 3.3V Power
2.  P1.0    - Unused
3.  P1.1    - UART TX (debug output when DEBUG_PID enabled)
4.  P1.2    - PWM output to heater (TA0.1)
5.  P1.3    - Unused
6.  P1.4    - Setpoint analog input (ADC10 A4)
7.  P1.5    - UART RX (debug input when DEBUG_PID enabled, unused in code)
8.  P2.0    - Unused
9.  P2.1    - Unused
10. P2.2    - Unused
11. P2.3    - Unused
12. P2.4    - Unused
13. P2.5    - DS18B20 temperature sensor data pin
14. P1.6    - TM1637 display CLK
15. P1.7    - TM1637 display DIO
16. RST     - Active-low reset input, used for programming
17. TEST    - Test pin for programming
18. XOUT    - Unused (crystal output)
19. XIN     - Unused (crystal input)
20. DVSS    - Ground
*/

// #define DEBUG_PID  // Закомментировать для финального релиза (отключит UART и отладку).

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
#define KD 0x0200 // дифференциальный коэффицинет
#define KI 0x0da7 // интегральный коэффициент (использовать в расчете как (KI * integral) >> 16) 0,0003/сек точность Q16.16

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

#define SETPOINT_MIN_Q6 1024  // (16.0 * 64)  = 1024 (16.0°C в Q10.6) минимальная уставка // 23*64 = 1472 для отладки в жару
#define SETPOINT_RANGE_Q6 600 // (9.375 * 64)  = 600 (9.375°C в Q10.6) диапазон уставки

#define ADC_DEADZONE_LOW 250                                // Нижняя граница "мертвой зоны" АЦП
#define ADC_DEADZONE_HIGH 815                               // Верхняя граница "мертвой зоны" АЦП (1023 - 100)
#define ADC_WORKZONE (ADC_DEADZONE_HIGH - ADC_DEADZONE_LOW) // Ширина рабочей зоны АЦП = 923-100

// Ограничения ШИМ
#define PWM_MIN 0
#define PWM_MAX 255

// Интервалы (в циклах таймера)
#define PWM_FREQ 47                 // Частота ШИМ (Гц) 12000/PWM_MAX
#define PD_UPDATE_INTERVAL PWM_FREQ // Обновление ПИ каждую 1 сек (в периодах ШИМ)
#define TEMP_MEASURE_INTERVAL 30    // Измерение температуры каждые 30 сек (в PD_UPDATE_INTERVAL)

// Глобальные переменные
// volatile uint16_t pwmCounter = 0;
volatile uint16_t updateCounter = 0;
volatile uint8_t measureFlag = 0;
volatile uint8_t updateFlag = 0;

const int32_t INTEGRAL_MAX = 2147483647L / KI - 1; // 2147450879;  // максимальное безопасное значение интеграла
const int32_t INTEGRAL_MIN = -INTEGRAL_MAX;        //-2147450879; // минимальное безопасное значение интеграла

#ifndef DEBUG_PID
TM1637TinyDisplay display(CLK, DIO); // 4-разрядный 7-сегментный дисплей с точками
#endif

// Прототипы функций
void initClock();
void initGPIO();
void initPWM();
void initADC();
uint16_t readADC();
int16_t readDS18B20();
uint8_t oneWireReset();
void oneWireWrite(uint8_t data);
uint8_t oneWireRead();
#ifdef DEBUG_PID
void initPIDDebug();
void sendPIDDebug(int16_t error, int32_t p_term, int32_t i_term, int16_t d_term, int32_t output, uint16_t pwm);
#else
void showLevel(uint8_t level, uint8_t pos);
#endif

int main(void)
{
    uint16_t adcValue = 0; // значние АЦП
    int16_t setpoint = 0;  // уставка
    int16_t temperature = 0;
    int32_t integral = 0; // Накопленная интегральная сумма (Q16.16)
    uint16_t pwmValue = 0;
    uint16_t lastADC = 0;         // последнее значение АЦП - нужно для детектирования изменения уставки
    boolean SPchangeFlag = false; // Флаг значительного изменения уставки
    int16_t lastTemperature = 0;  // последнее значение температуры
    int32_t d_term = 0;           // Дифференциальная составляющая
    int32_t filtered_d_term = 0;  // Отфильтрованное значение

    WDTCTL = WDTPW | WDTHOLD; // Остановить watchdog

    initClock();
    initGPIO();
    initPWM();
    initADC();
#ifdef DEBUG_PID
    initPIDDebug();
#else
    display.clear();
#endif

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
#ifndef DEBUG_PID
                display.showNumber((int)((temperature + 32) >> 6), false, 2, 2);
#endif
            }
            else
            {
#ifndef DEBUG_PID
                display.showString("Er", 2, 2); // показать ошибку
#endif
            }
#ifndef DEBUG_PID
            display.setBrightness(BRIGHT_1);
#endif
            SPchangeFlag = false;
        }

        if (updateFlag)
        {
            __disable_interrupt();
            updateFlag = 0;
            __enable_interrupt();
            // Чтение уставки (0-1023 -> 160-250, фиксированная точка 10.6)
            adcValue = (adcValue + readADC()) >> 1; // Безопасно, так как значение АЦП 10-битное

            // Если поменяли уставку, увеличить яркость и отложить измерение температуры, совмещённое с понижением яркости
            if (abs((int)adcValue - (int)lastADC) > 10)
            {
                SPchangeFlag = true;
#ifndef DEBUG_PID
                display.setBrightness(BRIGHT_HIGH);
#endif
                __disable_interrupt();
                updateCounter = TEMP_MEASURE_INTERVAL >> 1; // Отложить измерение на 15 секунд, чтобы показать уставку с максимальной яркостью
                __enable_interrupt();
                lastADC = adcValue; // Запомнить уставку для следующего сравнения
            }

            // Проверка крайних положений задающего органа
            if (adcValue <= ADC_DEADZONE_LOW)
            {
                output = PWM_MIN; // 0%
#ifndef DEBUG_PID
                display.showString("LO ", 3, 0); // Показать Low
#endif
            }
            else if (adcValue >= ADC_DEADZONE_HIGH)
            {
                output = PWM_MAX; // 100%
#ifndef DEBUG_PID
                display.showString("HI ", 3, 0); // Показать High
#endif
            }
            else
            {
                // Установка ШИМ
                if (temperature == TEMP_READ_ERROR)
                {                           // Действия при ошибке датчика
                    output = adcValue >> 2; // прямое управление ШИМ
#ifndef DEBUG_PID
                    display.showNumber((int)((output * 100) >> 8), true, 2, 0);
                    display.showString("%", 1, 2);
#endif
                }
                else
                {
                    // Корректировка adcValue с учетом "мертвой зоны"
                    uint16_t adjustedValue = adcValue - ADC_DEADZONE_LOW;
                    uint32_t scaledValue = (uint32_t)adjustedValue * SETPOINT_RANGE_Q6;

                    // Расчет уставки с масштабированием на новый диапазон АЦП (100-923 → 0-823) // 16.0-25.3°C
                    setpoint = SETPOINT_MIN_Q6 + (scaledValue + (ADC_WORKZONE >> 1)) / ADC_WORKZONE;
#ifndef DEBUG_PID
                    if (SPchangeFlag)
                    {
                        display.showNumberDec((int)(((int32_t)setpoint * 10 + 32) >> 6), 0b01000000, false, 3, 0); // Показать значение уставки с десятыми
                    }
                    else
                    {
                        display.showNumber((int)((setpoint + 32) >> 6), false, 2, 0); // Показать значение уставки
                        display.showString(" ", 1, 2);
                    }
#endif
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
                    // display.showNumber((integral_term >> 6), false, 4, 0);

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
#ifndef DEBUG_PID
            showLevel(pwmValue, 3);
#endif
            TA0CCR1 = pwmValue; // Записать регистр ШИМ

#ifdef DEBUG_PID
            sendPIDDebug(error, p_term, integral_term, filtered_d_term, raw_output, pwmValue); // Отправка отладочной информации
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
    TA0CCR0 = PWM_MAX;                // (12000 / PWM_FREQ) - 1; // Период ШИМ (47 Гц)
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
    static uint16_t pwmCounter = 0;
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
        LPM3_EXIT;
    }
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

uint8_t oneWireReset()
{
    const uint16_t TIMEOUT = 1000; // Максимальное время ожидания (настраивается)
    uint16_t timeoutCount = 0;

    // 1. Формирование импульса сброса
    DS18B20_PIN_DIR |= DS18B20_PIN;  // Пин как выход
    DS18B20_PIN_OUT &= ~DS18B20_PIN; // Низкий уровень
    __delay_cycles(480);             // Импульс 480 мкс
    DS18B20_PIN_DIR &= ~DS18B20_PIN; // Пин как вход
                                     //   DS18B20_PIN_OUT |= DS18B20_PIN;  // Включить подтяжку к VCC
    __delay_cycles(70);              // Ожидание 70 мкс

    // 2. Ожидание ответа датчика с таймаутом
    while ((DS18B20_PIN_IN & DS18B20_PIN) && (timeoutCount < TIMEOUT))
    {
        timeoutCount++;
    }

    __delay_cycles(410);             // Завершение тайминга
    return (timeoutCount < TIMEOUT); // 1 = датчик ответил, 0 = таймаут
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
    uint32_t timeout = 0;
    const uint32_t CONVERSION_TIMEOUT_CYCLES = 850; // Для 1MHz ~750ms

    // 1. Reset и проверка присутствия
    if (!oneWireReset())
    {
        return TEMP_READ_ERROR;
    }
    // 2. Запуск преобразования
    oneWireWrite(0xCC); // Skip ROM
    oneWireWrite(0x44); // Convert T

    // 3. Ожидание завершения с таймаутом (~750ms)
    while (timeout++ < CONVERSION_TIMEOUT_CYCLES)
    {
        __delay_cycles(1000); // Проверяем каждую 1ms
        if (oneWireRead())    // Читаем целый байт - 8 раз запрос бита с корректными таймингами
            break;            // если есть хоть одна 1 -> преобразование завершено
    }

    if (timeout >= CONVERSION_TIMEOUT_CYCLES)
    { // Timeout
        return TEMP_READ_ERROR;
    }

    // 4. Чтение результата
    if (!oneWireReset())
    {
        return TEMP_READ_ERROR;
    }
    oneWireWrite(0xCC); // Skip ROM
    oneWireWrite(0xBE); // Читаем scratchpad

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

#ifdef DEBUG_PID
// Заголовок CSV
void initPIDDebug()
{
    debugSerial.begin(38400);                                     // actual baud rate = 38400/16 = 2400 (library for 16MHz processors, but running @ 1MHz)
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
#else
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
#endif