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
P1.5  7 │       │ 14 P1.6   (Unused | TM1637 CLK)
P2.0  8 │       │ 13 P2.5   (Unused | DS18B20 Temperature Sensor)
P2.1  9 │       │ 12 P2.4   (Unused | Unused)
P2.2 10 │       │ 11 P2.3   (Unused | Unused)
        └───────┘

Pin Functions:
1.  DVCC    - 3.3V Power
2.  P1.0    - Unused
3.  P1.1    - Debug UART TX (bit-bang, 9600 8N1, transmit-only)
4.  P1.2    - PWM output to heater (TA0.1)
5.  P1.3    - Unused
6.  P1.4    - Setpoint analog input (ADC10 A4)
7.  P1.5    - Unused
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

#define DEBUG_PID  // Закомментировать для финального релиза (отключит UART и отладку).

#include <msp430.h>
#include <stdint.h>
#include <Arduino.h>
#include <TM1637TinyDisplay.h>

// Коэффициенты ПИД-регулятора (фиксированная точка Q8.8)
#define KP 0x0060 // пропорциональный коэффициент
#define KD 0x0200 // дифференциальный коэффициент
#define KI 0x0da7 // интегральный коэффициент (KI * integral) >> 16, 0.05/сек, Q16.16

// Display connection pins (Digital Pins)
#define CLK 14
#define DIO 15

// Пин для датчика DS18B20 (P2.5)
#define DS18B20_PIN_DIR P2DIR
#define DS18B20_PIN_OUT P2OUT
#define DS18B20_PIN_IN  P2IN
#define DS18B20_PIN     BIT5

// Пин для ШИМ нагревателя (P1.2 - TA0.1)
#define HEATER_PIN_SEL P1SEL
#define HEATER_PIN_DIR P1DIR
#define HEATER_PIN     BIT2

// Пин для аналогового входа (P1.4 - A4)
#define SETPOINT_ADC_IN INCH_4

#define MIN_VALID_TEMP -55     // -55.0°C
#define MAX_VALID_TEMP 80      //  80.0°C
#define TEMP_READ_ERROR 0x2000 // Значение при ошибке чтения

#define SETPOINT_MIN_Q6   1024 // 16.0°C в Q10.6
#define SETPOINT_RANGE_Q6  600 // 9.375°C в Q10.6

#define ADC_DEADZONE_LOW  250
#define ADC_DEADZONE_HIGH 815
#define ADC_WORKZONE (ADC_DEADZONE_HIGH - ADC_DEADZONE_LOW)

#define PWM_MIN 0
#define PWM_MAX 255

#define PWM_FREQ              47
#define PD_UPDATE_INTERVAL    PWM_FREQ
#define TEMP_MEASURE_INTERVAL 30

#define STATE_MEASURE   0b00000001
#define STATE_UPDATE    0b00000010
#define STATE_SP_CHANGE 0b00000100

// ============================================================================
//  DEBUG UART (bit-bang, transmit-only)
// ----------------------------------------------------------------------------
//  Enable by uncommenting "#define DEBUG_PID" above.
//
//  Connection parameters (standard serial monitor settings):
//      * Baud rate : 9600
//      * Data bits : 8
//      * Parity    : none
//      * Stop bits : 1
//      * Flow ctrl : none
//
//  Wiring:
//      MSP430G2452 P1.1  (Arduino/Energia pin 3, "DEBUG_TXD") -> USB-UART RXD
//      MSP430G2452 GND   (pin 20)                             -> USB-UART GND
//      (USB-UART TXD is NOT used — the firmware only transmits.)
//
//  Implementation notes:
//      * MCLK = 1 MHz (DCO calibrated). 1 bit time = 1000000 / 9600 ≈ 104 µs.
//      * Transmission is blocking (bit-banged with __delay_cycles).
//      * One CSV line is emitted per PID update (once per second):
//            Error,P-Term,I-Term,D-Term,Output,PWM
//
//  The TM1637 display is on P1.6/P1.7 and is completely independent of the
//  debug TX pin, so it works identically in both build modes.
//  When DEBUG_PID is undefined, initDebug()/debugPID() become no-ops and the
//  whole debug block is compiled out.
// ============================================================================
#ifdef DEBUG_PID
#define DEBUG_TX_PIN_SEL P1SEL
#define DEBUG_TX_PIN_DIR P1DIR
#define DEBUG_TX_PIN_OUT P1OUT
#define DEBUG_TX_PIN     BIT1

#define UART_BIT_CYCLES 104 // 1 MHz / 9600 baud

static void uartTxByte(uint8_t b)
{
    // Start bit (low)
    DEBUG_TX_PIN_OUT &= ~DEBUG_TX_PIN;
    __delay_cycles(UART_BIT_CYCLES);

    // 8 data bits, LSB first
    for (uint8_t i = 0; i < 8; i++)
    {
        if (b & 0x01)
            DEBUG_TX_PIN_OUT |= DEBUG_TX_PIN;
        else
            DEBUG_TX_PIN_OUT &= ~DEBUG_TX_PIN;
        __delay_cycles(UART_BIT_CYCLES);
        b >>= 1;
    }

    // Stop bit (high)
    DEBUG_TX_PIN_OUT |= DEBUG_TX_PIN;
    __delay_cycles(UART_BIT_CYCLES);
}

static void uartTxString(const char *s)
{
    while (*s)
        uartTxByte((uint8_t)*s++);
}

static void uartTxInt(int32_t n)
{
    char    buf[12];
    uint8_t len = 0;

    if (n < 0)
    {
        uartTxByte('-');
        n = -n;
    }
    if (n == 0)
    {
        uartTxByte('0');
        return;
    }
    while (n > 0 && len < sizeof(buf))
    {
        buf[len++] = (char)('0' + (n % 10));
        n /= 10;
    }
    while (len > 0)
        uartTxByte((uint8_t)buf[--len]);
}

// Debug init: configure TX pin and print the CSV header once at startup.
static void initDebug(void)
{
    DEBUG_TX_PIN_SEL &= ~DEBUG_TX_PIN; // GPIO function
    DEBUG_TX_PIN_DIR |=  DEBUG_TX_PIN; // Output
    DEBUG_TX_PIN_OUT |=  DEBUG_TX_PIN; // Idle high

    uartTxString("Error,P-Term,I-Term,D-Term,Output,PWM\r\n");
}

// The ONLY place in the firmware that emits debug output.
static void debugPID(int16_t error, int32_t p_term, int32_t i_term,
                     int32_t d_term, int32_t output, uint16_t pwm)
{
    uartTxInt(error);          uartTxByte(',');
    uartTxInt(p_term);         uartTxByte(',');
    uartTxInt(i_term);         uartTxByte(',');
    uartTxInt(d_term);         uartTxByte(',');
    uartTxInt(output);         uartTxByte(',');
    uartTxInt((int32_t)pwm);
    uartTxString("\r\n");
}
#else
// Debug disabled: all debug calls collapse to no-ops.
#define initDebug()                  ((void)0)
#define debugPID(e, p, i, d, o, w)   ((void)0)
#endif
// ============================================================================

// Глобальные переменные
volatile uint8_t  stateFlag     = 0;
volatile uint16_t updateCounter = 0;

const int32_t INTEGRAL_MAX = 2147483647L / KI - 1;
const int32_t INTEGRAL_MIN = -INTEGRAL_MAX;

TM1637TinyDisplay display(CLK, DIO); // 4-разрядный 7-сегментный дисплей

// Прототипы функций
void     initClock();
void     initGPIO();
void     initPWM();
void     initADC();
uint16_t readADC();
int16_t  readDS18B20();
uint8_t  oneWireReset();
void     oneWireWrite(uint8_t data);
uint8_t  oneWireRead();
void     showLevel(uint8_t level, uint8_t pos);

int main(void)
{
    uint16_t adcValue      = 512;
    int16_t  setpoint      = 0;
    int16_t  temperature   = 0;
    int32_t  integral      = 0;
    uint16_t pwmValue      = 0;
    uint16_t lastADC       = 0;
    int16_t  lastTemperature = 0;
    int32_t  d_term          = 0;
    int32_t  filtered_d_term = 0;

    WDTCTL = WDTPW | WDTHOLD; // Остановить watchdog

    initClock();
    initGPIO();
    initPWM();
    initADC();
    initDebug();          // no-op, если DEBUG_PID не определён
    display.clear();

    __enable_interrupt();

    stateFlag |= STATE_MEASURE;

    while (1)
    {
        int32_t output     = 0;
        int32_t raw_output = 0; // сохраняется для отладки (до масштабирования)
        int16_t error      = 0;
        int32_t integral_term = 0;
        int32_t p_term        = 0;

        if (stateFlag & STATE_MEASURE)
        {
            __disable_interrupt();
            stateFlag &= ~STATE_MEASURE;
            temperature = readDS18B20();
            __enable_interrupt();

            if (temperature != TEMP_READ_ERROR)
            {
                int16_t dError = lastTemperature - temperature;
                lastTemperature = temperature;
                d_term = (int32_t)KD * (int32_t)dError;
                filtered_d_term = (filtered_d_term + d_term) / 2;
                display.showNumber((int)((temperature + 32) >> 6), false, 2, 2);
            }
            else
            {
                display.showString("Er", 2, 2);
            }
            display.setBrightness(BRIGHT_2);
            stateFlag &= ~STATE_SP_CHANGE;
        }

        if (stateFlag & STATE_UPDATE)
        {
            __disable_interrupt();
            stateFlag &= ~STATE_UPDATE;
            __enable_interrupt();

            adcValue = (adcValue + readADC()) >> 1;

            if (abs((int)adcValue - (int)lastADC) > 10)
            {
                stateFlag |= STATE_SP_CHANGE;
                display.setBrightness(BRIGHT_HIGH);
                __disable_interrupt();
                updateCounter = TEMP_MEASURE_INTERVAL >> 1;
                __enable_interrupt();
                lastADC = adcValue;
            }

            if (adcValue <= ADC_DEADZONE_LOW)
            {
                output = PWM_MIN;
                display.showString("LO ", 3, 0);
            }
            else if (adcValue >= ADC_DEADZONE_HIGH)
            {
                output = PWM_MAX;
                display.showString("HI ", 3, 0);
            }
            else
            {
                if (temperature == TEMP_READ_ERROR)
                {
                    output = adcValue >> 2;
                    display.showNumber((int)((output * 100) >> 8), true, 2, 0);
                    display.showString("%", 1, 2);
                }
                else
                {
                    uint16_t adjustedValue = adcValue - ADC_DEADZONE_LOW;
                    uint32_t scaledValue   = (uint32_t)adjustedValue * SETPOINT_RANGE_Q6;

                    setpoint = SETPOINT_MIN_Q6 + (scaledValue + (ADC_WORKZONE >> 1)) / ADC_WORKZONE;

                    if (stateFlag & STATE_SP_CHANGE)
                    {
                        display.showNumberDec((int)(((int32_t)setpoint * 10 + 32) >> 6),
                                              0b01000000, false, 3, 0);
                    }
                    else
                    {
                        display.showNumber((int)((setpoint + 32) >> 6), false, 2, 0);
                        display.showString(" ", 1, 2);
                    }

                    error = setpoint - temperature;

                    integral += error;
                    if (integral > INTEGRAL_MAX) integral = INTEGRAL_MAX;
                    if (integral < INTEGRAL_MIN) integral = INTEGRAL_MIN;

                    integral_term = (KI * integral) >> 16;
                    p_term        = (int32_t)KP * (int32_t)error;

                    raw_output = p_term + filtered_d_term + integral_term;
                    output     = raw_output >> 6;

                    if (output < PWM_MIN)
                    {
                        output = PWM_MIN;
                        if (error < 0) integral -= error;
                    }
                    if (output > PWM_MAX)
                    {
                        output = PWM_MAX;
                        if (error > 0) integral -= error;
                    }
                }
            }

            pwmValue   = (uint16_t)output;
            showLevel(pwmValue, 3);
            TA0CCR1 = pwmValue;

            // Единственная точка вывода отладки (no-op при выключенном DEBUG_PID).
            debugPID(error, p_term, integral_term, filtered_d_term, raw_output, pwmValue);
        }
        LPM3;
    }
}

// Инициализация тактирования
void initClock()
{
    BCSCTL1 = CALBC1_1MHZ;
    DCOCTL  = CALDCO_1MHZ;
    BCSCTL3 |= LFXT1S_2; // ACLK = VLO (~12 кГц)
    BCSCTL2 |= DIVS_3;   // SMCLK = DCO/8 = 125 кГц
}

// Инициализация GPIO
void initGPIO()
{
    DS18B20_PIN_DIR &= ~DS18B20_PIN;
    DS18B20_PIN_OUT |=  DS18B20_PIN;
    HEATER_PIN_DIR  |=  HEATER_PIN;
    HEATER_PIN_SEL  |=  HEATER_PIN;
}

// Инициализация ШИМ и таймера
void initPWM()
{
    TA0CCR0  = PWM_MAX;
    TA0CCTL1 = OUTMOD_7;
    TA0CCR1  = 0;
    TA0CCTL0 = CCIE;
    TA0CTL   = TASSEL_1 + MC_1 + TACLR;
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

    if (pwmCounter >= PD_UPDATE_INTERVAL)
    {
        pwmCounter = 0;
        stateFlag |= STATE_UPDATE;
        updateCounter++;

        if (updateCounter >= TEMP_MEASURE_INTERVAL)
        {
            updateCounter = 0;
            stateFlag |= STATE_MEASURE;
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

// -------- DS18B20 --------
uint8_t oneWireReset()
{
    const uint16_t TIMEOUT = 1000;
    uint16_t timeoutCount = 0;

    DS18B20_PIN_DIR |=  DS18B20_PIN;
    DS18B20_PIN_OUT &= ~DS18B20_PIN;
    __delay_cycles(480);
    DS18B20_PIN_DIR &= ~DS18B20_PIN;
    __delay_cycles(70);

    while ((DS18B20_PIN_IN & DS18B20_PIN) && (timeoutCount < TIMEOUT))
        timeoutCount++;

    __delay_cycles(410);
    return (timeoutCount < TIMEOUT);
}

void oneWireWrite(uint8_t data)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        DS18B20_PIN_DIR |=  DS18B20_PIN;
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
        DS18B20_PIN_DIR |=  DS18B20_PIN;
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
    const uint32_t CONVERSION_TIMEOUT_CYCLES = 850;

    if (!oneWireReset())
        return TEMP_READ_ERROR;

    oneWireWrite(0xCC);
    oneWireWrite(0x44);

    while (timeout++ < CONVERSION_TIMEOUT_CYCLES)
    {
        __delay_cycles(1000);
        if (oneWireRead())
            break;
    }

    if (timeout >= CONVERSION_TIMEOUT_CYCLES)
        return TEMP_READ_ERROR;

    if (!oneWireReset())
        return TEMP_READ_ERROR;

    oneWireWrite(0xCC);
    oneWireWrite(0xBE);

    uint8_t lsb = oneWireRead();
    uint8_t msb = oneWireRead();
    int16_t raw_temp = (int16_t)(msb << 8 | lsb);

    int16_t integerPart   = raw_temp >> 4;
    uint8_t fractionalPart = raw_temp & 0x0F;
    int16_t converted_temp = integerPart * 64 + fractionalPart * 4;

    if (converted_temp < MIN_VALID_TEMP * 64 || converted_temp > MAX_VALID_TEMP * 64)
        return TEMP_READ_ERROR;

    return converted_temp;
}

// -------- Индикатор уровня ШИМ на 4-м разряде дисплея --------
void showLevel(uint8_t level, uint8_t pos)
{
    uint8_t digits[1] = {0};
    int bars = (int)(((level * 4) / 256) + 1);
    if (level == 0)
        bars = 0;
    switch (bars)
    {
    case 1: digits[0] = 0b10000000; break;
    case 2: digits[0] = 0b10001000; break;
    case 3: digits[0] = 0b11001000; break;
    case 4: digits[0] = 0b11001001; break;
    default: break;
    }
    display.setSegments(digits, 1, pos);
}