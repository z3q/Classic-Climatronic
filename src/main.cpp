#include <msp430.h>
#include <stdint.h>
#include <Arduino.h>
#include <TM1637TinyDisplay.h>

// Module connection pins (Digital Pins)
#define CLK 14
#define DIO 15

// Пин для датчика DS18B20 (P2.5)
#define DS18B20_PIN_DIR P2DIR
#define DS18B20_PIN_OUT P2OUT
#define DS18B20_PIN_IN  P2IN
#define DS18B20_PIN     BIT5

// Пин для ШИМ нагревателя (P1.2 - TA0.1)
#define HEATER_PIN_SEL  P1SEL
#define HEATER_PIN_DIR  P1DIR
#define HEATER_PIN      BIT2

// Пин для аналогового входа (P1.4 - A4)
#define SETPOINT_ADC_IN  INCH_4

// Коэффициенты ПД-регулятора (фиксированная точка Q8.8)
#define KP 0x0200  // 2.0
#define KD 0x0500  // 5.0

#define MIN_VALID_TEMP  -550  // -55.0°C (минимальная возможная температура для DS18B20)
#define MAX_VALID_TEMP  800  // 80.0°C (максимальная возможная температура)
#define TEMP_READ_ERROR 2000  // Значение при ошибке чтения


// Ограничения ШИМ
#define PWM_MIN 0
#define PWM_MAX 255

// Интервалы (в циклах таймера)
#define PWM_FREQ 47          // Частота ШИМ (Гц) 12000/PWM_MAX
#define PD_UPDATE_INTERVAL PWM_FREQ // Обновление ПД каждую 1 сек (в периодах ШИМ)
#define TEMP_MEASURE_INTERVAL 30 // Измерение температуры каждые 30 сек (в PD_UPDATE_INTERVAL)

// Глобальные переменные
volatile uint16_t setpoint = 0;
volatile uint16_t temperature = 0;
volatile int16_t lastError = 0;
volatile uint16_t pwmValue = 0;
volatile uint16_t pwmCounter = 0;
volatile uint16_t updateCounter = 0;
volatile uint8_t measureFlag = 0;
volatile uint8_t updateFlag = 0;

TM1637TinyDisplay display(CLK, DIO);

// Прототипы функций
void initClock();
void initGPIO();
void initPWM();
void initADC();
uint16_t readADC();
uint16_t readDS18B20();
void oneWireReset();
void oneWireWrite(uint8_t data);
uint8_t oneWireRead();

int main(void) {
    WDTCTL = WDTPW | WDTHOLD;  // Остановить watchdog
    
    initClock();
    initGPIO();
    initPWM();
    initADC();
        
    __enable_interrupt();
    display.setBrightness(BRIGHT_HIGH);
    // Первое измерение температуры сразу
    measureFlag = 1;
    
    while(1) {
        if(measureFlag) {
            measureFlag = 0;
            temperature = readDS18B20();
          }
        
        
        if(updateFlag) {
            updateFlag = 0;
            // Чтение уставки (0-1023 -> 160-250, фиксированная точка 10.6)
            uint16_t adcValue = readADC();
            if (adcValue > 1023) adcValue = 1023; // Защита от переполнения
            setpoint = 160 + ((adcValue * 90) >> 10);  // 16.0-25.0°C
            
            // Расчет ошибки (фиксированная точка 10.6)
            int16_t error = setpoint - temperature;
            
            // Расчет производной ошибки (dError/dt)
            int16_t dError = error - lastError;
            
            // Расчет выхода ПД-регулятора (фиксированная точка)
            int32_t output = (KP * error) + (KD * dError);
            
            // Масштабирование и ограничение выхода
            output >>= 6;
            if (output < PWM_MIN) output = PWM_MIN;
            if (output > PWM_MAX) output = PWM_MAX;
            
            // вывод на экран
            display.clear();
            display.showNumber((int)round(setpoint/10), true, 2, 0);
            display.showNumber((int)(temperature/10), true, 2, 2);
            // Установка ШИМ
            if(temperature == TEMP_READ_ERROR) {
              // Действия при ошибке (например, безопасный режим)
              pwmValue = adcValue >> 2; // прямое управление ШИМ
              display.showString("Er",2,2);
            } else {
            pwmValue = (uint16_t)output;
            }
            TA0CCR1 = pwmValue;
            lastError = error;
                
        LPM3;
          }
    }
  }

// Инициализация тактирования
void initClock() {
    BCSCTL1 = CALBC1_1MHZ;
    DCOCTL = CALDCO_1MHZ;
    BCSCTL3 |= LFXT1S_2;  // ACLK = VLO (~12 кГц)
    BCSCTL2 |= DIVS_3;    // SMCLK = DCO/8 = 125 кГц
}

// Инициализация GPIO
void initGPIO() {
    DS18B20_PIN_DIR &= ~DS18B20_PIN;
    DS18B20_PIN_OUT |= DS18B20_PIN;
    HEATER_PIN_DIR |= HEATER_PIN;
    HEATER_PIN_SEL |= HEATER_PIN;
}

// Инициализация ШИМ и таймера
void initPWM() {
    TA0CCR0 = PWM_MAX;    // (125000 / PWM_FREQ) - 1; // Период ШИМ (488 Гц)
    TA0CCTL1 = OUTMOD_7;  // Режим Reset/Set
    TA0CCR1 = 0;          // Начальная скважность 0%
    TA0CCTL0 = CCIE;      // Разрешить прерывания по CCR0
    TA0CTL = TASSEL_1 + MC_1 + TACLR; // ACLK, счет вверх
}

// Инициализация АЦП
void initADC() {
    ADC10CTL0 = ADC10SHT_2 + ADC10ON;
    ADC10CTL1 = SETPOINT_ADC_IN + ADC10SSEL_3;
    ADC10AE0 |= BIT4;
}

// Обработчик прерывания таймера
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer0_A0_ISR(void) {
    pwmCounter++;
    
    // Обновление ПД-регулятора каждую секунду (1000 циклов при 1 кГц)
    if(pwmCounter >= PD_UPDATE_INTERVAL) {
        pwmCounter = 0;
        updateFlag = 1;
        updateCounter++;
        
        // Измерение температуры каждые 30 сек
        if(updateCounter >= TEMP_MEASURE_INTERVAL) {
            updateCounter = 0;
            measureFlag = 1;
        }
    }
    LPM3_EXIT;
}

// Чтение АЦП
uint16_t readADC() {
    ADC10CTL0 |= ENC + ADC10SC;
    while (ADC10CTL1 & ADC10BUSY);
    return ADC10MEM;
}

// Функции работы с DS18B20 (остаются без изменений)
void oneWireReset() {
    DS18B20_PIN_DIR |= DS18B20_PIN;
    DS18B20_PIN_OUT &= ~DS18B20_PIN;
    __delay_cycles(480);
    DS18B20_PIN_DIR &= ~DS18B20_PIN;
    __delay_cycles(70);
    while (DS18B20_PIN_IN & DS18B20_PIN);
    __delay_cycles(410);
}

void oneWireWrite(uint8_t data) {
    for (uint8_t i = 0; i < 8; i++) {
        DS18B20_PIN_DIR |= DS18B20_PIN;
        DS18B20_PIN_OUT &= ~DS18B20_PIN;
        __delay_cycles(2);
        if (data & 0x01) DS18B20_PIN_DIR &= ~DS18B20_PIN;
        __delay_cycles(60);
        DS18B20_PIN_DIR &= ~DS18B20_PIN;
        data >>= 1;
    }
}

uint8_t oneWireRead() {
    uint8_t data = 0;
    for (uint8_t i = 0; i < 8; i++) {
        DS18B20_PIN_DIR |= DS18B20_PIN;
        DS18B20_PIN_OUT &= ~DS18B20_PIN;
        __delay_cycles(2);
        DS18B20_PIN_DIR &= ~DS18B20_PIN;
        __delay_cycles(8);
        if (DS18B20_PIN_IN & DS18B20_PIN) data |= 0x01 << i;
        __delay_cycles(50);
    }
    return data;
}

uint16_t readDS18B20() {
  uint16_t temp = TEMP_READ_ERROR; // Значение по умолчанию при ошибке
  
  oneWireReset();
  if (oneWireRead() != 0xCC) return TEMP_READ_ERROR; // Проверка присутствия датчика
  
  oneWireWrite(0x44);  // Запуск преобразования
  
  // Ожидание завершения преобразования с таймаутом
  uint32_t timeout = 0;
  DS18B20_PIN_DIR &= ~DS18B20_PIN;
  while (!(DS18B20_PIN_IN & DS18B20_PIN)) {
      __delay_cycles(1000);
      if (++timeout > 1000) return TEMP_READ_ERROR; // Таймаут 1 секунда
  }
  
  oneWireReset();
  oneWireWrite(0xCC);  // Skip ROM
  oneWireWrite(0xBE);  // Read Scratchpad
  
  uint8_t lsb = oneWireRead();
  uint8_t msb = oneWireRead();
  uint8_t crc = oneWireRead(); // Можно добавить проверку CRC при необходимости
  
  temp = (msb << 8) | lsb;
  
  // Проверка на корректность значения
  if (temp == 0xFFFF || temp == 0x0000) {
      return TEMP_READ_ERROR; // Ошибочные показания
  }
  
  // Конвертация в градусы Цельсия (фиксированная точка 10.6)
  int16_t converted_temp = (temp >> 4) * 10 + ((temp & 0x0F) * 10) / 16;
  
  // Проверка диапазона
  if (converted_temp < MIN_VALID_TEMP || converted_temp > MAX_VALID_TEMP) {
      return TEMP_READ_ERROR;
  }
  
  return (uint16_t)converted_temp;
}