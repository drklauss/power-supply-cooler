#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define ADC_DEPTH 4095         // максимальное значение АЦП, 2^8=1023, 2^10=4096
#define SERIAL_R 10000         // сопротивление последовательного резистора, 10 кОм
#define ACDC_RESISTANCE 9800   // номинальное сопротивления термистора, 10 кОм
#define DCDC_RESISTANCE 10000  // номинальное сопротивления термистора, 10 кОм
#define USBDC_RESISTANCE 10000 // номинальное сопротивления термистора, 10 кОм
#define NOMINAL_T 24           // номинальная температура (при которой TR = 10 кОм)
#define B 3950                 // B-коэффициент
#define KELVIN_K 273.15        // коэффициент перевода в Кельвины
#define ACDC_PIN A1            // датчик температуры блока AC-DC
#define DCDC_PIN A2            // датчик температуры блока DC-DC
#define TEMP_DANGER 60         // Температура, с которой начинает включаться вентилятор
#define TEMP_DISABLE 45        // Температура, до которой вентилятор снижает обороты, а затем отключается
#define TEMP_UPD 3000          // Температура обновления показаний
#define COOLER_PIN D9          // ШИМ сигнал на MOSFET
float currentTemp;             // текущее (максимальное) значение температуры
int sigPWM;                    // величина ШИМ-сигнала
char dotBot = char(161);
char dotTop = char(223);
char poses[4] = {dotBot, dotBot, dotBot, dotBot};
char cur = dotBot;

LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup()
{
  pinMode(ACDC_PIN, INPUT);
  pinMode(DCDC_PIN, INPUT);
  pinMode(COOLER_PIN, OUTPUT);
  lcd.init();
  // Пины D9 и D10 - задаем частоту PWM 16кГц, чтобы не пищал кулер
  TCCR1A = 0b00000001;
  TCCR1B = 0b00001010;
}

void loop()
{
  printWave();
  updateTemp();
  checkForDanger();
}

//
void printWave()
{
  // star
  static int starPos = 0;
  static bool direct = true;
  static unsigned long checkTime;
  if (millis() - checkTime < 500)
  {
    return;
  }

  lcd.setCursor(12 + starPos, 0);

  cur = poses[starPos];
  if (cur == dotBot) 
  {
    cur = dotTop;
  }
  else
  {
    cur = dotBot;
  }

  poses[starPos] = cur;
  lcd.print(cur);

  if (direct)
  {
    starPos++;
    if (starPos > 3)
    {
      starPos = 3;
      direct = false;
    }
  }
  else
  {
    starPos--;
    if (starPos < 0)
    {
      starPos = 0;
      direct = true;
    }
  }
  checkTime = millis();
}

/**
 * Мониторинг температуры
 * Организован гистерезис:
 * При росте температуры более TEMP_DANGER запускает вентилятор на мощности 30% (т.к. на более низкой он не стартует)
 * Далее наращивает обороты вплоть до 100% начиная с 90°
 *
 * При снижении температуры менее TEMP_DANGER снижает обороты ниже 30% (т.к. кулер уже крутится это не проблема)
 * Обороты могут падать вплоть до 15% что соответствует уровню TEMP_DISABLE
 * При дальнейшем снижении температуры вентилятор отключается
 *
 **/
void checkForDanger()
{
  static bool isRunning = false;
  if (currentTemp > TEMP_DANGER)
  {
    isRunning = true;
    sigPWM = map(currentTemp, TEMP_DANGER, 90, 80, 255);
    sigPWM = constrain(sigPWM, 80, 255);
    if (currentTemp > 90)
    {
      sigPWM = 255;
    }
    analogWrite(COOLER_PIN, sigPWM);

    return;
  }

  if (!isRunning)
  {
    return;
  }
  sigPWM = map(currentTemp, TEMP_DANGER, TEMP_DISABLE, 80, 40);
  sigPWM = constrain(sigPWM, 40, 80);
  if (currentTemp < TEMP_DISABLE)
  {
    isRunning = false;
    sigPWM = 0;
  }
  analogWrite(COOLER_PIN, sigPWM);
}

/**
 * Функция обновления температуры
 **/
void updateTemp()
{
  static unsigned long checkTime;
  if (millis() - checkTime < TEMP_UPD)
  {
    return;
  }

  // температура
  // lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("AC-DC: ");
  lcd.setCursor(6, 0);
  float tempACDC = medianACDC(getTemp(ACDC_PIN, ACDC_RESISTANCE));
  lcd.print(getFormatted(tempACDC));
  lcd.print((char)223);
  lcd.setCursor(0, 1);
  lcd.print("DC-DC: ");
  lcd.setCursor(6, 1);
  float tempDCDC = medianDCDC(getTemp(DCDC_PIN, DCDC_RESISTANCE));
  lcd.print(getFormatted(tempDCDC));
  lcd.print((char)223);

  // обороты
  lcd.setCursor(12, 1);
  if (sigPWM > 0)
  {
    lcd.print(getFormatted(sigPWM / 255.00 * 100));
    lcd.print("%");
  }
  else
  {
    lcd.print("idle");
  }
  currentTemp = tempACDC;

  if (tempDCDC > currentTemp)
  {
    currentTemp = tempDCDC;
  }
  checkTime = millis();
}

/**
 * Возвращает float отформатированный до одного знака после запятой
 **/
String getFormatted(float val)
{
  static char outstr[8];
  dtostrf(val, 4, 1, outstr);

  return outstr;
}

/**
 * Считывает и возвращает значение с запрошенного датчика
 **/
float getTemp(int pin, int int_resistance)
{
  int t = analogRead(pin);
  float tr = 4095.0 / t - 1;
  tr = SERIAL_R / tr;
  float steinhart;
  steinhart = tr / int_resistance;           // (R/Ro)
  steinhart = log(steinhart);                // ln(R/Ro)
  steinhart /= B;                            // 1/B * ln(R/Ro)
  steinhart += 1.0 / (NOMINAL_T + KELVIN_K); // + (1/To)
  steinhart = 1.0 / steinhart;               // Invert
  steinhart -= KELVIN_K;

  return steinhart;
}

/**
 *  Возвращает значение измерений температуры блока AC-DC, используя медианный фильтр на 3 значения со своим буфером
 **/
float medianACDC(float newVal)
{
  static float buf[3];
  static byte count = 0;
  buf[count] = newVal;
  if (++count >= 3)
    count = 0;

  return getMedian(buf[0], buf[1], buf[2]);
}

/**
 *  Возвращает значение измерений темепературы блока DC-DC, используя медианный фильтр на 3 значения со своим буфером
 **/
float medianDCDC(float newVal)
{
  static float buf[3];
  static byte count = 0;
  buf[count] = newVal;
  if (++count >= 3)
    count = 0;

  return getMedian(buf[0], buf[1], buf[2]);
}

/**
 * Реализация алгоритма простого медианного фильтра по трем значениям
 **/
float getMedian(float a, float b, float c)
{
  float middle;
  if ((a <= b) && (a <= c))
  {
    middle = (b <= c) ? b : c;
  }
  else
  {
    if ((b <= a) && (b <= c))
    {
      middle = (a <= c) ? a : c;
    }
    else
    {
      middle = (a <= b) ? a : b;
    }
  }
  return middle;
}