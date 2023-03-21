#include <Adafruit_GFX.h>         // Biblioteca gráfica;
#include <Adafruit_ST7789.h>      // Biblioteca específica oara o ST7789;
#include <SPI.h>                  // Arduino SPI library;

/* Por motivos de otimização, a biblioteca do display IPS usa:
  Usa a porta 13 do arduino para o pino CSL do display;
  Usa a porta 11 do arduino para o pino SDA do display;
*/

const int TFT_CS = 10;            // CS no display IPS;
const int TFT_RST = 8;            // RES no display IPS;
const int TFT_DC = 9;             // DC no display IPS;

#define BUZZER_PIN 7

// Cria uma instância da biblioteca ST7789 TFT library
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

int px = 120;                     // Posição x da base do desenho;
int py = 120;                     // Posição y da base do desenho;
int r = 10;                       // Valor do raio da circunferência;
unsigned red = ST77XX_RED;        // Cor em hexadeciaml disponível na biblioteca;

const int nRows = 7;
const int nColumn = 2;

float points[nRows][nColumn] = {
  { 0, 1.15 },
  { 29.6649, 0 },
  { 89.0298, 2.5 },
  { 25.3837, 0.75 },
  { 46.5112, 0 },
  { 166.3371, 1.45 },
  { 383.0733, 0 },
};

#define MAX_POINT_P_WAVE 1.15
#define MAX_POINT_QRS_COMPLEX 2.5

void setup()
{
  Serial.begin(9600);
  tft.init(240, 240, SPI_MODE2);  // Inicializa o display;
  tft.setRotation(2);             // Orientação da rotação da tela;
  tft.fillScreen(ST77XX_BLACK);   // Preenche a tela na cor preta (limpa a tela);
}

void loop()
{
  // drawHeart(px, py, r, red, 1); 
  // drawPulseHeart(px, py, 10, red, 5);
  // delay(3000);                      // Aguarda 3 segundos = 3000 milisegundos
  // tft.fillScreen(ST77XX_BLACK);

   double timeLostWithProcessing = getTimeLostWithProcessingDrawHeart(px, py, r, red, 5, &points[0]);

  while (true) {
    double startTime = millis();
    drawMyPulseHeart(px, py, r, red, 5, &points[0], nRows, timeLostWithProcessing);
    double endTime = millis();
    float timeInterval = endTime - startTime;

    tft.setCursor(0, 180);
    tft.setTextSize(2);
    tft.setTextColor(red);
    tft.print("time: "); tft.println(timeInterval);
    tft.print("bpm: "); tft.println(round((60/timeInterval)*1000));
    Serial.print("heartBeatTime: "); Serial.println(timeInterval);
  }
}

/* CODE */

int convertToCartesianY(int y) {
  return (240 - y);
}

void drawCircle(int cx, int cy, int r, float startAngle, float finalAngle, unsigned color) {
  int x, y, angle;
  float degreToRadians = 3.14/180;

  for (angle = startAngle; angle <= finalAngle; angle++) {
    y = cy + cos((angle) * degreToRadians) * r;
    x = cx + sin((angle) * degreToRadians) * r;

    tft.drawPixel(x, convertToCartesianY(y), color);
  }
}

void drawLine(float xi, float yi, float xf, float yf, unsigned color) {
  float m = (yf - yi)/(xf - xi);
  int x, y;

  if (xi < xf) {
    for (x = xi; x <= xf; x++) {
      y = (x * m) - (xi * m) + yi;
      tft.drawPixel(x, convertToCartesianY(y), color);
    }
  }

  if (xi > xf) {
    for (x = xi; x >= xf; x--) {
      y = (x * m) - (xi * m) + yi;
      tft.drawPixel(x, convertToCartesianY(y), color);
    }
  }
}

float getAngularCoefficientOfTheLine(int cx, int cy, int px, int py, int r) {
  float a = ((py - cy) * (py - cy)) - (r * r);
  float b = 2 * (cx - px) * (py - cy);
  return (a / b) * (-1);
 }

float getTangentPointX(int cx, int cy, int px, int py, float m) {
  float m2 = (-1/m);
  float a = (m * px) - (m2 * cx) + cy - py;
  float b = (m - m2);
  return a/b;
}

float getTangentPointY(int cx, int cy, int px, int py, float m, float tx) {
  float m2 = (-1 / m);
  return (m2 * tx) - (m2 * cx) + cy;
}

float getAngleByM(float m) {
  float radiansToDegree = 180/3.14;
  float angle = atan(m) * radiansToDegree;
  if (m < 0) { return -180 + (angle * (-1)); }
  if (m > 0) { return 180 - angle; }
  else { return 0.0f; }
}

void drawHeart(int px, int py, int r, unsigned color, int angleSteep) {
  int cx1 = px - r;
  int cx2 = px + r;
  int cy = py + 2 * r;

  float m1 = getAngularCoefficientOfTheLine(cx1, cy, px, py, r);
  float m2 = getAngularCoefficientOfTheLine(cx2, cy, px, py, r);

  float startAngle1 = getAngleByM(m1);
  float finalAngle1 = 90;
  float startAngle2 = -90;
  float finalAngle2 = getAngleByM(m2);

  float tx1 = getTangentPointX(cx1, cy, px, py, m1);
  float tx2 = getTangentPointX(cx2, cy, px, py, m2);
  float ty = getTangentPointY(cx2, cy, px, py, m2, tx2);

  
  int x, y, angle;
  float degreToRadians = 3.14/180;

  // Primeira metade da circunferência
  for (angle = startAngle1; angle <= finalAngle1; angle+= angleSteep) {
    y = cy + cos((angle) * degreToRadians) * r;
    x = cx1 + sin((angle) * degreToRadians) * r;
    tft.writePixel(x, convertToCartesianY(y), color);

    // Primeira reta inferior
    if (tx1 <= x) {
      y = (x * m1) - (tx1 * m1) + ty;
      tft.writePixel(x, convertToCartesianY(y), color);
    }
  }
  
  // Segunda metade da circunferência
  for (angle = startAngle2; angle <= finalAngle2; angle+= angleSteep) {
    y = cy + cos((angle) * degreToRadians) * r;
    x = cx2 + sin((angle) * degreToRadians) * r;
    tft.writePixel(x, convertToCartesianY(y), color);

    // // Segunda reta inferior
    if (x <= tx2) {
      y = (x * m2) - (tx2 * m2) + ty;
      tft.writePixel(x, convertToCartesianY(y), color);
    }
  }
}

void drawPulseHeart(int px, int py, int r, unsigned color, int angleSteep) {
  int i;

  for (i = 0; i < 3; i++) {
    drawHeart(px, py, i * r, color, angleSteep);
    drawHeart(px, py, i * r, ST77XX_BLACK, angleSteep);
  }

  for (i = 3; i > 0; i--) {
    drawHeart(px, py, i * r, color, angleSteep);
    drawHeart(px, py, i * r, ST77XX_BLACK, angleSteep);
  }
}

void drawMyPulseHeart(int px, int py, int r, unsigned color, int angleSteep, float (*points)[2], int nRows, double timeLostWithProcessing) {
  int i;
  float newR, interval;

  float compensationTime = 0;

  for (i = 0; i <= nRows - 1; i++) {

    if (points[i][1] == MAX_POINT_P_WAVE) { tone(BUZZER_PIN, 3000); }
    if (points[i][1] == MAX_POINT_QRS_COMPLEX) { tone(BUZZER_PIN, 3000); }

    if (i == nRows - 1) { interval = points[i][0] - compensationTime - timeLostWithProcessing; }
    else {
      if (points[i][0] > timeLostWithProcessing) { interval = points[i][0] - timeLostWithProcessing; }
      if (points[i][0] < timeLostWithProcessing) {
        interval = 0;
        compensationTime += points[i][0];
      }
    }

    if (points[i][1] != 0) { newR = points[i][1] * r; }
    else { newR = r; }
    
    static double startLoopTime = millis();

    drawHeart(px, py, newR, color, angleSteep);

    while (millis() - startLoopTime <= interval) {

    }
    noTone(BUZZER_PIN);
    drawHeart(px, py, newR, ST77XX_BLACK, angleSteep);
    startLoopTime = millis();
  }
}

double getTimeLostWithProcessingDrawHeart(int px, int py, int r, unsigned color, int angleSteep, float (*points)[2]) {
  int i;
  float newR;

  double timeLostWithProcessing;

  double startProcess = millis();
  newR = points[0][1];
  drawHeart(px, py, newR, color, angleSteep);
  drawHeart(px, py, newR, ST77XX_BLACK, angleSteep);
  double endProcess = millis();

  timeLostWithProcessing = endProcess - startProcess;

  return timeLostWithProcessing;
}
