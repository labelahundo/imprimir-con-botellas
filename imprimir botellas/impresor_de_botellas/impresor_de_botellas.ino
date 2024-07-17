#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <math.h>

// Definir las conexiones del encoder
const int clkPin = 7;   // Pin CLK del encoder
const int dtPin = 6;    // Pin DT del encoder
const int swPin = 4;    // Pin SW del encoder
int pushint = A2;  // pin de push button
int evrev = 1;          //guardar el estado del boton para evitar revotes o retardos
float evrev_new;
int led = 9;
int sound = 10;
int soundCount = 3;
int fin = 0;

// Definir conexiones del MOSFET y termistor
const int mosfetPin = 5;      // Pin de control del MOSFET
const int termistorPin = A0;  // Pin de lectura del termistor

// Variables para el encoder y pantalla LCD
volatile int temperatureD = 100;   // Temperatura deseada en grados Celsius
float temperatureA = 0;           // Temperatura actual en grados Celsius
unsigned long lastActivityTime = 0;     // Tiempo del último movimiento detectado en el encoder
unsigned long lastUpdateTime = 0;  // Tiempo de la última actualización de la pantalla
unsigned long lastReadTime = 0;    // Tiempo de la última lectura del termistor

// Variables para el encoder y estado del sistema
int currentStateClk;
int lastStateClk;
bool swState = HIGH;
int lastTemperatureD = temperatureD;

// Parámetros para el cálculo de la temperatura del termistor
const float BETA = 3950;  // Constante beta del termistor
const float R_FIXED = 10000;  // Resistencia fija en ohmios (10k ohmios)
const float T0 = 298.15;  // Temperatura de referencia en Kelvin (25°C)
const float R0 = 100000;   // Resistencia del termistor a 25°C (100k ohmios)

// Inicializar la pantalla LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Variables para el filtro de media móvil
const int numReadings = 10;
float readings[numReadings];
int readIndex = 0;
float total = 0;
float average = 0;

void setup() {
  Serial.begin(9600);
  // Inicializar los pines del encoder
  pinMode(clkPin, INPUT);
  pinMode(dtPin, INPUT);
  pinMode(swPin, INPUT_PULLUP);

  //Inicializar el pin del led
  pinMode(led, OUTPUT);

  //inicializar pin del pushbutton
  pinMode(pushint, INPUT_PULLUP);

  // Inicializar el pin del MOSFET como salida
  pinMode(mosfetPin, OUTPUT);

  //Inicializar el pin de la vocina como salida
  pinMode(sound, OUTPUT);

  // Inicializar la pantalla LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Impresor de");
  lcd.setCursor(0, 1);
  lcd.print("botellas");
  delay(3000);
  lcd.clear();

  // Inicializar el estado del encoder
  lastStateClk = digitalRead(clkPin);

  //inicializar el estado del led
  digitalWrite(led, HIGH);

  // Inicializar las lecturas de temperatura
  for (int i = 0; i < numReadings; i++) {
    readings[i] = 0;
  }

  //indicar que esta listo con el sonido de la bocina
  for(int i =0; i < soundCount; i++){
    digitalWrite(sound, HIGH);
    delay(1000);
    digitalWrite(sound, LOW);
  }
}

void loop() {
  //leer el estado actual del pulsador
  evrev_new = digitalRead(pushint);
  if (evrev_new == LOW){
    fin = 1;
  }
  Serial.println(digitalRead(pushint));
  //apagar el funcionamiento segun el estado del boton
  if (fin == 1){
    temperatureD = 0;
    digitalWrite(led, HIGH);

    if(soundCount > 0){
      for(int i =0; i < 3; i++){
        digitalWrite(sound, HIGH);
        delay(300);
        digitalWrite(sound, LOW);
        delay(300);
        soundCount -=1;
      }
    }
  }

  // Leer el estado actual del pin CLK
  currentStateClk = digitalRead(clkPin);

  // Si el estado ha cambiado, verificar la dirección del giro para ajustar la temperatura
  if (currentStateClk != lastStateClk) {
    if (millis() - lastActivityTime > 50) { // debounce de 50 ms
      if (digitalRead(dtPin) != currentStateClk) {
        temperatureD++;
        digitalWrite(led, HIGH);
      } else {
        digitalWrite(led, HIGH);
        temperatureD--;
      }
      lastActivityTime = millis();
      digitalWrite(led, LOW);
    }
  }

  // Guardar el estado actual como el último estado
  lastStateClk = currentStateClk;

  // Leer la temperatura actual del termistor cada 200 ms
  if (millis() - lastReadTime > 200) {
    readTemperature();
    lastReadTime = millis();
  }

  // Control del MOSFET basado en la temperatura
  if (temperatureA >= temperatureD - 0.5) {
    digitalWrite(mosfetPin, LOW);  // Apagar MOSFET
  } else {
    digitalWrite(mosfetPin, HIGH); // Encender MOSFET
  }

  // Actualizar la pantalla cada 500 ms
  if (millis() - lastUpdateTime > 500) {
    updateTemperatureScreen();
    lastUpdateTime = millis();
  }
}

// Función para actualizar la pantalla con la temperatura deseada y actual
void updateTemperatureScreen() {
  if (lastTemperatureD != temperatureD) {
    lcd.setCursor(0, 0);
    lcd.print("T. D.");
    lcd.print(temperatureD);
    lcd.print(" C   ");
    lastTemperatureD = temperatureD;
  }
  lcd.setCursor(0, 1);
  lcd.print("T. A. ");
  lcd.print((int)temperatureA);
  lcd.print(" C   ");
}

// Función para leer la temperatura actual del termistor
void readTemperature() {
  int analogValue = analogRead(termistorPin);
  float resistance = R_FIXED * (1024.0 / analogValue - 1.0);
  float temperatureK = 1.0 / (1.0 / T0 + log(resistance / R0) / BETA);
  temperatureA = temperatureK - 273.15; // Convertir a Celsius

  // Filtro de media móvil
  total = total - readings[readIndex];
  readings[readIndex] = temperatureA;
  total = total + readings[readIndex];
  readIndex = (readIndex + 1) % numReadings;
  average = total / numReadings;
  temperatureA = average;
}
