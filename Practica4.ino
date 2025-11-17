#include <Controllino.h>
#include "Stone_HMI_Define.h"
#include "Procesar_HMI.h"

// ==== PARÁMETROS PID ====
float salidaPID = 0;
// Variables globales para las ganancias (protegidas)
float Kp = 0.0;
float Ki = 0.0;
float Kd = 0.0;
const float Ts = 0.05;       // 50 ms (Frecuencia de control de 20 Hz)
float errorPID[3] = {0, 0, 0};

// ==== MOTOR Y HMI ====
const int pinPWM = CONTROLLINO_D0;
int referenciaRPM = 0;       // 0-5000 RPM
char txt_setpoint[10];
char txt_rpm[10];
char txt_pwm[10];

// ==== ENCODER ====
const int pinEncoder = CONTROLLINO_IN1;
volatile unsigned long pulsos = 0;
float velocidadRPM = 0;
const uint16_t PPR = 36;
const float fs = 1.0 / Ts; // fs = 20 Hz

// ==== TIEMPOS ====
unsigned long t_lecturaHMI = 0;
unsigned long t_envioHMI = 0;

// ==== PROTOTIPO ====
void ISR_pulso();

// ==================================================
void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);

  pinMode(pinPWM, OUTPUT);
  pinMode(pinEncoder, INPUT);
  analogWrite(pinPWM, 0);

  // Inicialización HMI
  Stone_HMI_Set_Value("slider", "slider1", NULL, 0);   // Slider (0-5000)
  Stone_HMI_Set_Value("spin_box", "spin_box1", NULL, 0);   // Kp = 0
  Stone_HMI_Set_Value("spin_box", "spin_box2", NULL, 0);   // Ki = 0
  Stone_HMI_Set_Value("spin_box", "spin_box3", NULL, 0);   // Kd = 0

  // Configuración de las series
  STONE_push_series("line_series", "line_series1", 0);
  STONE_push_series("line_series", "line_series2", 0);
  STONE_push_series("line_series", "line_series3", 0);

  attachInterrupt(digitalPinToInterrupt(pinEncoder), ISR_pulso, FALLING);

  // Timer1 modo CTC para el bucle PID (50 ms)
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1B |= B00000100;    // prescaler 256
  TIMSK1 |= B00000010;    // habilita interrupción por comparación A
  OCR1A = 3125;           // 50 ms
  interrupts();

  HMI_init();
}

// ==================================================
void loop() {
  // Lectura HMI cada 10 ms
  if (millis() - t_lecturaHMI >= 10) {
    t_lecturaHMI = millis();

    // 1. Leemos el Setpoint (0-5000 RPM)
    int slider_val = HMI_get_value("slider", "slider1");
    if (slider_val >= 0) {
      referenciaRPM = slider_val;
    }

    // 2. Leemos las constantes PID
    float new_Kp = HMI_get_value("spin_box", "spin_box1");
    float new_Ki = HMI_get_value("spin_box", "spin_box2");
    float new_Kd = HMI_get_value("spin_box", "spin_box3");

    // 3. Protegemos la escritura de las ganancias
    noInterrupts();
    if (new_Kp >= 0) Kp = new_Kp / 100.0;
    if (new_Ki >= 0) Ki = new_Ki / 100.0;
    if (new_Kd >= 0) Kd = new_Kd / 100.0;
    interrupts();
  }

  // Envío HMI cada 100 ms
  if (millis() - t_envioHMI >= 100) {
    t_envioHMI = millis();

    float rpm_actual;
    float pid_actual;

    noInterrupts();
    rpm_actual = velocidadRPM;
    pid_actual = salidaPID;
    interrupts();

    int duty_percent = (int)(pid_actual / 255.0 * 100.0);

    dtostrf(referenciaRPM, 7, 2, txt_setpoint);
    dtostrf(rpm_actual, 7, 2, txt_rpm);
    dtostrf(duty_percent, 4, 0, txt_pwm);

    Stone_HMI_Set_Text("label", "label12", txt_setpoint); 
    Stone_HMI_Set_Text("label", "label4", txt_rpm);       
    Stone_HMI_Set_Text("label", "label2", txt_pwm);     

    STONE_push_series("line_series", "line_series1", duty_percent);
    STONE_push_series("line_series", "line_series2", referenciaRPM);
    STONE_push_series("line_series", "line_series3", (int)rpm_actual);
  }
}

// ==================================================
// BUCLE DE CONTROL (Se ejecuta cada 50ms)
// ==================================================
ISR(TIMER1_COMPA_vect) {
  TCNT1 = 0; // Reinicia el timer
  
  // 1. Calcular RPM
  velocidadRPM = (float(pulsos) * 60.0 * fs) / (float)PPR;
  pulsos = 0;

  // 2. Copias locales de las ganancias
  noInterrupts();
  float Kp_local = Kp;
  float Ki_local = Ki;
  float Kd_local = Kd;
  interrupts();
  
  // 3. Actualización de errores
  errorPID[2] = errorPID[1];
  errorPID[1] = errorPID[0];
  errorPID[0] = referenciaRPM - velocidadRPM;

  // 4. PID incremental
  float deltaU = Kp_local * (errorPID[0] - errorPID[1])
               + (Ki_local * Ts) * errorPID[0]
               + (Kd_local / Ts) * (errorPID[0] - 2 * errorPID[1] + errorPID[2]);
  
  salidaPID += deltaU;

  // 5. Saturación
  if (salidaPID > 255) salidaPID = 255;
  if (salidaPID < 0)   salidaPID = 0;

  // 6. Aplicar salida
  if (referenciaRPM > 0) {
    analogWrite(pinPWM, (int)salidaPID);
  } else {
    analogWrite(pinPWM, 0);
    salidaPID = 0;
    errorPID[0] = 0;
    errorPID[1] = 0;
    errorPID[2] = 0;
  }
}

// ==================================================
// Interrupción de Hardware (Encoder)
// ==================================================
void ISR_pulso() {
  pulsos++;
}
