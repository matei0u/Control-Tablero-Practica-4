// LIBRERIAS
#include <Controllino.h>
#include "Stone_HMI_Define.h" // Librería oficial de HMI Stone
#include "Procesar_HMI.h"     // Librería implementada para procesar respuestas del HMI

// --- VARIABLES DE CONTROL Y ESTADO ---
const int pin_motor = CONTROLLINO_D0; // Pin de salida PWM al motor
bool is_running = false;              // Estado del motor (true=corriendo, false=parado)

// --- VARIABLES DE SETPOINT (HMI) ---
float setpoint_rpm = 0; // RPM deseadas (leído del slider1, 0-5000)
char label12_text[10];  // Char para mostrar el Setpoint en el label12

// --- VARIABLES DE MEDICIÓN (RPM) ---
const int entrada = CONTROLLINO_IN1;                       // Pin de entrada de pulsos
volatile unsigned long conteo_pulsos = 0;                  // Contador de pulsos
float rpm = 0;                                             // RPM calculadas
char label4_text[10];                                      // Char para mostrar las RPM en el label4
const uint16_t PULSOS_POR_REV = 36;                        // Pulsos por revolución
const float fs = 1;                                        // Frecuencia de muestreo (1 Hz)

// --- VARIABLES DEL PID ---
// Constantes (leídas de la HMI)
float Kp = 1.0; // Ganancia Proporcional (desde spin_box1)
float Ki = 0.5; // Ganancia Integral (desde spin_box2)
float Kd = 0.1; // Ganancia Derivativa (desde spin_box3)

// Variables de cálculo del PID
float error = 0;
float integral = 0;
float derivative = 0;
float last_error = 0;
float output_pid = 0; // Salida del PID (0-255)
const float pid_min = 0;
const float pid_max = 255;

// --- VARIABLES DE TEMPORIZACIÓN ---
unsigned long t_previo_hmi = 0;    // Para enviar datos a HMI
unsigned long t_previo_read = 0; // Para leer datos de HMI

// FUNCIONES ADICIONALES
void contarPulso();

void setup() {
  Serial.begin(115200);  // Comunicación serial con el PC
  Serial2.begin(115200); // Comunicación serial con el HMI

  // --- Configuración HMI ---
  STONE_push_series("line_series", "line_series1", 0); // Gráfica 1 (Setpoint)
  STONE_push_series("line_series", "line_series2", 0); // Gráfica 2 (RPM)
  Stone_HMI_Set_Value("slider", "slider1", NULL, 0); // Poner slider en 0
  HMI_init(); // Inicialización del sistema de colas

  // --- Configuración de Pines ---
  pinMode(entrada, INPUT);
  pinMode(pin_motor, OUTPUT);
  analogWrite(pin_motor, 0); // Asegurarse que el motor esté apagado

  // --- Configuración de Interrupciones ---
  // Interrupción de hardware para contar pulsos
  attachInterrupt(digitalPinToInterrupt(entrada), contarPulso, FALLING);
  
  // Interrupción de Timer1 para el bucle de control (1 Hz)
  noInterrupts();
  TCCR1A = 0b00000000;  // Modo normal
  TCCR1B = 0b00000000;  // Limpiar registros
  TCCR1B |= B00000100;  // Preescaler 256
  TIMSK1 |= B00000010;  // Habilitar interrupción por comparación (CTC)
  OCR1A = 62500 / fs;   // TOP para 1 segundo (16MHz / 256 / 1Hz = 62500)
  interrupts();
}

void loop() {
  // --- TAREA 1: Leer HMI (cada 10ms) ---
  if (millis() - t_previo_read >= 10) {
    t_previo_read = millis();

    // 1. Leer Setpoint del slider (0-5000 RPM)
    float slider_val = HMI_get_value("slider", "slider1");
    if (slider_val > 0) {
      setpoint_rpm = slider_val; // Asigna 0-5000 RPM
      is_running = true;       // Si el usuario mueve el slider, arranca
    }

    // 2. Leer constantes PID de los spin_box
    Kp = HMI_get_value("spin_box", "spin_box1");
    Ki = HMI_get_value("spin_box", "spin_box2");
    Kd = HMI_get_value("spin_box", "spin_box3");
    
    // 3. Leer botón de paro
    int stop_button = HMI_get_value("tab_button", "tab_button1");
    if (stop_button == 1) { // Asumiendo 1 = presionado
      is_running = false;
      setpoint_rpm = 0; // Poner setpoint a 0
      Stone_HMI_Set_Value("slider", "slider1", NULL, 0); // Resetear slider en HMI
    }
  }

  // --- TAREA 2: Escribir en HMI (cada 100ms) ---
  if (millis() - t_previo_hmi >= 100) {
    t_previo_hmi = millis();

    // Convertir valores a texto
    dtostrf(setpoint_rpm, 7, 2, label12_text); // Valor deseado (del slider)
    dtostrf(rpm, 7, 2, label4_text);           // Valor medido (RPM reales)

    // Enviar texto a etiquetas
    Stone_HMI_Set_Text("label", "label12", label12_text); // Envía Setpoint a label12
    Stone_HMI_Set_Text("label", "label4", label4_text);   // Envía RPM reales a label4

    // Enviar datos a las gráficas
    STONE_push_series("line_series", "line_series1", setpoint_rpm); // Gráfica 1: Setpoint
    STONE_push_series("line_series", "line_series2", rpm);          // Gráfica 2: RPM real
  }
}


// =================================================================
// BUCLE DE CONTROL PID (Se ejecuta 1 vez por segundo)
// =================================================================
ISR(TIMER1_COMPA_vect) {
  TCNT1 = 0; // Resetea el timer

  // --- 1. Calcular RPM (Variable de Proceso) ---
  rpm = (float(conteo_pulsos) * 60) * fs / (PULSOS_POR_REV);
  conteo_pulsos = 0; // Resetea los pulsos para el próximo segundo

  // --- 2. Lógica de Paro ---
  if (!is_running) {
    output_pid = 0;     // Apagar salida
    integral = 0;     // Resetear integral
    last_error = 0;   // Resetear derivativa
    analogWrite(pin_motor, 0); // Apagar motor
    return; // Salir de la interrupción
  }

  // --- 3. Calcular Error ---
  error = setpoint_rpm - rpm; // Ej: 3500 - 3450

  // --- 4. Calcular Término Integral (con Anti-Windup) ---
  integral = integral + (error * (1.0 / fs));
  // Limitar (sujetar) la integral para evitar windup
  if (integral > pid_max) integral = pid_max;
  if (integral < pid_min) integral = pid_min;

  // --- 5. Calcular Término Derivativo ---
  derivative = (error - last_error) * fs;
  last_error = error; // Guardar error actual para la próxima iteración

  // --- 6. Calcular Salida PID ---
  output_pid = (Kp * error) + (Ki * integral) + (Kd * derivative);

  // --- 7. Saturar la Salida (0-255) ---
  // A_A_quí el PID convierte el error grande (0-5000)
  // en una salida de potencia (0-255)
  if (output_pid > pid_max) output_pid = pid_max;
  if (output_pid < pid_min) output_pid = pid_min;

  // --- 8. Aplicar Salida al Motor ---
  analogWrite(pin_motor, (int)output_pid);

  // (Debug) Imprimir en el monitor serial
  Serial.print("SP: "); Serial.print(setpoint_rpm);
  Serial.print(" | RPM: "); Serial.print(rpm);
  Serial.print(" | P: "); Serial.print(Kp * error);
  Serial.print(" | I: "); Serial.print(Ki * integral);
  Serial.print(" | D: "); Serial.print(Kd * derivative);
  Serial.print(" | PWM: "); Serial.println(output_pid);
}

// =================================================================
// Interrupción por Hardware para contar los pulsos del motor
// =================================================================
void contarPulso() {
  conteo_pulsos++; // Incrementar contador al detectar pulso
}