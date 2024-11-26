#define TRIG_PIN 9          // Pin TRIG del sensor ultrasónico
#define ECHO_PIN 10         // Pin ECHO del sensor ultrasónico
#define POT_PIN A0          // Pin del potenciómetro para nivel deseado
#define MOTOR1_PWM_PIN 11   // Pin PWM para controlar el motor DC1
#define MOTOR2_PWM_PIN 12   // Pin PWM para controlar el motor DC2
#define SIGNAL_TYPE 1      // 1: Senoidal, 2: Triangular, 3: Cuadrada, 4: Diente de sierra

// Variables para el controlador PID
float Kp = 2.0;             // Ganancia proporcional
float Ki = 0.5;             // Ganancia integral
float Kd = 1.0;             // Ganancia derivativa
float integral = 0;
float previousError = 0;
int mode = 1;               // Modo actual: 1 = Potenciómetro, 2 = Señales

void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(MOTOR1_PWM_PIN, OUTPUT);
  pinMode(MOTOR2_PWM_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
  Serial.begin(9600);
}

// Función para medir la distancia con el sensor ultrasónico
float measureDistance() {
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);
  float distance = duration * 0.034 / 2; // Convertir tiempo en distancia (cm)
  return distance;
}

// Función para controlar los motores en base a una señal
void ControlMotor(int signalValue, int flag) {
  //Serial.print("AQUI");
  signalValue= signalValue*flag;
  if (signalValue > 0) {
    // Motor 1 para valores positivos
    digitalWrite(MOTOR1_PWM_PIN, constrain(signalValue, 0, 255)); // Limitar a 0-255
    digitalWrite(MOTOR2_PWM_PIN, 0);  // Apagar Motor 2
  } else if (signalValue < 0) {
    // Motor 2 para valores negativos
    digitalWrite(MOTOR1_PWM_PIN, 0);  // Apagar Motor 1
    digitalWrite(MOTOR2_PWM_PIN, constrain(abs(signalValue), 0, 255)); // Absoluto y limitar
  } else {
    // Apagar ambos motores si la señal es 0
    digitalWrite(MOTOR1_PWM_PIN, 0);
    digitalWrite(MOTOR2_PWM_PIN, 0);
  }
}

void generateSineWave() {
  const int amplitude = 255;  // Amplitud de la señal (rango de 0 a 255)
  const int offset = 128;     // Desplazamiento para centrar la señal
  const int period = 100;     // Número de pasos para un ciclo completo (ajustable)
  int flag=1;
  for (int i = 0; i < period; i++) {
    float sineValue = sin(2 * PI * i / period);  // Genera un valor senoidal
    int signalValue = (sineValue * amplitude / 2) + offset;  // Escala la señal a 0-255
    
    Serial.println(signalValue);  // Enviar la señal al Plotter
    if(signalValue == 0){
      flag = -1;
    } else if( signalValue == 255){
      flag = 1;
    }
    ControlMotor(signalValue, flag);  // Controlar motores
    
  }
  delay(1000);  // Esperar un poco antes de empezar el siguiente ciclo
}

// Función para generar una señal triangular
void generateTriangleWave() {
  const int amplitude = 255;  // Amplitud de la señal (rango de 0 a 255)
  const int period = 100;     // Número de pasos para un ciclo completo (ajustable)

  for (int i = 0; i < period; i++) {
    int signalValue;
    if (i < period / 2) {
      signalValue = map(i, 0, period / 2, 0, amplitude);  // Parte ascendente
      ControlMotor(signalValue,1);
    } else {
      signalValue = map(i, period / 2, period, amplitude, 0);  // Parte descendente
      ControlMotor(signalValue,-1);  // Controlar motores
    }
    Serial.println(signalValue);  // Enviar la señal al Plotter
   
  }
  delay(500);  // Esperar un poco antes de empezar el siguiente ciclo
}

// Función para generar una señal cuadrada
void generateSquareWave() {
  const int highValue = 255;  // Valor máximo de la señal
  const int lowValue = 1;     // Valor mínimo de la señal
  const int period = 100;     // Número de pasos para un ciclo completo (ajustable)
  const int halfPeriod = period / 2;  // Mitad del ciclo

  for (int i = 0; i < period; i++) {
    int flag = (i < halfPeriod) ? 1 : -1;  // Alterna entre alto y bajo
    int signalValue = (i < halfPeriod) ? highValue : lowValue;  // Alterna entre alto y bajo
    ControlMotor(signalValue,flag);  // Controlar motores
    Serial.println(signalValue);  // Enviar la señal al Plotter
  
  }
  delay(100);  // Esperar un poco antes de empezar el siguiente ciclo
}

// Función para generar una señal diente de sierra
void generateSawtoothWave() {
  const int amplitude = 255;  // Amplitud de la señal (rango de 0 a 255)
  const int period = 100;     // Número de pasos para un ciclo completo (ajustable)

  for (int i = 0; i < period; i++) {
    int signalValue = map(i, 0, period, 0, amplitude);  // Aumenta linealmente de 0 a 255
    ControlMotor(signalValue,1);  // Controlar motores
    Serial.println(signalValue);  // Enviar la señal al Plotter
  }
  delay(100);  // Esperar un poco antes de empezar el siguiente ciclo
}

// Función para manejar la generación de señales
void loop_signal_generation() {
  switch (SIGNAL_TYPE) {
    case 1:
      generateSineWave();
      break;
    case 2:
      generateTriangleWave();
      break;
    case 3:
      generateSquareWave();
      break;
    case 4:
      generateSawtoothWave();
      break;
    default:
      generateSineWave();
      break;
  }
}

void loop_potentiometer_control() {
  // Leer el nivel deseado desde el potenciómetro (0 a 100 cm)
  int potValue = analogRead(POT_PIN);
  float targetLevel = map(potValue, 0, 1023, 0, 100); // Nivel deseado en cm

  // Medir el nivel actual con el sensor ultrasónico
  float currentLevel = measureDistance();

  // Calcular el error
  float error = targetLevel - currentLevel;

  // Controlador PID
  integral += error;                                   // Acumular el error
  float derivative = error - previousError;           // Cambio del error
  float output = (Kp * error) + (Ki * integral) + (Kd * derivative); // Salida PID
  previousError = error;                              // Actualizar el error previo

  // Limitar la salida del PID al rango del PWM (0 a 255)
  int motorSpeed = constrain(abs(output), 0, 255);

  // Activar el motor si la distancia está en el rango
  if (currentLevel > 2 && currentLevel < 15 ) {
      Serial.print("Resta: ");
      Serial.println((((targetLevel*14)/100) - (14-currentLevel)));
    if((((targetLevel*14)/100) - (14-currentLevel)) > 0){
      // Bucle para variar la velocidad del motor de forma cíclica
      for (int speed = 0; speed <= motorSpeed; speed += 10) { // Incrementar velocidad
        digitalWrite(MOTOR1_PWM_PIN, speed);
        //delay(50); // Controlar el tiempo de incremento
      }
      for (int speed = motorSpeed; speed >= 0; speed -= 10) { // Reducir velocidad
        digitalWrite(MOTOR1_PWM_PIN, speed);
        //delay(50); // Controlar el tiempo de decremento
      }
    }else {
    // Apagar el motor fuera del rango
      digitalWrite(MOTOR1_PWM_PIN, 0);
    }
  } else {
    // Apagar el motor fuera del rango
    digitalWrite(MOTOR1_PWM_PIN, 0);
  }

  // Imprimir valores para depuración
  Serial.print("Nivel deseado: ");
  Serial.print((targetLevel*14)/100);
  Serial.print(" cm | Nivel actual: ");
  Serial.print((14-currentLevel));
  Serial.print(" cm | Error: ");
  Serial.print(error);
  Serial.print(" | Salida PID: ");
  Serial.println(output);

  //delay(1000); // Pausa para evitar lecturas rápidas
}

void loop() {
  // Revisar el modo actual
  if (mode == 1) {
    loop_potentiometer_control();
  } else if (mode == 2) {
    loop_signal_generation();
  }
}
