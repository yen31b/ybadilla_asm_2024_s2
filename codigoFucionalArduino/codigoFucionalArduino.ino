#define TRIG_PIN 9          // Pin TRIG del sensor ultrasónico
#define ECHO_PIN 10         // Pin ECHO del sensor ultrasónico
#define POT_PIN A0          // Pin del potenciómetro para nivel deseado
#define MOTOR_PWM_PIN 11    // Pin PWM para controlar el motor

// Variables para el controlador PID
float Kp = 2.0;             // Ganancia proporcional
float Ki = 0.5;             // Ganancia integral
float Kd = 1.0;             // Ganancia derivativa
float integral = 0;
float previousError = 0;

void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(MOTOR_PWM_PIN, OUTPUT); // Configurar el pin PWM para el motor
  digitalWrite(TRIG_PIN, LOW);
  Serial.begin(9600);
}

// Función para medir la distancia con el sensor ultrasónico
float measureDistance() {
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Leer el tiempo del pulso en microsegundos
  long duration = pulseIn(ECHO_PIN, HIGH);

  // Calcular la distancia en centímetros
  float distance = duration * 0.034 / 2; // Velocidad del sonido: 0.034 cm/μs
  return distance;
}

void loop() {
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
        digitalWrite(MOTOR_PWM_PIN, speed);
        //delay(50); // Controlar el tiempo de incremento
      }
      for (int speed = motorSpeed; speed >= 0; speed -= 10) { // Reducir velocidad
        digitalWrite(MOTOR_PWM_PIN, speed);
        //delay(50); // Controlar el tiempo de decremento
      }
    }else {
    // Apagar el motor fuera del rango
      digitalWrite(MOTOR_PWM_PIN, 0);
    }
  } else {
    // Apagar el motor fuera del rango
    digitalWrite(MOTOR_PWM_PIN, 0);
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
