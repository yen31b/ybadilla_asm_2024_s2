from machine import Pin, PWM, ADC, Timer
import time
import sys

# Configuración de pines
#sensor = ADC(26)  # Sensor de presión
voltaje = ADC(27)
potenciometro = ADC(28)
pwm = PWM(Pin(20)) 
pwm.freq(40000)

out_pin = Pin(3, Pin.IN)  # HX710 OUT conectado a GPIO2
sck_pin = Pin(2, Pin.OUT)  # HX710 SCK conectado a GPIO3

# Variables globales
t = 0
level = 0
dt = 1

# Parámetros del sistema
K = 5
tau = 350
theta = 3
Ts = 10  # Periodo de muestreo
L = theta + Ts / 2

e = [0, 0, 0]  # Vector de error
u = [0, 0]  # Vector de Ley de Control PID
setpoint = 0

# Parámetros del controlador PID
kp = (1.2 * tau) / (K * L)
ti = 2 * L
td = 0.5 * L
q0 = kp * (1 + Ts / (2 * ti) + td / Ts)
q1 = -kp * (1 - Ts / (2 * ti) + (2 * td) / Ts)
q2 = (kp * td) / Ts

# Funciones auxiliares
def update_past(v, kT):
    for i in range(1, kT, 1):
        v[i - 1] = v[i]
    return v

def PID_Controller(u, e, q0, q1, q2):
    # Controlador PID
    lu = u[0] + q0 * e[2] + q1 * e[1] + q2 * e[0]  # Ley de control PID discreto
    
    # Anti-windup
    if lu >= 100.0:
        lu = 100.0
    if lu <= 0.0:
        lu = 0.0

    return lu

def temporizador(timer):
    global u, e, setpoint, level, q0, q1, q2
    # Actualiza los vectores u y e
    u = update_past(u, len(u))
    e = update_past(e, len(e))
    
    # Calcula el error actual
    e[len(e) - 1] = setpoint - level
    # Calcula la acción de control PID
    u_end = PID_Controller(u, e, q0, q1, q2)  # Max = 100, Min = 0
    u[len(u) - 1] = u_end
    
    # Aplica la acción de control al PWM
    velocidad = int(u[len(u) - 1] * 65535 / 100)
    pwm.duty_u16(velocidad)

def read_hx710():
    # Espera a que termine la lectura actual
    while out_pin.value() == 1:
        pass

    # Lee 24 bits
    result = 0
    for i in range(24):
        sck_pin.on()
        time.sleep_us(1)  # Pequeña demora para estabilizar la señal de reloj
        sck_pin.off()
        result = result << 1
        if out_pin.value():
            result += 1

    # Convierte a complemento a 2
    result = result ^ 0x800000

    # Pulsa la línea de reloj 3 veces para iniciar la siguiente lectura
    for i in range(3):
        sck_pin.on()
        time.sleep_us(1)
        sck_pin.off()

    return result

def read_analog_filtered(adc_pin, samples=10):
    total = 0
    for _ in range(samples):
        total += adc_pin.read_u16()
        time.sleep_us(100)  # Breve pausa entre lecturas
    return total // samples


def main():
    global level, t, setpoint, u, e

    tim = Timer()
    tim.init(period=10000, mode=Timer.PERIODIC, callback=temporizador)

    while True:
        # Leer presión en la forma correcta, calculando la medida final
        pressure_raw = read_hx710()
        level = pressure_raw / 1000  # Ajusta según lo que necesites

        # Leer setpoint del potenciómetro (puedes cambiarlo si necesitas otro tipo de entrada)
        setpoint = read_analog_filtered(potenciometro) * 18 / (65535)

        # Imprimir los valores en la consola
        print("Presión medida:", level)
        print("Setpoint:", setpoint)
        print("Control PWM:", pwm.duty_u16())
        
        # Esperar un poco antes de hacer la siguiente lectura
        time.sleep(1)

if __name__ == '__main__':
    main()
