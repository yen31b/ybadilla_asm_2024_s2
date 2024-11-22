"""
Nivel de un Tanque con Modos Manual y Automático

by: Sergio Andrés Castaño Giraldo (base)
Adaptación por: ChatGPT
"""

from machine import Pin, PWM, ADC
from utime import sleep_ms, ticks_ms
import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Configuración de PINES
sensor = ADC(26)
voltaje = ADC(27)
potenciometro = ADC(28)
pwm = PWM(Pin(20))
pwm.freq(40000)
modo = Pin(15, Pin.IN)  # Interruptor digital para seleccionar modo

# Parámetros para generación de señales
amplitud = 32767  # Valor máximo para PWM (0 a 65535)
frecuencia = 0.5  # Frecuencia de la señal (Hz)
signal_type = 'senoidal'  # Cambia a: 'triangular', 'diente', 'cuadrada' según desees

# Parámetros del sensor MPS20N0040D-HX10B
Vmin = 0.5  # Voltaje mínimo (0 kPa)
Vmax = 4.5  # Voltaje máximo (40 kPa)
Pmax = 40   # Presión máxima (kPa)
rho = 1000  # Densidad del agua (kg/m^3)
g = 9.8     # Gravedad (m/s^2)


def generar_senal(tipo, t):
    """
    Genera diferentes tipos de señales según el tipo y el tiempo.
    """
    if tipo == 'senoidal':
        return int((math.sin(2 * math.pi * frecuencia * t) + 1) * (amplitud / 2))
    elif tipo == 'triangular':
        valor = (t % (1 / frecuencia)) * frecuencia * 2
        return int(amplitud * (1 - abs(valor - 1)))
    elif tipo == 'diente':
        return int((t % (1 / frecuencia)) * frecuencia * amplitud)
    elif tipo == 'cuadrada':
        return int(amplitud if (t % (1 / frecuencia)) < (1 / (2 * frecuencia)) else 0)


def calcular_presion(sensor_adc):
    """
    Calcula la presión en kPa a partir de la lectura ADC del sensor.
    """
    factor_16 = 3.3 / (65535)  # Conversión de ADC a voltaje (Raspberry Pi Pico)
    Vout = sensor_adc.read_u16() * factor_16
    if Vout < Vmin:
        Vout = Vmin  # Aseguramos que el voltaje no sea menor al rango del sensor
    presion = (Vout - Vmin) / (Vmax - Vmin) * Pmax
    return presion


def calcular_nivel(presion):
    """
    Calcula el nivel de agua en cm a partir de la presión en kPa.
    """
    nivel = (presion * 1000) / (rho * g) * 100  # Conversión de presión a altura (cm)
    return nivel


# --- Código para graficar en la computadora ---
# Variables globales para la gráfica
niveles = []
tiempos = []
start_time = ticks_ms()


def actualizar_grafica(i):
    global niveles, tiempos, start_time

    # Cálculo de presión y nivel
    level_sum = 0
    for _ in range(200):
        presion = calcular_presion(sensor)
        nivel = calcular_nivel(presion)
        level_sum += nivel
    nivel = level_sum / 200

    # Generar señal o usar potenciómetro
    tiempo_actual = (ticks_ms() - start_time) / 1000  # Tiempo en segundos
    velocidad = generar_senal(signal_type, tiempo_actual) if modo.value() == 1 else int(potenciometro.read_u16())
    pwm.duty_u16(velocidad)

    # Actualizar datos para graficar
    niveles.append(nivel)
    tiempos.append(tiempo_actual)

    # Mostrar solo los últimos 100 puntos
    niveles = niveles[-100:]
    tiempos = tiempos[-100:]

    # Actualizar la gráfica
    ax.clear()
    ax.plot(tiempos, niveles, label="Nivel del tanque (cm)")
    ax.set_title("Nivel de agua en el tanque")
    ax.set_xlabel("Tiempo (s)")
    ax.set_ylabel("Nivel (cm)")
    ax.legend()
    ax.grid()


# Configuración inicial de matplotlib
fig, ax = plt.subplots()
ani = animation.FuncAnimation(fig, actualizar_grafica, interval=500)

# Inicia el programa
plt.show()
