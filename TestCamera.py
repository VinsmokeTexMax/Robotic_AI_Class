import cv2
import numpy as np
import socket
import json
import time
import random

# Crear un socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(('localhost', 12345))

# Iniciar la captura de video desde la cámara web
cap = cv2.VideoCapture(1)

# Definir el kernel para operaciones morfológicas
kernel = np.ones((5, 5), np.uint8)

# Calcular distancia euclidiana entre dos puntos
def distancia(p1, p2):
    return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

# Calcular la dispersión de puntos (distancia promedio entre los puntos)
def dispersion(puntos):
    if len(puntos) < 2:
        return 0
    distancias = [distancia(p1, p2) for i, p1 in enumerate(puntos) for p2 in puntos[i+1:]]
    return np.mean(distancias)

# Calcular el color óptimo a recolectar en función de cercanía al centro superior y dispersión
def seleccionar_color_optimo(punto_referencia, posiciones):
    mejor_color = None
    menor_valor = float('inf')
    for color, puntos in posiciones.items():
        if puntos:
            distancia_promedio_ref = np.mean([distancia(punto_referencia, p) for p in puntos])
            dispersión_color = dispersion(puntos)
            # Valor combinado de proximidad al punto de referencia y dispersión
            valor = distancia_promedio_ref + dispersión_color
            if valor < menor_valor:
                menor_valor = valor
                mejor_color = color
    return mejor_color

# Modificar función `ruta_mas_corta` para priorizar los más cercanos al punto de referencia
def ruta_mas_corta(puntos, punto_referencia):
    # Ordenar puntos por distancia al punto de referencia
    puntos_ordenados = sorted(puntos, key=lambda p: distancia(punto_referencia, p))
    return puntos_ordenados

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Aplicar filtro para aumentar el contraste, bajar el brillo y subir la saturación 
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) 
    h, s, v = cv2.split(hsv) 
    v = cv2.add(v, -60) 
    # Bajar el brillo 
    s = cv2.add(s, 10)
    # # Subir la saturación 
    hsv = cv2.merge([h, s, v]) 
    frame = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR) 
    frame = cv2.convertScaleAbs(frame, alpha=1, beta=0) # Aumentar el contraste

    # Punto de referencia en la parte superior central de la imagen
    punto_referencia = (frame.shape[1] // 2, 0)

    # Convertir la imagen al espacio de color HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Ajustar rangos de color en HSV para detectar rojos más claros y naranjas
    lower_red_1 = np.array([0, 50, 50])
    upper_red_1 = np.array([14, 255, 255])
    mask_red = cv2.inRange(hsv, lower_red_1, upper_red_1)


    lower_blue = np.array([90, 50, 50])
    upper_blue = np.array([130, 255, 255])
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

    lower_green = np.array([35, 50, 50])
    upper_green = np.array([85, 255, 255])
    mask_green = cv2.inRange(hsv, lower_green, upper_green)

    mask_red = cv2.erode(mask_red, kernel, iterations=1)
    mask_red = cv2.dilate(mask_red, kernel, iterations=1)
    mask_blue = cv2.erode(mask_blue, kernel, iterations=1)
    mask_blue = cv2.dilate(mask_blue, kernel, iterations=1)
    mask_green = cv2.erode(mask_green, kernel, iterations=1)
    mask_green = cv2.dilate(mask_green, kernel, iterations=1)

    # Almacenar posiciones de cada color
    posiciones = {"Rojo": [], "Azul": [], "Verde": []}
    for mask, color_name, box_color in [(mask_red, 'Rojo', (0, 0, 255)), (mask_blue, 'Azul', (255, 0, 0)), (mask_green, 'Verde', (0, 255, 0))]:
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 500:
                x, y, w, h = cv2.boundingRect(contour)
                cx, cy = x + w // 2, y + h // 2
                posiciones[color_name].append((cx, cy))
                cv2.rectangle(frame, (x, y), (x + w, y + h), box_color, 2)
                cv2.circle(frame, (cx, cy), 5, box_color, -1)
                cv2.putText(frame, f"{color_name} ({cx}, {cy})", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, box_color, 2)

    # Dibujar el punto de referencia en la parte superior central de la imagen
    cv2.circle(frame, punto_referencia, 5, (255, 255, 255), -1)  # Blanco para que sea visible
    cv2.putText(frame, "Punto de referencia", (punto_referencia[0] - 50, punto_referencia[1] + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    # Seleccionar el color más conveniente para recolectar primero
    color_optimo = seleccionar_color_optimo(punto_referencia, posiciones)
    ruta = []
    if color_optimo:
        box_color = (0, 0, 255) if color_optimo == "Rojo" else (255, 0, 0) if color_optimo == "Azul" else (0, 255, 0)
        ruta = ruta_mas_corta(posiciones[color_optimo], punto_referencia)
        # Dibujar la ruta óptima
        for i in range(len(ruta)):
            if i > 0:
                # Dibujar la línea desde el punto anterior al actual
                cv2.line(frame, ruta[i - 1], ruta[i], box_color, 2)
            # Etiquetar cada punto en la ruta con su número
            cv2.putText(frame, f"{i+1}", ruta[i], cv2.FONT_HERSHEY_SIMPLEX, 0.5, box_color, 2)
        # Indicar el último punto de la ruta como "Fin"
        if ruta:
            cv2.putText(frame, "Fin", ruta[-1], cv2.FONT_HERSHEY_SIMPLEX, 0.6, box_color, 2)

    # Mostrar el video en vivo con las detecciones y rutas
    cv2.imshow("Detección de Colores con Ruta de Recolección", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    pos = []
    for i in range(len(ruta)):
        pos.append(ruta[i])

    datos = {
        'Color:' : color_optimo,
        'Posiciones:' : pos
    }
    print(datos)
    sock.sendall((json.dumps(datos) + '\n').encode('utf-8'))
    time.sleep(1)  # Esperar 5 segundos antes de enviar la siguiente posición

sock.close()
cap.release()
cv2.destroyAllWindows()
