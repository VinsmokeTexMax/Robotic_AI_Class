# **Implementación de un Brazo Robótico de 6 GDL con Visión Artificial y Algoritmo A* para la Clasificación Automatizada de Objetos por Color**

![Robotic Arm](https://github.com/VinsmokeTexMax/Robotic_AI_Class/blob/main/brazo.png?raw=true) <!-- Inserta aquí un enlace a una imagen ilustrativa del proyecto -->

Este proyecto presenta el diseño e implementación de un **brazo robótico de 6 grados de libertad (6GDL)**, capaz de clasificar objetos según su color utilizando técnicas de visión artificial. La solución combina herramientas avanzadas como **MATLAB**, **Python** y **Arduino**, implementando el algoritmo de planificación de trayectorias **A\*** para optimizar los movimientos del brazo.

## **Características del Sistema**

- **6 Grados de Libertad:** Permite una alta flexibilidad y precisión en los movimientos del brazo robótico.  
- **Visión Artificial:** Implementación de detección y clasificación de objetos en tiempo real basada en color.  
- **Planificación de Trayectorias:** Uso del algoritmo A\* para calcular rutas óptimas y evitar colisiones.  
- **Integración Multiplataforma:** Uso de MATLAB para simulaciones, Python para procesamiento de imágenes y Arduino para control del hardware.  
- **Aplicaciones:** Automatización industrial, clasificación de productos, educación en robótica y más.

---

## **Tecnologías Utilizadas**

- **Python:** Procesamiento de imágenes y visión artificial mediante OpenCV.
- **MATLAB:** Simulación y modelado del brazo robótico.
- **Arduino:** Control de los servomotores y comunicación con el sistema.
- **Algoritmo A\*:** Optimización de trayectorias para movimientos eficientes.

---

## **Arquitectura del Sistema**

### **1. Adquisición de Imágenes**
- Captura de imágenes en tiempo real con una cámara.
- Procesamiento mediante Python y OpenCV para detección de colores.

### **2. Clasificación de Objetos**
- Análisis y segmentación por color.
- Asignación de cada objeto a una categoría específica.

### **3. Planificación y Movimiento**
- Uso de A\* para generar trayectorias optimizadas.
- Control del brazo mediante Arduino para ejecutar la manipulación.

---

## **Requisitos del Proyecto**

### **Hardware**
- Brazo robótico de 6GDL.
- Cámara compatible con USB.
- Placa Arduino (Uno o Mega recomendada).
- Módulos de servomotores.

### **Software**
- Python 3.x con OpenCV y NumPy.
- MATLAB R202x (versión recomendada).
- Arduino IDE.

---

## **Cómo Ejecutar el Proyecto**

1. **Configuración del Hardware:**
   - Conecta la cámara al sistema.
   - Ensambla y conecta el brazo robótico al Arduino.
   - Asegúrate de que los servomotores estén calibrados.

2. **Preparación del Software:**
   - Instala las dependencias necesarias para Python:
     ```bash
     pip install opencv-python numpy
     ```
   - Configura MATLAB para simular el brazo robótico.
   - Carga el código del Arduino desde el Arduino IDE.

3. **Ejecución:**
   - Ejecuta el script de procesamiento de imágenes en Python.
   - En MATLAB, simula y verifica los movimientos planeados.
   - Controla el brazo mediante el firmware cargado en Arduino.

---

## **Resultados Esperados**

- **Detección precisa de colores** en tiempo real.
- Clasificación eficiente y rápida de los objetos.
- Movimientos optimizados y sin colisiones gracias al algoritmo A\*.

---

## **Contribuciones**

¡Contribuciones son bienvenidas! Si tienes ideas para mejorar este proyecto, no dudes en hacer un *fork* del repositorio, trabajar en tus cambios y enviar un *pull request*.

---

## **Autor**

Proyecto desarrollado por:  
**Santiago Jesus Marquez Calderon**  
- Email: [smarquezc@unal.edu.co](mailto:smarquezc@unal.edu.co)  
- GitHub: [VisnmokeTexMax](https://github.com/VinsmokeTexMax)
- **Raul Ricardo Reales Cohen**  
- Email: [rreales@unal.edu.co](mailto:rreales@unal.edu.co)  
- **Erick Enrique Bastidas Santana**  
- Email: [erbastidass@unal.edu.co](mailto:erbastidass@unal.edu.co)  

---

