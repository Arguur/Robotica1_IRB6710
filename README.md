# IRB 6710-210/2.65 - Análisis Cinemático y Planificación de Trayectorias

<div align="center">

![Robot](https://img.shields.io/badge/Robot-ABB%20IRB%206710-red?style=for-the-badge)
![MATLAB](https://img.shields.io/badge/MATLAB-R2020a+-orange?style=for-the-badge&logo=mathworks)
![Unity](https://img.shields.io/badge/Unity-2021.3+-blue?style=for-the-badge&logo=unity)

**Proyecto Final - Robótica I**  
Universidad Nacional de Cuyo - Facultad de Ingeniería

</div>

## Soldadura por Puntos en Línea de Ensamblaje Automotriz
![unnamed](https://github.com/user-attachments/assets/a7bdd463-5123-4ad6-bbfe-29cdd4d6b1b5)

## Demo en Unity

**[Ver simulación completa en YouTube](https://youtu.be/tIxCD4fexeM)**

## 📋 Descripción del Proyecto

Este proyecto implementa un análisis cinemático completo del robot industrial **ABB IRB 6710-210/2.65** aplicado a una línea de ensamblaje de carrocerías automotrices. El robot realiza soldadura por puntos de forma automatizada, trabajando en paralelo con otro robot en la celda de trabajo.

### Características Principales

- ✅ **Robot Industrial**: ABB IRB 6710-210/2.65 (6 GDL)
- ✅ **Cinemática Directa e Inversa**: Implementación completa con método geométrico
- ✅ **Análisis Jacobiano**: Cálculo de singularidades y relación de velocidades
- ✅ **Planificación de Trayectorias**: Combinación de trayectorias articulares y cartesianas
- ✅ **Simulación en Unity**: Visualización 3D de la celda de trabajo
- ✅ **Comunicación TCP/IP**: Envío de trayectorias desde MATLAB a Unity

---

## 🎓 Autores - Grupo 2

- **Juan Francisco Huertas** - 12620
- **Renzo Scaglia** - 11761
- **Gabriel Mamaní** - 13401
- **Germán Ricco** - 13653

---

## Animación realizada por:

- **Juan Francisco Huertas** - 12620
  
---

## 🛠️ Tecnologías Utilizadas

- **MATLAB R2023a+** con Robotics System Toolbox
- **Peter Corke Robotics Toolbox** ([RTB](https://petercorke.com/toolboxes/robotics-toolbox/))
- **Unity 2021.3+** para simulación 3D
- **TCP/IP** para comunicación MATLAB-Unity

---

## 📁 Estructura del Repositorio

```
Robotica1_IRB6710/
│
├── robot.m                      # Definición del robot (parámetros DH, límites)
├── cin_inv_IRB6710.m            # Cinemática inversa con desacople cinemático
│
├── Trayectoria.m                # Script principal de generación de trayectorias
├── trayectoria_articular.m     # Interpolación en espacio articular (jtraj)
├── trayectoria_cartesiana.m    # Interpolación en espacio cartesiano (ctraj)
├── trayectoria_mstraj.m        # Trayectorias suavizadas con mstraj
│
├── graficar_trayectoria.m      # Visualización de trayectorias y análisis
├── enviar_trayectoria_unity.m  # Comunicación TCP/IP con Unity
│
└── Proyecto_Final_Robotica_1_grupo_2.pdf  # Informe completo
```

---

## 🚀 Uso

### 1. Configuración Inicial

Asegúrate de tener instalado:
- MATLAB con Robotics System Toolbox
- Peter Corke Robotics Toolbox en el path de MATLAB

```matlab
% Agregar Robotics Toolbox al path (ajustar ruta según tu instalación)
addpath(genpath('rvctools'));
```

### 2. Definición del Robot

```matlab
run('robot.m');
```

Este script crea el objeto `R` del robot con:
- Parámetros de Denavit-Hartenberg
- Límites articulares
- Transformaciones de base y herramienta

### 3. Generación de Trayectorias

```matlab
run('Trayectoria.m');
```

El script principal:
1. Define una secuencia de puntos de soldadura
2. Genera trayectorias combinadas (articulares + cartesianas)
3. Ofrece opciones para:
   - Graficar en MATLAB
   - Visualizar gráficas de posición/velocidad/aceleración
   - Enviar a Unity para simulación 3D

### 4. Cinemática Inversa

```matlab
% Ejemplo: Calcular configuración articular para una pose deseada
T_deseada = transl(2.0, 0.5, 1.8) * roty(pi/2) * rotz(pi);
q_actual = [0, 0, 0, 0, 0, 0];

% Solución óptima (más cercana a q_actual)
q_sol = cin_inv_IRB6710(R, T_deseada, q_actual, true);

% Todas las soluciones posibles
q_todas = cin_inv_IRB6710(R, T_deseada, q_actual, false);
```

### 5. Simulación en Unity

Para enviar la trayectoria a Unity:

```matlab
% Generar trayectoria
Q_traj = ...;  % Matriz Nx6 con configuraciones articulares

% Enviar a Unity (fps=20, puerto=55001)
enviar_trayectoria_unity(Q_traj, 20, 55001);
```

**Nota**: Unity debe estar ejecutándose y presionar `C` para conectar.

---

## 📊 Características Técnicas del Robot

| Parámetro | Valor |
|-----------|-------|
| **Alcance Máximo** | 2.65 m |
| **Capacidad de Carga** | 210 kg |
| **Repetibilidad (ISO 9283)** | 0.04 mm |
| **Grados de Libertad** | 6 |
| **Tipo de Muñeca** | Esférica |

### Rangos Articulares

| Articulación | Rango [°] | Velocidad Máx [°/s] |
|--------------|-----------|---------------------|
| J1 (Base) | ±170 | 110 |
| J2 (Hombro) | -65 / +85 | 110 |
| J3 (Codo) | -180 / +70 | 110 |
| J4 (Muñeca 1) | ±300 | 200 |
| J5 (Muñeca 2) | ±130 | 150 |
| J6 (Muñeca 3) | ±360 | 210 |

---

## 🔬 Análisis Implementado

### Cinemática Directa
- Parámetros de Denavit-Hartenberg estándar
- Validación mediante análisis del espacio de trabajo

### Cinemática Inversa
- **Desacople cinemático** (posición + orientación)
- **Método geométrico** para los primeros 3 GDL
- Hasta **8 soluciones** para una pose dada
- Criterio de selección por mínima distancia articular

### Análisis de Velocidades
- Cálculo del **Jacobiano geométrico** (6×6)
- Detección de **singularidades**:
  - **Singularidad de muñeca**: q₅ = kπ
  - **Singularidad de hombro**: centro de muñeca sobre eje base
  - **Singularidad de alcance**: brazo completamente extendido

### Planificación de Trayectorias
- **Trayectorias articulares** (`jtraj`): reposicionamiento rápido
- **Trayectorias cartesianas** (`ctraj`): precisión en soldadura
- **Trayectorias suavizadas** (`mstraj`): control de aceleración

---

## 📈 Gráficas Generadas

El script `graficar_trayectoria.m` genera 9 figuras:

1. **Trayectoria 3D** en espacio cartesiano
2. **Posiciones articulares** (todas las articulaciones)
3-8. **Por articulación**: posición, velocidad, aceleración
9. **Variables cartesianas**: posición, velocidad, aceleración del efector

---

## Aplicación: Soldadura por Puntos

El proyecto simula una **celda de trabajo automotriz** donde:

- Soldadura de carrocerías en una línea de ensamblaje
- Secuencia automatizada de puntos de soldadura:
  - Puerta trasera
  - Lateral del vehículo
  - Puerta delantera
  - Parabrisas

**Ventajas de la automatización**:
- ✅ Consistencia en la calidad
- ✅ Mayor productividad
- ✅ Repetibilidad < 0.04 mm
- ✅ Reducción de tiempos de ciclo

---

## 📄 Informe Completo

Para más detalles sobre el análisis matemático, desarrollo de ecuaciones y resultados, consulta el [**informe completo en PDF**]

---

## 📝 Licencia

Este proyecto fue desarrollado con fines académicos para la asignatura **Robótica I** de la carrera de Ingeniería Mecatrónica en la Universidad Nacional de Cuyo.

---

## 🙏 Agradecimientos

Agradecemos a la **Facultad de Ingeniería de la UNCuyo** y al equipo docente de Robótica I por el apoyo durante el desarrollo de este proyecto.

---

## Referencias

1. ABB Robotics - [IRB 6710 Datasheet](https://www.abb.com/global/en/areas/robotics/products/robots/articulated-robots/large-robots/irb-6710)
2. Craig, J. J. (2018). Introduction to Robotics: Mechanics and Control (4th ed.). Pearson.
3. Corke, P. (2017). Robotics, Vision and Control (2nd ed.). Springer.
4. [Peter Corke's Robotics Toolbox](https://petercorke.com/toolboxes/robotics-toolbox/)


Proyecto académico - Universidad Nacional de Cuyo  
Robótica I - 2025

---

**Desarrollado con MATLAB y Unity**
