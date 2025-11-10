# IRB 6710-210/2.65 - Análisis Cinemático y Planificación de Trayectorias

<div align="center">

![Robot](https://img.shields.io/badge/Robot-ABB%20IRB%206710-red?style=for-the-badge)
![MATLAB](https://img.shields.io/badge/MATLAB-R2020a+-orange?style=for-the-badge&logo=mathworks)
![Unity](https://img.shields.io/badge/Unity-2021.3+-blue?style=for-the-badge&logo=unity)

**Proyecto Final - Robótica I**  
Universidad Nacional de Cuyo - Facultad de Ingeniería

</div>
---

## Descripción

Análisis cinemático completo del robot industrial ABB IRB 6710-210/2.65 aplicado a **soldadura por puntos en línea de ensamblaje automotriz**. 

Este proyecto implementa:
- Modelado Denavit-Hartenberg completo
- Cinemática directa e inversa
- Análisis de velocidades (Jacobiano)
- Detección de singularidades
- Planificación de trayectorias optimizadas
- Simulación 3D en Unity

---

## Estructura del Repositorio
```
Robotica1_IRB6710/
├── robot.m                          # Definición del robot (DH, límites, base, tool)
├── cin_inv_IRB6710.m                # Cinemática inversa con desacople
├── Trayectoria.m                    # Script principal de planificación
├── trayectoria_articular.m          # Interpolación articular (jtraj)
├── trayectoria_cartesiana.m         # Interpolación cartesiana (ctraj)
├── trayectoria_mstraj.m             # Trayectorias con perfiles trapezoidales
├── graficar_trayectoria.m           # Visualización de resultados
├── enviar_trayectoria_unity.m       # Comunicación MATLAB-Unity (TCP)
└── README.md
```

---

## Requisitos Previos

### Software Necesario

- **MATLAB** R2020a o superior
- **Robotics Toolbox** de Peter Corke (versión 10.4+)
- **Unity** 2021.3+ (opcional, para simulación 3D)

### Instalación del Robotics Toolbox
```matlab
% Desde MATLAB Add-Ons:
% Home → Add-Ons → Get Add-Ons → Buscar "Robotics Toolbox"

% O manual desde: https://petercorke.com/toolboxes/robotics-toolbox/
addpath(genpath('ruta/a/rvctools'));
```

---

## Inicio Rápido

### 1. Definir el Robot
```matlab
run('robot.m');
% Crea la variable 'R' (SerialLink) con todos los parámetros
```

### 2. Cinemática Directa
```matlab
q = [0, -pi/4, pi/3, 0, pi/6, 0];
T = R.fkine(q);
pos = T.t';  % Posición [x, y, z]

% Visualizar
R.plot(q, 'workspace', [-3 3 -3 3 -0.5 3]);
```

### 3. Cinemática Inversa
```matlab
% Pose deseada
T_deseada = transl(2, 0.5, 1.5) * roty(pi/2) * rotz(pi);

% Configuración inicial
q0 = [0, 0, 0, 0, 0, 0];

% Calcular CI (mejor solución)
q_solucion = cin_inv_IRB6710(R, T_deseada, q0, true);

% Obtener todas las soluciones
Q_todas = cin_inv_IRB6710(R, T_deseada, q0, false);
```

### 4. Ejecutar Trayectoria Completa
```matlab
run('Trayectoria.m');

% El script solicita interactivamente:
% 1. ¿Graficar trayectoria en MATLAB?
% 2. ¿Mostrar gráficas de análisis?
% 3. ¿Enviar a Unity?
```

### 5. Enviar a Unity
```matlab
puerto = 55001;
fps = 20;
enviar_trayectoria_unity(Q_total, fps, puerto);

% En Unity: Presionar 'C' para conectar
```

---

## Funciones Principales

### cin_inv_IRB6710(R, T, q0, mejor)

Resuelve la cinemática inversa con desacople cinemático.

**Parámetros:**
- `R` - Objeto SerialLink del robot
- `T` - Matriz de transformación 4x4 o SE3
- `q0` - Configuración inicial [1x6]
- `mejor` - true para mejor solución, false para todas

**Retorna:**
- `Q` - Solución(es) [6xN]

**Características:**
- Desacople posición/orientación
- Hasta 8 soluciones posibles
- Filtrado por límites articulares
- Manejo de singularidades

---

### graficar_trayectoria(Q_traj, R, Ts)

Genera 9 figuras con análisis completo:

1. Trayectoria 3D en espacio cartesiano
2. Posiciones articulares (qplot)
3-8. Análisis por articulación (posición, velocidad, aceleración)
9. Variables cartesianas (X, Y, Z)

Incluye estadísticas:
- Velocidades máximas articulares
- Aceleraciones máximas articulares
- Velocidades cartesianas máximas

---

### enviar_trayectoria_unity(Q_traj, fps, puerto)

Envía trayectoria a Unity mediante TCP/IP.

**Protocolo:**
1. Servidor TCP en MATLAB (puerto 55001)
2. Unity conecta como cliente
3. Envío: `TOTAL:N` → `q1,q2,...,q6` → `END`
4. Confirmación ACK por cada punto

---

## Tipos de Trayectorias

### Trayectoria Articular (jtraj)

Movimientos rápidos punto a punto.
```matlab
q_inicio = [0, 0, 0, 0, 0, 0];
q_final = [pi/4, -pi/6, pi/3, 0, pi/4, 0];
Q_traj = jtraj(q_inicio, q_final, 50);
```

**Ventajas:**
- Más suave y eficiente
- Sin singularidades internas
- Menor tiempo de ciclo

---

### Trayectoria Cartesiana (ctraj)

Para soldadura u operaciones en línea recta.
```matlab
T_inicio = transl(2, 0, 1) * roty(pi/2);
T_final = transl(2, 0.5, 1) * roty(pi/2);
TT = ctraj(T_inicio, T_final, 50);
Q_traj = trayectoria_cartesiana(R, TT, q0, 50, true);
```

**Ventajas:**
- Trayectoria cartesiana precisa
- Control exacto del efector final

---

### Trayectoria Trapezoidal (mstraj)

Movimientos con aceleración/desaceleración controladas.
```matlab
waypoints = [
    2.0, -0.5, 1.5;
    2.3, -0.3, 1.2;
    2.5, 0.0, 1.8
];

Q_cart = mstraj(waypoints, [1,1,1], [], p_actual, 0.06, 0.2);
```

**Ventajas:**
- Transiciones suaves
- Control de aceleraciones
- Configurable

---

## Análisis de Singularidades

### Singularidad de Muñeca

**Condición:** q5 = k·π (k = 0, 1, 2, ...)
```matlab
J = R.jacob0(q);
Jw = J(4:6, 4:6);
det_Jw = det(Jw);  % = -sin(q5)

if abs(det_Jw) < 1e-6
    warning('Singularidad de muñeca');
end
```

**Consecuencias:**
- Ejes 4 y 6 colineales
- Pérdida de 1 GDL en orientación
- Velocidades infinitas requeridas

---

### Singularidad de Hombro

**Condición:** Muñeca sobre eje Z0 (proyección nula en XY)
```matlab
u = sqrt(xm^2 + ym^2);
if u < 1e-3
    warning('Singularidad de hombro');
end
```

**Consecuencias:**
- Centro de muñeca sobre eje de base
- Pérdida de 1 GDL en posición
- q1 no afecta posición cartesiana

---

## Resultados

### Espacio de Trabajo

- **Alcance máximo:** 2.65 m
- **Carga útil:** 210 kg
- **Repetibilidad:** ±0.04 mm (ISO 9283)


---

## Autores

**Grupo 2 - Robótica I (2025)**

- Juan Francisco Huertas (13653)
- Renzo Scaglia (11761)
- Gabriel Mamaní (13401)
- Germán Ricco (13653)

**Universidad Nacional de Cuyo**  
Facultad de Ingeniería

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
