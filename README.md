# Tarea 2 - *Navegación por planeación*

***Fundamentos de Robótica móvil - 2024 1S)***.
***Universidad Nacional de Colombia***
***Departamento de Ingeniería Mecánica y Mecatrónica***

Equipo 1:

* *Daniel Esteban Molano*
* *Camilo Esteban Zambrano*
* *Cristhian David Sandoval*
* *Juan Sebastián Dueñas*

<!--
resolución de mapa 3 celdas/m   
entrada inferior izquierda (1.2,0) superior derecha (17.5,16)
-->

En este taller de navegación se utilizan algoritmos de planeación de rutas como PRM y RRT para resolver un laberinto. Se utiliza un mapa de ocupación binaria y se realiza la simulación de la navegación en *MATLAB* y *CoppeliaSim*.

## Modelo cinemático

El robot asignado para esta tarea es el *Robotnik Summit XL*. Este robot tiene medidas:

* Diámetro de rueda $d=235\ mm$
* Distancia entre ejes (wheel base) de $458\ mm$
* Ancho de $614\ mm$
* Largo $731\ mm$
* Alto $720\ mm$

<figure align="center">
    <img
    src="https://github.com/mobile-robotics-unal/Tarea-2-Navegaci-n-por-planeaci-n/assets/161974694/261a02a0-10d5-4597-ab1b-78cae4a81da4"
    alt="RB-SUMMIT-datasheets"
    width="60%" />
    <figcaption> <b>Figura 1:</b> Medidas RB Summit </figcaption>
</figure>

Se realiza el modelo  cinemático de robot.

Posición:
$$
\begin{align*}
v &= \frac{{v_r + v_l}}{2} \\
w &= \frac{{v_r - v_l}}{l} \\
\end{align*}
$$

$$
\begin{bmatrix}
v \\
w
\end{bmatrix}
=
\begin{bmatrix}
\frac{1}{2} & \frac{1}{2} \\
-\frac{1}{l} & \frac{1}{l}
\end{bmatrix}
\begin{bmatrix}
v_l \\
v_r
\end{bmatrix}
$$

Velocidad:
$$
\begin{align*}
\dot{\theta} &= w \cdot dt \\
\dot{x} &= v \cdot \cos(\dot{\theta}) \cdot dt \\
\dot{y} &= v \cdot \sin(\dot{\theta}) \cdot dt \\
\end{align*}
$$

$$
\begin{bmatrix}
\dot{x} \\
\dot{y} \\
\dot{\theta}
\end{bmatrix}
=
\begin{bmatrix}
\cos(\dot{\theta}) & 0 \\
\sin(\dot{\theta}) & 0 \\
0 & 1
\end{bmatrix}
\begin{bmatrix}
v \\
w
\end{bmatrix}
\cdot dt
$$
<!--
construir modelo cinemático del robot en matlab
-->

## Mapas

Para la generación del mapa, primero se debe asignar un objeto con las dimensiones de ocupación binaria que tendrá el mapa, en este caso de acuerdo al archivo suministrado por los docentes, el mapa tiene unas dimensiones de $52 \times 52$ celdas, además de una asignación de 3 celdas por cada metro, lo cual lleva a tener un mapa cuyas dimensiones son de $\frac{52}{3}\times \frac{52}{3}\,m^2$. El fragmento de código para la implementación de estos datos se presenta a continuación:

```matlab
%% Create Occupancy Map Object
myMap=binaryOccupancyMap((52/3),(52/3),3);
```

Ahora, al haber asignado las características al mapa, se llama la matriz en la cual se encuentra el laberinto o mapa para esta actividad , en esta matriz un 0 representa un espacio vacío, mientras que el 1 hace referencia a un obstáculo.

```matlab
%% Assign array to occupancy values
setOccupancy(myMap,[1,1],sm4b,'grid')
show(myMap)
```

Al ejecutar este código junto con la sección anterior de código, se obtiene el mapa base.

<figure align="center">
    <img
    src="https://github.com/mobile-robotics-unal/Tarea-2-Navegaci-n-por-planeaci-n/assets/161974694/e9b4a38a-4fc9-4b26-968c-e792b2caa9ae"
    alt="Mapa base"
    width="60%" />
    <figcaption> <b>Figura 2: </b>Mapa base </figcaption>
</figure>

Sabiendo las dimensiones del robot se encuentra el radio mínimo que lo encierra siendo este la diagonal del robot, Con este radio se realiza la inflación del mapa obteniendo el siguiente mapa inflado:

<figure align="center"">
    <img
    src="https://github.com/mobile-robotics-unal/Tarea-2-Navegaci-n-por-planeaci-n/assets/25491198/9c436187-75d1-43bc-9a89-71e01e055841"
    width="60%" />
    <figcaption><b>Figura 3:</b> Radio de inflación</figcaption>
</figure>

<figure align="center"">
    <img
    src="https://github.com/mobile-robotics-unal/Tarea-2-Navegaci-n-por-planeaci-n/assets/161974694/1729c636-5331-418a-8a02-57af4ae5b0a6"
    width="60%" />
    <figcaption><b>Figura 4:</b> Mapa inflado </figcaption>
</figure>

### Alternativa

Una alternativa al `BinaryOccupancyMap`, es el `vehicleCostmap` this is part of the *Automated Driving Toolbox* el cual brinda una estructura para modificar parámetros basado en las características del vehículo utilizado para saber más sobre su uso se puede ejecutar el siguiente comando el cual abre la documentación.

```matlab
doc vehicleCostmap
```

Se empieza por definir las características del vehículo en un objeto `vehicleDim` este sera usado en `inflationCollisionChecker` para definir el radio de inflación. `inflationCollisionChecker` tiene la flexibilidad de poder definir un vehículo con varios radios de inflación, esto mejora la precisión en caso de que el vehículo tenga una dimensión mayor a la otra. En el caso del vehículo summit no es necesario y el código utilizado es:

```matlab
%% Create Occupancy Map Object
cellSize = 1/3
costmap = vehicleCostmap(double(sm4b),cellSize=cellSize)

% vehicle dimensions 731x614x720 mm
vehicleDims = vehicleDimensions(0.731 , 0.614, 0.720, "FrontOverhang",0.131,"RearOverhang", 0.131, "Wheelbase",0.458);    %[m]

ccConfig = inflationCollisionChecker(vehicleDims);
costmap.CollisionChecker = ccConfig;
```

Para establecer los parámetros de FrontOverhang y RearOverhang se realiza la resta entre el largo y el wheelBase  dividiendo entre 2 dando como resultado $131\ mm$.

<figure align="center">
    <img
    src="https://github.com/mobile-robotics-unal/Tarea-2-Navegaci-n-por-planeaci-n/assets/25491198/4ed9155d-64a8-4f1a-aed3-cc2cf0e76fed"
    width="60%" />
    <figcaption><b>Figura 5</b>: Parámetros vehicleDim</figcaption>
</figure>

Podemos usar el siguiente comando para visualizar el vehículo y el mapa de colisión inflado.

```matlab
figure
plot(ccConfig)
title('Vehicle characteristics')

figure
plot(costmap)
title('Collision Checking with One Circle')
```

<figure align="center">
    <img
    src="https://github.com/mobile-robotics-unal/Tarea-2-Navegaci-n-por-planeaci-n/assets/25491198/6c23dabc-4afa-42f3-b0d8-b164b6435b4e"
    width="90%" />
    <figcaption><b>Figura 6: </b> Mapa inflado</figcaption>
</figure>

Para la planeación de trayectoria se define una posición inicial en la esquina inferior derecha (1.2, 0.5) y posición final (17,16).

## Planeación PRM

Para la ejecución del algoritmo PRM (Probabilistic Road Map ) se crea un objeto `mobileRobotPRM`. El planner tiene unos valores por defecto que no son los adecuados para el robot, por lo que se deben modificar.

* Número de nodos utilizados: 200 nodos
* Distancia de conexión: 7

```matlab
start = [1.2, 0.5];  
goal = [17,16]; 
map = binaryOccupancyMap(sm4b,Resolution=3); 
radius =sqrt((0.731/2)^2 + (0.614/2)^2)     %radios:= diagonal of the vehicle
inflatedMap = map
inflate(inflatedMap,0.6)

figure 
show(map)

show(inflatedMap)

PRM = mobileRobotPRM(map)
PRM.NumNodes = 200;
PRM.ConnectionDistance = 7;

PRM.findpath(start,goal)

figure 
PRM.show()
```

<figure align="center">
  <img
    src="https://github.com/mobile-robotics-unal/Tarea-2-Navegaci-n-por-planeaci-n/assets/25491198/e6110bf5-ad81-4b20-a13c-b6a41b6c9c8c"
    width="60%" />
  <figcaption><b> Figura 7: </b> Solución algoritmo PRM</figcaption>
</figure>

<!---
función de costo y valor de ruta optima
-->

## Planeación RRT

Para la ejecución del algoritmo RRT (optimal rapidly exploring random tree) se crea un objeto `pathPlannerRRT`. El planner tiene unos valores por defecto que no son los adecuados para el robot, por lo que se deben modificar.

```matlab
startPose = [1.2, 0.5, 90];   % [meters, meters, degrees]
goalPose = [17,16, 0]; 

planner = pathPlannerRRT(costmap)
planner.ConnectionDistance = 4;
planner.MinTurningRadius = 0.5;
planner.MinIterations = 100;
planner.MaxIterations= 80000;
[refPath,tree] = planner.plan(startPose,goalPose);

pathFound = ~isempty(refPath.PathSegments)

if (pathFound) 
    plot(planner,'Tree','on')
end
```

* `ConnectionDistance`: Distance entre nodos consecutivos en el árbol de búsqueda.
* `MinTurningRadius`: Determina el radio de giro más pequeño que el vehículo puede realizar.
* `MinIterations`: Número mínimo de iteraciones.
* `MaxIterations`: Número máximo de iteraciones.

Existe otro parámetro que define el método de conexión entre poses consecutivas, este puede ser `Dubins` o `Reeds-Shepp`. Si solo se permiten movimientos hacia adelante se debe usar `Dubins`. Ya que permite 3 tipos de movimientos:

* Straight (forward)
* Left turn at the maximum steering angle of the vehicle (forward)
* Right turn at the maximum steering angle of the vehicle (forward)

Por otro lado el método `Reeds-Shepp` permite 5 tipos de movimientos:

* Straight (forward or reverse)
* Left turn at the maximum steering angle of the vehicle (forward or reverse)
* Right turn at the maximum steering angle of the vehicle (forward or reverse)

Se obtiene como resultado una trayectoria que se muestra en la siguiente imagen. Se evidencia también todos los demás trayectos recorridos por árbol de búsqueda que generó la trayectoria.

<figure align="center">
    <img
    src="https://github.com/mobile-robotics-unal/Tarea-2-Navegaci-n-por-planeaci-n/assets/25491198/dfce2fe3-9b0c-4aae-83ba-59ca5b6d9065"
    alt="RRT algorithm"
    width="60%" />
    <figcaption><b>Figura 8:</b> Solución RRT</figcaption>
</figure>  

## Simulación en *MATLAB* y *CoppeliaSim*
<!--
Seleccione a su gusto una de las dos rutas obtenidas (PRM o RRT).

6.2. Realice un algoritmo en Matlab para aplicar el controlador PurePursuit al robot y realice
la ruta planeada. Ajuste los par´ametros de control que correspondan. Repasar el algoritmo
Path Following for a Differential Drive Robot.
6.3. Modifique el algoritmo de forma adecuada de manera que el valor de velocidad de ruedas
dado por el controlador sea transmitido a CoppeliaSim y el robot cumpla con la trayectoria
del punto de inicio al punto objetivo.
6.4. Capture la simulaci´on con un v´ıdeo y an´exelo a su informe
-->

## Conclusiones

* Algoritmos de navegación son sensibles a los parámetros seleccionados, por lo que debe seleccionarse aquellos que se ajusten a las características del robot. De no ser asi lo más probable es que no se encuentre una solución.

* La inflación de mapa utilizando un radio que contenga al vehículo permite encontrar la zona segura que el robot puede recorrer. Sin embargo puede ser muy conservador y para robots con dimensiones  que no sean proporcionales se pueden utilizar varios círculos de inflación para aproximarse mejor a su forma.

## Referencias

1. [Documentación inflationCollisionChecker](https://la.mathworks.com/help/driving/ref/driving.costmap.inflationcollisionchecker.html)
2. [Localization and Kalman filters](https://la.mathworks.com/matlabcentral/fileexchange/109485-lessons-on-mobile-robot-localization-and-kalman-filters?s_tid=prof_contriblnk)
3. [Documentación vehicleCostmap](https://la.mathworks.com/help/robotics/ug/probabilistic-roadmaps-prm.html)
4. [Documentación controllerPurePursuit](https://la.mathworks.com/help/robotics/ref/controllerpurepursuit-system-object.html)
