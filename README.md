# Tarea 2 - *Navegación por planeación*
***Fundamentos de Robótica móvil - 2024 1S)***.
***Universidad Nacional de Colombia***
***Departamento de Ingeniería Mecánica y Mecatrónica***

Equipo 1:
* _Daniel Esteban Molano_
* _Camilo Esteban Zambrano_
* _Cristhian David Sandoval_
* _Juan Sebastián Dueñas_

<!--
resolución de mapa 3 celdas/m   
entrada inferior izquierda (1.2,0) superior derecha (17.5,16)
-->

En este taller de navecación se utilizan algortimos de planeación de rutas como PRM y RRT para resolver un laberinto. Se utiliza un mapa de ocupación binaria y se realiza la simulación de la navegación en *MATLAB* y *CoppeliaSim*.

## Modelo cinemático

El robot asignado para esta tarea es el *Robotnik Summit XL*. Este robot tiene medidas:
- Diámetro  de rueda d= 235 mm
- Distancia entre ejes (wheel base) de 458 mm
- Ancho de 614 mm
- largo 731 mm 
- Alto 720 mm


<div  align="center">
    <img
    src="https://github.com/mobile-robotics-unal/Tarea-2-Navegaci-n-por-planeaci-n/assets/161974694/261a02a0-10d5-4597-ab1b-78cae4a81da4"
    alt="RB-SUMMIT-datasheets"
    width="60%" />
    <p> <b>Figure 1: </b> medidas RB Summit </p>
</div> 

Se realiza el modelo  cinemático de robot.



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

Al ejecutar este código junto con la sección anterior de código, se obtiene el mapa preliminar.

<div  align="center">
    <img
    src="https://github.com/mobile-robotics-unal/Tarea-2-Navegaci-n-por-planeaci-n/assets/161974694/e9b4a38a-4fc9-4b26-968c-e792b2caa9ae"
    alt="Mapa base"
    width="60%" />
    <p> <b>Figure 2: </b>Mapa base </p>
</div> 

Luego, de acuerdo a las dimensiones del robot, agranda cada posición con un 1 lógico (ocupada) por el radio indicado en metros...



<div  align="center"">
    <img
    src="https://github.com/mobile-robotics-unal/Tarea-2-Navegaci-n-por-planeaci-n/assets/161974694/1729c636-5331-418a-8a02-57af4ae5b0a6"
    width="60%" />
    <p> <b>Figure 3:</b> Mapa inflado </p>
</div> 

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


Para establecer los parámetros de FrontOverhang y RearOverhang se realiza la resta entre el largo y el wheelBase  diviendo entre 2 dando como resultado 131 mm. 

<div align="center">
    <img
    src="https://github.com/mobile-robotics-unal/Tarea-2-Navegaci-n-por-planeaci-n/assets/25491198/4ed9155d-64a8-4f1a-aed3-cc2cf0e76fed"
    width="60%" />
    <p> <b>Figure 4</b>: parámetros vehicleDim </p>
</div>

Podemos usar el siguiente comando para visualizar el vehículo y el mapa de colisión inflado.

```matlab

figure
plot(ccConfig)
title('Vehicle characteristics')

figure
plot(costmap)
title('Collision Checking with One Circle')

```

<div  align="center">
    <img
    src="https://github.com/mobile-robotics-unal/Tarea-2-Navegaci-n-por-planeaci-n/assets/25491198/6c23dabc-4afa-42f3-b0d8-b164b6435b4e"
    width="90%" />
    <p> <b> Figure 5: </b> Mapa inflado </p>
</div>

## Planeación PRM

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
   
        
    %,ConnectionDistance=0.5,MinTurningRadius=0.4
    
    plot(planner,'Tree','on')
    
end
```

* `ConnectionDistance`: Distance entre nodos consecutivos en el árbol de búsqueda.
* `MinTurningRadius`: Determina el radio de giro más pequeño que el vehículo puede realizar.
* `MinIterations`: Número mínimo de iteraciones.
* `MaxIterations`: Número máximo de iteraciones.

Existe otro parámetro que define el método de conexión entre poses consecutivas, este puede ser 'Dubins' o 'Reeds-Shepp'. Si solo se permiten movimientos hacia adelante se debe usar 'Dubins'.
Ya que permite 3 tipos de movimientos:
* Straight (forward)
* Left turn at the maximum steering angle of the vehicle (forward)
* Right turn at the maximum steering angle of the vehicle (forward)

Por otro lado el método 'Reeds-Shepp' permite 5 tipos de movimientos:
* Straight (forward or reverse)
* Left turn at the maximum steering angle of the vehicle (forward or reverse)
* Right turn at the maximum steering angle of the vehicle (forward or reverse)


Se obtine como resultado una trayectoria que se muestra en la siguiente imagen. Se evidencia también todos los demás trayectos recorridos por árbol de búsqueda que generó la trayectoria.

<div align="center">
    <img
    src="https://github.com/mobile-robotics-unal/Tarea-2-Navegaci-n-por-planeaci-n/assets/25491198/dfce2fe3-9b0c-4aae-83ba-59ca5b6d9065"
    alt="RRT algorithm"
    width="60%" />
    <p><b>Figure 6:</b> Solucción RRT   </p>
</div>  

## Simulasión en *MATLAB* y *CoppeliaSim*

## Referencias
1. [doc inflationCollisionChecker](https://la.mathworks.com/help/driving/ref/driving.costmap.inflationcollisionchecker.html)
2. [localization and kalman filters](https://la.mathworks.com/matlabcentral/fileexchange/109485-lessons-on-mobile-robot-localization-and-kalman-filters?s_tid=prof_contriblnk)

