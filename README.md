# Tarea 2 - *Navegación por planeación*
***Fundamentos de Robótica móvil - 2024 1S)***.
***Universidad Nacional de Colombia***
***Departamento de Ingeniería Mecánica y Mecatrónica***

Equipo 1:
* _Daniel Esteban Molano_
* _Camilo Esteban Zambrano_
* _Cristhian David Sandoval_
* _Juan Sebastián Dueñas_

## Modelo cinemático.

## Mapas.
Para la generación del mapa, primero se debe asignar un objeto con las dimensiones de ocupación binaria que tendra el mapa, en este caso de acuerdo al archivo suministrado por los docentes, el mapa tiene unas dimensiones de $52 \times 52$ celdas, además de una asignación de 3 celdas por cada metro, lo cual lleva a tener un mapa cuyas dimensiones son de $\frac{52}{3}\times \frac{52}{3}\,m^2$. El fragmento de código para la implementación de estos datos se presenta a continuación:

```matlab
%% Create Occupancy Map Object
myMap=binaryOccupancyMap((52/3),(52/3),3);
```

Ahora, al haber asignado las características al mapa, se llama la matriz en la cual se encuentra el laberinto o mapa para esta actividad , en esta matriz un 0 representa un espacio vacío, mientras que el 1 hace referencia a un obstaculo.

```matlab
%% Assign array to occupansy values
setOccupancy(myMap,[1,1],sm4b,'grid')
show(myMap)
```

Al ejecutar este código junto con la sección anterior de código, se obtiene el mapa preliminar.

*Figura 1: Mapa preliminar.*

![mapa_base](https://github.com/mobile-robotics-unal/Tarea-2-Navegaci-n-por-planeaci-n/assets/161974694/e9b4a38a-4fc9-4b26-968c-e792b2caa9ae)

Luego, de acuerdo a las dimensiones del robot, agranda cada posición con un 1 lógico (ocupada) por el radio indicado en metros...

![robotnik_dimensiones](https://github.com/mobile-robotics-unal/Tarea-2-Navegaci-n-por-planeaci-n/assets/161974694/261a02a0-10d5-4597-ab1b-78cae4a81da4)

![mapa_inflate](https://github.com/mobile-robotics-unal/Tarea-2-Navegaci-n-por-planeaci-n/assets/161974694/1729c636-5331-418a-8a02-57af4ae5b0a6)


## Planeación PRM.

## Planeación RRT.

## Simulasión en *MATLAB* y *CeppeliaSim*
