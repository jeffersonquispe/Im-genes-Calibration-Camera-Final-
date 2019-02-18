# Calibration Pattern Detection Algorithm 
CAMERA calibration  consist  on  check  the
physical   parameters   of   the   camera,   as lenses or focal length, with the aim to determine
the geometry of the objects watched, parameters pattern detection is an important stage in Camera Calibration because this bring us the metrics to work with 3D images and Augmented Reality
mds

![Alt text](images/Fig2.png?raw=true "Title")

## Instruccions:

Clone the repository:
```
git clone https:https://github.com/jeffersonquispe/ImagenesSemana1.git
``` 

### install requeriments to linux

Instalar OpenCV:

```
apt install libopencv-dev
```


### Calibrated Camera RIngs
move to download file:

exec the next comand to run:

```
 g++ preprocessing.cpp -o output `pkg-config --cflags --libs opencv`

```

exe:
```
./output
```
## Document:
To latex document visualization

https://www.overleaf.com/read/hfjrystrfdfv


## Calibrated Camera Chessboard -  Circles
move to download file:

exec the next comand to run:

```
 g++ -std=c++11 -O3 main.cpp addfunctions.cpp `pkg-config opencv --cflags --libs` -o main && 


```

exe:
```
./main
```
