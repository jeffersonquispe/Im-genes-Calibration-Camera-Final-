# Calibration Pattern Detection Algorithm 
CAMERA calibration  consist  on  check  the
physical   parameters   of   the   camera,   as lenses or focal length, with the aim to determine
the geometry of the objects watched, parameters pattern detection is an important stage in Camera Calibration because this bring us the metrics to work with 3D images and Augmented Reality
mds

![Alt text](images/Fig2.png?raw=true "Title")

Solarized dark             |  Solarized Ocean
:-------------------------:|:-------------------------:
![](images/pipelining.png?raw=true "Title")  |  ![](images/pipelining2.png?raw=true "Title")

![Alt text](images/pipelining.png?raw=true "Title")


![Alt text](images/pipelining2.png?raw=true "Title")


![Alt text](images/rms-a.jpg?raw=true "Title")


![Alt text](images/rms-b.jpg?raw=true "Title")


![Alt text](images/ezgif.com-video-to-gif.gif?raw=true "Title")

## Instruccions:

Clone the repository:
```
https://github.com/jeffersonquispe/Im-genes-Calibration-Camera-Final-.git
``` 

### install requeriments to linux

Instalar OpenCV:

```
apt install libopencv-dev
```


### Calibrated Camera Rings
move to download file:

exec the next comand to run on the root:

```
g++ main.cpp utils.cpp `pkg-config opencv --cflags --libs` -o main
./main

```

## Calibrated Camera Chessboard -  Circles
move to download file:

exec the next comand to run:

```
 g++ camera_calibration.cpp `pkg-config opencv --cflags --libs` -o output
 ./output

```
## Document:
To latex document visualization

https://www.overleaf.com/read/khkqtvgrgzkd

## Video:
https://www.youtube.com/watch?v=UnKER4F6m_U
