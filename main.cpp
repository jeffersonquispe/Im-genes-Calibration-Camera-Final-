
#define WinName "frame"
#include "global.h"

#define path "../2/"
//#define video_path "anillos.avi"
#define video_path "anillos.mp4"
//=====================================VARIABLES GLOBALES===================================================
int patternType = RINGS_GRID;
int noImages = 20; // Numero de imagenes para la Calibración
int noIterations = 25;
float squareSize = 0.04540;//meters
cv::Size imgPixelSize = Size(640,480); // Tamaño de la imagen
cv::Size patternSize = cv::Size(5,4);
cv::Size detectionGridSize = Size(4,3);
int noOutputImages = 20;
cv::Mat gray;
cv::Mat mat1;
cv::Mat mat2;
cv::Mat mat3;
cv::Mat mat4;
cv::Mat In;
bool autoselector=false;
//==========================================================================================================

//==============================================FUNCIONES===================================================
//Visualizar varias ventanas
void ShowManyImages(string title, int nArgs, ...) {
  int size;
  int i;
  int m, n;
  int x, y;

  // w - Maximum number of images in a row
  // h - Maximum number of images in a column
  int w, h;

  // scale - How much we have to resize the image
  float scale;
  int max;

  // If the number of arguments is lesser than 0 or greater than 12
  // return without displaying
  if(nArgs <= 0) {
      printf("Number of arguments too small....\n");
      return;
  }
  else if(nArgs > 14) {
      printf("Number of arguments too large, can only handle maximally 12 images at a time ...\n");
      return;
  }
  // Determine the size of the image,
  // and the number of rows/cols
  // from number of arguments
  else if (nArgs == 1) {
      w = h = 1;
      size = 300;
  }
  else if (nArgs == 2) {
      w = 2; h = 1;
      size = 300;
  }
  else if (nArgs == 3 || nArgs == 4) {
      w = 2; h = 2;
      size = 300;
  }
  else if (nArgs == 5 || nArgs == 6) {
      w = 3; h = 2;
      size = 200;
  }
  else if (nArgs == 7 || nArgs == 8) {
      w = 4; h = 2;
      size = 200;
  }
  else {
      w = 4; h = 3;
      size = 150;
  }

  // Create a new 3 channel image
  Mat DispImage = Mat::zeros(Size(100 + size*w, 60 + size*h), CV_8UC3);

  // Used to get the arguments passed
  va_list args;
  va_start(args, nArgs);

  // Loop for nArgs number of arguments
  for (i = 0, m = 20, n = 20; i < nArgs; i++, m += (20 + size)) {
      // Get the Pointer to the IplImage
      Mat img = va_arg(args, Mat);

      // Check whether it is NULL or not
      // If it is NULL, release the image, and return
      if(img.empty()) {
          printf("Invalid arguments");
          return;
      }

      // Find the width and height of the image
      x = img.cols;
      y = img.rows;

      // Find whether height or width is greater in order to resize the image
      max = (x > y)? x: y;

      // Find the scaling factor to resize the image
      scale = (float) ( (float) max / size );

      // Used to Align the images
      if( i % w == 0 && m!= 20) {
          m = 20;
          n+= 20 + size;
      }
      // Set the image ROI to display the current image
      // Resize the input image and copy the it to the Single Big Image
      Rect ROI(m, n, (int)( x/scale ), (int)( y/scale ));
      Mat temp; resize(img,temp, Size(ROI.width, ROI.height));
      temp.copyTo(DispImage(ROI));
  }

  // Create a new window, and show the Single Big Image
  namedWindow( title, 1 );
  imshow( title, DispImage);

  // End the number of arguments
  va_end(args);
}


//Realizar reconocimiento de patrones y otras FUNCIONES
bool findRingsGridPattern(cv::Mat Input, cv::Size size, std::vector<cv::Point2f>& points, bool& isTracking, std::vector<cv::Point2f>& oldPoints){
		//prepreocessing

    cv::cvtColor(Input,gray,CV_BGR2GRAY);
    GaussianBlur(gray,gray,Size(3,3),0);
    adaptiveThreshold(gray,gray,255,ADAPTIVE_THRESH_GAUSSIAN_C,THRESH_BINARY,41,6);
		//copia de adaptative folder (no dibuja)
		//mat2=gray.clone();
    //deteccion de elipses
    vector<vector<Point>> contours;
    vector<Vec4i> hierachy;
    findContours(gray.clone(),contours,hierachy,CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE,Point(0,0));
    vector<RotatedRect>minEllipse(contours.size());
    cv::cvtColor(gray,gray, CV_GRAY2BGR); // <------------- Cambiamos a color
    // Fitear una elipse a los contornos detectados
    for( int i = 0; i < contours.size(); i++ ){
        //minRect[i] = minAreaRect( Mat(contours[i]) );
        //Scalar color( rand()&255, rand()&255, rand()&255 );
        drawContours( gray, contours, i, Scalar(0,255,255), 1, 8);
        if( contours[i].size() > 4 ){
            minEllipse[i] = fitEllipse( Mat(contours[i]) ); }
        //if( contours[i].size() == 4)
        //    minEllipse[i] = cv::minAreaRect( Mat(contours[i]) );
    }

    //Filtrar las ellipses
    vector<RotatedRect> selected;
    for( int i = 0; i< contours.size(); i++ ){
        float w = minEllipse[i].size.width;
        float h = minEllipse[i].size.height;
        float c_x = minEllipse[i].center.x;
        float c_y = minEllipse[i].center.y;
        float dif = w - h;

        //ellipse( gray, minEllipse[i], Scalar(0,0,255), 1, 8 );
        if(abs(dif) < 40){ //-->>> CAMBIAR ESTE PARAMETRO PARA FILTRAR LAS ELIPSES
            if(hierachy[i][2] != -1){ // Si el Contour tiene Hijo que hijo sea unico
                int child_index = hierachy[i][2];
                if(hierachy[child_index][0] == -1 && hierachy[child_index][1] == -1 && hierachy[child_index][2] == -1){
                    selected.push_back(minEllipse[i]);
                    selected.push_back(minEllipse[child_index]);
                    ellipse( gray, minEllipse[i], Scalar(0,0,255), 1, 8 );
                    ellipse( gray, minEllipse[child_index], Scalar(0,0,255), 1, 8 );
                }
            }
        }
    }

    //Como minimo debemos capturar 40 elipses para continuar

    if(selected.size() < 40) return false;

    //cv::imshow("g",gray);
    //Extraemos los centros de todas las elipses Seleccionadas
    //cout << "Number Selected Ellipsises: " << selected.size() << endl;
    vector<Point2f> centers;
    for( int i = 0; i < selected.size(); i++ ){
        centers.push_back(selected[i].center);
    }

    //Eliminamos Duplicados y nos quedamos con el promedio de Centros similares('permanezcan a un conjunto de elipses')
    vector<Point2f> CPs;
    CPs = getControlPoints(centers);
    for(int i = 0; i < CPs.size();i++)
        circle(gray,CPs[i],2,Scalar(255,255,0),3,5);

    GaussianBlur(gray,gray, Size(5, 5), 0);
		mat1=gray.clone();
    //cv::imshow(windowGray,gray);
    if(CPs.size() < 20) return false;


    //cv::imshow(windowGray,gray);

    //caculando la mediana
    vector<Point2f> tmpCPs;
    vector<Point2f> c1 = CPs;
    vector<Point2f> c2 = CPs;
    sort(c1.begin(),c1.end(),cmpx);
    sort(c2.begin(),c2.end(),cmpy);

    int n = c1.size()/2;
    float xm = c1[ n ].x;
    float ym = c2[ n ].y;

    // Esta parte debe ayudar a validar nuestros PC
    // Como maximo deberiamos tener 20
    // Ademas debe ordenarlos de la sgte manera fila por fila, columna por columna
    // Debe retornar true si se cumplen los requisitos
    int r;
    for(r = 1; r < 200; r++){
        int count = 0;
        for(int i = 0; i < CPs.size(); i++){
            if(dist(Point2f(xm,ym),CPs[i]) < r)
                count++;
        }
        // Hace una verificacion fuerte de 20 elementos!!!!
        if(count >= 20){
            break;
        }

    }

    //Displaying the range Circles
    int padding = 30;
    for(int i = 0; i < CPs.size(); i++)
        if(dist(Point2f(xm,ym),CPs[i]) < r +padding)
            tmpCPs.push_back(CPs[i]);
    //cout << "Filtered Control Points(Median): "<< tmpCPs.size() <<endl;

    CPs.clear();
    CPs = tmpCPs;

    // for(int i = 0; i < CPs.size();i++){
    //     circle(gray,CPs[i],5,Scalar(255,0,0),3,8);
    // }

    //cv::imshow(windowGray,gray);
    if(CPs.size() < 20) return false;

    // ORDERING AND SETTINGS POINTS ( Tracking )
    std::vector<Point2f> trackedPoints;
    int numTrackedItems = 20;

    if(isTracking){
        trackedPoints.resize(numTrackedItems);
        std::vector<float> distances;
        for(int k = 0; k < numTrackedItems;k++){
            Point2f tmp = oldPoints[k]; // Aqui esta el error
            float min = 100000.0f;
            int index = 0;
            for(int i = 0; i < CPs.size(); i++){
                if( min > dist(oldPoints[k],CPs[i]) ){
                    min = dist(oldPoints[k],CPs[i]);
                    index = i;
                }
            }
            distances.push_back(dist(oldPoints[k],CPs[index]));
            trackedPoints[k] = CPs[index]; // Actualizamos la posicion de los puntos
        }
        bool isCorrect = true;

        float dstddev = StandarDesviation(distances);

        //Aumentar validaciones en esta zona
        if(dstddev > 3.0f)
            isCorrect = false;

        //Revisar que np haya duplicados
        for(int i = 0; i < trackedPoints.size()-1;i++)
            for(int j = i+1; j < trackedPoints.size();j++)
                if(trackedPoints[i] == trackedPoints[j])
                    isCorrect = false;

        //Si no es correcto el mandar señal para tratar de capturar el tracking
        if(!isCorrect){
            cout << "Couldnt keep tracking\n";
            isTracking = false;
        }
    }
    //isTracking = false;
    //if(!isTracking){
    else{
        //cout << "Start Tracking\n";
        // Buscamos encontrar el patron, devolvemos solo el numero correspondiente de nodos
        // Ademas Ordenamos los nodos, primero por fila, luego por columna
        //VizVector(CPs);
        bool patternWasFound = FindRingPattern(CPs,gray,4,5);
        //patternWasFound = false;
        //Esta parte del codigo debe enviar 20 puntos Ordenados y en grilla hacia TrackedPoints
        //En cualquier otro caso debe pasar al siguiente frame y tratar otra vez
        //El ordenamiento a pasar es el siguiente

        if(patternWasFound){
            trackedPoints.clear();
            for(int i = 0; i < numTrackedItems; i++){
                trackedPoints.push_back(CPs[i]);
                circle(gray,CPs[i],5,Scalar(0,255,255),3,8);
            }
            isTracking = true;
        }
    }

    //cv::imshow(windowGray,gray);

    // Copiamos el vector a points que seran nuestros CPs
    points = trackedPoints;
    return isTracking;
}
//==========================================================================================================

// FUNCION MAIN

int main(){
	cv::Mat frame;
	std::vector< std::vector<cv::Point3f> > objPoints; // Puntos de nuestro objeto(Patron de calibracion)
	// Suponemos que el patron se encuentra de forma paralela a la camara, y a una misma altura
	std::vector< std::vector<cv::Point2f> > imgPoints; // 2D Points en la Imagen(Pixels)
	// Inital Calibration
	objPoints.resize(1);
	calcBoardCornerPositions(cv::Size(5,4),squareSize,objPoints[0],patternType);
	objPoints.resize(noImages,objPoints[0]);
	bool isTracking; // Variable para ayudar a la función FindRingGridPattern
	std::vector<cv::Point2f> oldPoints; // Punto usados para el Tracking en RingGrid
	cv::namedWindow(windowName,0);
	cv::resizeWindow(windowName,640,480);
	//Variables para guardar los Valores de Correccion
	double rms;
  cv::Mat cameraMatrix = cv::Mat::eye(3,3,CV_64F); // Matriz para guardar la camera Intrinsics
  cv::Mat distCoeffs = cv::Mat::zeros(8, 1,CV_64F); // Aqui guardamos los coeficientes de Distorsion
  std::vector<cv::Mat> rvecs,tvecs; //Vectores de rotacion y de traslacion para cada frame
	Mat cloud = Mat::zeros( 480, 640, CV_8UC3 );

		VideoCapture cap(video_path);
		if( !(cap.isOpened()) ){
			cout << "No se pudo leer\n";
			return 0;
		}
		//==============================parametros para autoselector=========================
		//===================================================================================
		//==============================parametros para autoselector=========================
		//===================================================================================
		//==============================parametros para autoselector=========================
		//===================================================================================
		//==============================Parametros para autoselector=========================
		std::vector< cv::Mat > voMats;
		// Guarda los cuadrantes que fueron contabilizados en cada uno de los frames capturados
		std::vector< std::vector<bool> > voAffections;
		//===================================================================================
		//=======================Autoselector de imagenes=====================================

    if(autoselector==true){
  		//Captura frame por frame

  		for(;;){
  			cv::Mat frame;
  			cap >> frame;

  			if(frame.empty()) break; // Verificación de que hayamos capturado un frame
  			std::vector<cv::Point2f> PointBuffer;
  			isTracking = false; // Para que busque en todas las imagenes

  			bool found = findRingsGridPattern(frame,patternSize, PointBuffer, isTracking,oldPoints);

  			if(found){
  				//cv::drawChessboardCorners(frame,patternSize, PointBuffer,found);
  				// Registramos el frame
  				voMats.push_back(frame);
  				// Registrar los cuadrantes donde afecto la detección
  				voAffections.push_back( calc_affection(PointBuffer, imgPixelSize, detectionGridSize) );
  			}

  			ShowManyImages("Processing", 2, frame,mat1);
  			if(waitKey(10) == 27) break;
  		}

  		// Numero total de frames donde se detecto el patron
  		cout << "Numero Total de Frames Capturados: " << voMats.size() << endl;

  		// FOR(i,voMats.size()){
  		// 	cv::imshow(WinName,voMats[i]);
  		// 	if(waitKey(10) == 27) break;
  		// }
  		// Obtenemos la mejor combinación de frames
  		std::vector<int> bestFrames = calc_BestFrameCombination(voAffections,noOutputImages);
  		// Escribimos los frames correspondientes
  		FOR(i, bestFrames.size()){
  			std::string str = path + std::to_string(i)+".jpg";
  			bool captured = cv::imwrite(str,voMats[ bestFrames[i] ]);
  			if(!captured) cout << "No se pudo Capturar Imagen " << i << endl;
		}
		//===================================================================================
    }
    //Capturamos las matrices

	FOR(i,noImages){
		string filename = path + std::to_string(i)  +  ".jpg";

		frame = cv::imread(filename,CV_LOAD_IMAGE_COLOR);
		std::vector<cv::Point2f> PointBuffer;
		isTracking = false; // Para que busque en todas las imagenes
		bool found = findRingsGridPattern(frame,patternSize, PointBuffer, isTracking,oldPoints);
		//VizVector(PointBuffer);
		if(found){
			imgPoints.push_back(PointBuffer);
			cv::drawChessboardCorners(frame,patternSize, PointBuffer,found);
		}
		else{
			cout << "Patron no encontrado\n";
		}

		for(int i=0;i<PointBuffer.size();i++){
					circle(cloud,PointBuffer[i],3,Scalar(180,0,200),-1);
		}

		cv::imshow(windowName,frame);
		//cv::imshow("cloud",cloud);
		ShowManyImages("first calibration", 2, frame,cloud);
		int key = cv::waitKey(100);
		bool c = true;
		switch(key){
			case 27:{
				c = false;
				break;
			}
			case 'c': //Pasar al siguiente frame
				break;
			default:
				break;
		}
		if(c) continue;
		else break;
	}

	// Calibracion Iterativa
	vector<float> rms_set;
	std::vector<cv::Point2f> fronto_corners = getFrontoParallelCorners(imgPixelSize,patternSize);
	FOR(it,noIterations)
	{
		// Limpiamosc variables
		rvecs.clear(); tvecs.clear();
		// cout << imgPoints.size() << endl;
		// Comenzamos la Calibracion
		rms = cv::calibrateCamera(objPoints,imgPoints, imgPixelSize,cameraMatrix,distCoeffs,rvecs,tvecs);
		std::cout << it << " " << cameraMatrix.at<double>(0,0) << " " << cameraMatrix.at<double>(1,1) <<
		" " << cameraMatrix.at<double>(0,2) << " " << cameraMatrix.at<double>(1,2) << " " << rms << " "<<endl;
    //VizVector(rvecs);
    //cout << "tvecs" << endl;
    //VizVector(tvecs);
		//cout << "El error de reproyeccion obtenido fue de " << rms << endl;
		//cout << "Matriz Intrinseca:" << endl << cameraMatrix << endl;
		//cout << "Coeficientes de Distorsion: " << endl << distCoeffs << endl;
		rms_set.push_back(rms);
		std::vector< std::vector<cv::Point2f> > imgPoints2;
		vector<float> avrColi; // Para sacar un promedio de las colinealidades del vector
		Mat imagen=imread("presentacion.png",CV_LOAD_IMAGE_COLOR);
		// Mostrar imagens sin Distorsion
		FOR(i,noImages){

			string filename = path + std::to_string(i)  +  ".jpg";
			frame = cv::imread(filename,CV_LOAD_IMAGE_COLOR);
			cv::Mat temp = frame.clone();
			cv::Mat OptimalMatrix = cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, Size(640,480), 1.0);
			cv::undistort(temp,frame,cameraMatrix,distCoeffs,OptimalMatrix);
			std::vector<cv::Point2f> PointBuffer;
			cv::undistortPoints(imgPoints[i], PointBuffer, cameraMatrix, distCoeffs, cv::noArray(),OptimalMatrix);
      //std::cout << "i: " <<i<< '\n';
			//VizVector(PointBuffer);

			//cv::drawChessboardCorners(frame,patternSize, PointBuffer,true);
			float m = getAvgColinearityFromVector( PointBuffer, patternSize);
			//std::cout << "m" << m << '\n';
			avrColi.push_back(m);
      //cin.get();
			// Almacenamos solo cuatro esquinas
			std::vector<cv::Point2f> corners1 = extractCorners(PointBuffer,patternSize);

	    cv::Mat H = cv::findHomography(corners1,fronto_corners);
					//VizVector(fronto_corners);
	        //cout << "H:\n" << H << endl;
	        //cout << "H.inv:\n" << H.inv() << endl;
	        //Transformacion Fronto Parallel
	    cv::Mat imgWarp;
	    cv::warpPerspective(frame,imgWarp,H,Size(320,240));
			//cv::imshow("a", frame);
	    PointBuffer.clear();
			isTracking = false; // Para que busque en todas las imagenes

			//VizVector(PointBuffer);
			bool found2 = findRingsGridPattern(imgWarp,patternSize, PointBuffer, isTracking,oldPoints);
      //std::cout << "i: "<< i<< '\n';
      //std::cout << "found "<< found2<< '\n';
			//VizVector(PointBuffer);
			//cin.get();
      //std::cout << "noIamges" <<noImages<< '\n';
			waitKey(1);
			if(!found2){
				//cv::drawChessboardCorners(imgWarp,patternSize, PointBuffer,found);
        //noImages=noImages+1;
				cout << "no se pudo enconrtar el patron en la proyeccion FrontoParallel\n";
        //continue;
				//return 0;
			}
			//Transformacion Fronto Parallel Inversa
			cv::Mat imgWarp_inv;
	    cv::warpPerspective(imgWarp,imgWarp_inv,H.inv(),frame.size());
	    vector<Point2f> points_buffer2;
	    cv::perspectiveTransform( PointBuffer, points_buffer2, H.inv() );
	    std::vector<cv::Point2f> corrected_points;
      // Distorsión Inversa
	    cv::undistortPoints(points_buffer2,corrected_points,OptimalMatrix,-distCoeffs,cv::noArray(),cameraMatrix);
	    //cv::drawChessboardCorners(imgWarp_inv, patternSize, corrected_points, true);
	    //cv::drawChessboardCorners(imgWarp_inv, patternSize, imgPoints[i], true);
	    imgPoints2.push_back(corrected_points);
			ShowManyImages("Image2", 4, frame,imgWarp,imgWarp_inv,gray);
		}

		FOR(i,noImages)
			FOR(j,patternSize.width * patternSize.height){
				imgPoints[i][j].x = (imgPoints[i][j].x +  imgPoints2[i][j].x) / 2.0;
				imgPoints[i][j].y = (imgPoints[i][j].y +  imgPoints2[i][j].y) / 2.0;
			}
		 //cout << printAvgColinearity(avrColi) << endl;
		//rms = cv::calibrateCamera(objPoints,imgPoints2, imgPixelSize,cameraMatrix,distCoeffs,rvecs,tvecs);
		//cout << "El error de reproyeccion obtenido fue de " << rms << endl;
	}
	//std::cout << std::min_element( std::begin(rms_set), std::end(rms_set) ) << std::endl;
	//std::sort( rms_set.begin(), rms_set.end() );
	// cout << "El menor rms obtenido: "<< rms_set[0] << endl;
	//terminando el programa
    cv::destroyAllWindows();

	return 0;
}

//para ejecutar
//g++ -std=c++11 -O3 main.cpp utils.cpp `pkg-config opencv --cflags --libs` -o main && ./main
