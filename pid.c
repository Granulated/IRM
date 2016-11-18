/*
 * pid.c
 * 
 * Copyright 2015  <ubuntu@udoobuntu>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 * 
 * 
 */



#include "pid.h"

float constrain(float x, float min, float max){
	if(x>max) return max;
	if(x<min) return min;
	return x;
}

int hmin = 0, hmax = 40, smin = 50, smax = 255, vmin = 50, vmax = 255, startTime;

// initialise serial ports
int serialport_init(const char* serialport, int baud)
{
    struct termios toptions;
    int fd;
    
    fd = open(serialport, O_RDWR | O_NONBLOCK);
    
    if (fd == -1)  
        return -1;
    
    if (tcgetattr(fd, &toptions) < 0) 
        return -1;
	
    speed_t brate = baud;
    switch (baud) 
    {
		case 4800:   brate = B4800;   
		break;
		case 9600:   brate = B9600;   
		break;
		#ifdef B14400
		case 14400:  brate = B14400;  
		break;
		#endif
		case 19200:  brate = B19200;  
		break;
		#ifdef B28800
		case 28800:  brate = B28800;  
		break;
		#endif
		case 38400:  brate = B38400;  
		break;
		case 57600:  brate = B57600;  
		break;
		case 115200: brate = B115200; 
		break;
    }
	
    cfsetispeed(&toptions, brate);
    cfsetospeed(&toptions, brate);

    toptions.c_cflag &= ~PARENB;
    toptions.c_cflag &= ~CSTOPB;
    toptions.c_cflag &= ~CSIZE;
    toptions.c_cflag |= CS8;
    toptions.c_cflag &= ~CRTSCTS;
    toptions.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines
    toptions.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl
    toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
    toptions.c_oflag &= ~OPOST; // make raw	
    toptions.c_cc[VMIN]  = 0;
    toptions.c_cc[VTIME] = 0;
    
    tcsetattr(fd, TCSANOW, &toptions);
    if (tcsetattr(fd, TCSAFLUSH, &toptions) < 0) 
        return -1;

    return fd;
}

int serialport_close(int fd)
{
    return close(fd);
}

// write data to serial port
int serialport_write(int fd, const char* str)
{
    int len = strlen(str);
    int n = write(fd, str, len);
	
    if (n != len) 
        return -1;
		
    return 0;
}

// read from serial port
int serialport_read_until(int fd, char* buf, char until, int buf_max, int timeout)
{
    char b[1];  // read expects an array, so we give it a 1-byte array
    int i=0;
    int n=0;
	
    do 
    { 
        n = read(fd, b, 1);  // read a char at a time		
        if (n == -1) 
			return -1;    // couldn't read
			
        if (n == 0) 
        {
            usleep( 1 * 1000 );  // wait 1 msec try again
            timeout--;
            continue;
        }		
        buf[i] = b[0]; 
        i++;
    } while(b[0] != until && i < buf_max && timeout>0);

    buf[i] = 0;  // null terminate the string
	
    return n;
}


// draws crosshair at desired location
IplImage* CrossTarget (IplImage* inImg, int x, int y, int size, int line_thickness)
{	
	IplImage* outImg = inImg;
	
	cvLine(outImg, cvPoint(x-size,y), cvPoint(x+size,y), cvScalar(0,0,0,0), line_thickness, 8, 0);
	cvLine(outImg, cvPoint(x,y-size), cvPoint(x,y+size), cvScalar(0,0,0,0), line_thickness, 8, 0);
	
	return outImg;
}

// Color tracker
int ColorTracking (IplImage* img, int* positionX , int* positionY, int color, int* posX , int* posY, int count, int drawTraj)
{
	
	// Create new image
	IplImage* imgHSV = cvCreateImage (cvGetSize(img), 8, 3);
			
	// Convert source image to HSV
	cvCvtColor(img, imgHSV, CV_BGR2HSV);
	
	// Smooth the gray image by using "cvSmooth" function and using gaussian method (CV_GAUSSIAN)
	cvSmooth(imgHSV,imgHSV,CV_GAUSSIAN);
	
	// shift color range by 20
	cvAddS(imgHSV, cvScalar(20,0,0),imgHSV);
	
	// Create new image 
	IplImage* imgThresh  = cvCreateImage(cvGetSize(img), 8, 1);	
	if (color == 1)
	{
		// Threshold the image so that only blue color pixels are set to 255 (rest is set to 0)
		cvInRangeS(imgHSV, cvScalar(20,100,100,0), cvScalar(40,255,255,0), imgThresh); 
	}		
	else if (color == 2)
	{
		// Threshold the image so that only yellow color pixels are set to 255 (rest is set to 0)
		cvInRangeS(imgHSV, cvScalar(75,100,100,0), cvScalar(130,255,255,0), imgThresh); 
	}
	else if (color == 3)
	{
		// Threshold the image so that only red color pixels are set to 255 (rest is set to 0)
		cvInRangeS(imgHSV, cvScalar(hmin,smin,vmin,0), cvScalar(hmax,smax,vmax,0), imgThresh); 
	}
	
	// Apply image filters:
	cvErode(imgThresh, imgThresh, NULL,1);
	cvDilate(imgThresh, imgThresh, NULL, 1);
	cvErode(imgThresh, imgThresh, NULL,2);
	cvDilate(imgThresh, imgThresh, NULL, 3);
	
	
	// Create image moments
	CvMoments *moments_y = (CvMoments*)malloc(sizeof(CvMoments));
	// Calculate all of the moments
	cvMoments(imgThresh, moments_y, 1);

	// Extract spatial moments
	double moment10_y = cvGetSpatialMoment(moments_y, 1, 0);
    double moment01_y = cvGetSpatialMoment(moments_y, 0, 1);
    double area_y = cvGetCentralMoment(moments_y, 0, 0);

	// Determine the color center position
    *positionX = moment10_y/area_y;
    *positionY = moment01_y/area_y;

	// Display images
     //~ cvShowImage("Original image", img); 
	 //~ cvShowImage("HSV image", imgHSV);
     cvShowImage("Thresholded image", imgThresh); 	
	 //~ cvShowImage("Original image with target", CrossTarget (img, *positionX, *positionY, 20, 2));
	
	if(drawTraj){
		posX[count] = *positionX;
		posY[count] = *positionY;
		//~ printf("1\n");
		
		for(int i = 1; i<=count; i++){
				cvLine(img, cvPoint(posX[i-1], posY[i-1]), cvPoint(posX[i], posY[i]), cvScalar(0,0,0,0), 1, 8, 0);
				//~ printf("%i, %i, %i, %i\n", posX[i-1], posY[i-1], posX[i], posY[i]);
			}
	}
	
    cvReleaseImage(&imgHSV);
    cvReleaseImage(&imgThresh);
    free(moments_y);
	
	return 0;
}


// blob detection algorithm --> see other file
int BlobDetect (IplImage* img, int thresh) 
{
    // add your blob detection algorithm here
		
	return 0;	
}


// template tracking algorithm
int TemplateTracking (IplImage* img, IplImage* blob, int* positionX , int* positionY) {
	// create result array
	CvSize resultSize;
	resultSize.width = cvGetSize(img).width-cvGetSize(blob).width+1;
	resultSize.height = cvGetSize(img).height-cvGetSize(blob).height+1;
	IplImage* result = cvCreateImage (resultSize, 32, 1);
	
	// template matching
	cvMatchTemplate(img, blob, result, CV_TM_SQDIFF);
	
	
	CvPoint maxPoint;
	cvMinMaxLoc( result, NULL, NULL, NULL, &maxPoint);
	*positionX = maxPoint.x + cvGetSize(blob).width;
	*positionY = maxPoint.y + cvGetSize(blob).height;
	
	return 0;
}

// function to connect points in trajectory
int drawTraj (IplImage* img, int* positionX , int* positionY, int* posX , int* posY, int * count) {
	posX[*count] = *positionX;
	posY[*count] = *positionY;
	//~ printf("1\n");
	
	for(int i = 1; i<=*count; i++){
			cvLine(img, cvPoint(posX[i-1], posY[i-1]), cvPoint(posX[i], posY[i]), cvScalar(0,0,0,0), 1, 8, 0);
			//~ printf("%i, %i, %i, %i\n", posX[i-1], posY[i-1], posX[i], posY[i]);
		}
	*count++;
	return 0;
}

// Difference of Gaussian tracking
int DoGTracking (IplImage* img, int* positionX , int* positionY) {
    // Create new images for gaussians
    IplImage* imgG5 = cvCreateImage (cvGetSize(img), 8, 1);
    IplImage* imgG8 = cvCreateImage (cvGetSize(img), 8, 1);

    // Copy green channel
    cvSplit(img, NULL, imgG5, NULL, NULL);

    // Apply gaussian filter for sigma = 5 (sqrt(5^2-3^2)=4) with reduced kernel
    cvSmooth(imgG5, imgG5, CV_GAUSSIAN, 11, 11, 5);
    
    // Apply gaussian filter for sigma = 8 (sqrt(8^2-5^2)=sqrt(39)) with reduced kernel
    cvSmooth(imgG5, imgG8, CV_GAUSSIAN, 13, 13, 6.25);
    
    // Calculate difference of gaussian
    cvSub(imgG8, imgG5, imgG8);
  

    // Get maximum position
    CvPoint maxPoint;
    cvMinMaxLoc( imgG8, NULL, NULL, NULL, &maxPoint);
    *positionX = maxPoint.x;
    *positionY = maxPoint.y;
    
    // clean up memory
    cvReleaseImage(&imgG8);
    cvReleaseImage(&imgG5);

    return 0;
}


// more precise difference of Gaussian tracker
int DoGTrackingPrecise (IplImage* img, int* positionX , int* positionY) {
	// Create new image for extracting channel
	IplImage* imgG = cvCreateImage (cvGetSize(img), 8, 1);
	
    // Create new images for gaussians
    IplImage* imgG2 = cvCreateImage (cvGetSize(img), 32, 1);
    IplImage* imgG3 = cvCreateImage (cvGetSize(img), 32, 1);
    IplImage* imgG5 = cvCreateImage (cvGetSize(img), 32, 1);
    IplImage* imgG8 = cvCreateImage (cvGetSize(img), 32, 1);

    // Copy green channel
    cvSplit(img, NULL, imgG, NULL, NULL);
    
    // Convert to 32bit
    cvConvertScale(imgG, imgG2);

    // Apply gaussian filter for sigma = 2
    cvSmooth(imgG2, imgG2, CV_GAUSSIAN, 0, 0, 2);
    // Apply gaussian filter for sigma = 3 (sqrt(3^2-2^2)=sqrt(5))
    cvSmooth(imgG2, imgG3, CV_GAUSSIAN, 0, 0, sqrt(5));
    // Apply gaussian filter for sigma = 5 (sqrt(5^2-3^2)=4)
    cvSmooth(imgG3, imgG5, CV_GAUSSIAN, 0, 0, 4);
    // Apply gaussian filter for sigma = 8 (sqrt(8^2-5^2)=sqrt(39))
    cvSmooth(imgG5, imgG8, CV_GAUSSIAN, 0, 0, sqrt(39));

    // Calculate difference of gaussian (0.4: magic number)
    cvSub(imgG2, imgG3, imgG2);
    cvSub(imgG8, imgG5, imgG8);
    cvScaleAdd(imgG2, cvScalar(0.4), imgG8, imgG2);
    
    // Get maximum position
    CvPoint maxPoint;
    cvMinMaxLoc( imgG2, NULL, NULL, NULL, &maxPoint);
    *positionX = maxPoint.x;
    *positionY = maxPoint.y;
    
    // clean up memory
    cvReleaseImage(&imgG2);
    cvReleaseImage(&imgG3);
    cvReleaseImage(&imgG5);
    cvReleaseImage(&imgG8);

    return 0;
}


// send commands from the UDOO to the Arduino
int constructCommand (char* command, int u, int motor)
{
	// First byte determines the motor to be moved
	command[0] = motor + '0';
	
	// Second byte determines the direction of movement
	command[1] = u>0?'1':'2';
	u = u<0?-u:u;
	// Third to fifth bytes determine the number of steps to be moved
    // Remember to convert integers to char first
    command[2] = (u/100)%10 + '0';
    command[3] = (u/10)%10 + '0';
    command[4] = u%10 + '0';
	command[5] = '\0';
	
	return 0;
}

// send a command to the Arduino to control motor
int MoveMotor (int fd, float distance, int motor)
{
    // initialize variables here
	char command[6];
	char buf[1];
	
    // The distance must be below the maximum range.
	distance = (distance>5)?5:distance;
	
	// Create appropriate 5 byte command and write it to the serial port
	constructCommand(command, distance/0.005, motor);
	printf("Command: %s, ", command);
	serialport_write(fd, command);
	
  
	// Wait until SAM3x microcontroller responds
	serialport_read_until(fd, buf, '\0', 1, 10000);
    printf("Buf: %s\n", buf);
    
    // check if one of the switches was pressed
	if (buf[0] >= '1' && buf[0] <= '4'){
			printf("Switch %s pressed.\n", buf);
	}
	
	return 0;
}

// move in a rectangular fashion
int MoveMotorRectangular (int fd, float distance, int steps, int useVision, int camIndex)
{
    // initialize your variables here
    FILE *fp;
	CvCapture* capture = 0;
	IplImage* frame = 0;

	int positionX, positionY;
	
    // Create a .txt file
    fp = fopen("RectangularCoord.txt", "w+");
    if (!fp)
    {
		printf("Could not create a .txt file...\n");
		return -1;
	}

	// If vision is used, initialize camera and open a .txt file to store the coordinates
	if(useVision){
		// Get camera
		capture = cvCaptureFromCAM(camIndex);
		
		if (!capture){
			printf("Could not initialize capturing...\n");
			return -1;
		}
        
	
	
		for(int k = 0; k<10;k++) frame = cvQueryFrame(capture);
		DoGTrackingPrecise	 (frame, &positionX , &positionY); 
		printf("X: %d Y: %d\n", positionX, positionY);
		frame = CrossTarget (frame, positionX, positionY, 10, 1);
		fprintf(fp, "%i;%i\n", positionX, positionY); 
		cvShowImage("Original image", frame);
		cvWaitKey(10);	
	
		for(int i = 0; i<steps; i++){
		// Move the stage along all 4 sides
			for(int j = 0; j<5; j++){
				MoveMotor (fd, -distance/5, 1);
				for(int k = 0; k<10;k++) frame = cvQueryFrame(capture);
				DoGTrackingPrecise	 (frame, &positionX , &positionY); 
				printf("X: %d Y: %d\n", positionX, positionY);
				frame = CrossTarget (frame, positionX, positionY, 10, 1);
				fprintf(fp, "%i;%i\n", positionX, positionY); 
				cvShowImage("Original image", frame);
				cvWaitKey(10);	
			}
			for(int j = 0; j<5; j++){
				MoveMotor (fd, -distance/5, 2);
				for(int k = 0; k<10;k++) frame = cvQueryFrame(capture);
				DoGTrackingPrecise	 (frame, &positionX , &positionY); 
				printf("X: %d Y: %d\n", positionX, positionY);
				frame = CrossTarget (frame, positionX, positionY, 10, 1);
				fprintf(fp, "%i;%i\n", positionX, positionY); 
				cvShowImage("Original image", frame);
				cvWaitKey(10);	
			}
			for(int j = 0; j<5; j++){
				MoveMotor (fd, distance/5, 1);
				for(int k = 0; k<10;k++) frame = cvQueryFrame(capture);
				DoGTrackingPrecise	 (frame, &positionX , &positionY); 
				printf("X: %d Y: %d\n", positionX, positionY);
				frame = CrossTarget (frame, positionX, positionY, 10, 1);
				fprintf(fp, "%i;%i\n", positionX, positionY); 
				cvShowImage("Original image", frame);
				cvWaitKey(10);	
			}
			for(int j = 0; j<5; j++){
				MoveMotor (fd, distance/5, 2);
				for(int k = 0; k<10;k++) frame = cvQueryFrame(capture);
				DoGTrackingPrecise	 (frame, &positionX , &positionY); 
				printf("X: %d Y: %d\n", positionX, positionY);
				frame = CrossTarget (frame, positionX, positionY, 10, 1);
				fprintf(fp, "%i;%i\n", positionX, positionY); 
				cvShowImage("Original image", frame);
				cvWaitKey(10);	
			}
		}
	}else{
		for(int i = 0; i<steps; i++){
			MoveMotor (fd, -distance, 1);
			MoveMotor (fd, -distance, 2);
			MoveMotor (fd, distance, 1);
			MoveMotor (fd, distance, 2);
		}
	}
	fclose(fp);
	
	return 0;
}

// PID control
int PID (int fd, int targetPositionX, int targetPositionY, CvCapture* capture, FILE *fp, struct timeval startTime)
{
    // initialize your variables here
    struct timeval currentTime;
    struct timeval prevTime;
	IplImage* frame;
	
	int positionX, positionY;
	float Px, Py, dX, dY;
	int posX[10000], posY[10000];
	float timestamp[10000];
	float errorX, errorY, errorOldX = 0, errorOldY  = 0;
	float intX = 0, intY = 0, intOldX = 0, intOldY = 0;
	
	int i = 0;
	float res = 17;	// px/mm`
	float tol = 2; // tolerance in px
	float Kp = 0.5, Ki = 0, Kd = 1;
	float dt = 1;
	
	float cmdX, cmdY;
	
	
	
	// Grab the frame from the camera
	frame = cvQueryFrame(capture);

    // Find the X and Y coordinates of an object
    DoGTrackingPrecise (frame, &positionX , &positionY);

	// Get current time and initial error
	errorX = (targetPositionX - positionX);
	errorY = (targetPositionY - positionY);
	
	
    // write your do - while loop here
	do
	{
		// Determine the time interval
		gettimeofday(&prevTime, NULL);
		
        // Determine the P, I and D
		Px = Kp * errorX;
		Py = Kp * errorY;
		intX = intOldX + Ki * (errorX + errorOldX)/2*dt;
		intY = intOldY + Ki * (errorY + errorOldY)/2*dt;
		dX = Kd*(errorX - errorOldX)/dt;
		dY = Kd*(errorY - errorOldY)/dt;
		
		// Compute the control command
		cmdX = (Px + intX + dX)/res;
		cmdY = (Py + intY + dY)/res;
		
        // Move the stage axis X
        if( abs(errorX) > tol ){
			MoveMotor (fd, constrain(cmdX, -5, 5), 1);
		}
		// Move the stage axis Y
		if( abs(errorY) > tol ){
			MoveMotor (fd, constrain(cmdY, -5, 5), 2);
		}
		//update old values
		intOldX = intX;
		intOldY = intY;
		errorOldX = errorX;
		errorOldY = errorY;
		
		// Grab the new frame from the camera
		for(int j = 0; j<10; j++){
			frame = cvQueryFrame(capture);
			usleep(10*1000);
		}
		
		// Determine the new position
		DoGTracking (frame, &positionX , &positionY);
    
		// Save the new position as current position
		posX[i] = positionX;
		posY[i] = positionY;
		
		
		
		// Get current time and update the error
		gettimeofday(&currentTime, NULL);
		timestamp[i] = currentTime.tv_sec - startTime.tv_sec + (currentTime.tv_usec - startTime.tv_usec)/1e6;
		dt = currentTime.tv_sec - prevTime.tv_sec + (currentTime.tv_usec - prevTime.tv_usec)/1e6;
		
		fprintf(fp, "%i;%i;%f\n", positionX, positionY, timestamp[i]); 
		
		errorX = (targetPositionX - positionX);
		errorY = (targetPositionY - positionY);	
		printf("ErrorXY: %f/%f, cmXY: %f/%f\n", errorX, errorY, cmdX, cmdY);
		i++;
		
		
	}
	while (abs(errorX) > tol || abs(errorY) > tol);
	
	
	

	
    return 0;
}







