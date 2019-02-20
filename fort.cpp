#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>   /* File Control Definitions           */
#include <termios.h> /* POSIX Terminal Control Definitions */
#include <unistd.h>  /* UNIX Standard Definitions 	   */ 
#include <errno.h>   /* ERROR Number Definitions           */

using namespace cv;
using namespace std;
int fd;
Mat img=imread("index.png",1);
Mat img1(img.rows,img.cols,CV_8UC1,Scalar(0));
Mat src_gray;
Mat frame;
Mat frame1;
int thresh = 230;
int max_thresh = 255;
RNG rng(12345);
int coun=0;
//variabls of path planning
int d1,d2,d;
double angle;
int front = 1;
//dilation size
int dilation_size=1;
//points of centre of bot and endpoint


int counter=0; //for running the array


Point a1,a2,c1,c2;
//parameters for area and scaling factors
int area,areac;
double sc;
Mat temp;
Mat temp1;

//coordinates of center
int cenx,ceny;
int x[5],y[5];
enum status {
    START_POINT=1,
    IN_BETWEEN_PATH,
    END_POINT,
    BLINK_LED,
    END_POINT_TC
}state=START_POINT;

 void bounding_rect();
 void scaling_factor();
 void MatchingMethod();
 void arena();


void settings(const char *abc)
    {
      fd = open(abc,O_RDWR | O_NOCTTY);	/* ttyUSB0 is the FT232 based USB2SERIAL Converter   */
      usleep(3500000);
			   						/* O_RDWR Read/Write access to serial port           */
									/* O_NOCTTY - No terminal will control the process   */
									/* O_NDELAY -Non Blocking Mode,Does not care about-  */
									/* -the status of DCD line,Open() returns immediatly */                                        
									
        	if(fd == -1)						/* Error Checking */
            	   printf("\n  Error! in Opening ttyUSB0  ");
        	else
            	   printf("\n  ttyUSB0 Opened Successfully ");
       struct termios toptions;         /* get current serial port settings */
       tcgetattr(fd, &toptions);        /* set 9600 baud both ways */
       cfsetispeed(&toptions, B9600);
       cfsetospeed(&toptions, B9600);   /* 8 bits, no parity, no stop bits */
       toptions.c_cflag &= ~PARENB;
       toptions.c_cflag &= ~CSTOPB;
       toptions.c_cflag &= ~CSIZE;
       toptions.c_cflag |= CS8;         /* Canonical mode */
       toptions.c_lflag |= ICANON;       /* commit the serial port settings */
       tcsetattr(fd, TCSANOW, &toptions);
      }
 void send(const char *abc)
      {
       write(fd, abc, 1);
      }

 void colourdetector(int lr,int hr,int lg,int hg,int lb,int hb)
  { 
   int i,j,a,b,c;
imshow("color",img);
   for(i=0;i<img.rows;i++)
    {
     for(j=0;j<img.cols;j++)
     {
       a = img.at<Vec3b>(i, j)[0];
       b = img.at<Vec3b>(i, j)[1];
       c = img.at<Vec3b>(i, j)[2];
    if(((a>=lb)&&(a<=hb))&&((b>=lg)&&(b<=hg))&&((c<=hr)&&(c>=lr)))
            {

                img1.at<uchar>(i, j) = 255;
            }
            else
            {
                img1.at<uchar>(i, j) = 0;
            }
        }
     }
 imshow("window111",img1);
waitKey(400);
    bounding_rect();
	
   }





void bounding_rect()
 {
  img1.copyTo( src_gray);

  /// Convert image to gray and blur it
 printf("bounding");
 //cvtColor( img1, src_gray, CV_BGR2GRAY );
  //blur( src_gray, src_gray, Size(3,3) );

  Mat threshold_output;
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;

  /// Detect edges using Threshold
  threshold( src_gray, threshold_output, thresh, 255, THRESH_BINARY );

  /// Find contours
  findContours( threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

  /// Approximate contours to polygons + get bounding rects and circles
  vector<vector<Point> > contours_poly( contours.size() );
  vector<Rect> boundRect( contours.size() );
  vector<Point2f>center( contours.size() );
 // vector<float>radius( contours.size() );

  for( int i = 0; i < contours.size(); i++ )
     { approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
       boundRect[i] = boundingRect( Mat(contours_poly[i]) );
      }


  /// Draw polygonal contour + bonding rects + circles
  Mat drawing = Mat::zeros( threshold_output.size(), CV_8UC3 );
  for( int i = 0; i< contours.size(); i++ )
     {
       a1=boundRect[i].tl();
       a2=boundRect[i].br();
       if((a1.x!=0)&&(a1.y!=0))
        {
       Scalar color = Scalar( 255,255,255 );
       drawContours( drawing, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point());
       area=contourArea(contours_poly[i]);
	cout<<area<<endl;
      // rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
       c1=boundRect[i].tl();
       c2=boundRect[i].br();
      // cout<<c1<<endl;
       //cout<< c2<<endl;
	 temp = drawing(boundRect[i]);
        }
      }
    scaling_factor(temp);
}





void scaling_factor(Mat temp2)
{
imshow("win",temp);
waitKey(0);
    resize(temp2,temp1,Size(),0,0);
MatchingMethod();   
}

void MatchingMethod()
{
Point matchLoc;
  /// Source image to display
  Mat img_display;
  frame.copyTo( img_display );

  /// Create the result matrix
  int result_cols =  frame.cols - temp1.cols + 1;
  int result_rows = frame.rows - temp1.rows + 1;

 Mat result(frame.rows,frame.cols,CV_8UC1,Scalar(0));

  /// Do the Matching and Normalize
  matchTemplate( frame, temp1, result, CV_TM_SQDIFF );
  normalize( result, result, 0, 1, NORM_MINMAX, -1, Mat() );

  /// Localizing the best match with minMaxLoc
  double minVal; double maxVal; Point minLoc; Point maxLoc;
  

  minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );

  rectangle( img_display, matchLoc, Point( matchLoc.x + temp1.cols , matchLoc.y + temp1.rows ), Scalar::all(0), 2, 8, 0 );
  rectangle( result, matchLoc, Point( matchLoc.x + temp1.cols , matchLoc.y + temp1.rows ), Scalar::all(0), 2, 8, 0 );
 cenx=(int)(2*matchLoc.x+temp1.cols)/2;
 ceny=(int)(2*matchLoc.y+temp1.rows)/2;
 x[coun]=cenx;
 y[coun]=ceny;
coun++;
}

void arena()
{
Mat img2;

Mat element=getStructuringElement(MORPH_RECT,Size(2*dilation_size+1,2*dilation_size+1),Point(dilation_size,dilation_size));
	dilate(frame,img2,element);
  cout<<"arena"<<endl;
  cvtColor( img2, src_gray, CV_BGR2GRAY );
  blur( src_gray, src_gray, Size(3,3) );

  Mat threshold_output;
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;

  /// Detect edges using Threshold
  threshold( src_gray, threshold_output, thresh, 255, THRESH_BINARY );

  /// Find contours
  findContours( threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

  /// Approximate contours to polygons + get bounding rects and circles
  vector<vector<Point> > contours_poly( contours.size() );
  vector<Rect> boundRect( contours.size() );
  vector<Point2f>center( contours.size() );
  vector<float>radius( contours.size() );

  for( int i = 0; i < contours.size(); i++ )
     { approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
       boundRect[i] = boundingRect( Mat(contours_poly[i]) );
      }


  /// Draw polygonal contour + bonding rects + circles
  Mat drawing = Mat::zeros( threshold_output.size(), CV_8UC3 );
  for( int i = 0; i< contours.size(); i++ )
     {
     // if((a1.x!=0)&&(a1.y!=0))
      
       Scalar color = Scalar( 255,255,255 );
       drawContours( drawing, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point());
       int area=contourArea(contours_poly[i]);
      if(area<(7000))
       {cout<<area<<endl;
        areac=area;}
	
      }
	imshow("temp", drawing);
	imshow("dilated",img2);
	waitKey(10);
}

void bot_location()
{
int r,b,g;
int xmin=frame.rows,ymin=frame.cols,ymax=0,xmax=0;
int xmin1=frame.rows,ymin1=frame.cols,ymax1=0,xmax1=0;
int i,j;
Mat copy=frame1.clone();
Mat copy1=frame1.clone();
 for(i=0;i<frame1.rows;i++)
     {
      for(j=0;j<frame1.cols;j++)
       {
        r=frame1.at<Vec3b>(i,j)[2];
	g=frame1.at<Vec3b>(i,j)[1];
	b=frame1.at<Vec3b>(i,j)[0];
        if((r<hrf&& r>lrf) &&(g>lgf && g<hgf)&&(b<hbf && b>lbf))
	{
		copy1.at<Vec3b>(i,j)={255,255,255};
                if((xmin>i)&&(ymin>j))
                  {
                  xmin=i;ymin=j;
                   }
               if((xmax<i)&&(ymin<j))
                  {
                  xmax=i;ymax=j;
                   }
                 

	}
	else
        {
               copy1.at<Vec3b>(i,j)={0,0,0};
         }
        
        if((r<hrb&& r>lrb) &&(g>lgb && g<hgb)&&(b<hbb && b>lbb))
	{
		copy.at<Vec3b>(i,j)={255,255,255};
                if((xmin1>i)&&(ymin1>j))
                  {
                  xmin1=i;ymin1=j;
                   }
               if((xmax1<i)&&(ymin1<j))
                  {
                  xmax1=i;ymax1=j;
                   }
	}
	else
        {
               copy.at<Vec3b>(i,j)={0,0,0};
         }
         cfx=(xmin+xmax)/2;cfy=(ymin+ymax)/2;cbx=(xmin1+xmax1)/2;cby=(ymin1+ymax1)/2;cx=(cfx+cbx)/2;cy=(cfy+cby)/2;
   head_point.x=cfx; end_point.x=x[counter]; tail_point.x=cbx;head_point.y=cfy; end_point.y=y[counter]; tail_point.y=cby;
        }

    }
 
}

void get_path() {
    line(path_img, , end_point, Scalar(255, 255, 255), 2, 8);
}



double dist(Point a, Point b) {
    return (double) sqrt(pow(b.x - a.x, 2) + pow(b.y - a.y, 2));
}



double angle_between(Point a, Point b, Point c) {
    double slope1 = (double) (c.y - b.y) / (a.x - b.x);
    double slope2 = (double) (c.y - b.y) / (c.x - b.x);

    double inter_angle = (double) (atan((slope1 - slope2) / (1 + (slope1 * slope2)))) * 180 / 3.14;

    return inter_angle;
}

void blink_led(int counter)
{ 
 if(counter==0)
send("M");
else if (counter==1)
  {send("N");}
else if (counter==2)
  {send("O");}
else if (counter==3)
  {send("P");}
else if (counter==4)
  {send("Q");}
 else cout<<"error\n"<<endl;
}

void move_bot() {
   // printf("End Point: (%d, %d)\n", end_point.x, end_point.y);
      if (state == START_POINT){
        state== IN_BETWEEN_PATH;
       }
     if (state == BLINK_LED) {
        cout<<"Blinking LED"<<endl;
        blink_led(counter);
        state=IN_BETWEEN_PATH;
        move_bot();
    }

    if (state == IN_BETWEEN_PATH) {
        cout<<"status = IN_BETWEEN_PATH"<<endl;
        char previous;
        double d, d1, d2, angle;
        do {
            d1 = dist(head_point, end_point);
            d2 = dist(tail_point, end_point);
            d = d1 >= d2 ? d1 : d2;
            angle = angle_between(head_point, end_point, tail_point)
            if (angle <= 10 && angle >= -10) {
                if (previous != 'W') {
                    previous = 'W';
                    send("W");
                    printf("W\n");
                }
            }
            if (front) {
                if (angle < -10) {
                    if (previous != 'A') {
                        previous = 'A';
                        send("A");
                        printf("A\n");
                    }
                } else if (angle > 10) {
                    if (previous != 'D') {
                        previous = 'D';
                        send("D");
                        printf("D\n");
                    }
                }
            } else {
                if (angle < -10) {
                    if (previous != 'D') {
                        previous = 'A';
                        send("A");
                        printf("A\n");
                    }
                } else if (angle > 10) {
                    if (previous != 'A') {
                        send("D");
                        printf("D");
                    }
                }
            }
            
        } while (d > 60);
        send("S");
        printf("S\n");
        printf("status = BLINK_LED\n");
        state = BLINK_LED;
        move_bot();
    }
}

int main(int argc, const char **argv)
{
  int cou;
  settings(argv[1]);
  VideoCapture vid(0); // open the default camera
    if (argc < 2) {
        fprintf(stderr, "Please enter the ardunio dev file\n");
        return -1;
    }
    if (!vid.isOpened()) {
        fprintf(stderr, "ERROR!\n");
        return -1;
    }

//Fcout<<"enter the sequence of rgb"<<endl;
vid.read(frame);
arena();
/////////////////////////////////
colourdetector(180,255,30,200,30,225);
while(1)  
 {
	vid.read(frame);
	imshow("win",frame);
	waitKey(10);
        //move_bot();
        
 }
   
  return 0;
}
