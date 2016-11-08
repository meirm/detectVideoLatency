/*
***********************************
*
* Program name: detectVideoLatency
*
* Detect latency by examinating 
* the lag between the serial signal 
* and the capture of a LED that was
* light on the event.
*
* Version: 0.9
*
* Author: Meir Michanie <meirm@riunx.com>
*
* Usage: This program is to be used together with a
* serial device that sends characters at 115200 bps
*
* latest version of this file can be obtained at 
* http://www.riunx.com/public/detectVideoLatency-latest.zip
*
* to compile run:
* g++ -O3 FPS.cpp detectVideoLatency.cpp `pkg-config --libs --cflags opencv`  -std=gnu++11 -lboost_program_options  -pthread -o detectVideoLatency
* 
*
* for more usage info run:
* detectVideoLatency --help
*
*  ./detectVideoLatency --video 1 --window 1
* window set to 1.
* video set to 1.
* image was not set. Using default
* device was not set. Using default
* HIGHGUI ERROR: V4L/V4L2: VIDIOC_S_CROP
* I read 1 bytes, char 0
* I read 1 bytes, char 1
* Total time elapsed: 18.929
* FPS: 29.3201

***********************************
*/

#include <errno.h>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <string.h>
#include <cstdlib>
#include <thread>
#include "opencv2/core/core_c.h"
#include "opencv2/core/core.hpp"
#include "opencv2/flann/miniflann.hpp"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/video.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/ml/ml.hpp"
#include "opencv2/highgui/highgui_c.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/opencv.hpp"
#include "FPS.hpp"
#include <boost/program_options.hpp>
#include <iterator>


using namespace cv;
using namespace std;
namespace po = boost::program_options;

// Default values
int capfps=30;
string port="/dev/ttyUSB0";
int videoport =0;
string imagefile="evidence.jpg";
int showWindow=0;
int saveVideo=0;
int useDummyTTY=0;
int x=640;
int y=480;
int outFormat=0;
float pid = 0.9;
int flag=-1;
int serial; //fd

Mat frame[3];
Mat *currentFrame;
int current=0;

//outFormat= CV_FOURCC('M', 'J', 'P', 'G');
//outFormat= 0; // uncompressed
//outFormat= CV_FOURCC('Y', 'U', 'Y', 'V');
//int fourCC_code = CV_FOURCC('M','P','V','4');
//int fourCC_code = CV_FOURCC('M','J','P','G'); // M-JPEG codec (may not
//be reliable).
//int fourCC_code = CV_FOURCC('P','I','M','1'); // MPEG 1 codec.
//int fourCC_code = CV_FOURCC('D','I','V','3'); // MPEG 4.3 codec.
//int fourCC_code = CV_FOURCC('I','2','6','3'); // H263I codec.
//int fourCC_code = CV_FOURCC('F','L','V','1'); // FLV1 codec.
//outFormat = static_cast<int>(cap->get(CV_CAP_PROP_FOURCC));
	
VideoCapture *cap;
Mat gray_image;
int delay = 1000 / capfps;

void sleepMillis(int millis){
	for(int i=0 ; i < millis; i++){
	usleep(1000);
	}
}

void sleep(int secs){
	for(int i=0 ; i < secs; i++){
	sleepMillis(1000);
	}
}


void ShowFrames(){
	if (showWindow == 1){
		cvNamedWindow( "Image", CV_WINDOW_AUTOSIZE );
		while( flag != 2){
			if (!currentFrame->empty()) imshow("Image", *currentFrame);
			if (waitKey(1) >= 0 ){
					flag=2;
					break;
				}
		}
		cvDestroyWindow( "Image" );
	}
}
void SaveVideoThread() {
	FPS fpsout;
	
	if (saveVideo ==1 ){
		cv::VideoWriter output_cap;
		output_cap.open(imagefile, outFormat,capfps,cv::Size(x,y));
	    if (!output_cap.isOpened()){
	        cout << "!!! Output video could not be opened" << endl;
	        return;
		}
	
	struct timespec realstart,start, finish;
	fpsout.start();
	cout << "Saving video" << endl;
	unsigned long counter=0;
	clock_gettime(CLOCK_MONOTONIC, &realstart);
	while (true) {
	    if (!currentFrame->empty()) {
			//clock_gettime(CLOCK_MONOTONIC, &start);
			//do image processing functions (these worked before implementing threads and are not causing errors)
			//cvtColor( capF, gray_image, CV_BGR2GRAY );
			//Mat gray_image_small;
			//resize(capF,gray_image,cv::Size(x,y));
			output_cap.write(*currentFrame);
			fpsout.update();
	        counter++;
			fpsout.stop();
			if (counter < 3){
				sleepMillis(delay);
			}else{
				float howmuchWait = delay *((fpsout.fps() / float(capfps)) * pid + (1 - pid)*(fpsout.fps() - float(capfps)));
				if (int(howmuchWait) > 0)sleepMillis(int(howmuchWait));
			}
			if (flag == 2){
				break;
			}
		}
	}
	fpsout.stop();
	output_cap.release();
	
    cout << "Output time elapsed: " << fpsout.elapsed() << endl;
	cout << "Output FPS: " << fpsout.fps() << endl;
	}
}

int set_interface_attribs (int fd, int speed, int parity)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                printf ("error %d from tcgetattr", errno);
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 0.5;            // 0.05 seconds read timeout
        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
                printf ("error %d from tcsetattr", errno);
                return -1;
        }
        return 0;
}



void set_blocking (int fd, int should_block)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                printf ("error %d from tggetattr", errno);
                return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
                printf ("error %d setting term attributes", errno);
}


void init(){
    serial = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    set_interface_attribs (serial, B115200, 0);  // set speed to 115,200 bps, 8n1 (no parity)
    //set_blocking (fd, 0);                // set no blocking
    set_blocking (serial, 1);                // set blocking
    if (serial < 0){
        //Error
      }else{
        //run loop
    }
}

void serial_loop(){
       unsigned char finished=0;
	char buf [1];
        while(flag!=1){
            int n = read (serial, buf, sizeof buf);  // read up to 100 characters if ready to read
            if (n > 0 ) {
                printf("I read %d bytes, char %c\n",n,*buf);
		flag++;
           }
        }
    sleep(4);
	flag=2;

}

void arduino(){
	init();
	serial_loop();
}


void CaptureFrames(){
	  cap= new VideoCapture(videoport);
    // open the default camera, use something different from 0 otherwise;
    // Check VideoCapture documentation.
    if(cap->isOpened()){
	cout << "Capturing" << endl;
    cap->set(CV_CAP_PROP_FRAME_WIDTH,x);
    cap->set(CV_CAP_PROP_FRAME_HEIGHT,y);
    FPS fps;
    fps.start();
    for(;;){
		  
		  current++;
		  current%= 3;
          *cap >> frame[current];
          fps.update();
          if( frame[current].empty() ) break; // end of video stream
		  if(flag == 0){
			flip(frame[current],frame[current],0);
		  }
		  currentFrame = &frame[current ];
          
          if ( flag == 2) break;
    }
    fps.stop();
    cap->release();
    
    cout << "Capture time elapsed: " << fps.elapsed() << endl;
    cout << "Capture FPS: " << fps.fps() << endl;
	}else{
		cerr << "Error opening capture device" << endl;
	}
}


void simpleTimer(){
    sleep(5);
    flag = 0;
    sleep(3);
    flag = 1;
    sleep(3);
    flag = 2;

}



void init_prog(int ac, char * av[] ){
try {

        po::options_description desc("Allowed options");
        desc.add_options()
            ("help", "produce help message")
            ("serial", po::value<string>(), "set serial device /dev/tty...")
            ("video", po::value<int>(), "set video device <N>")
            ("output", po::value<string>(), "set (output) image/video filename")
            ("window", "show image on a window")
            ("write", "save video to file")
            ("timer", "capture for x secs without serial device intervention")
            ("xres", po::value<int>(), "x resolution")
            ("yres", po::value<int>(), "y resolution")
            ("fps", po::value<int>(), "Output FPS")
            ("codec", po::value<int>(), "<\\d> int FOURCC codec number")
            ("pid", po::value<float>(), "<0.0-1.0> saving ratio ...")
        ;

        po::variables_map vm;
        po::store(po::parse_command_line(ac, av, desc), vm);
        po::notify(vm);

        if (vm.count("help")) {
            cout << desc << "\n";
            exit(0);
        }

        if (vm.count("timer")) {
		useDummyTTY=1;
        } else {
		useDummyTTY=0;
        }

        if (vm.count("window")) {
		showWindow=1;
        } else {
		showWindow=0;
        }
        if (vm.count("codec")) {
            cout << "codec set to "
                 << vm["codec"].as<int>() << ".\n";
			outFormat=vm["codec"].as<int>();
        } else {
            cout << "codec was not set. Using default\n";
        }
        if (vm.count("pid")) {
            cout << "pid set to "
                 << vm["pid"].as<float>() << ".\n";
			pid=vm["pid"].as<float>();
        } else {
            cout << "pid was not set. Using default\n";
        }
        
        if (vm.count("fps")) {
            cout << "fps set to "
                 << vm["fps"].as<int>() << ".\n";
		capfps=vm["fps"].as<int>();
		delay = 1000 / capfps;
        } else {
            cout << "fps was not set. Using default\n";
        }

        if (vm.count("video")) {
            cout << "video set to "
                 << vm["video"].as<int>() << ".\n";
		videoport=vm["video"].as<int>();
        } else {
            cout << "video was not set. Using default\n";
        }
        
		if (vm.count("xres")) {
            cout << "xres set to "
                 << vm["xres"].as<int>() << ".\n";
		x=vm["xres"].as<int>();
        } else {
            cout << "xres was not set. Using default\n";
        }
        
        if (vm.count("yres")) {
            cout << "yres set to "
                 << vm["yres"].as<int>() << ".\n";
		y=vm["yres"].as<int>();
        } else {
            cout << "yres was not set. Using default\n";
        }
        
        if (vm.count("output")) {
            cout << "output set to "
                 << vm["output"].as<string>() << ".\n";
		imagefile=vm["output"].as<string>();
        } else {
            cout << "image was not set. Using default\n";
        }

        if (vm.count("serial")) {
            cout << "serial device set to "
                 << vm["device"].as<string>() << ".\n";
		port=vm["serial"].as<string>();
        } else {
            cout << "serial device was not set. Using default\n";
        }
        
        if (vm.count("write")) {
		saveVideo=1;
        } else {
		saveVideo=0;
        }
    }
    catch(exception& e) {
        cerr << "error: " << e.what() << "\n";
        exit(1);
    }
    catch(...) {
        cerr << "Exception of unknown type!\n";
    }
}

int main(int ac, char * av[] ){
    init_prog(ac, av);
    cout << "starting arduino thread" << endl;
	thread *runarduino;
	if (useDummyTTY==1){
		 runarduino= new thread(simpleTimer);
	}else{
		runarduino= new thread(arduino);
	}
	cout << "starting CaptureFrames thread" << endl;
	thread runCapFrame(CaptureFrames);	
	sleep(2);
	cout << "starting save video thread" << endl;
	thread runSaveVideo(SaveVideoThread);	
	ShowFrames();
	runarduino->join();
	runCapFrame.join();
	runSaveVideo.join();

	return(0);
}
