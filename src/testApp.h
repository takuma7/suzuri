#ifndef _TEST_APP
#define _TEST_APP

#include "ofxOpenCv.h"
#include "ofxARToolkitPlus.h"
#include "ofxOpenNI.h"

#include "ofMain.h"

// Uncomment this to use a camera instead of a video file
#define KINECT_CONNECTED

class testApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();
        void exit();

		void keyPressed  (int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void windowResized(int w, int h);
	
		/* Size of the image */
		int width, height;
	
		/* Use either camera or a video file */
		//#ifdef KINECT_CONNECTED
		//ofVideoGrabber vidGrabber;
		//#else
		//ofVideoPlayer vidPlayer;
		//#endif

		/* ARToolKitPlus class */	
		ofxARToolkitPlus artk;	
		int artkThreshold;
	
		/* OpenCV images */
		ofxCvColorImage colorImage;
		ofxCvGrayscaleImage grayImage;
		ofxCvGrayscaleImage	grayThres;
        ofxCvGrayscaleImage depthThres;
        int depthNearThreshold;
        int depthFarThreshold;
    
        ofxCvContourFinder 	contourFinder;
	
		/* Image to distort on to the marker */
		ofImage displayImage;
		/* The four corners of the image */
		vector<ofPoint> displayImageCorners;
    
        //void userEvent(ofxOpenNIUserEvent & event);
        ofxOpenNI openNIDevice;
        ofTrueTypeFont verdana;
};

#endif
