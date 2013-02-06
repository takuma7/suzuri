#ifndef _TEST_APP
#define _TEST_APP

#include "ofxOpenCv.h"
#include "ofxARToolkitPlus.h"
#include "ofxKinect.h"
#include "ofxBlobTracker.h"
#include "ofxJSONElement.h"
#include "ofxOsc.h"
#include "ofxSimpleGuiToo.h"
#include "ofxThreadedImageLoader.h"

#define CAMERA_HOST "localhost"
#define SERVER_HOST "localhost"
#define TABLE_HOST "localhost"

#define SERVER_PORT_FOR_CAMERA 5555
#define SERVER_PORT_FOR_TABLE 5556
#define CAMERA_PORT_FOR_SERVER 5557
#define CAMERA_PORT_FOR_TABLE 5558
#define TABLE_PORT_FOR_SERVER 5559
#define TABLE_PORT_FOR_CAMERA 5560

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
    void dragEvent(ofDragInfo dragInfo);
    void gotMessage(ofMessage msg);

    void blobAdded(ofxBlob &_blob);
    void blobMoved(ofxBlob &_blob);
    void blobDeleted(ofxBlob &_blob);

    void dumpOSC(ofxOscMessage m);

    /* gui */
    ofxSimpleGuiToo gui;
    bool isFullScreenMode;

    /* Size of the image */
    int width, height;

    /* ARToolKitPlus class */	
    ofxARToolkitPlus artk;	
    int artkThreshold;

    /* OpenCV images */
    ofxCvColorImage colorImage;
    ofxCvGrayscaleImage grayImage;
    ofxCvGrayscaleImage	grayThres;
    ofxCvGrayscaleImage depthImage;
    ofxCvGrayscaleImage depthThres;
    ofImage depthThresTransparent;
    int depthNearThreshold;
    int depthFarThreshold;

    ofxBlobTracker blobTracker;
    int minBlobSize;
    int maxBlobSize;
    int handTrackingDistThres;
    bool drawHandTrackingDistThres;

    /* Image to distort on to the marker */
    ofImage displayImage;
    /* The four corners of the image */
    vector<ofPoint> displayImageCorners;

    ofxKinect kinect;
    int angle;
    ofTrueTypeFont verdana;

    /* table */
    ofPoint tableOriginPosition;
    float tableWidth;
    float tableHeight;
    bool showTableCalibrationGuide;

    /* osc */
    ofxOscReceiver oscRecieverForServer;
    ofxOscSender oscSenderForServer;

    ofxOscReceiver oscRecieverForTable;
    ofxOscSender oscSenderForTable;

    /* image loader */
    ofxThreadedImageLoader imageLoader;

    std::map<int, int> trackedHands;    //map<blobID, uid>
    std::set<int> trackedMarkers;       //set<markerID>
    std::set<int> users;                //users<uid>
};

#endif
