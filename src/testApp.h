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
#define TABLE_HOST "10.0.1.43"

#define SERVER_PORT_FOR_CAMERA 5555
#define CAMERA_PORT_FOR_SERVER 5557
#define CAMERA_PORT_FOR_TABLE 5558
#define TABLE_PORT_FOR_CAMERA 5560

#include "ofMain.h"

// Uncomment this to use a camera instead of a video file
#define KINECT_CONNECTED

class TouchPoint {
public:
    TouchPoint(){}
    TouchPoint(ofVec2f initialPt, ofColor color) : mColor(color), mTimeOfDeath(-1.0){
        mLine.push_back(initialPt);
    }
    
    void addPoint(ofVec2f pt){
        mLine.push_back(pt);
    }
    
    void draw(int _x, int _y, int _w, int _h){
        int _t = 20;
        if(mLine.size()<=_t) return;
        ofPushMatrix();
        ofTranslate(_x, _y);
        ofPushStyle();
        ofSetColor(mColor);
        ofNoFill();
        for(int i=_t; i<mLine.size()-_t; i+=_t){
            ofLine(mLine[i-_t].x * _w, mLine[i-_t].y * _h, mLine[i].x * _w, mLine[i].y * _h);
        }
        ofPopStyle();
        ofPopMatrix();
    }
    
    std::vector<ofVec2f> mLine;
    ofColor mColor;
    float mTimeOfDeath;
    int uid;
    int brushSize;
    
    void setColor(string hexColorCode){
        int _hex;
        std::stringstream ss;
        ss << std::hex << hexColorCode.substr(1, 6);
        ss >> _hex;
        mColor.setHex(_hex);
    }
    
    void setColor(ofColor color){
        mColor = color;
    }
    
    void setBrushSize(int _brushSize){
        brushSize = _brushSize;
    }
};

class User{
public:
    User(){}
    User(int _uid){
        uid = _uid;
    }
    int uid;
    string username;
    string first_name;
    string last_name;
    ofImage img;
    string img_url;
    string url;
    
    int size;
    ofColor color;
    
    void setup(string _username, string _first_name, string _last_name, string _img_url, ofxThreadedImageLoader &imageLoader, string _url, int _size, string _color){
        username = _username;
        first_name = _first_name;
        last_name = _last_name;
        img_url = _img_url;
        url = _url;
        size = _size;
        int _hex;
        std::stringstream ss;
        ss << std::hex << _color.substr(1, 6);
        ss >> _hex;
        color = ofColor::fromHex(_hex);
        
        //load image
        imageLoader.loadFromURL(&img, img_url);
    }
    
    void setBrushSize(int _size){
        size = _size;
    }
    
    void setColor(string hexColorCode){
        int _hex;
        std::stringstream ss;
        ss << std::hex << hexColorCode.substr(1, 6);
        ss >> _hex;
        color.setHex(_hex);
    }
    
    void setColor(ofColor _color){
        color = _color;
    }
};

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
    
    /* touch */
    bool showTouchPoints;
    float touchDistThres;

    /* osc */
    ofxOscReceiver oscRecieverForServer;
    ofxOscSender oscSenderForServer;

    ofxOscReceiver oscRecieverForTable;
    ofxOscSender oscSenderForTable;

    /* image loader */
    ofxThreadedImageLoader imageLoader;

    std::map<int, int> trackedHands;    //map<blobID, uid>
    std::set<int> trackedMarkers;       //set<markerID>
    std::map<int, User> users;
    std::map<int, TouchPoint> touchPoints;
};

#endif
