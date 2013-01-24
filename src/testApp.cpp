#include "testApp.h"

//--------------------------------------------------------------
void testApp::setup(){
	width = 640;
	height = 480;
		
	colorImage.allocate(width, height);
	grayImage.allocate(width, height);
	grayThres.allocate(width, height);
    depthThres.allocate(width, height);
    depthNearThreshold = 0;
    depthFarThreshold = 680;
	
	// Load the image we are going to distort
	displayImage.loadImage("of.jpg");
	// Load the corners of the image into the vector
	int displayImageHalfWidth = displayImage.width / 2;
	int displayImageHalfHeight = displayImage.height / 2;	
	displayImageCorners.push_back(ofPoint(0, 0));
	displayImageCorners.push_back(ofPoint(displayImage.width, 0));
	displayImageCorners.push_back(ofPoint(displayImage.width, displayImage.height));
	displayImageCorners.push_back(ofPoint(0, displayImage.height));
	
	// This uses the default camera calibration and marker file
	artk.setup(width, height);

	// The camera calibration file can be created using GML:
	// http://graphics.cs.msu.ru/en/science/research/calibration/cpp
	// and these instructions:
	// http://studierstube.icg.tu-graz.ac.at/doc/pdf/Stb_CamCal.pdf
	// This only needs to be done once and will aid with detection
	// for the specific camera you are using
	// Put that file in the data folder and then call setup like so:
	// artk.setup(width, height, "myCamParamFile.cal", "markerboard_480-499.cfg");
	
	// Set the threshold
	// ARTK+ does the thresholding for us
	// We also do it in OpenCV so we can see what it looks like for debugging
	artkThreshold = 240;
	artk.setThreshold(artkThreshold);
    
    #ifdef KINECT_CONNECTED
    openNIDevice.setup();
    #else
    openNIDevice.setupFromONI("test.oni");
    #endif
    openNIDevice.addDepthGenerator();
    openNIDevice.addImageGenerator();
    openNIDevice.setRegister(true);
    openNIDevice.setMirror(false);
    ofxOpenNIDepthThreshold openNIDepthThreshold = ofxOpenNIDepthThreshold(depthNearThreshold, depthFarThreshold);
    openNIDepthThreshold.setMaskPixelFormat(OF_PIXELS_MONO);
    openNIDevice.addDepthThreshold(openNIDepthThreshold);
    openNIDevice.start();
    
    // Blob tracking
    ofAddListener(blobTracker.blobAdded, this, &testApp::blobAdded);
    ofAddListener(blobTracker.blobMoved, this, &testApp::blobMoved);
    ofAddListener(blobTracker.blobDeleted, this, &testApp::blobDeleted);
    minArea = 450;
}

//--------------------------------------------------------------
void testApp::update(){
    openNIDevice.update();
    if(openNIDevice.isNewFrame()){
        ofBackground(0, 0, 0);
        colorImage.setFromPixels(openNIDevice.getImagePixels());
        grayImage = colorImage;
        grayThres = grayImage;
        grayThres.threshold(artkThreshold);
        artk.update(grayImage.getPixels());
        
        depthThres.setFromPixels(openNIDevice.getDepthThreshold(0).getMaskPixels());
        blobTracker.update(depthThres, 80, minArea);
    }
}

//--------------------------------------------------------------
void testApp::draw(){
	ofPushMatrix();
    if(openNIDevice.isRecording()){
        ofSetHexColor(0xff0000);
        ofDrawBitmapString("Rec", 650, 20);
    }else if(openNIDevice.isPlaying()){
        ofSetHexColor(0x00ff00);
        ofDrawBitmapString("Playing", 650, 20);
    }
	// Main image
	ofSetHexColor(0xffffff);
	grayImage.draw(0, 0, width/2, height/2);
	ofSetHexColor(0x666666);
	ofDrawBitmapString(ofToString(artk.getNumDetectedMarkers()) + " marker(s) found", 650, 60);

	// Threshold image
	ofSetHexColor(0xffffff);
	grayThres.draw(width/2, 0, width/2, height/2);
	ofSetHexColor(0x666666);
	ofDrawBitmapString("artkThreshold: " + ofToString(artkThreshold), 650, 80);
	ofDrawBitmapString("Use the Up/Down keys to adjust the gray scale threshold", 650, 100);

    // Depth threshold image
    ofSetHexColor(0x333333);
    depthThres.draw(0, height/2, width, height);
    ofSetHexColor(0x666666);
    ofDrawBitmapString("Near Threashold: " + ofToString(depthNearThreshold), 650, 140);
    ofDrawBitmapString("Far Threashold:  " + ofToString(depthFarThreshold), 650, 160);
    ofDrawBitmapString("Use the Left/Rgith keys to adjust the depth threshold", 650, 180);
    
    // Blob tracking image
    ofSetHexColor(0xffffff);
    blobTracker.draw(0, height/2, width, height);
    ofSetHexColor(0x666666);
    ofDrawBitmapString(ofToString((int)blobTracker.size()) + " blob(s) tracked", 650, 220);
    ofDrawBitmapString("minArea: " + ofToString(minArea), 650, 240);
    ofDrawBitmapString("User the j/k keys to adjust minArea size", 650, 260);
    /*ofPushMatrix();
    for (int i = 0; i < blobTracker.size(); i++){
        ofFill();
        ofSetColor(255,0,0);
        ofCircle(blobTracker[i].centroid.x * width,
                 height/2 + blobTracker[i].centroid.y * height,
                 10);
        ofSetColor(0);
        ofDrawBitmapString(ofToString( blobTracker[i].id ),
                           blobTracker[i].centroid.x * width,
                           height/2 + blobTracker[i].centroid.y * height);
    }
    ofPopMatrix();*/
    ofDisableBlendMode();
    
    // draw some info regarding frame counts etc
	ofSetColor(0, 255, 0);
	//string msg = " MILLIS: " + ofToString(ofGetElapsedTimeMillis()) + " FPS: " + ofToString(ofGetFrameRate()) + " Device FPS: " + ofToString(openNIDevice.getFrameRate());
    
	//verdana.drawString(msg, 20, openNIDevice.getNumDevices() * 480 - 20);
    
	// ARTK draw
	// An easy was to see what is going on
	// Draws the marker location and id number
	artk.draw(0, height/2, width, height);
    artk.draw(width/2, 0, width/2, height/2);
	/*
	// ARTK 2D stuff
	// See if marker ID '0' was detected
	// and draw blue corners on that marker only
	int myIndex = artk.getMarkerIndex(0);
	if(myIndex >= 0) {
		// Get the corners
		vector<ofPoint> corners;
		artk.getDetectedMarkerBorderCorners(myIndex, corners);
		// Can also get the center like this:
		// ofPoint center = artk.getDetectedMarkerCenter(myIndex);
		ofSetHexColor(0x0000ff);
		for(int i=0;i<corners.size();i++) {
			ofCircle(corners[i].x, corners[i].y, 10);
		}
	}
	
	// Homography
	// Here we feed in the corners of an image and get back a homography matrix
	ofMatrix4x4 homo = artk.getHomography(myIndex, displayImageCorners);
	// We apply the matrix and then can draw the image distorted on to the marker
	ofPushMatrix();
	glMultMatrixf(homo.getPtr());
	ofSetHexColor(0xffffff);
	displayImage.draw(0, 0);
	ofPopMatrix();
	
	
	// ARTK 3D stuff
	// This is another way of drawing objects aligned with the marker
	// First apply the projection matrix once
	artk.applyProjectionMatrix();
	// Find out how many markers have been detected
	int numDetected = artk.getNumDetectedMarkers();
	ofEnableAlphaBlending();
	// Draw for each marker discovered
	for(int i=0; i<numDetected; i++) {
		
		// Set the matrix to the perspective of this marker
		// The origin is in the middle of the marker	
		artk.applyModelMatrix(i);		
		
		// Draw a line from the center out
		ofNoFill();
		ofSetLineWidth(5);
		ofSetHexColor(0xffffff);
//		glBegin(GL_LINES);
//		glVertex3f(0, 0, 0); 
//		glVertex3f(0, 0, 50);
//		glEnd();
		
		// Draw a stack of rectangles by offseting on the z axis
		ofNoFill();
		ofEnableSmoothing();
		ofSetColor(255, 255, 0, 50);	
		for(int i=0; i<10; i++) {		
			ofRect(-25, -25, 50, 50);
			ofTranslate(0, 0, i*1);
		}
	}*/
	ofPopMatrix();
}


//--------------------------------------------------------------
void testApp::blobAdded(ofxBlob &_blob){
    ofLog(OF_LOG_NOTICE, "Blob ID " + ofToString(_blob.id) + " added" );
}

void testApp::blobMoved(ofxBlob &_blob){
    ofLog(OF_LOG_NOTICE, "Blob ID " + ofToString(_blob.id) + " moved" );
}

void testApp::blobDeleted(ofxBlob &_blob){
    ofLog(OF_LOG_NOTICE, "Blob ID " + ofToString(_blob.id) + " deleted" );
}

//--------------------------------------------------------------
void testApp::exit(){
    openNIDevice.stop();
}

//--------------------------------------------------------------
void testApp::keyPressed(int key){
    switch(key){
        case ' ':
            #ifdef KINECT_CONNECTED
            #else
            openNIDevice.setPaused(!openNIDevice.isPaused());
            #endif
        case 'r':
            #ifdef KINECT_CONNECTED
            if(!openNIDevice.isRecording()){
                openNIDevice.startRecording(ofToDataPath("test.oni"));
            }else{
                openNIDevice.stopRecording();
            }
            #endif
            break;
        case 'p':
            #ifdef KINECT_CONNECTED
            if(!openNIDevice.isRecording() && !openNIDevice.isPlaying()){
                openNIDevice.startPlayer("test.oni");
            }else if(openNIDevice.isPlaying()){
                openNIDevice.firstFrame();
            }
            #else
            openNIDevice.firstFrame();
            #endif
            break;
        case OF_KEY_UP:
            artk.setThreshold(++artkThreshold);
            break;
        case OF_KEY_DOWN:
            artk.setThreshold(--artkThreshold);
            break;
        case OF_KEY_RIGHT:
            depthFarThreshold+=5;
            openNIDevice.getDepthThreshold(0).setFarThreshold(depthFarThreshold);
            break;
        case OF_KEY_LEFT:
            depthFarThreshold-=5;
            openNIDevice.getDepthThreshold(0).setFarThreshold(depthFarThreshold);
            break;
        case 'j':
            minArea-=10;
            break;
        case 'k':
            minArea+=10;
            break;
    }
}

//--------------------------------------------------------------
void testApp::keyReleased(int key){

}

//--------------------------------------------------------------
void testApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void testApp::gotMessage(ofMessage msg){
    
}

//--------------------------------------------------------------
void testApp::dragEvent(ofDragInfo dragInfo){
    
}

