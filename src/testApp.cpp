#include "testApp.h"

//--------------------------------------------------------------
void testApp::setup(){
    // enable depth->video image calibration
	kinect.setRegistration(true);
	kinect.init();
	//kinect.init(true); // shows infrared instead of RGB video image
	//kinect.init(false, false); // disable video image (faster fps)
	kinect.open();		// opens first available kinect    
    angle = 0;
	kinect.setCameraTiltAngle(angle);
    //kinect.enableDepthNearValueWhite();


	width = 640;
	height = 480;
		
	colorImage.allocate(width, height);
	grayImage.allocate(width, height);
	grayThres.allocate(width, height);
    depthImage.allocate(width, height);
    depthThres.allocate(width, height);
    depthNearThreshold = 0;
    depthFarThreshold = 255;
	
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
    #else
    #endif
    
    // Blob tracking
    ofAddListener(blobTracker.blobAdded, this, &testApp::blobAdded);
    ofAddListener(blobTracker.blobMoved, this, &testApp::blobMoved);
    ofAddListener(blobTracker.blobDeleted, this, &testApp::blobDeleted);
    minBlobSize = 450;
    maxBlobSize = 100000;
    
    oscRecieverForServer.setup(CAMERA_PORT_FOR_SERVER);
    oscSenderForServer.setup(SERVER_HOST, SERVER_PORT_FOR_CAMERA);
    
    oscRecieverForTable.setup(CAMERA_PORT_FOR_TABLE);
    oscSenderForTable.setup(TABLE_HOST, TABLE_PORT_FOR_CAMERA);
    
    drawHandTrackingDistThres = true;
    
    isFullScreenMode = true;
    ofSetFullscreen(isFullScreenMode);
    gui.setup();
    gui.setAutoSave(true);
    gui.config->gridSize.x = 335;
    gui.addTitle("suzuri-camera");
    gui.addToggle("toggle fullscreen", isFullScreenMode);
    gui.addContent("grayImage", grayImage);
    gui.addTitle("ARTK+ Setting");
    gui.addContent("grayThres", grayThres);
    gui.addSlider("gray threshold", artkThreshold, 0, 255);
    //kinect setting
    gui.addPage("Kinect Setting");
    gui.addContent("depth map", depthImage);
    gui.addContent("depthThres", depthThres);
    gui.addSlider("near depth threshold", depthNearThreshold, 400, 500);
    gui.addSlider("far depth threshold", depthFarThreshold, 500, 1000);
    //Blob Segging
    gui.addPage("Blob Setting");
    gui.addSlider("min blob size", minBlobSize, 10, 1000);
    gui.addSlider("max blob size", maxBlobSize, 10, 100000);
    gui.addSlider("hand tracking dist thres", handTrackingDistThres, 0, 500);
    gui.addToggle("show the thres", drawHandTrackingDistThres);
    //table setting
    gui.addPage("Table Setting");
    gui.addTitle("Table Position");
    gui.addToggle("show guide", showTableCalibrationGuide);
    gui.addSlider("table width", tableWidth, 0, 1);
    gui.addSlider("table height", tableHeight, 0, 1);
    gui.loadFromXML();
    gui.show();
    
    ofSetFullscreen(isFullScreenMode);
    ofEnableSmoothing();
    ofBackground(0, 0, 0);
}

//--------------------------------------------------------------
void testApp::update(){
    ofSetFullscreen(isFullScreenMode);
    kinect.update();
    if(kinect.isFrameNew()){
        colorImage.setFromPixels(kinect.getPixels(), width ,height);
        grayImage = colorImage;
        grayThres = grayImage;
        grayThres.threshold(artkThreshold);
        
        artk.setThreshold(artkThreshold);
        artk.update(grayImage.getPixels());
        std::vector<int> newMarkers;
        std::vector<int> lostMarkers;
        for(std::set<int>::iterator it = trackedMarkers.begin(); it != trackedMarkers.end(); it++){
            if(artk.getMarkerIndex(*it) == -1){
                lostMarkers.push_back(*it);
                trackedMarkers.erase(it);
            }
        }
        for(int i=0; i<artk.getNumDetectedMarkers(); i++){
            if(trackedMarkers.find(artk.getMarkerID(i)) == trackedMarkers.end()){
                newMarkers.push_back(artk.getMarkerID(i));
                trackedMarkers.insert(artk.getMarkerID(i));
            }
        }
        for(std::vector<int>::iterator it = newMarkers.begin(); it != newMarkers.end(); it++){
            ofxOscMessage sm;
            sm.setAddress("/camera/marker/tracked");
            sm.addIntArg(*it);
            oscSenderForServer.sendMessage(sm);
        }
        for(std::vector<int>::iterator it = lostMarkers.begin(); it != lostMarkers.end(); it++){
            ofxOscMessage sm;
            sm.setAddress("/camera/marker/lost");
            sm.addIntArg(*it);
            oscSenderForServer.sendMessage(sm);
        }
        
        depthImage.setFromPixels(kinect.getDepthPixels(), width ,height);
        ofPixels pix;
        pix.allocate(width, height, OF_PIXELS_MONO);
        float *ddpix = kinect.getDistancePixels();
        for(int i=0; i<width*height; i++){
            if(ddpix[i] > depthNearThreshold && ddpix[i] < depthFarThreshold){
                pix[i] = 255;
            }else{
                pix[i] = 0;
            }
        }
        depthThres.flagImageChanged();
        depthThres.setFromPixels(pix);
        blobTracker.update(depthThres, 80, minBlobSize, maxBlobSize);
    }
        
    while( oscRecieverForServer.hasWaitingMessages() )
    {
        ofxOscMessage m;
        oscRecieverForServer.getNextMessage( &m );
        
        dumpOSC(m);
        if(m.getAddress()=="/client/tap"){
            int markerID = m.getArgAsInt32(0);
            int uid = m.getArgAsInt32(1);
            std::set<int>::iterator it = trackedMarkers.find(markerID);
            if(it!=trackedMarkers.end()){ //if the marker found on the table...
                int markerIndex = artk.getMarkerIndex(markerID);
                ofPoint markerCenter = artk.getDetectedMarkerCenter(markerIndex);
                int trackedBlobsNum = blobTracker.size();
                bool handFound = false;
                cout << "finding the closest finger to " << markerID << "\n";
                for(int i=0; i<trackedBlobsNum && !handFound; i++){
                    cout << "blob#" << blobTracker.trackedBlobs[i].id << "\n";
                    if(blobTracker.trackedBlobs[i].gotFingers){ // if the blob is a hand
                        cout << "\t" << "has fingers" << "\n";
                        for(int j=0; j<blobTracker.trackedBlobs[i].nFingers; j++){  //find whose finger is pointing at the marker
                            cout << "\tfinger#" << j << "\n";
                            ofPoint fingerPos = blobTracker.trackedBlobs[i].fingers[j];
                            float fingerMarkerDistSquared = (fingerPos.x*width - markerCenter.x)*(fingerPos.x*width - markerCenter.x) + (fingerPos.y*height - markerCenter.y)*(fingerPos.y*height - markerCenter.y);
                            cout << "\t\t" << "fingerPos.x: " << fingerPos.x*width << "\n";
                            cout << "\t\t" << "fingerPos.y: " << fingerPos.y*height << "\n";
                            cout << "\t\t" << "markerCenter.y: " << markerCenter.x << "\n";
                            cout << "\t\t" << "markerCenter.y: " << markerCenter.y << "\n";
                            cout << "\t\t" << "finger-marker distance squared: " << fingerMarkerDistSquared << "\n";
                            cout << "\t\t" << "distance threshold squared: " << handTrackingDistThres * handTrackingDistThres << "\n";
                            if(fingerMarkerDistSquared < handTrackingDistThres * handTrackingDistThres){    //if the finger close enough to the marker...
                                cout << "\t\t...this is close enough\n";
                                if(trackedHands.find(blobTracker.trackedBlobs[i].id) == trackedHands.end()){
                                    cout << "\t\t" << "not tracked yet\n";
                                    trackedHands.insert(std::pair<int, int>( blobTracker.trackedBlobs[i].id, uid));
                                    if(users.find(uid) == users.end()){
                                        cout << "\t\t" << "user not found. adding...(uid:" << uid << ")\n";
                                        users.insert(uid);
                                        cout << "\t\t" << "user#" << uid << " added\n";
                                    }
                                    ofxOscMessage sm;
                                    sm.setAddress("/camera/hand/tracked");
                                    sm.addIntArg(uid);
                                    int trackedHandsNum = 0;
                                    for(std::map<int, int>::iterator it=trackedHands.begin(); it!=trackedHands.end(); it++){
                                        if(it->second == uid){
                                            trackedHandsNum++;
                                        }
                                    }
                                    sm.addIntArg(trackedHandsNum);
                                    oscSenderForServer.sendMessage(sm);
                                    handFound = true;
                                    break;
                                }
                            }
                        }
                    }
                }
            }
        }else if(m.getAddress()=="/client/img/res"){
            int markerID = m.getArgAsInt32(0);
            int uid = m.getArgAsInt32(1);
            string imgURL = m.getArgAsString(2);
            ofImage img;
            imageLoader.loadFromURL(&img, imgURL);
        }
    }
}

//--------------------------------------------------------------
void testApp::draw(){
    ofBackground(0, 0, 0);
	ofPushMatrix();
    ofEnableAlphaBlending();

    int fullWidth = ofGetHeight()*width/height;
    int fullHeight = ofGetHeight();
    
    tableOriginPosition.x = 0.5 - tableWidth/2;
    tableOriginPosition.y = 0.5 - tableHeight/2;
    
    ofSetHexColor(0xcccccc);
    colorImage.draw(0, 0, fullWidth, fullHeight);
    if(showTableCalibrationGuide){
        ofSetColor(255, 255, 255, 125);
        ofLine(0, 0, fullWidth, fullHeight);
        ofLine(fullWidth, 0, 0, fullHeight);
        ofLine(0, fullHeight/2, fullWidth, fullHeight/2);
        ofLine(fullWidth/2, 0, fullWidth/2, fullHeight);
        ofSetColor(255, 146, 3);
        ofNoFill();
        ofSetLineWidth(1);
        int r = 5;
        ofCircle(
                 tableOriginPosition.x * fullWidth,
                 tableOriginPosition.y * fullHeight,
                 r
                 );
        ofCircle(
                 tableOriginPosition.x * fullWidth,
                 tableOriginPosition.y * fullHeight,
                 r+4
                 );
        ofCircle(
                 tableOriginPosition.x * fullWidth,
                 (tableOriginPosition.y + tableHeight) * fullHeight,
                 r
                 );
        ofCircle(
                 (tableOriginPosition.x + tableWidth) * fullWidth,
                 tableOriginPosition.y * fullHeight,
                 r
                 );
        ofCircle(
                 (tableOriginPosition.x + tableWidth) * fullWidth,
                 (tableOriginPosition.y + tableHeight) * fullHeight,
                 r
                 );
        ofLine(
               (tableOriginPosition.x + tableWidth/2)* fullWidth,
               tableOriginPosition.y * fullHeight,
               (tableOriginPosition.x + tableWidth/2) * fullWidth,
               (tableOriginPosition.y + tableHeight) * fullHeight
               );
        ofLine(
               tableOriginPosition.x * fullWidth,
               (tableOriginPosition.y + tableHeight/2) * fullHeight,
               (tableOriginPosition.x + tableWidth) * fullWidth,
               (tableOriginPosition.y + tableHeight/2) * fullHeight
               );
    }
    ofSetColor(255, 146, 3);
    ofNoFill();
    ofSetLineWidth(2);
    ofRect(
           tableOriginPosition.x * fullWidth,
           tableOriginPosition.y * fullHeight,
           tableWidth * fullWidth,
           tableHeight * fullHeight
           );
    ofSetColor(255, 146, 3, 50);
    ofFill();
    ofRect(
           tableOriginPosition.x * fullWidth,
           tableOriginPosition.y * fullHeight,
           tableWidth * fullWidth,
           tableHeight * fullHeight
           );
    artk.draw(0, 0, fullWidth, fullHeight);
    glPushMatrix();
	glScalef((float)fullWidth/width, (float)fullHeight/height, 1);
    if(drawHandTrackingDistThres){
        ofSetColor(0, 255, 0);
        ofNoFill();
        for(int i=0; i<artk.getNumDetectedMarkers(); i++){
            ofCircle(artk.getDetectedMarkerCenter(i), handTrackingDistThres);
            ofDrawBitmapString(ofToString(artk.getMarkerID(i)), artk.getDetectedMarkerCenter(i).x, artk.getDetectedMarkerCenter(i).y+handTrackingDistThres+20);
        }
    }
    glPopMatrix();

    ofSetHexColor(0xffffff);
    blobTracker.draw(0, 0, fullWidth, fullHeight, trackedHands);
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
    /*ofDisableBlendMode();
    
    // draw some info regarding frame counts etc
	ofSetColor(0, 255, 0);
	//string msg = " MILLIS: " + ofToString(ofGetElapsedTimeMillis()) + " FPS: " + ofToString(ofGetFrameRate()) + " Device FPS: " + ofToString(openNIDevice.getFrameRate());
    
	//verdana.drawString(msg, 20, openNIDevice.getNumDevices() * 480 - 20);
    
	// ARTK draw
	// An easy was to see what is going on
	// Draws the marker location and id number
	artk.draw(0, height/2, width, height);
    artk.draw(width/2, 0, width/2, height/2);
     
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
    
    ofSetHexColor(0xffffff);
    std::vector<string> status;
    status.push_back(ofToString((int)trackedMarkers.size()) + " marker(s) found");
    status.push_back(ofToString((int)blobTracker.size()) + " blob(s) tracked");
    status.push_back(ofToString((int)users.size()) + " user(s) now on the system");
    status.push_back(ofToString((int)trackedHands.size()) + " hand(s) tracked");
    for(int i=0; i<status.size(); i++){
        ofDrawBitmapString(status[i], fullWidth + 10, 100 + i*40);
    }
    gui.draw();
}


//--------------------------------------------------------------
void testApp::blobAdded(ofxBlob &_blob){
    //ofLog(OF_LOG_NOTICE, "Blob ID " + ofToString(_blob.id) + " added" );
}

void testApp::blobMoved(ofxBlob &_blob){
    //ofLog(OF_LOG_NOTICE, "Blob ID " + ofToString(_blob.id) + " moved" );
}

void testApp::blobDeleted(ofxBlob &_blob){
    //ofLog(OF_LOG_NOTICE, "Blob ID " + ofToString(_blob.id) + " deleted" );
    std::map<int, int>::iterator it;
    it = trackedHands.find(_blob.id);
    if(it != trackedHands.end()){
        int uid = it->second;
        ofxOscMessage m;
        m.setAddress("/camera/hand/lost");
        m.addIntArg(uid);
        trackedHands.erase(it);
        int trackedHandsNum = 0;
        for(it=trackedHands.begin(); it!=trackedHands.end(); it++){
            if(it->second == uid){
                trackedHandsNum++;
            }
        }
        m.addIntArg(trackedHandsNum);
        oscSenderForServer.sendMessage(m);
        if(trackedHandsNum == 0){
            users.erase(users.find(uid));
        }
    }
}


//OSCメッセージをコンソールに出力する関数
void testApp::dumpOSC(ofxOscMessage m) {
    string msg_string;
    msg_string = m.getAddress();
    for (int i=0; i<m.getNumArgs(); i++ ) {
        msg_string += " ";
        if(m.getArgType(i) == OFXOSC_TYPE_INT32)
            msg_string += ofToString( m.getArgAsInt32(i));
        else if(m.getArgType(i) == OFXOSC_TYPE_FLOAT)
            msg_string += ofToString( m.getArgAsFloat(i));
        else if(m.getArgType(i) == OFXOSC_TYPE_STRING)
            msg_string += m.getArgAsString(i);
    }
    cout << msg_string << endl;
}
//--------------------------------------------------------------
void testApp::exit(){
    kinect.setCameraTiltAngle(0);
    kinect.close();
    gui.saveToXML();
}

//--------------------------------------------------------------
void testApp::keyPressed(int key){
    switch(key){
        case ' ':
            gui.toggleDraw();
            break;
        case '[':
            gui.prevPage();
            break;
        case ']':
            gui.nextPage();
            break;
        case 'p':
            #ifdef KINECT_CONNECTED
            #else
            #endif
        case 'r':
            #ifdef KINECT_CONNECTED
            #endif
            break;
        case 'f':
            #ifdef KINECT_CONNECTED
            #else
            #endif
            break;
        case OF_KEY_RIGHT:
            #ifdef KINECT_CONNECTED
            #else
            #endif
            break;
        case OF_KEY_LEFT:
            #ifdef KINECT_CONNECTED
            #else
            #endif
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

