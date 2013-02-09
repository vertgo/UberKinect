#include "testApp.h"

#define TWEEN_TIME 10.f
//--------------------------------------------------------------
void testApp::setup() {
	ofSetLogLevel(OF_LOG_VERBOSE);
    Tweenzor::init();
	
	// enable depth->video image calibration
	kinect.setRegistration(true);
    
	kinect.init();
	//kinect.init(true); // shows infrared instead of RGB video image
	//kinect.init(false, false); // disable video image (faster fps)
	
	kinect.open();		// opens first available kinect
	//kinect.open(1);	// open a kinect by id, starting with 0 (sorted by serial # lexicographically))
	//kinect.open("A00362A08602047A");	// open a kinect using it's unique serial #
	
    curVizVarIndex = 0;
#ifdef USE_TWO_KINECTS
	kinect2.init();
	kinect2.open();
#endif
	
	colorImg.allocate(kinect.width, kinect.height);
	grayImage.allocate(kinect.width, kinect.height);
	grayThreshNear.allocate(kinect.width, kinect.height);
	grayThreshFar.allocate(kinect.width, kinect.height);
	
	nearThreshold = 230;
	farThreshold = 70;
	bThreshWithOpenCV = true;
	
	ofSetFrameRate(60);
	
	// zero the tilt on startup
	angle = 0;
	kinect.setCameraTiltAngle(angle);
	
	// start from the front
	bDrawPointCloud = false;
    
    step = 2;
    pointSize = 2;
    
    //ui stuff
    useEasyCam = true; //have to prevent easycam to be able to widget
    float dim = 16; 
    float xInit = OFX_UI_GLOBAL_WIDGET_SPACING; 
    float length = 320-xInit; 
    colorWeight = 1;
    
    bgColorR = bgColorG = bgColorB = 0.f;
    pixelColorR = .45f;
    pixelColorG = .88f; 
    pixelColorB = .925f;
    
    gui = new ofxUICanvas(0,0,length+xInit*2.0,ofGetHeight());
    drawFill = true;     
    gui->addWidgetDown(new ofxUILabel("BGS FEBRUARY BETA", OFX_UI_FONT_LARGE)); 
    gui->addSlider("STEP", 1.f, 20.f, &step, length,dim);
    gui->addSlider("POINT SIZE", .1f, 20.f, &pointSize, length,dim);
    gui->addSlider("COLOR WEIGHT", 0.f, 1.f, &colorWeight, length,dim);
    
    gui->addSlider("PixelR", 0.f, 1.f, &pixelColorR, length,dim);
    gui->addSlider("PixelG", 0.f, 1.f, &pixelColorG, length,dim);
    gui->addSlider("PixelB", 0.f, 1.f, &pixelColorB, length,dim);
    
    
    gui->addSlider("BG R", 0.f, 255.f, &bgColorR, length,dim);
    gui->addSlider("BG G", 0.f, 255.f, &bgColorG, length,dim);
    gui->addSlider("BG B", 0.f, 255.f, &bgColorB, length,dim);
    
    
    
    
    
    vizVarSequence.push_back(vizVars()) ;
    vizVarSequence.push_back(vizVars()) ;
    vizVarSequence.push_back(vizVars()) ;
    vizVarSequence.push_back(vizVars()) ;    
    vizVarSequence.push_back(vizVars()) ;

    
    vizVarSequence[0].step = 8.88f;
    vizVarSequence[0].pointSize = 9.36f;
    vizVarSequence[0].colorWeight = 0.44f;
    
    vizVarSequence[1].step = 1.78f;
    vizVarSequence[1].pointSize = 2.37f;
    vizVarSequence[1].colorWeight = 0.49f;
    
    vizVarSequence[2].step = 8.88f;
    vizVarSequence[2].pointSize = 1.f;
    vizVarSequence[2].colorWeight = 0.9f;
    
    vizVarSequence[3].step = 8.88f;
    vizVarSequence[3].pointSize = 20.f;
    vizVarSequence[3].colorWeight = 0.14f;
    
    
    vizVarSequence[4].step = 2.f;
    vizVarSequence[4].pointSize = 5.f;
    vizVarSequence[4].colorWeight = 0.f;
    
    
    tweening = true;
    
    gotoNextViz();
    
    
}

//--------------------------------------------------------------
void testApp::update() {
    
	
    if ( tweening )
        Tweenzor::update(ofGetElapsedTimeMillis());
    cout << "testApp::update::point:" << pointSize << ", step:" << step << ", colorWeight:" << colorWeight <<endl;
	kinect.update();
	
	// there is a new frame and we are connected
	if(kinect.isFrameNew()) {
		
		// load grayscale depth image from the kinect source
		grayImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);
		
		// we do two thresholds - one for the far plane and one for the near plane
		// we then do a cvAnd to get the pixels which are a union of the two thresholds
		if(bThreshWithOpenCV) {
			grayThreshNear = grayImage;
			grayThreshFar = grayImage;
			grayThreshNear.threshold(nearThreshold, true);
			grayThreshFar.threshold(farThreshold);
			cvAnd(grayThreshNear.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);
		} else {
			
			// or we do it ourselves - show people how they can work with the pixels
			unsigned char * pix = grayImage.getPixels();
			
			int numPixels = grayImage.getWidth() * grayImage.getHeight();
			for(int i = 0; i < numPixels; i++) {
				if(pix[i] < nearThreshold && pix[i] > farThreshold) {
					pix[i] = 255;
				} else {
					pix[i] = 0;
				}
			}
		}
		
		// update the cv images
		grayImage.flagImageChanged();
		
		// find contours which are between the size of 20 pixels and 1/3 the w*h pixels.
		// also, find holes is set to true so we will get interior contours as well....
		contourFinder.findContours(grayImage, 10, (kinect.width*kinect.height)/2, 20, false);
	}
	
#ifdef USE_TWO_KINECTS
	kinect2.update();
#endif
}

//--------------------------------------------------------------
void testApp::draw() {
	
	ofSetColor(255, 255, 255);
	
	if(bDrawPointCloud) {
		if ( useEasyCam )
            easyCam.begin();
		drawPointCloud();
		if ( useEasyCam )
            easyCam.end();
	} else {
		// draw from the live kinect
        /*
		kinect.drawDepth(10, 10, 400, 300);
		
		
		grayImage.draw(10, 320, 400, 300);
		contourFinder.draw(10, 320, 400, 300);
        */
        //kinect.draw(420, 10, 400, 300);
        kinect.draw(0, 0, ofGetWidth(), ofGetHeight());
		
#ifdef USE_TWO_KINECTS
		kinect2.draw(420, 320, 400, 300);
#endif
	}
	
	// draw instructions
	ofSetColor(255, 255, 255);
	stringstream reportStream;
	reportStream << "accel is: " << ofToString(kinect.getMksAccel().x, 2) << " / "
	<< ofToString(kinect.getMksAccel().y, 2) << " / "
	<< ofToString(kinect.getMksAccel().z, 2) << endl
	<< "press p to switch between images and point cloud, rotate the point cloud with the mouse" << endl
	<< "using opencv threshold = " << bThreshWithOpenCV <<" (press spacebar)" << endl
	<< "set near threshold " << nearThreshold << " (press: + -)" << endl
	<< "set far threshold " << farThreshold << " (press: < >) num blobs found " << contourFinder.nBlobs
	<< ", fps: " << ofGetFrameRate() << endl
	<< "press c to close the connection and o to open it again, connection is: " << kinect.isConnected() << endl
	<< "press UP and DOWN to change the tilt angle: " << angle << " degrees" << endl
	<< "press 1-5 & 0 to change the led mode (mac/linux only)" << endl;
	ofDrawBitmapString(reportStream.str(),20,652);
    
    
    if(drawFill)
    {
        ofFill(); 
    }
    else
    {
        ofNoFill(); 
    }
}

void testApp::drawPointCloud() {
    ofBackground(bgColorR, bgColorG, bgColorB);
    
	int w = 640;
	int h = 480;
	ofMesh mesh;
	mesh.setMode(OF_PRIMITIVE_POINTS);
	
    ofFloatColor blue( pixelColorR, pixelColorG, pixelColorB);
	for( float y = 0; y < h; y += step) {
		for( float x = 0; x < w; x += step) {
			if(kinect.getDistanceAt(x, y) > 0) {
                
				//mesh.addColor( blue );//kinect.getColorAt(x,y));
                //mesh.addColor(kinect.getColorAt(x,y));
                mesh.addColor( getWeightedColor(colorWeight, blue, x, y));
				mesh.addVertex(kinect.getWorldCoordinateAt(x, y));
			}
		}
	}
	glPointSize( pointSize);
	ofPushMatrix();
	// the projected points are 'upside down' and 'backwards' 
	ofScale(1, -1, -1);
	ofTranslate(0, 0, -1000); // center the points a bit
	glEnable(GL_DEPTH_TEST);
	mesh.drawVertices();
	glDisable(GL_DEPTH_TEST);
	ofPopMatrix();
}
//--------------------------------------------------------------
ofFloatColor testApp::getWeightedColor(float inWeight, ofFloatColor inConstant, int inX, int inY){
    ofFloatColor returnColor = inConstant;
    ofFloatColor pointColor = kinect.getColorAt(inX, inY);
    float notWeight = 1-inWeight;
    
    returnColor.r = returnColor.r * inWeight + pointColor.r * notWeight;
    returnColor.g = returnColor.g * inWeight + pointColor.g * notWeight;
    returnColor.b = returnColor.b * inWeight + pointColor.b * notWeight;
    
    return returnColor;
    
}


//--------------------------------------------------------------
void testApp::exit() {
	//kinect.setCameraTiltAngle(0); // zero the tilt on exit
	kinect.close();
	
#ifdef USE_TWO_KINECTS
	kinect2.close();
#endif
}

//--------------------------------------------------------------
void testApp::keyPressed (int key) {
	switch (key) {
		case ' ':
			bThreshWithOpenCV = !bThreshWithOpenCV;
			break;
			
		case'p':
			bDrawPointCloud = !bDrawPointCloud;
			break;
			
		case '>':
		case '.':
			farThreshold ++;
			if (farThreshold > 255) farThreshold = 255;
			break;
			
		case '<':
		case ',':
			farThreshold --;
			if (farThreshold < 0) farThreshold = 0;
			break;
			
		case '+':
		case '=':
			nearThreshold ++;
			if (nearThreshold > 255) nearThreshold = 255;
			break;
			
		case '-':
			nearThreshold --;
			if (nearThreshold < 0) nearThreshold = 0;
			break;
			
		case 'w':
			kinect.enableDepthNearValueWhite(!kinect.isDepthNearValueWhite());
			break;
			
		case 'o':
			kinect.setCameraTiltAngle(angle); // go back to prev tilt
			kinect.open();
			break;
			
		case 'c':
			kinect.setCameraTiltAngle(0); // zero the tilt
			kinect.close();
			break;
			
		case '1':
			kinect.setLed(ofxKinect::LED_GREEN);
			break;
			
		case '2':
			kinect.setLed(ofxKinect::LED_YELLOW);
			break;
			
		case '3':
			kinect.setLed(ofxKinect::LED_RED);
			break;
			
		case '4':
			kinect.setLed(ofxKinect::LED_BLINK_GREEN);
			break;
			
		case '5':
			kinect.setLed(ofxKinect::LED_BLINK_YELLOW_RED);
			break;
			
		case '0':
			kinect.setLed(ofxKinect::LED_OFF);
			break;
        case 'e':
			useEasyCam = !useEasyCam;
			break;
        case 't':
            tweening = !tweening;
            break;
			
		case OF_KEY_UP:
			angle++;
			if(angle>30) angle=30;
			kinect.setCameraTiltAngle(angle);
			break;
			
		case OF_KEY_DOWN:
			angle--;
			if(angle<-30) angle=-30;
			kinect.setCameraTiltAngle(angle);
			break;
	}
}
//--------------------------------------------------------------
void testApp::gotoNextViz(){

    Tweenzor::add( &step, step, vizVarSequence[ curVizVarIndex].step , 0.f, (float)TWEEN_TIME );
    Tweenzor::add( &colorWeight, colorWeight, vizVarSequence[ curVizVarIndex].colorWeight , 0.f, (float)TWEEN_TIME );
    Tweenzor::add( &pointSize, pointSize, vizVarSequence[ curVizVarIndex].pointSize , 0.f, (float)TWEEN_TIME );
    Tweenzor::addCompleteListener(Tweenzor::getTween(&step), this, &testApp::onVizTweenComplete );

}

//--------------------------------------------------------------
void testApp::onVizTweenComplete(float *arg){
    curVizVarIndex++;
    if (curVizVarIndex >= vizVarSequence.size() ){
        curVizVarIndex = 0;
    }
    gotoNextViz();
    
}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h)
{}
