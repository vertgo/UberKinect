#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxUI.h"
#include "ofxTweenzor.h"

// uncomment this to read from two kinects simultaneously
//#define USE_TWO_KINECTS
struct vizVars{
    float step;
    float pointSize;
    float colorWeight;
    
};

class testApp : public ofBaseApp {
public:
	
	void setup();
	void update();
	void draw();
	void exit();
	
	void drawPointCloud();
	
	void keyPressed(int key);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void windowResized(int w, int h);
    
	
	ofxKinect kinect;
	
#ifdef USE_TWO_KINECTS
	ofxKinect kinect2;
#endif

	ofxCvColorImage colorImg;
	
	ofxCvGrayscaleImage grayImage; // grayscale depth image
	ofxCvGrayscaleImage grayThreshNear; // the near thresholded image
	ofxCvGrayscaleImage grayThreshFar; // the far thresholded image
	
	ofxCvContourFinder contourFinder;
	
	bool bThreshWithOpenCV;
	bool bDrawPointCloud;
	
	int nearThreshold;
	int farThreshold;
	
	int angle;
    bool tweening;
	
	// used for viewing the point cloud
	ofEasyCam easyCam;
    //ofCamera easyCam;
        
    //gui stuff
    ofxUICanvas *gui;   
    bool drawFill; 
    float step;
    float pointSize;
    
    bool useEasyCam;
    
    vector<vizVars> vizVarSequence;
    
    float colorWeight;
    ofFloatColor getWeightedColor( float inWeight, ofFloatColor inConstant, int inX, int inY );
    
    int curVizVarIndex;
    void gotoNextViz();
    void onVizTweenComplete( float * arg);
    
    float pixelColorR;
    float pixelColorG;
    float pixelColorB;
    
    float curPitch;
    
    float bgColorR;
    float bgColorG;
    float bgColorB;
    
    bool camMoved;
    
    float xPosSinIndex;
    ofImage bgImage;
    
    
};
