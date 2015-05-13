#pragma once

#include "ofMain.h"
#include "ofxGui.h"
#include "ofxFlowTools.h"
#include "ofxKinect.h"
#include "ofxOpenCv.h"




#define USE_PROGRAMMABLE_GL					

using namespace flowTools;

class flowToolsApp : public ofBaseApp{
public:
    void setup();
    void update();
    void draw();
    void exit();

    //-------------------------------Kinect------------------------------------------------------------
    void drawPointCloud();
    
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
    ofParameter<bool>	doFlipKinect;

    int nearThreshold;
    int farThreshold;
    bool didKinectUpdate;
    ofFbo				kinectFbo;
    
    int angle;
    // used for viewing the point cloud
    ofEasyCam easyCam;
    //-------------------------------Kinect------------------------------------------------------------

    
    // Interaction
    void				keyPressed(int key);
    void				mouseMoved(int x, int y);
    void				mouseDragged(int x, int y, int button);
    ofVec2f				lastMouse;
    
    // GUI
    ofxPanel			gui;
    void				setupGui();
    ofParameter<float>	guiFPS;
    
    ofParameter<bool>	doFullScreen;
    void				setFullScreen(bool& _value) { ofSetFullscreen(_value);}
    ofParameter<bool>	toggleGuiDraw;
    ofParameter<bool>	doFlipCamera;
    ofParameter<int>	visualisationMode;
    ofParameter<string> visualisationName;
    int					numVisualisationModes;
    string				*visualisationModeTitles;
    ofParameterGroup	visualisationParameters;
    
    ofParameterGroup	drawForceParameters;
    ofParameter<bool>	doResetDrawForces;
    void				resetDrawForces(bool& _value) { if (_value) {for (int i=0; i<numDrawForces; i++) flexDrawForces[i].reset();} doResetDrawForces.set(false);}
    ofParameterGroup	leftButtonParameters;
    ofParameterGroup	rightButtonParameters;
    ofParameter<bool>	showScalar;
    ofParameter<bool>	showField;
    ofParameter<float>	displayScalarScale;
    void				setDisplayScalarScale(float& _value) { displayScalar.setScale(_value); }
    ofParameter<float>	velocityFieldArrowScale;
    void				setVelocityFieldArrowScale(float& _value) { velocityField.setVectorSize(_value); }
    ofParameter<float>	temperatureFieldBarScale;
    void				setTemperatureFieldBarScale(float& _value) { temperatureField.setVectorSize(_value); }
    ofParameter<bool>	visualisationLineSmooth;
    void				setVisualisationLineSmooth(bool& _value) { velocityField.setLineSmooth(_value); }
    
    
    // Camera
    ofVideoGrabber		simpleCam;
    bool				didCamUpdate;
    ofFbo				cameraFbo;
    
    // Time
    float				lastTime;
    float				deltaTime;
    
    // FlowTools
    int					flowWidth;
    int					flowHeight;
    int					drawWidth;
    int					drawHeight;
    
    flowTools::ftOpticalFlow		opticalFlow;
    flowTools::ftVelocityMask
    velocityMask;
    flowTools::ftFluidSimulation	fluid;
    flowTools::ftParticleFlow		particleFlow;
    
    flowTools::ftDisplayScalar		displayScalar;
    flowTools::ftVelocityField		velocityField;
    flowTools::ftTemperatureField	temperatureField;
    
    int					numDrawForces;
    flowTools::ftDrawForce*		flexDrawForces;
    
    ofImage				flowToolsLogoImage;
    bool				showLogo;
};
