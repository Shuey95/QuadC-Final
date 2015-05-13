#include "ofApp.h"

//--------------------------------------------------------------
void flowToolsApp::setup(){
    
    ofSetFrameRate(70);
    //-------------------------------Kinect------------------------------------------------------------
    ofSetLogLevel(OF_LOG_VERBOSE);
    
    kinect.setRegistration(true);
    
    kinect.init();
    
    
    kinect.open();
    
    if(kinect.isConnected()) {
        ofLogNotice() << "sensor-emitter dist: " << kinect.getSensorEmitterDistance() << "cm";
        ofLogNotice() << "sensor-camera dist:  " << kinect.getSensorCameraDistance() << "cm";
        ofLogNotice() << "zero plane pixel size: " << kinect.getZeroPlanePixelSize() << "mm";
        ofLogNotice() << "zero plane dist: " << kinect.getZeroPlaneDistance() << "mm";
    }
    
    colorImg.allocate(kinect.width, kinect.height);
    grayImage.allocate(kinect.width, kinect.height);
    grayThreshNear.allocate(kinect.width, kinect.height);
    grayThreshFar.allocate(kinect.width, kinect.height);
    
    nearThreshold = 230;
    farThreshold = 70;
    bThreshWithOpenCV = true;
    
    ofSetFrameRate(60);
    
    // -------------zero the tilt on startup-------------
    angle = 0;
    kinect.setCameraTiltAngle(angle);

    //-------------------------------Kinect------------------------------------------------------------

    ofSetVerticalSync(true);
    
    drawWidth = 980;
    drawHeight = 720;
    // -------------process all but the density on 16th resolution-------------
    flowWidth = drawWidth/4;
    flowHeight = drawHeight/4;
    
    // -------------Flow and Mask
    opticalFlow.setup(flowWidth, flowHeight);
    velocityMask.setup(drawWidth, drawHeight);
    
    //-------------Fluid commands-------------
#ifdef USE_FASTER_INTERNAL_FORMATS
    fluid.setup(flowWidth, flowHeight, drawWidth, drawHeight, true);
#else
    fluid.setup(flowWidth, flowHeight, drawWidth, drawHeight, false);
#endif
    
    
    // -------------Particles-------------
    particleFlow.setup(flowWidth, flowHeight, drawWidth, drawHeight);
    
    //-------------Visualisation-------------
    displayScalar.allocate(flowWidth, flowHeight);
    velocityField.allocate(flowWidth /3, flowHeight / 4);
    temperatureField.allocate(flowWidth / 3, flowHeight / 4);
    
    //-------------Draw Forces-------------
    numDrawForces = 6;
    flexDrawForces = new ftDrawForce[numDrawForces];
    flexDrawForces[0].setup(drawWidth, drawHeight, FT_DENSITY, true);
    flexDrawForces[0].setName("draw full res");
    flexDrawForces[1].setup(flowWidth, flowHeight, FT_VELOCITY, true);
    flexDrawForces[1].setName("draw flow res 1");
    flexDrawForces[2].setup(flowWidth, flowHeight, FT_TEMPERATURE, true);
    flexDrawForces[2].setName("draw flow res 2");
    flexDrawForces[3].setup(drawWidth, drawHeight, FT_DENSITY, false);
    flexDrawForces[3].setName("draw full res");
    flexDrawForces[4].setup(flowWidth, flowHeight, FT_VELOCITY, false);
    flexDrawForces[4].setName("draw flow res 1");
    flexDrawForces[5].setup(flowWidth, flowHeight, FT_TEMPERATURE, false);
    flexDrawForces[5].setName("draw flow res 2");
    
//-------------Kinect initialize-------------
   didKinectUpdate = false;
   kinectFbo.allocate(drawWidth, drawHeight);
   kinectFbo.begin(); ofClear(0); kinectFbo.end();
    
    
    setupGui();
    
    lastTime = ofGetElapsedTimef();
    lastMouse.set(0,0);
    
}

//--------------------------------------------------------------
void flowToolsApp::setupGui() {
    
    gui.setup("settings");
    gui.setDefaultBackgroundColor(ofColor(0, 0, 0, 0));
    gui.setDefaultFillColor(ofColor(0, 0, 0, 0));
    gui.add(guiFPS.set("FPS", 90, 90, 90));
    numVisualisationModes =100;
    gui.add(visualisationMode.set("visualisation mode",50,50, numVisualisationModes -100));
    
  visualisationModeTitles = new string[numVisualisationModes];

    
    int guiColorSwitch = ofRandom(100);
    ofColor guiHeaderColor[2];
    guiHeaderColor[0].set(0, 0, 0, 0);
    guiHeaderColor[1].set(0, 0, 0, 0);
    ofColor guiFillColor[2];
    guiFillColor[0].set(0, 0, 0, 0);
    guiFillColor[1].set(0, 0, 0, 0);
    
    gui.setDefaultHeaderBackgroundColor(guiHeaderColor[guiColorSwitch]);
    gui.setDefaultFillColor(guiFillColor[guiColorSwitch]);
    guiColorSwitch = 1 - guiColorSwitch;
    gui.add(opticalFlow.parameters);
    
    gui.setDefaultHeaderBackgroundColor(guiHeaderColor[guiColorSwitch]);
    gui.setDefaultFillColor(guiFillColor[guiColorSwitch]);
    guiColorSwitch = 1 - guiColorSwitch;
    gui.add(velocityMask.parameters);
    
    gui.setDefaultHeaderBackgroundColor(guiHeaderColor[guiColorSwitch]);
    gui.setDefaultFillColor(guiFillColor[guiColorSwitch]);
    guiColorSwitch = 1 - guiColorSwitch;
    gui.add(fluid.parameters);
    
    gui.setDefaultHeaderBackgroundColor(guiHeaderColor[guiColorSwitch]);
    gui.setDefaultFillColor(guiFillColor[guiColorSwitch]);
    guiColorSwitch = 1 - guiColorSwitch;
    gui.add(particleFlow.parameters);
    
    visualisationParameters.setName("visualisation");
    visualisationParameters.add(showScalar.set("show scalar", true));
    visualisationParameters.add(showField.set("show field", true));
    visualisationParameters.add(displayScalarScale.set("display scalar scale", 50.25, 5.05, 5.5));
    displayScalarScale.addListener(this, &flowToolsApp::setDisplayScalarScale);
    visualisationParameters.add(velocityFieldArrowScale.set("arrow scale", 10.6, 10.2, 1));
    velocityFieldArrowScale.addListener(this, &flowToolsApp::setVelocityFieldArrowScale);
    visualisationParameters.add(temperatureFieldBarScale.set("temperature scale", 0.25, 0.05, 0.5));
    temperatureFieldBarScale.addListener(this, &flowToolsApp::setTemperatureFieldBarScale);
    visualisationParameters.add(visualisationLineSmooth.set("line smooth", false));
    visualisationLineSmooth.addListener(this, &flowToolsApp::setVisualisationLineSmooth);
    
    gui.setDefaultHeaderBackgroundColor(guiHeaderColor[guiColorSwitch]);
    gui.setDefaultFillColor(guiFillColor[guiColorSwitch]);
    guiColorSwitch = 1 - guiColorSwitch;
    gui.add(visualisationParameters);
    
    leftButtonParameters.setName("mouse left button");
    for (int i=0; i<3; i++) {
        leftButtonParameters.add(flexDrawForces[i].parameters);
    }
    gui.setDefaultHeaderBackgroundColor(guiHeaderColor[guiColorSwitch]);
    gui.setDefaultFillColor(guiFillColor[guiColorSwitch]);
    guiColorSwitch = 1 - guiColorSwitch;
    gui.add(leftButtonParameters);
    
    rightButtonParameters.setName("mouse right button");
    for (int i=3; i<6; i++) {
        rightButtonParameters.add(flexDrawForces[i].parameters);
    }
    gui.setDefaultHeaderBackgroundColor(guiHeaderColor[guiColorSwitch]);
    gui.setDefaultFillColor(guiFillColor[guiColorSwitch]);
    guiColorSwitch = 1 - guiColorSwitch;
    gui.add(rightButtonParameters);
    
    gui.setDefaultHeaderBackgroundColor(guiHeaderColor[guiColorSwitch]);
    gui.setDefaultFillColor(guiFillColor[guiColorSwitch]);
    guiColorSwitch = 1 - guiColorSwitch;
    gui.add(doResetDrawForces.set("reset draw forces (D)", false));
    doResetDrawForces.addListener(this,  &flowToolsApp::resetDrawForces);
    
    gui.loadFromFile("settings.xml");
    gui.minimizeAll();
    
    toggleGuiDraw = true;
    
}

//--------------------------------------------------------------
void flowToolsApp::update(){
    //-------------------------------Kinect------------------------------------------------------------
    
    //ofBackground(0, 255, 255);
    
    kinect.update();
    
    //-------------there is a new frame and we are connected-------------
    if(kinect.isFrameNew()) {
    grayImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);
        
        if(bThreshWithOpenCV) {
            grayThreshNear = grayImage;
            grayThreshFar = grayImage;
            grayThreshNear.threshold(nearThreshold, true);
            grayThreshFar.threshold(farThreshold);
            cvAnd(grayThreshNear.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);
        } else {
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
        
        grayImage.flagImageChanged();
        contourFinder.findContours(grayImage, 10, (kinect.width*kinect.height)/2, 20, false);
    }
    
#ifdef USE_TWO_KINECTS
    kinect2.update();
#endif

    //-------------------------------Kinect------------------------------------------------------------

    didKinectUpdate = false;
    kinect.update();
    if (kinect.isFrameNew())
        didKinectUpdate = true;
    
    deltaTime = ofGetElapsedTimef() - lastTime;
    lastTime = ofGetElapsedTimef();
    
}

//--------------------------------------------------------------
void flowToolsApp::draw(){
    
    //-------------------------------Kinect------------------------------------------------------------
    
    ofSetColor(255, 255, 255);
    
    if(bDrawPointCloud) {
        easyCam.begin();
        drawPointCloud();
        easyCam.end();
    //-------------------------------Kinect------------------------------------------------------------

    ofClear(0);
    
    if (didKinectUpdate) {
        ofPushStyle();
        ofEnableBlendMode(OF_BLENDMODE_DISABLED);
        kinectFbo.begin();
        if (doFlipKinect)
            kinect.draw(kinectFbo.getWidth(), 0, -kinectFbo.getWidth(), kinectFbo.getHeight());  // Flip Horizontal
        else
            kinect.draw(0, 0, kinectFbo.getWidth(), kinectFbo.getHeight());
        kinectFbo.end();
        ofPopStyle();
        
        opticalFlow.setSource(kinectFbo.getTextureReference());
        opticalFlow.update(deltaTime);
        
        velocityMask.setDensity(kinectFbo.getTextureReference());
        velocityMask.setVelocity(opticalFlow.getOpticalFlow());
        velocityMask.update();
    }
    
    
    fluid.addVelocity(opticalFlow.getOpticalFlowDecay());
    fluid.addDensity(velocityMask.getColorMask());
    fluid.addTemperature(velocityMask.getLuminanceMask());
    
    for (int i=0; i<numDrawForces; i++) {
        flexDrawForces[i].update();
        if (flexDrawForces[i].didChange()) {
            float strength = flexDrawForces[i].getStrength();
            if (!flexDrawForces[i].getIsTemporary())
                strength *=deltaTime;
            switch (flexDrawForces[i].getType()) {
                case FT_DENSITY:
                    fluid.addDensity(flexDrawForces[i].getTextureReference(), strength);
                    break;
                case FT_VELOCITY:
                    fluid.addVelocity(flexDrawForces[i].getTextureReference(), strength);
                    particleFlow.addFlowVelocity(flexDrawForces[i].getTextureReference(), strength);
                    break;
                case FT_TEMPERATURE:
                    fluid.addTemperature(flexDrawForces[i].getTextureReference(), strength);
                    break;
                case FT_PRESSURE:
                    fluid.addPressure(flexDrawForces[i].getTextureReference(), strength);
                    break;
                case FT_OBSTACLE:
                    fluid.addTempObstacle(flexDrawForces[i].getTextureReference());
                default:
                    break;
            }
        }
    }
    
    fluid.update();
    
    if (particleFlow.isActive()) {
        particleFlow.setSpeed(fluid.getSpeed());
        particleFlow.setCellSize(fluid.getCellSize());
        particleFlow.addFlowVelocity(opticalFlow.getOpticalFlow());
        particleFlow.addFluidVelocity(fluid.getVelocity());
        particleFlow.setObstacle(fluid.getObstacle());
    }
    particleFlow.update();
    
    
    int windowWidth = ofGetWindowWidth();
    int windowHeight = ofGetWindowHeight();
    ofClear(0,0);
    //Switch ksys-------------
    switch(visualisationMode.get()) {
        case 0:
            kinectFbo.draw(0,0, windowWidth, windowHeight);
            break;
       
        case 2: //-------------Optical Flow Mask-------------
            ofPushStyle();
            ofEnableBlendMode(OF_BLENDMODE_DISABLED);
            ofBackgroundGradient(ofColor::whiteSmoke, ofColor::blanchedAlmond, OF_GRADIENT_CIRCULAR);
            	ofSetColor(ofRandom(0), 0, 0);
             ofRect(0,0,windowWidth, windowHeight);
             ofEnableBlendMode(OF_BLENDMODE_ALPHA);
             ofSetColor(255, 255, 255);
            //-----COOL EARTHQUAKE EFFECT
             	//velocityMask.draw(ofRandom(255), ofRandom(255), windowWidth, windowHeight);
            velocityMask.draw(0, 0, windowWidth, windowHeight);
            ofPopStyle();
            break;
            
        case 10: // Fluid Color
            ofPushStyle();
            fluid.draw(0, 0, windowWidth, windowHeight);
            ofEnableBlendMode(OF_BLENDMODE_MULTIPLY);
            if (particleFlow.isActive())
                particleFlow.draw(0, 0, windowWidth, windowHeight);
            if (showLogo) {
                ofEnableBlendMode(OF_BLENDMODE_MULTIPLY);
                ofSetColor(ofRandom(255),255,255,255);
            }
            ofPopStyle();
            break;
        case 11: // pFluid Composite
            ofPushStyle();
            ofEnableBlendMode(OF_BLENDMODE_DISABLED);
            kinectFbo.draw(0,0, windowWidth, windowHeight);
            
            ofEnableBlendMode(OF_BLENDMODE_ADD);
            fluid.draw(0, 0, windowWidth, windowHeight);
            
            }
        
                    ofPopStyle();
        
    }
    
    if (toggleGuiDraw) {
        guiFPS = ofGetFrameRate();
        if (visualisationMode.get() >= numVisualisationModes)
            visualisationMode.set(numVisualisationModes-1);
        visualisationName.set(visualisationModeTitles[visualisationMode.get()]);
        gui.draw();
    }
}

//-------------------------
void flowToolsApp::drawPointCloud() {
    int w = 640;
    int h = 480;
    ofMesh mesh;
    mesh.setMode(OF_PRIMITIVE_POINTS);
    int step = 2;
    for(int y = 0; y < h; y += step) {
        for(int x = 0; x < w; x += step) {
            if(kinect.getDistanceAt(x, y) > 0) {
                mesh.addColor(kinect.getColorAt(x,y));
                mesh.addVertex(kinect.getWorldCoordinateAt(x, y));
            }
        }
    }
    glPointSize(3);
    ofPushMatrix();
    // the projected points are 'upside down' and 'backwards'
    ofScale(1, -1, -1);
    ofTranslate(0, 0, -1000); // center the points a bit
    ofEnableDepthTest();
    mesh.drawVertices();
    ofDisableDepthTest();
    ofPopMatrix();
}
//------
void flowToolsApp::exit() {
    kinect.setCameraTiltAngle(0); // zero the tilt on exit
    kinect.close();
    
#ifdef USE_TWO_KINECTS
    kinect2.close();
#endif
}


//--------------------------------------------------------------
void flowToolsApp::keyPressed(int key){
    switch (key) {
            //kinect
        case ' ':
            bThreshWithOpenCV = !bThreshWithOpenCV;
            break;
            
        case'p':
            bDrawPointCloud = !bDrawPointCloud;
            break;
            //kinect
        case 'G' :
            toggleGuiDraw = !toggleGuiDraw;
            break;
        case 'F':
            ofToggleFullscreen();
            break;
        case 'R':
            fluid.reset();
            showLogo = false;
            for (int i=0; i<numDrawForces; i++)
                flexDrawForces[i].reset();
            break;
        case 'D':
            doResetDrawForces.set(true);
            break;
        case '0':
            visualisationMode.set(0);
            break;
        case '1':
            visualisationMode.set(1);
            break;
        case '2':
            visualisationMode.set(2);
            break;
        case '3':
            visualisationMode.set(3);
            break;
        case '4':
            visualisationMode.set(4);
            break;
        case '5':
            visualisationMode.set(5);
            break;
        case '6':
            visualisationMode.set(6);
            break;
        case '7':
            visualisationMode.set(7);
            break;
        case '8':
            visualisationMode.set(8);
            break;
        case '9':
            visualisationMode.set(9);
            break;
        case '-':
            visualisationMode.set(10);
            break;
        case '=':
            visualisationMode.set(11);
            break;
        default:
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
void flowToolsApp::mouseDragged(int x, int y, int button) {
    ofVec2f mouse;
    
    mouse.set(x / (float)ofGetWindowWidth(), y / (float)ofGetWindowHeight());
    ofVec2f velocity = mouse - lastMouse;
    if (button == 0) {
        
        for (int i=0; i<3; i++) {
            if (flexDrawForces[i].getType() == FT_VELOCITY)
                flexDrawForces[i].setForce(velocity);
            flexDrawForces[i].applyForce(mouse);
        }
    }
    else {
        
        for (int i=3; i<numDrawForces; i++) {
            if (flexDrawForces[i].getType() == FT_VELOCITY)
                flexDrawForces[i].setForce(velocity);
            flexDrawForces[i].applyForce(mouse);
        }
    }
    lastMouse.set(mouse.x, mouse.y);
    
}

//--------------------------------------------------------------
void flowToolsApp::mouseMoved(int x, int y){
    ofVec2f mouse;
    mouse.set(x / (float)ofGetWindowWidth(), y / (float)ofGetWindowHeight());
    lastMouse.set(mouse.x, mouse.y);
    
}
