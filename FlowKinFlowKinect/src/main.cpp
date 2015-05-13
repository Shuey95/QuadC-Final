#include "ofMain.h"
#include "ofApp.h"

#ifdef USE_PROGRAMMABLE_GL
#include "ofGLProgrammableRenderer.h"
#endif

//========================================================================
int main( ){
    
#ifdef 	USE_PROGRAMMABLE_GL
    ofPtr<ofBaseRenderer> renderer(new ofGLProgrammableRenderer(false));
    ofSetCurrentRenderer(renderer, false);
    //	ofSetCurrentRenderer(ofGLProgrammableRenderer::TYPE);
#endif
    ofSetupOpenGL(980, 780, OF_WINDOW);
    
    ofRunApp( new flowToolsApp());
    
}
