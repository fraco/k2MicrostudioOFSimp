#include "ofMain.h"
#include "ofxKinectV2OSC.h"
#include "ofxGenerative.h"

class ofxColorChangingBehavior : public ofxBehavior
{
    void actUpon(ofxRParticle *particle, ofVec3f &pos, ofVec3f &vel, ofVec3f &acc, float dt)
    {
        ofVec3f &home = particle->getHome();
        ofVec3f delta = home - pos;
        ofColor clr = ofColor::orangeRed;
        float len = delta.length()*2.0;
        clr.setHsb(len, len, 255, len);
        particle->setColor(clr);
    }
};

class ofApp : public ofBaseApp{
    
    public:
        ofxKinectV2OSC kinect;
        Skeleton* skeleton;
        vector<Skeleton>* skeletons;
        ofTrueTypeFont smallFont, largeFont;
        
        BodyRenderer rendererK;
        
        //GENERATIVE
        float easing;
        ofPoint targetMouse;
        ofPoint chaseMouse;
        ofPoint difMouse;
        
        int width;
        int height;
        int interval;
        ofxRParticleSystem *ps;
        ofxRParticleRenderer *renderer;
        ofxDistorterBehavior *distorter;
        ofxDamperBehavior *damper;
        float expFac;
        
        float ergM;
        
        void setup(){
            smallFont.loadFont("selena.otf", 12); //http://openfontlibrary.org/en/font/selena
            largeFont.loadFont("selena.otf", 18);
            
            ofSetLineWidth(8);
            ofSetFrameRate(60);
            
            //The Kinect here is just an OSC receiver and parser
            //It just needs a port number and font for the debug text
            kinect.setup(12345, smallFont);
            
            //Here we get a pointer to the list of skeletons it has parsed
            //from OSC
            skeletons = kinect.getSkeletons();
            
            //We could inspect the skeletons and draw them here in ofApp
            //but for now let's pass the list to a default renderer class
            rendererK.setup(skeletons); //, largeFont);
            
            
            ///ÑGENERATIVE
            ps = new ofxRParticleSystem();
            renderer = ps->getRendererPtr();
            renderer->setAdditiveBlending(true);
            renderer->setPointSize(10.0);
            
            interval = 8;
            width = ofGetWidth();
            height = ofGetHeight();
            
            for(float y = 0; y <= height; y+=interval)
            {
                for(float x = 0; x <= width; x+=interval)
                {
                    ps->addParticle( new ofxRParticle( ofVec3f( x, y, 0 ) ) );
                }
            }
            
            ps->setAccerationLimit(2);
            ps->setVelocityLimit(10);
            
            ofxHomingBehavior *homing = new ofxHomingBehavior();
            homing->setMagnitude(0.175);
            ps->addBehavior(homing);
            
            damper = new ofxDamperBehavior();
            damper->setMagnitude(0.95);
            ps->addBehavior(damper);
            
            distorter = new ofxDistorterBehavior();
            distorter->setPosition( ofVec3f( ofGetWidth()*0.5, ofGetHeight()*0.5,0.0 ) );
            distorter->setRadius(1000);
            distorter->setExpFactor(expFac);
            ps->addBehavior(distorter);
            
            ps->addBehavior(new ofxColorChangingBehavior());
            
            expFac = .5;
        }
        
        void update(){
            //Each frame check for new Kinect OSC messages
            kinect.update();
            
            ///ÑGENERATIVE
            distorter->setMagnitude(distorter->getMagnitude()*0.5);
            distorter->setExpFactor(expFac);
            ps->update();
        }
        
        void draw(){
            
            //	ofBackground(ofColor::darkGray);
            ofBackground(0);
            ofDisableDepthTest();
            ofEnableAlphaBlending();
            
            easing = 0.05;
            
            //Print out strings with the values from the network
            kinect.drawDebug();
            
            //We passed the skeleton list pointer to the renderer earlier,
            //now we tell it to draw them
            rendererK.draw();
            
            //If you want to stop using the default renderer and start
            //drawing your own graphics, uncomment this for a starting point:
            for(int i = 0; i < skeletons->size(); i++) {
                ofSetColor(ofColor::fromHsb(ofGetFrameNum() % 255, 255, 255));
                Joint handLeft = skeletons->at(i).getHandLeft();
                //        ofCircle(handLeft.x(), handLeft.y(), 60);
                Joint elbLeft = skeletons->at(i).getElbowLeft();
                
                Joint handRight = skeletons->at(i).getHandRight();
                //        ofCircle(handRight.x(), handRight.y(), 60);
                
                ergM = handLeft.getPoint().distance(elbLeft.getPoint());
                cout<<ergM<<endl;
                
                ///ÑGENERATIVE
                targetMouse = handLeft.getPoint();//ofPoint(mouseX,mouseY);
                
                difMouse = targetMouse - chaseMouse;
                
                if(targetMouse.distance(chaseMouse) > 2){
                    chaseMouse += difMouse * easing;
                }
                if(targetMouse.distance(chaseMouse) > 30){
                    //        ofSetColor(ofColor::limeGreen);
                    //        ofCircle(targetMouse, 44);
                    distorter->setMagnitude(100);
                    distorter->setPosition(targetMouse);
                    expFac = expFac - (.0005 * 1.0002);
                    if (expFac <= 1.0) {
                        expFac = .95;
                    }
                } else {
                    expFac = expFac + .0075;
                    if (expFac >= 2.0){
                        expFac = 2.0;
                    }
                }
                ofSetColor(ofColor::lightYellow,56);
                ofCircle(chaseMouse, 18);
                
                ps->draw();
            }
            
            //Print out commands and text
            string commands = "COMMANDS\n\n";
            commands.append("d = debug\n");
            commands.append("j = joints\n");
            commands.append("b = bones\n");
            commands.append("h = hands\n");
            commands.append("r = ranges\n");
            
            ofSetColor(ofColor::white);
            smallFont.drawString(commands, 20, 40);
            largeFont.drawString("fps:\n" + ofToString(ofGetFrameRate()), 20, ofGetHeight() - 100);
        }
        
        void keyPressed(int key){
            if(key == 'd') kinect.toggleDebug();
            if(key == 'j') rendererK.toggleJoints();
            if(key == 'b') rendererK.toggleBones();
            if(key == 'h') rendererK.toggleHands();
            //    if(key == 'r') rendererK.toggleRanges();
        }
    
};

int main( ){
    ofSetupOpenGL(1024,768,OF_WINDOW);
    ofRunApp(new ofApp());
}
