#include "testApp.h"


//--------------------------------------------------------------
void testApp::setup()
{
	kinect.init();
	kinect.setVerbose(true);
	kinect.open();
	kinect.enableDepthNearValueWhite(true);

	
	KW = kinect.width;
	KH = kinect.height;
	
	DHF.setDimensions(KW,KH);
	DHF.setDepthHistory(1); 

	colorImg.allocate				(KW,KH);
	
	ofxCv8uC1_Depth.allocate		(KW,KH);
	ofxCv8uC1_DepthRaw.allocate		(KW,KH);
	ofxCv8uC1_DepthRawThreshed.allocate	(KW,KH);
	
	ofxCv8uC1_ThreshN.allocate		(KW,KH);
	ofxCv8uC1_ThreshF.allocate		(KW,KH);
	ofxCv8uC1_ThreshNF.allocate		(KW,KH);
	
	
	cvImgTemp = cvCreateImage( cvSize(KW,KH), IPL_DEPTH_8U, 1); 
	MAX_N_CONTOUR_POINTS = 4000;
	cvpts    = new CvPoint*[1];
	cvpts[0] = new CvPoint[MAX_N_CONTOUR_POINTS];
	for (int i=0; i<MAX_N_CONTOUR_POINTS; i++){
		cvpts[0][i] = cvPoint(0,0);
	}
	ncvpts = new int[1];
	ncvpts[0] = 0;
	
	

	//-------------------------------
	depthHistogramSize = 256;
	depthHistogramWidth = depthHistogramSize;
	depthHistogramHeight = 96;
	float range_0[]={0, depthHistogramSize};
	float* histRanges[] = { range_0 };	
	depthHistogram = cvCreateHist(1, &depthHistogramSize, CV_HIST_ARRAY, histRanges, 1); 
	depthHistogramData = new float[depthHistogramSize];
	
	kinectPrevFrameMillis = 0;
	
	gW = 256;
	gH = (gW*3)/4;
	gM = 8; 

	
	// ofSetFrameRate(60);
	ofSetVerticalSync(false);
	bComputeDepthHistogram	= false;
	
	
	
		
	gui.setup("App Controls", gM, gH*2+gM*3, 320, 360);
	gui.addPanel(" Main Controls", 1, false);
	gui.addPanel(" Hole Filling", 1, false);
	
	//--------- PANEL 1
	gui.setWhichPanel(0);
	gui.setWhichColumn(0);
	gui.addSlider("Far Threshold",		"LO_THRESHOLD", 30, 0, 255, true);	
	gui.addSlider("Near Threshold",		"HI_THRESHOLD", 255, 0, 255, true);	
	gui.addToggle("Fill Using History?",	"FILL_USING_HISTORY", 0);
	gui.addToggle("Fill Using Closing?",	"FILL_USING_CLOSING", 0);
	gui.addToggle("Fill Using Contours?",	"FILL_USING_CONTOURS", 0);
	
	gui.addToggle("Compute Histogram?", "DO_HISTOGRAM", 0);
	
	//--------- PANEL 2
	gui.setWhichPanel(1);
	gui.setWhichColumn(0);
	gui.addSlider("Depth History",		"N_HISTORY",		2, 0, 16, true);
	gui.addSlider("# Closing Passes",	"N_CLOSING_PASSES",	1, 0, 5,  true);	
	gui.addSlider("Max Area to Fill",	"MAX_HOLE_AREA",	400, 1, 10000, true);
	gui.addSlider("Max #Holes to Fill",	"MAX_N_CONTOURS_TO_FILL",	100, 0, 1000, true);			  
				  

		
	gui.loadSettings("controlPanelSettings.xml");
	
}




//--------------------------------------------------------------
void testApp::update()
{
	ofBackground(100, 100, 100);
	gui.update();
	kinect.update();
	if (kinect.isFrameNew()){
		
		// compute Kinect frame rate
		computeFrameRate();
		
		
		int nearThreshold		= gui.getValueI("LO_THRESHOLD");
		int farThreshold		= gui.getValueI("HI_THRESHOLD");

		colorImg.setFromPixels(kinect.getPixels(), KW,KH);
		colorImg.mirror(false, true);
		
		// Retrieve the current depth buffer.
		ofxCv8uC1_DepthRaw.setFromPixels(kinect.getDepthPixels(), KW,KH);
		ofxCv8uC1_DepthRaw.mirror(false, true);

		// Compute a double-ended threshold of the depth image
		ofxCv8uC1_ThreshN = ofxCv8uC1_DepthRaw;
		ofxCv8uC1_ThreshF = ofxCv8uC1_DepthRaw;
		ofxCv8uC1_ThreshN.threshold(nearThreshold, false);
		ofxCv8uC1_ThreshF.threshold(farThreshold,  true);
		cvAnd(	ofxCv8uC1_ThreshN.getCvImage(), 
				ofxCv8uC1_ThreshF.getCvImage(), 
				ofxCv8uC1_ThreshNF.getCvImage(), NULL);
		cvAnd(	ofxCv8uC1_ThreshNF.getCvImage(), 
				ofxCv8uC1_DepthRaw.getCvImage(), 
				ofxCv8uC1_DepthRawThreshed.getCvImage(), NULL);
		
		
		ofxCv8uC1_Depth = ofxCv8uC1_DepthRawThreshed;
		
		
		
		bool bFillHolesUsingHistory		= gui.getValueB("FILL_USING_HISTORY", 0);
		bool bFillHolesUsingContours	= gui.getValueB("FILL_USING_CONTOURS", 0);
		bool bFillHolesUsingClosing		= gui.getValueB("FILL_USING_CLOSING", 0);
			
		if (bFillHolesUsingHistory){
			int depthHistory = gui.getValueI("N_HISTORY");
			DHF.setDepthHistory(depthHistory); 
			DHF.updatePreProcessingDepthHistory (ofxCv8uC1_Depth );
			DHF.fillHolesUsingHistory (ofxCv8uC1_Depth); 
		}
		
		if (bFillHolesUsingClosing){
			int nClosingPasses = gui.getValueI("N_CLOSING_PASSES");
			DHF.performMorphologicalClose(ofxCv8uC1_Depth,nClosingPasses);
		}
		
		if (bFillHolesUsingContours){
			int maxContourHoleToFillArea = gui.getValueI("MAX_HOLE_AREA");
			int maxNContoursToFill = gui.getValueI("MAX_N_CONTOURS_TO_FILL");
			DHF.fillHolesUsingContourFinder (ofxCv8uC1_Depth, maxContourHoleToFillArea, maxNContoursToFill);
		}
		
		
		
		
		
		
		bComputeDepthHistogram	= gui.getValueB("DO_HISTOGRAM", 0);
		if (bComputeDepthHistogram){
			computeDepthHistogram (ofxCv8uC1_Depth); 
		}

	}
}









//--------------------------------------------------------------
void testApp::computeDepthHistogram (ofxCvGrayscaleImage depthImage){
	// Compute the histogram of the depth-colored difference-from-background image. 
	
	IplImage*  iplDepthImg = depthImage.getCvImage();
	cvCalcHist( &iplDepthImg, depthHistogram, 0, NULL );
	float *depthHistArr = cvGetHistValue_1D (depthHistogram, 0);
	
	int maxVal = 0;
	int startIndex = 1; // don't count black pixels. 
	for (int i=startIndex; i<depthHistogramSize; i++){
		if (depthHistArr[i] > maxVal){
			maxVal = depthHistArr[i];
		}
	}
	
	for (int i=0; i<depthHistogramSize; i++){
		depthHistogramData[i] = depthHistArr[i] / (float)maxVal;
	}
}



//--------------------------------------------------------------
void testApp::draw()
{
	glColor3f(1,1,1);
	colorImg.draw					(gM*2+gW*1,	gM*1,		gW, gH);
	ofxCv8uC1_DepthRaw.draw			(gM*1+gW*0,	gM*1,		gW, gH);
	contourFinder.draw				(gM*1+gW*0, gM*1,		gW,	gH);
	DHF.ofxCv8uC1_Blobs.draw		(gM*1+gW*0, gM*2+gH,		gW,	gH);

	float sc = 2.0;//2.75;
	float gr = 1.0;
	glColor3f(gr,gr,gr);
	ofxCv8uC1_Depth.draw			(gM*3+gW*2,	gM*1,		gW*sc, gH*sc);
	
	if (bComputeDepthHistogram){ 
		drawDepthHistogram(); 
	}
	
	
	
	char reportStr[1024];
	sprintf(reportStr, "Kfps: %f", kinectFrameRate);//ofGetFrameRate());
	ofSetColor(255, 255, 255);
	ofDrawBitmapString(reportStr, gW*0+gM*1, ofGetHeight()-gM);
	
	gui.draw();
}






//--------------------------------------------------------------
void testApp::drawDepthHistogram(){
	
	
	glPushMatrix();
	glTranslatef(gM*2+gW*1, gM*3+gH*2,0);
	
	glColor3f(0.5,0.5,0.5);
	ofRect(0,0,depthHistogramWidth,depthHistogramHeight); 
	glColor3f(1,1,1);
	for (int i=1; i<depthHistogramSize; i++){ // skip 0 (black)
		float y = depthHistogramData[i] * depthHistogramHeight;
		ofLine(i, 0, i, y);
	}
	glPopMatrix();
}






//--------------------------------------------------------------
void testApp::exit(){
	kinect.close();
}

//--------------------------------------------------------------
void testApp::keyPressed (int key)
{
	switch (key)
	{
		

	}
}

//--------------------------------------------------------------
void testApp::mouseMoved(int x, int y){
}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button){
	gui.mouseDragged(x, y, button);
}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button){
	gui.mousePressed(x, y, button);
}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button){
	gui.mouseReleased();
}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h){
}


//--------------------------------------------------------------
void testApp::computeFrameRate(){
	float now = ofGetElapsedTimeMillis();
	float FR = 1000.0/(now - kinectPrevFrameMillis);
	float fA = 0.95; 
	float fB = 1.0-fA;
	kinectFrameRate = (fA*kinectFrameRate) + (fB*FR); 
	kinectPrevFrameMillis = now;
}








