#ifndef _TEST_APP
#define _TEST_APP

#include "ofMain.h"

#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxControlPanel.h"

#include "DepthHoleFiller.h"


// include these 2 lines for median-of-5 code:
#include <algorithm>
using std::swap;

	


class testApp : public ofBaseApp
{
	

	public:

		void setup();
		void update();
		void draw();
		void exit();

		void keyPressed  (int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void windowResized(int w, int h);
	


	ofxControlPanel			gui;
	ofxKinect				kinect;
	DepthHoleFiller			DHF;
	
	
	//--------------------------------------------
	int						KW;
	int						KH;
	ofxCvColorImage			colorImg;
	ofxCvGrayscaleImage		ofxCv8uC1_Depth;
	ofxCvGrayscaleImage		ofxCv8uC1_DepthRaw;
	ofxCvGrayscaleImage		ofxCv8uC1_DepthRawThreshed;

	ofxCvGrayscaleImage		ofxCv8uC1_ThreshN;
	ofxCvGrayscaleImage		ofxCv8uC1_ThreshF;
	ofxCvGrayscaleImage		ofxCv8uC1_ThreshNF;

	
	//--------------------------------------------
	ofxCvContourFinder		contourFinder;
	int						MAX_N_CONTOUR_POINTS;
	CvPoint					**cvpts;
	int						*ncvpts;
	IplImage*				cvImgTemp;
	

	
	//--------------------------------------------
	bool				bComputeDepthHistogram;
	int					depthHistogramSize;
	int					depthHistogramWidth;
	int					depthHistogramHeight;
	CvHistogram*		depthHistogram;
	float*				depthHistogramData;
	void				drawDepthHistogram();
	void				computeDepthHistogram (ofxCvGrayscaleImage depthImage);
	
	//--------------------------------------------
	void				fillHolesPre();
	void				fillHolesPost();
	void				captureBackground();
	void				processBackground();

	bool				bLastCaptureBg;
	


	//--------------------------------------------	
	int					gW;
	int					gH;
	int					gM;
	
	void				computeFrameRate();
	float				kinectPrevFrameMillis;
	float				kinectFrameRate;
	
	void				drawPointCloud();

};

#endif
