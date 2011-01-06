/*
 *  DepthHoleFiller.h
 *  ofxKinect
 *
 *  Created by Golan Levin on 1/4/11.
 *  Copyright 2011 Carnegie Mellon University. All rights reserved.
 *
 */


#ifndef _DEPTH_HOLE_FILLER_
#define _DEPTH_HOLE_FILLER_

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxControlPanel.h"

#define	 MAX_DEPTH_HISTORY	16

class DepthHoleFiller {
	
	public:
		DepthHoleFiller ();
	
	int						KW;
	int						KH;
	void					setDimensions (int w, int h); 
	
	void					updatePreProcessingDepthHistory ( ofxCvGrayscaleImage ofxCv8uC1_DepthRawThreshed );
	void					setDepthHistory (int dh); 
	void					fillHolesUsingHistory ( ofxCvGrayscaleImage &ofxCv8uC1_Depth); 
	ofxCvGrayscaleImage		ofxCv8uC1_DepthHistory[MAX_DEPTH_HISTORY];
	int						nDepthHistory;
	
	
	void					fillHolesUsingContourFinder (ofxCvGrayscaleImage &input, int maxContourArea, int maxNContours);
	void					computeBlobContours (ofxCvGrayscaleImage &input, int maxContourArea, int maxNContours);
	void					computeBlobImage();
	void					fillBlobsWithInterpolatedData (ofxCvGrayscaleImage &input);
		
	void					performMorphologicalClose ( ofxCvGrayscaleImage &input);
	void					performMorphologicalClose ( ofxCvGrayscaleImage &input, int nTimes);
	void					performMorphologicalOpen  ( ofxCvGrayscaleImage &input);
	void					performMorphologicalOpen  ( ofxCvGrayscaleImage &input, int nTimes);
	void					performProperClose		  ( ofxCvGrayscaleImage &input);
	void					performProperClose		  ( ofxCvGrayscaleImage &input, int diameter);
	
	
	
	ofxCvGrayscaleImage		ofxCv8uC1_Temp1;
	ofxCvGrayscaleImage		ofxCv8uC1_Blobs;
	ofxCvGrayscaleImage		ofxCv8uC1_DepthInvalid;	
	ofxCvGrayscaleImage		ofxCv8uC1_DepthInvalidCopy;
		
	//--------------------------------------------
	ofxCvContourFinder		contourFinder;
	int						MAX_N_CONTOUR_POINTS;
	CvPoint					**cvpts;
	int						*ncvpts;
	IplImage*				cvImgTemp;
	
	
	
	
	
	
	/*
		int						nCloseOperations;
		int						nDilations;
		int						nDepthDilate;
		int						nDepthHistory;
		bool					bUseMedian;
	
		
	

		ofxCvGrayscaleImage		ofxCv8uC1_DepthPrev1;
		ofxCvGrayscaleImage		ofxCv8uC1_DepthPrev2;
		ofxCvGrayscaleImage		ofxCv8uC1_DepthPrev3;
		ofxCvGrayscaleImage		ofxCv8uC1_DepthPrev4;
		ofxCvGrayscaleImage		ofxCv8uC1_Depth;
	 */
	

	 
	
};

#endif