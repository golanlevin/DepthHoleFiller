/*
 *  depthHoleFiller.cpp
 *  ofxKinect
 *
 *  Created by Golan Levin on 1/4/11.
 *  Copyright 2011 Carnegie Mellon University. All rights reserved.
 *
 */

#include "DepthHoleFiller.h"

DepthHoleFiller::DepthHoleFiller(){
	KW = 640;
	KH = 480;
}

//--------------------------------------------------------------
void DepthHoleFiller::setDimensions (int w, int h){
	KW = w; 
	KH = h;
	
	ofxCv8uC1_Temp1.allocate		(KW,KH);
	ofxCv8uC1_Blobs.allocate		(KW,KH);
	ofxCv8uC1_DepthInvalid.allocate		(KW,KH);
	ofxCv8uC1_DepthInvalidCopy.allocate (KW,KH);
	
	//----------------------------
	cvImgTemp = cvCreateImage( cvSize(KW,KH), IPL_DEPTH_8U, 1); 
	MAX_N_CONTOUR_POINTS = 4000;
	cvpts    = new CvPoint*[1];
	cvpts[0] = new CvPoint[MAX_N_CONTOUR_POINTS];
	for (int i=0; i<MAX_N_CONTOUR_POINTS; i++){
		cvpts[0][i] = cvPoint(0,0);
	}
	ncvpts = new int[1];
	ncvpts[0] = 0;
	
	//----------------------------
	nDepthHistory = 0;
	for (int i=0; i<MAX_DEPTH_HISTORY; i++){
		ofxCv8uC1_DepthHistory[i].allocate(KW,KH);
		ofxCv8uC1_DepthHistory[i].set(0);
	}				
	
}

//--------------------------------------------------------------
void DepthHoleFiller::updatePreProcessingDepthHistory ( ofxCvGrayscaleImage ofxCv8uC1_DepthRawThreshed ){
	
	if (nDepthHistory > 0){
		for (int i=(nDepthHistory-1); i>0; i--){
			int olderId = i;
			int newerId = i-1;
			cvCopy(ofxCv8uC1_DepthHistory[newerId].getCvImage(), ofxCv8uC1_DepthHistory[olderId].getCvImage(), NULL);
		}
		cvCopy(ofxCv8uC1_DepthRawThreshed.getCvImage(), ofxCv8uC1_DepthHistory[0].getCvImage(), NULL);
	}
}

//--------------------------------------------------------------
void DepthHoleFiller::fillHolesUsingHistory(ofxCvGrayscaleImage &ofxCv8uC1_Depth){
	
	for (int i=1; i<nDepthHistory; i++){
		
		// get all currently non-valid pixels.
		// note that ofxCv8uC1_Depth gets more and more filled with each iteration. 
		bool bInvert = true;
		int FIRST_VALID_VALUE = 1;
		ofxCv8uC1_DepthInvalid = ofxCv8uC1_Depth;
		ofxCv8uC1_DepthInvalid.threshold(FIRST_VALID_VALUE, bInvert);
		
		// And them with the previous frame, to extract
		// those pixels from the previous frame which are invalid in the current one
		cvAnd(ofxCv8uC1_DepthInvalid.getCvImage(), 
			  ofxCv8uC1_DepthHistory[i].getCvImage(), 
			  ofxCv8uC1_DepthInvalid.getCvImage(), NULL);
		
		// Add them in together
		cvOr (ofxCv8uC1_Depth.getCvImage(), 
			  ofxCv8uC1_DepthInvalid.getCvImage(), 
			  ofxCv8uC1_Depth.getCvImage(), NULL);
	}
}
											
										


//--------------------------------------------------------------
void DepthHoleFiller::setDepthHistory (int dh){
	if (dh > MAX_DEPTH_HISTORY){
		printf("Requested depth history (%d) cannot be larger than %d\n", dh, MAX_DEPTH_HISTORY); 
	}
	dh = MAX(0,MIN(MAX_DEPTH_HISTORY, dh));
	nDepthHistory = dh;
}


//--------------------------------------------------------------
void DepthHoleFiller::fillHolesUsingContourFinder (ofxCvGrayscaleImage &depthImage, 
												   int maxContourArea, 
												   int maxNContours ){
	
	// Find just the holes, as geometric contours.
	computeBlobContours (depthImage, maxContourArea, maxNContours);
	
	// Rasterize those holes as filled polys, white on a black background.
	computeBlobImage();
	
	// Re-color the blobs with graylevels interpolated from the depth image.
	fillBlobsWithInterpolatedData (depthImage);
	
	// Add the interpolated holes back into the depth image
	cvMax(depthImage.getCvImage(), 
		  ofxCv8uC1_Blobs.getCvImage(), 
		  depthImage.getCvImage());
	
	// Flag changes to the images. 
	ofxCv8uC1_Blobs.flagImageChanged();
	depthImage.flagImageChanged();
}


//--------------------------------------------------------------
void DepthHoleFiller::computeBlobContours (ofxCvGrayscaleImage &depthImage, int maxContourArea, int maxNContours){
	
	// Note: "find holes" is set to true so we just get the exterior contour.
	int minContourArea = 1;
	contourFinder.findContours(depthImage, minContourArea, maxContourArea, maxNContours, true);
}

//--------------------------------------------------------------
void DepthHoleFiller::computeBlobImage (){
	
	ofxCv8uC1_Blobs.set(0); // clear any previous blobs.
	CvScalar color_white = CV_RGB(255,255,255);
	
	int nHolesToFill = contourFinder.blobs.size();
	for (int i=0; i<nHolesToFill; i++){
		ofxCvBlob blob = contourFinder.blobs.at(i);
		int nPts = MIN(blob.nPts, MAX_N_CONTOUR_POINTS);
		
		for (int j=0; j<nPts; j++){ 
			ofPoint pt = blob.pts.at(j);
			int px = (int) pt.x; // 0...imgw
			int py = (int) pt.y; // 0...imgh
			cvpts[0][j].x = px;
			cvpts[0][j].y = py;
		}
		ncvpts[0] = nPts;
		cvFillPoly (ofxCv8uC1_Blobs.getCvImage(), cvpts, ncvpts, 1, color_white, 8, 0 );
	}
}



//==============================================================
void DepthHoleFiller::fillBlobsWithInterpolatedData (ofxCvGrayscaleImage &depthImage){
	
	// interpolates between one edge of the hole and the other
	
	unsigned char *depthPixels =      depthImage.getPixels();
	unsigned char *blobsPixels = ofxCv8uC1_Blobs.getPixels();
	
	const unsigned char HOLE = 255;
	int row = 0;
	int index=0;
	
	int runIndexA;
	int runIndexB;
	unsigned char runValueA;
	unsigned char runValueB;
	
	for (int y=0; y<KH; y++){
		runIndexA = 0;
		runIndexB = 0;
		
		bool bRunStarted = false;
		unsigned char bval0;
		unsigned char bval1;
		row = y*KW;
		
		for (int x=1; x<KW; x++){
			index = row + x; 
			bval0 = blobsPixels[index-1];
			bval1 = blobsPixels[index  ];
			
			if ((bval0 != HOLE) && (bval1 == HOLE)){
				runIndexA = index;
				runValueA = depthPixels[index-1];
				bRunStarted = true;
			}
			if (bRunStarted){
				if ((bval0 == HOLE) && (bval1 != HOLE)){
					runIndexB = index-1;
					runValueB = depthPixels[index];
					bRunStarted = false;
					
					// since we have identified a hole run, fill it appropriately
					int runlen = (runIndexB - runIndexA);
					int vallen = (runValueB - runValueA);
					
					// interpolate between the values.
					if (runlen <= 1){
						int va = runValueA;
						int vb = runValueB;
						unsigned char val = (unsigned char)((va+vb)/2);
						for (int i=runIndexA; i<=runIndexB; i++){
							blobsPixels[i] = val;
						}
					} else {
						float runlenf = (float)(runlen);
						float vallenf = (float)(vallen);
						for (int i=runIndexA; i<=runIndexB; i++){
							float tf = (float)(i-runIndexA)/runlenf;
							float vf = runValueA + tf*vallenf; 
							unsigned char val = (unsigned char)(vf + 0.5);
							blobsPixels[i] = val;
						}
					}
					
				}
			}
			
		}
		
	}
}



//--------------------------------------------------------------
void DepthHoleFiller::performProperClose		  ( ofxCvGrayscaleImage &input){
	
	// http://homepages.inf.ed.ac.uk/rbf/HIPR2/close.htm 
	// Defined as Max(f, O(C(O(f))))
	
	ofxCv8uC1_Temp1 = input; // temp copy of original
	
	performMorphologicalOpen	(input);
	performMorphologicalClose	(input);
	performMorphologicalOpen	(input);
	
	cvMax(input.getCvImage(), 
		  ofxCv8uC1_Temp1.getCvImage(), 
		  input.getCvImage());
}

//--------------------------------------------------------------
void DepthHoleFiller::performProperClose		  ( ofxCvGrayscaleImage &input, int diameter){
	
	// http://homepages.inf.ed.ac.uk/rbf/HIPR2/close.htm 
	// Defined as Max(f, O(C(O(f))))
	
	ofxCv8uC1_Temp1 = input; // temp copy of original
	
	performMorphologicalOpen	(input, diameter);
	performMorphologicalClose	(input, diameter);
	performMorphologicalOpen	(input, diameter);
	
	cvMax(input.getCvImage(), 
		  ofxCv8uC1_Temp1.getCvImage(), 
		  input.getCvImage());
}


//--------------------------------------------------------------
void DepthHoleFiller::performMorphologicalClose ( ofxCvGrayscaleImage &input){
	
	// Clean up the holes using morphological close. 
	// http://homepages.inf.ed.ac.uk/rbf/HIPR2/close.htm
	
	input.dilate();
	input.erode();
}

//--------------------------------------------------------------
void DepthHoleFiller::performMorphologicalOpen ( ofxCvGrayscaleImage &input){
	
	// Clean up the holes using morphological close. 
	// http://homepages.inf.ed.ac.uk/rbf/HIPR2/open.htm
	
	input.erode();
	input.dilate();
}

//--------------------------------------------------------------
void DepthHoleFiller::performMorphologicalClose ( ofxCvGrayscaleImage &input, int diameter){
	
	// Clean up the holes using morphological close. 
	// use a "larger structural element" by repeated passes.
	// http://homepages.inf.ed.ac.uk/rbf/HIPR2/close.htm
	
	for (int i=0; i<diameter; i++){
		input.dilate();
	}
	for (int i=0; i<diameter; i++){
		input.erode();
	}
}

//--------------------------------------------------------------
void DepthHoleFiller::performMorphologicalOpen ( ofxCvGrayscaleImage &input, int nTimes){
	
	// Clean up the holes using morphological close. 
	// http://homepages.inf.ed.ac.uk/rbf/HIPR2/open.htm
	
	for (int i=0; i<nTimes; i++){
		input.erode();
	}
	for (int i=0; i<nTimes; i++){
		input.dilate();
	}	
}





/*
//--------------------------------------------------------------
void DepthHoleFiller::fillHolesPre (ofxCvGrayscaleImage ofxCv8uC1_DepthRawThreshed){
	
	// Copy the current depth into the previous depth buffer(s)
	cvCopy(ofxCv8uC1_DepthPrev3.getCvImage(),		ofxCv8uC1_DepthPrev4.getCvImage(), NULL);
	cvCopy(ofxCv8uC1_DepthPrev2.getCvImage(),		ofxCv8uC1_DepthPrev3.getCvImage(), NULL);
	cvCopy(ofxCv8uC1_DepthPrev1.getCvImage(),		ofxCv8uC1_DepthPrev2.getCvImage(), NULL);
	cvCopy(ofxCv8uC1_DepthRawThreshed.getCvImage(), ofxCv8uC1_DepthPrev1.getCvImage(), NULL);
}
 */

/*
 
 void DepthHoleFiller::setMorphologicalCloseCount (int n){
 n = MAX(0, n);
 nCloseOperations = n;
 }
 
//--------------------------------------------------------------
void DepthHoleFiller::performMorphologicalOperations(){
	
	
	// Clean up the holes using morphological close. 
	// http://homepages.inf.ed.ac.uk/rbf/HIPR2/close.htm
	for (int i=0; i<nCloseOperations; i++){
		ofxCv8uC1_DepthRawThreshed.dilate();
		ofxCv8uC1_DepthRawThreshed.erode();
	}
	
	// Dilate the thresholded depth image to clean up small spatial holes.
	// int nDepthDilate = gui.getValueI("N_DILATIONS", 0); 
	for (int i=0; i<nDepthDilate; i++){
		ofxCv8uC1_DepthRawThreshed.dilate();
	}
	
	
}
 */

/*
//--------------------------------------------------------------
void DepthHoleFiller::fillHolesPost (ofxCvGrayscaleImage ofxCv8uC1_DepthRawThreshed) {
	
	
	performMorphologicalOperations();
	
	// Fill temporal holes by taking the max of the past N depth frames
	// int nDepthHistory = gui.getValueI("N_HISTORY", 0); 
	// bool bUseMedian = gui.getValueB("DO_DEPTH_MEDIAN", 0);	
	if ((nDepthHistory == 0) && (bUseMedian == false)) {
		ofxCv8uC1_Depth = ofxCv8uC1_DepthRawThreshed;
		
	} else {
		
		
		if (bUseMedian){
			
			int medianSize = 5; // 3 or 5
			switch (medianSize) {
					
				default:
				case 3:
					// Do median of 3
					//nDepthHistory = 3;
					//gui.setValueI("N_HISTORY", nDepthHistory, 0);
					
					// Max of (A,B) in Temp1
					cvMax(ofxCv8uC1_DepthRawThreshed.getCvImage(), 
						  ofxCv8uC1_DepthPrev1.getCvImage(), 
						  ofxCv8uC1_Temp1.getCvImage());
					
					// Max of (A,C) in Temp2
					cvMax(ofxCv8uC1_DepthRawThreshed.getCvImage(), 
						  ofxCv8uC1_DepthPrev2.getCvImage(), 
						  ofxCv8uC1_Temp2.getCvImage());
					
					// Min of (Temp1, Temp2) in Depth
					cvMin(ofxCv8uC1_Temp1.getCvImage(), 
						  ofxCv8uC1_Temp2.getCvImage(), 
						  ofxCv8uC1_Depth.getCvImage());
					
					break;
					
					//---------------------------------------------
				case 5:
					// Do median of 5
					//nDepthHistory = 4;
					//gui.setValueI("N_HISTORY", nDepthHistory, 0);
					
					
					unsigned char* pix0		= ofxCv8uC1_DepthRawThreshed.getPixels();
					unsigned char* pix1		= ofxCv8uC1_DepthPrev1.getPixels();
					unsigned char* pix2 	= ofxCv8uC1_DepthPrev2.getPixels();
					unsigned char* pix3 	= ofxCv8uC1_DepthPrev3.getPixels();
					unsigned char* pix4 	= ofxCv8uC1_DepthPrev4.getPixels();
					unsigned char* currPix	= ofxCv8uC1_Depth.getPixels();
					
					int count = 0; 
					int numPixels = KW*KH;
					for (int i = 0; i < numPixels; i++){
						unsigned char  a0 = pix0[i];
						unsigned char  a1 = pix1[i];
						unsigned char  a2 = pix2[i];
						
						if ((a0>0) || (a1>0) || (a2>0)){
							
							unsigned char  a3 = pix3[i];
							unsigned char  a4 = pix4[i];
							
							if (a1 < a0) 	swap(a0, a1);
							if (a2 < a0)	swap(a0, a2); 
							if (a3 < a0)	swap(a0, a3);
							if (a4 < a0)	swap(a0, a4);
							if (a2 < a1)	swap(a1, a2);
							if (a3 < a1) 	swap(a1, a3);
							if (a4 < a1)	swap(a1, a4);
							if (a3 < a2)	swap(a2, a3);
							if (a4 < a2)	swap(a2, a4);
							currPix[i] = a2;
							
						} else {
							currPix[i] = 0;
						}
					}
					break;
			}
			
			
			
		} else {
			// The simplest design:
			// just use the Max of all depth history images instead.
			if (nDepthHistory > 0){
				cvMax(ofxCv8uC1_DepthRawThreshed.getCvImage(), 
					  ofxCv8uC1_DepthPrev1.getCvImage(), 
					  ofxCv8uC1_Depth.getCvImage());
			}
			if (nDepthHistory > 1){
				cvMax(ofxCv8uC1_Depth.getCvImage(), 
					  ofxCv8uC1_DepthPrev2.getCvImage(), 
					  ofxCv8uC1_Depth.getCvImage());
			}
			if (nDepthHistory > 2){
				cvMax(ofxCv8uC1_Depth.getCvImage(), 
					  ofxCv8uC1_DepthPrev3.getCvImage(), 
					  ofxCv8uC1_Depth.getCvImage());
			}
			if (nDepthHistory > 3){
				cvMax(ofxCv8uC1_Depth.getCvImage(), 
					  ofxCv8uC1_DepthPrev4.getCvImage(), 
					  ofxCv8uC1_Depth.getCvImage());
			}
		}
	}
	ofxCv8uC1_Depth.flagImageChanged();
}
*/
