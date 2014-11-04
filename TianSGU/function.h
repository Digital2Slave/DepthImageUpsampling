#ifndef _FUNCTION_H_
#define _FUNCTION_H_

#include "head.h"

// core function
Mat& TJBU(Mat& source, Mat &refIm, Mat &dest, int WinWidth);

// Mask F
Mat& ExtractMaskF   (Mat &inImage,Mat &TmpF);

// Variance
Mat& ExtractVariaceF(Mat &inImage,Mat &VarF);

#endif