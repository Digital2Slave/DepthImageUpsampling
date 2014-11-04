#include "head.h"
#include "function.h"
                        
Mat F, VarCF,  VarDF,  VarF;
int height = 0,    width = 0;
int s_height = 0,  s_width = 0;
int ScaleRate = 0, WinSize = 0;

int main(int argc,char * argv[])
{ 
	Mat RefImage  = imread(argv[1],0);
	Mat SrcImage  = imread(argv[2],0); 
	ScaleRate = atoi(argv[3]);

	if (argc != 6)
	{
		cout<<"Error!"<<endl;
		cout<<"Please input the file again!"<<endl;
		return -1;
	}

	height = SrcImage.rows;
	width  = SrcImage.cols;
	s_height = height/ScaleRate;
	s_width  = width /ScaleRate;

	Mat TmpCDown = Mat::zeros(s_height, s_width,RefImage.type()); 
	Mat TmpDDown = Mat::zeros(s_height, s_width,SrcImage.type()); 
	resize(RefImage,TmpCDown,TmpCDown.size(),0,0,INTER_NEAREST ); 
	resize(SrcImage,TmpDDown,TmpDDown.size(),0,0,INTER_NEAREST );  

	//------------------------- TJBU----------------------
	double start_time,end_time; 
	start_time = clock();

	Mat DepthEdge  = Mat::zeros(TmpDDown.size(),TmpDDown.type());
	F = DepthEdge.clone();

	VarCF       = Mat::zeros(s_height, s_width, CV_32FC1);
	VarDF       = Mat::zeros(s_height, s_width, CV_32FC1);
	VarF        = Mat::zeros(s_height, s_width, CV_32FC1);
	ExtractVariaceF(TmpCDown,VarCF);
	ExtractVariaceF(TmpDDown,VarDF);

#pragma region CA

	const int w = 3;
	const int bordval = 2;
	const int val = 1;
	int top,bottom,left,right;
	top = bottom = left = right = val;

	Mat TmpCDownBorder = Mat::zeros(s_height+bordval, s_width+bordval, TmpCDown.type());
	Mat TmpDDownBorder = Mat::zeros(s_height+bordval, s_width+bordval, TmpDDown.type());
	copyMakeBorder(TmpCDown, TmpCDownBorder, top, bottom, left, right, BORDER_REPLICATE);
	copyMakeBorder(TmpDDown, TmpDDownBorder, top, bottom, left, right, BORDER_REPLICATE);

	for (int j = 1; j < s_height+1; j++)
	{
		for (int i = 1; i < s_width+1; i++)
		{
			double a1 = 0.0, a2 = 0.0;
			double sumPix1 = 0.0,sumPix2 = 0.0, sumPix = 0.0;
			double meanPix1 = 0.0, meanPix2 = 0.0, meanPix= 0.0;
			for (int m = -w/2; m<=w/2; m++)
			{
				int y = j + m;
				y = (y > 0 ? (y < s_height + 2 ? y : s_height + 1 ) : 0);  

				for (int n = -w/2; n<=w/2; n++)
				{
					int x = i + n;
					x = (x > 0 ? (x < s_width + 2 ? x : s_width + 1 ) : 0); 
					a1 = TmpCDownBorder.at<uchar>(y,x);
					a2 = TmpDDownBorder.at<uchar>(y,x);
					sumPix1 += a1;
					sumPix2 += a2;
					sumPix += (a1 * a2);
				}//end for n
			}//end for m

			meanPix1 = sumPix1/(w*w);//EX
			meanPix2 = sumPix2/(w*w);//EY
			meanPix = sumPix/(w*w);  //EXY
			double CA = meanPix - meanPix1 * meanPix2;//EXY - EXEY
			double dx = VarCF.at<float>(j - 1,i - 1);
			double dy = VarDF.at<float>(j - 1,i - 1);   
			CA /= sqrt(dx * dy);                      //CA = ((EXY - EXEY)/sqrt(DXDY))
			VarF.at<float>(j-1, i-1) = CA;
		}//end for i
	}//end for j
#pragma endregion

	ExtractMaskF(TmpDDown, DepthEdge);

	threshold(DepthEdge, F, 0, 255, THRESH_OTSU);
   
	WinSize = ScaleRate * ScaleRate + 1;             //window's size for different scals downsampling
	Mat SrcUp = Mat::zeros(SrcImage.size(), SrcImage.type());

	TJBU(TmpDDown, RefImage, SrcUp, WinSize);

	end_time = (clock() - start_time) / CLOCKS_PER_SEC; 
	cout << "Imagescale #:" << ScaleRate << endl;
	cout << "The time of Tian method is : " << end_time << endl;

	imwrite(argv[4],SrcUp);

#pragma region Criterion BPR MSE RMSE PSNR

	Mat BadImage = Mat::zeros(SrcUp.size(),SrcUp.type());
	double BRP = 0.0,Mse = 0.0 , Rmse = 0.0, Psnr = 0.0;
	double blackcnt = 0.0,cnt = 0.0;
	int dv = 0;
	long sum = 0;

	for (int j = 0; j < height; j++)
	{
		for (int i = 0; i < width; i++)
		{
			if (SrcImage.at<uchar>(j,i) != 0)
			{
				//MSE RMSE PSNR
				dv = abs(SrcImage.at<uchar>(j,i) - SrcUp.at<uchar>(j,i));
				dv = pow(dv,2);
				sum += dv;
				//BPR
				if (dv > 1)
				{
					BadImage.at<uchar>(j,i) = 0;
					cnt++;
				}
				else
				{
					BadImage.at<uchar>(j,i) = 255;
				}
			} 
			else
			{
				BadImage.at<uchar>(j,i) = 255;
				blackcnt++;
			}
		}
	}

	BRP = 1.0 * cnt / (height * width - blackcnt)*100;
	Mse = 1.0 * sum/(height*width - blackcnt);
	Rmse = sqrt(Mse);
	Psnr = 10 * log10(255*255/(Mse));

	cout << "BPR  : " << BRP << "%" << endl;
	cout << "MSE  : " << Mse << endl;
	cout << "RMSE : " << Rmse << endl;
	cout << "PSNR : " << Psnr << endl;
	cout << endl << endl;
	imwrite(argv[5],BadImage);
#pragma endregion

	return EXIT_SUCCESS;
}

Mat& ExtractMaskF(Mat& inImage,Mat &TmpF)
{
	int height = inImage.rows;
	int width = inImage.cols;

	Mat grad_x, grad_y;  
	Mat abs_grad_x, abs_grad_y;  
	int scale0 = 1;       
	int delta0 = 0;       
	int ddepth = CV_16S;

	// Gradient X grad_x 
	Scharr( inImage, grad_x, ddepth, 1, 0, scale0, delta0, BORDER_DEFAULT );  
	convertScaleAbs( grad_x, abs_grad_x );  

	// Gradient Y grad_y
	Scharr( inImage, grad_y, ddepth, 0, 1, scale0, delta0, BORDER_DEFAULT );  
	convertScaleAbs( grad_y, abs_grad_y );  

	addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, TmpF);

	return TmpF;
}

Mat& ExtractVariaceF(Mat &inImage,Mat &VarF)
{
	//---Lee Mask 3*3 Varance image---
	const int w = 3;
	const int bordval = 2;
	const int val = 1;

	Mat TmpDownBorder = Mat::zeros(s_height+bordval, s_width+bordval, inImage.type());

	int top,bottom,left,right;
	top = bottom = left = right = val;

	copyMakeBorder(inImage, TmpDownBorder, top, bottom, left, right, BORDER_REPLICATE);

	for (int j = 1; j < s_height+1; j++)
	{
		for (int i = 1; i < s_width+1; i++)
		{
			double sumPix = 0.0,sumPix2 = 0.0;
			double meanPix = 0.0, meanPix2 = 0.0, variancePix = 0.0;
			for (int m = -w/2; m<=w/2; m++)
			{
				int y = j + m;
				y = (y > 0 ? (y < s_height + 2 ? y : s_height + 1 ) : 0);  // y coordinate

				for (int n = -w/2; n<=w/2; n++)
				{
					int x = i + n;
					x = (x > 0 ? (x < s_width + 2 ? x : s_width + 1 ) : 0); // x coordinate
					uchar a = TmpDownBorder.at<uchar>(y,x);
					sumPix += a;
					sumPix2 += (a*a);
				}//end for n
			}//end for m

			meanPix = sumPix/(w*w);
			meanPix2 = sumPix2/(w*w);
			variancePix = meanPix2 - meanPix*meanPix;  
			VarF.at<float>(j-1, i-1) = variancePix;
		}//end for i
	}//end for j
	return VarF;
}