#include "head.h"
#include "function.h"

extern Mat F;          // the refine mask of downsampling ground truth image
extern Mat VarCF;      
extern Mat VarDF;
extern Mat VarF;
extern int height,width;
extern int s_height,s_width;
extern int ScaleRate;

inline double dist(double x1, double y1, int x2, int y2)
{
	return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}
inline double StandardGauss( double x, double sigma)
{
	double exponent = exp(-1.0 * (x * x ) / (2 * sigma * sigma));   
	return exponent;
}

Mat& TJBU(Mat& source, Mat &refIm, Mat &dest, int WinWidth)
{
	double scale     = 1.0*s_width/width;
	const double  mu = 0.0;
	const double sigmad = 0.50;
	const double sigmar = 25.5;
	const int num_neighbors = WinWidth/2;   

	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			// coordinate maybe fractional
			double o_y = y * scale;            
			double o_x = x * scale;
			int r_y = 0, r_ys = 0, r_x = 0, r_xs = 0;
		
			double varCFval = VarCF.at<float>(o_y,o_x);                  // Low resolution color image's variance VarCF 
			double varDFval = VarDF.at<float>(o_y,o_x);                  // Low resolution depth image's variance VarDF 
			double afa = VarF.at<float>(o_y,o_x);                        // Correlation Value CA

			uchar FPix = F.at<uchar>(o_y, o_x);                          // Mask F
			uchar srcPix  = source.at<uchar>(o_y,o_x);                   // source Low resolution depth image Dl 
			uchar refPix  = refIm.at<uchar>(y, x);                       // refIm  High resolution color imag I
			
			//Joint bilateral upsampling
			uchar nsrcPix = 0, nrefPix = 0;
			double dis = 0.0, dgauss = 0.0, sgauss = 0.0, rgauss = 0.0, totalgauss = 0.0;
			double normalizing_factor = 0.0, total_val = 0.0;
			
			//for each neighbour for the source 
			for (int j = -num_neighbors; j <= num_neighbors; j++)
			{
				//find y coordinates in source
				r_y = o_y + j;
				r_y = (r_y > 0 ? (r_y < s_height ? r_y :s_height - 1) : 0) ;
				r_ys = r_y / scale;  

				for (int i = -num_neighbors; i <= num_neighbors; i++)
				{
					//find x coordinates in source
					r_x = o_x + i;
					r_x = (r_x > 0 ? (r_x < s_width ? r_x : s_width- 1) : 0);
					r_xs = r_x / scale;  
				
					nsrcPix = source.at<uchar>(r_y, r_x);            // Low resolution depth image Dl
					nrefPix = refIm.at<uchar>(r_ys, r_xs);           // High resolution color image I

					if (nsrcPix != 0)
					{	
						if (FPix == 255)
						{
							dis = dist(o_x, o_y, r_x, r_y);
							sgauss = StandardGauss(dis, sigmad);                        // spatial filter 
							rgauss = StandardGauss(abs(refPix - nrefPix), varCFval);    // color filter   
							dgauss = StandardGauss(abs(srcPix - nsrcPix), varDFval);    // depth filter   	
							totalgauss = sgauss * (afa*rgauss + (1 - afa)*dgauss);
							total_val += nsrcPix * totalgauss;
							normalizing_factor += totalgauss;
						} 
						else
						{//Kopf's JBU							
							dis = dist(o_x, o_y, r_x, r_y);
							sgauss = StandardGauss(dis, sigmad);                  // spatial filter 
							rgauss = StandardGauss(abs(refPix - nrefPix), sigmar);// range filter
							totalgauss = sgauss * rgauss;
							total_val += nsrcPix * totalgauss;
							normalizing_factor += totalgauss;
						}
					}
				
				}//end for i
			}//end for j

			dest.at<uchar>(y, x) = ceil(total_val / normalizing_factor);
		
#pragma region Plan_1
			if (varDFval >= 0.01)  
			{
				//===============refine depth value============
				const int w = 2;
				uchar centPix = dest.at<uchar>(y, x);
				uchar wpix[25] = {0};
				int wdiffpix[25] = {0};
				int indexpix = 0;

				for (int j = -w; j <= w; j++)
				{
					int r_y = o_y + j;
					r_y = (r_y > 0 ? (r_y < s_height ? r_y :s_height - 1) : 0) ;

					for (int i = -w; i <= w; i++)
					{
						int r_x = o_x + i;
						r_x = (r_x > 0 ? (r_x < s_width ? r_x : s_width- 1) : 0);
						uchar srcPix = source.at<uchar>(r_y, r_x);  

						if (srcPix != 0)
						{
							wpix[indexpix] = srcPix;
							wdiffpix[indexpix] = abs(srcPix - centPix);
							indexpix++;
						}
					}
				}

				int pixid = 0;
				int minpix = wdiffpix[0];
				for (int i=0; i<indexpix; i++) 
				{
					if (minpix > wdiffpix[i] )
					{
						minpix = wdiffpix[i];
						pixid = i;
					}
				}
				dest.at<uchar>(y, x) = wpix[pixid];
			}
			//=============high resolution color image's edge=============
#pragma endregion

		}//end for x
	}//end for y

	return dest;
}