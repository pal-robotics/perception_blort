/**
 * @file cvgeometry.h
 * @author Markus Bader
 * @date Mo Jul 9 2009
 * @version 0.1
 * @brief
 *
 * @see
 **/

#ifndef CV_GEOMETRY_H
#define CV_GEOMETRY_H

#include <stdio.h>
//#include <opencv/cv.h>
#include <opencv2/core/core.hpp>


/**
* @brief Distorts a point 
* @param pIntrinsicDistort	Intrinsic Matrinx of the distort image (camera image) in [3x3] matrix
* @param pDistortion	DistortionCoeff pointer to an array [k1, k2, p1, p2]
* @param pSrc	source point [x, y]
* @param pDes	destination point [x, y]
**/
inline void cvDistort (const double *pIntrinsicDistort, const double *pDistortion, const double *pSrc, double *pDes ) {
    //std::cout << "k1: " << pDistortion[0] << ", k2: " << pDistortion[1] << ",p1: " << pDistortion[2] << ", p2: " << pDistortion[3] << std::endl;
    double dx = ( pSrc[0] - pIntrinsicDistort[2] ) / pIntrinsicDistort[0];
    // relative y position in distorted image to the center
    double dy = ( pSrc[1] - pIntrinsicDistort[5] ) / pIntrinsicDistort[4];
    // relative distance of the distorted point to the center
    double dxdy = dx*dy;
    double dx2 = dx*dx;
    double dy2 = dy*dy;
    double r = sqrt ( dx2 + dy2 );
    double r2 = r*r;
    double r4 = r2*r2;

    //x = dx*(1 + k1r^2 + k2r^4) + 2*p1*dx*dy + p2(r^2+2*dx^2)
    double Tx1 = dx * ( 1.0 + pDistortion[0] * r2 + pDistortion[1] * r4 );
    double Tx2 = 2 * pDistortion[2] * dxdy;
    double Tx3 = pDistortion[3] * ( r2 + 2 * dx2 );

    //y = dy*(1 + k1r^2 + k2r^4) + p1(r^2+2*dy^2) + 2*p2*dx*dy
    double Ty1 = dy * ( 1.0 + pDistortion[0] * r2 + pDistortion[1] * r4 );
    double Ty2 = pDistortion[2] * ( r2 + 2 * dy2 );
    double Ty3 = 2 * pDistortion[3] * dxdy;

    pDes[0] = ( Tx1 + Tx2 + Tx3 ) * pIntrinsicDistort[0] + pIntrinsicDistort[2];
    pDes[1] = ( Ty1 + Ty2 + Ty3 ) * pIntrinsicDistort[4] + pIntrinsicDistort[5];

}


/**
* @brief Distorts a point
* @param pIntrinsicDistort	Intrinsic Matrinx of the distort image (camera image) in [3x3] matrix
* @param pDistortion	DistortionCoeff pointer to an array [k1, k2, p1, p2]
* @param pSrc	source point [x, y]
* @param pDes	destination point [x, y]
**/
inline void cvDistort (const CvMat *pIntrinsicDistort, const CvMat *pDistortion, const CvPoint2D64f *pSrc, CvPoint2D64f *pDes ) {
    cvDistort (pIntrinsicDistort->data.db, pDistortion->data.db, (const double *) pSrc, (double *) pDes );
}

/**
* @brief Distorts a point
* @param pIntrinsicDistort	Intrinsic Matrinx of the distort image (camera image) in [3x3] matrix
* @param pDistortion	DistortionCoeff pointer to an array [k1, k2, p1, p2]
* @param pSrc	source point [x, y]
* @param pDes	destination point [x, y]
**/
inline void cvDistort (const CvMat *pIntrinsicDistort, const CvMat *pDistortion, const CvPoint *pSrc, CvPoint *pDes ) {
    double src[] = {static_cast<double>(pSrc->x), static_cast<double>(pSrc->y)}, des[2];
    cvDistort (pIntrinsicDistort->data.db, pDistortion->data.db, (const double *) &src, (double *) &des );
    pDes->x = cvRound(des[0]), pDes->y = cvRound(des[1]);
}

/**
* @brief  Undistort a image by using the equations
* <br/> x = dx*(1 + k1r^2 + k2r^4) + 2*p1*dx*dy + p2(r^2+2*dx^2)
* <br/> y = dy*(1 + k1r^2 + k2r^4) + p1(r^2+2*dy^2) + 2*p2*dx*dy
* @param pIntrinsicDistort	Intrinsic Matrinx of the distort image (camera image) in [3x3] matrix
* @param pDistortion	DistortionCoeff pointer to an array [k1, k2, p1, p2]
* @param pMapX	destination X map (it must be of the type 32fC1 -> float)
* @param pMapY	destination Y map (it must be of the type 32fC1 -> float)
**/
inline void cvInitUndistortMapExact (const CvMat *pIntrinsicDistort, const CvMat *pDistortion, CvMat *pMapX,  CvMat *pMapY) {
    CvPoint2D64f undistort, distort;
		if((cvGetElemType(pMapX) == CV_32FC1) && (cvGetElemType(pMapY) == CV_32FC1)){
    	float *pX = pMapX->data.fl, *pY = pMapY->data.fl;
			for ( undistort.y = 0; undistort.y < pMapX->height; undistort.y++) {
					for ( undistort.x = 0; undistort.x < pMapX->width; undistort.x++) {
							cvDistort(pIntrinsicDistort, pDistortion, &undistort, &distort);
							*pX++ = (float) distort.x, *pY++ = (float) distort.y;
					}
			}
		} else if((cvGetElemType(pMapX) == CV_64FC1) && (cvGetElemType(pMapY) == CV_64FC1)){
    	double *pX = pMapX->data.db, *pY = pMapY->data.db;
			for ( undistort.y = 0; undistort.y < pMapX->height; undistort.y++) {
					for ( undistort.x = 0; undistort.x < pMapX->width; undistort.x++) {
							cvDistort(pIntrinsicDistort, pDistortion, &undistort, &distort);
							*pX++ = distort.x, *pY++ = distort.y;
					}
			}
		}
}

/**
* @brief  Undistort a image by using the equations
* <br/> x = dx*(1 + k1r^2 + k2r^4) + 2*p1*dx*dy + p2(r^2+2*dx^2)
* <br/> y = dy*(1 + k1r^2 + k2r^4) + p1(r^2+2*dy^2) + 2*p2*dx*dy
* @param pIntrinsicDistort	Intrinsic Matrinx of the distort image (camera image) in [3x3] matrix
* @param pDistortion	DistortionCoeff pointer to an array [k1, k2, p1, p2]
* @param pSrc	source point [x, y]
* @param pDes	destination point [x, y]
**/
inline void cvUndistortExact (const CvMat *pIntrinsicDistort, const CvMat *pDistortion, const IplImage *pSrc,  IplImage *pDes) {

    CvMat *pMapX  = cvCreateMat ( pSrc->height,  pSrc->width, CV_32FC1 );
    CvMat *pMapY  = cvCreateMat ( pSrc->height,  pSrc->width, CV_32FC1 );
		cvInitUndistortMapExact (pIntrinsicDistort, pDistortion, pMapX,  pMapY);
    cvRemap ( pSrc, pDes, pMapX, pMapY );
    cvReleaseMat ( &pMapX ), cvReleaseMat ( &pMapY );
}

/**
* @brief Undistorts a point
* @param pIntrinsicDistort	Intrinsic Matrinx of the distort image (camera image) in [3x3] matrix
* @param pDistortion	DistortionCoeff pointer to an array [k1, k2, p1, p2]
* @param pSrc	source point [x, y]
* @param pDes	destination point [x, y]
**/
inline bool cvUndistort (const double *pIntrinsicDistort, const double *pDistortion, const double *pSrc, double *pDes, double maxError = 1) {
    const int iMaxInterations = 100;
    double fError = (pIntrinsicDistort[2]+pIntrinsicDistort[5])*4;
    double cradient[] = {0, 0};
    pDes[0] = pSrc[0], pDes[1] = pSrc[1];
    double current[2];
    int i;
    for ( i = 0; ( ( i < iMaxInterations ) && ( fError > maxError ) ); i++ ) {
        pDes[0] += cradient[0];
        pDes[1] += cradient[1];
        cvDistort (pIntrinsicDistort, pDistortion, pDes, (double *) &current );
        cradient[0] = pSrc[0] - current[0];
        cradient[1] = pSrc[1] - current[1];
        fError = cradient[0] * cradient[0] + cradient[1] * cradient[1];
    }
    if ( i < iMaxInterations ) return true;
    else return false;
}

/**
* @brief Undistorts a point
* @param pIntrinsicDistort	Intrinsic Matrinx of the distort image (camera image) in [3x3] matrix
* @param pDistortion	DistortionCoeff pointer to an array [k1, k2, p1, p2]
* @param pSrc	source point [x, y]
* @param pDes	destination point [x, y]
**/
inline bool cvUndistort (const CvMat *pIntrinsicDistort, const CvMat *pDistortion, const CvPoint2D64f *pSrc, CvPoint2D64f *pDes, double maxError = 1) {
    return cvUndistort (pIntrinsicDistort->data.db, pDistortion->data.db, (const double *) pSrc, (double *) pDes, maxError );
}

/**
* @brief Undistorts a point
* @param pIntrinsicDistort	Intrinsic Matrinx of the distort image (camera image) in [3x3] matrix
* @param pDistortion	DistortionCoeff pointer to an array [k1, k2, p1, p2]
* @param pSrc	source point [x, y]
* @param pDes	destination point [x, y]
**/
inline bool Undistort (const CvMat *pIntrinsicDistort, const CvMat *pDistortion, const CvPoint *pSrc, CvPoint *pDes, double maxError = 1) {
    double src[] = {static_cast<double>(pSrc->x), static_cast<double>(pSrc->y)}, des[2];
    bool result = cvUndistort (pIntrinsicDistort->data.db, pDistortion->data.db, (const double *) src, (double *) &des, maxError );
    pDes->x = round(des[0]);
    pDes->y = round(des[1]);
    return result;
}
/**
* @brief projects a 3D point into the image plane
* @param pMintMext  = Mint*Mext [4x4]
* @param pSrc	source point 3D [x, y, z]
* @param pDes	destination point 2D [x, y]
**/
inline void cvProjectPoint(const double *pMintMext, const double *pSrc, double *pDes) {
    pDes[0] = pMintMext[0] * pSrc[0] + pMintMext[1] * pSrc[1] + pMintMext[2] * pSrc[2] + pMintMext[3];
    pDes[1] = pMintMext[4] * pSrc[0] + pMintMext[5] * pSrc[1] + pMintMext[6] * pSrc[2] + pMintMext[7];
    double n = pMintMext[8] * pSrc[0] + pMintMext[9] * pSrc[1] + pMintMext[10] * pSrc[2] + pMintMext[11];
    pDes[0] /= n;
    pDes[1] /= n;
}

/**
* @brief projects a 3D point into the image plane
* @param pMintMext  = Mint*Mext [4x4]
* @param pSrc	source point 3D [x, y, z]
* @param pDes	destination point 2D [x, y]
**/
inline void cvProjectPoint(const CvMat *pMintMext, const CvPoint2D64f *pSrc, CvPoint2D64f *pDes) {
    cvProjectPoint(pMintMext->data.db, (const double *) pSrc, (double *) pDes);
}

/**
* @brief projects a 3D point into the image plane
* @param pMintMext  = Mint*Mext [4x4]
* @param pSrc	source point 3D [x, y, z]
* @param pDes	destination point 2D [x, y]
**/
template <typename A, typename B> inline void cvProjectPoint(const CvMat *pMintMext, const A *pSrc, B *pDes) {
    double src[] = {pSrc->x, pSrc->y, pSrc->z}, des[2];
    cvProjectPoint(pMintMext->data.db, (const double *) src, (double *) des);
    pDes->x = cvRound(des[0]), pDes->y = cvRound(des[1]);
}

#endif //CV_GEOMETIRY_H
