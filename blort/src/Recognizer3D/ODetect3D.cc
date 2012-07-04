/**
 * $Id: ODetect3D.cc 34111 2012-07-03 14:29:54Z student5 $
 *
 * Johann Prankl, 2010-01-27 
 * prankl@acin.tuwien.ac.at
 *
 * TODO:
 * - now we assume undistorted images 
 * -> introduce distortion paramter
 */


#include <blort/Recognizer3D/ODetect3D.hh>
#include <ros/console.h>
#include <blort/TomGine/tgPose.h>
#include <blort/blort/pal_util.h>
#include <blort/Recognizer3D/Recognizer3D.h>

namespace P 
{


    /********************** ODetect3D ************************
     * Constructor/Destructor
     */
    ODetect3D::ODetect3D()
        : dbg(0)
    {
        nn_far_enough_threshold = (Def::DO_MIN_DESC_DIST*Def::DO_MIN_DESC_DIST)/100.0;
        n_points_to_match = 4;

        cameraMatrix = cvCreateMat( 3, 3, CV_32F );
        distCoeffs = cvCreateMat( 4, 1, CV_32F );

        matcherSize = 4096;
        matcher = new SiftMatchGPU(4096);
        matcher->VerifyContextGL();

        //we assume undistort images!!!!
        cvmSet( distCoeffs, 0, 0, 0. );
        cvmSet( distCoeffs, 1, 0, 0. );
        cvmSet( distCoeffs, 2, 0, 0. );
        cvmSet( distCoeffs, 3, 0, 0. );
    }

    ODetect3D::~ODetect3D()
    {
        delete matcher;
        if (cameraMatrix!=0) cvReleaseMat(&cameraMatrix);
        if (distCoeffs!=0) cvReleaseMat(&distCoeffs);
        DeletePairs(matches);
    }

    /**
     * DeletePairs
     */
    void ODetect3D::DeletePairs(Array<KeyClusterPair*> &matches)
    {
        for (unsigned i=0; i<matches.Size(); i++)
            delete matches[i];
        matches.Clear();
    }


    /**
     * Match keypoints with codebook
     * check second nearest neighbour
     */
    void ODetect3D::MatchKeypoints2(Array<KeypointDescriptor *> &keys, Array<CodebookEntry *> &cb, Array<KeyClusterPair* > &matches)
    {
        double ticksBefore = cv::getTickCount();
        float dist, dist1, dist2;
        unsigned idx = 0;
        KeypointDescriptor *key;

        for (unsigned i=0; i<keys.Size(); i++)
        {
            if (keys[i]->id != 0)
            {
                dist1 = dist2 = FLT_MAX;
                key = (KeypointDescriptor*)keys[i];
                for (unsigned j=0; j<cb.Size(); j++)
                {
                    if(key->GetType() == cb[j]->model->GetType())
                    {
                        dist = key->DistSqr(key->GetVec(), cb[j]->model->GetVec(), key->GetSize());
                        if (dist < dist1)
                        {
                            dist2=dist1;
                            dist1=dist;
                            idx=j;
                        }
                        else
                        {
                            if (dist<dist2) dist2=dist;
                        }
                    }
                }
                if(double(dist1)/dist2 < nn_far_enough_threshold)
                    matches.PushBack(new KeyClusterPair(key, cb[idx]));
            }
        }
        ROS_INFO("odetect3d::matchkeypoints() run: %f ms",
                 1000*(cv::getTickCount() - ticksBefore)/cv::getTickFrequency());
    }

    /**
     * Match keypoints with codebook
     * (use threshold)
     */
    void ODetect3D::MatchKeypoints(Array<KeypointDescriptor *> &keys, Array<CodebookEntry *> &cb, Array<KeyClusterPair* > &matches)
    {
        float dist, dist1;
        unsigned idx = 0;
        KeypointDescriptor *key;

        for (unsigned i=0; i<keys.Size(); i++)
        {
            if (keys[i]->id != 0)                //upppps HACK!!!!
            {
                dist1=FLT_MAX;
                key = (KeypointDescriptor*)keys[i];
                for (unsigned j=0; j<cb.Size(); j++)
                {
                    dist = key->Match(cb[j]->model);
                    if (dist < dist1)
                    {
                        dist1=dist;
                        idx=j;
                    }
                }
                if (!IsEqual(dist1,FLT_MAX))
                    matches.PushBack(new KeyClusterPair(key, cb[idx], dist1));
            }
        }
    }

    /**
     * Match keypoints (GPU version)
     */
    void ODetect3D::MatchKeypointsGPU(Array<KeypointDescriptor *> &keys, Array<CodebookEntry *> &cb, Array<KeyClusterPair* > &matches)
    {
        if (keys.Size()<4)
            return;

        int num;
        int (*match_buf)[2] = new int[(int)keys.Size()][2];

        if (matcherSize < (int)keys.Size()) matcher->SetMaxSift((int)keys.Size());
        if (matcherSize < (int)cb.Size()) matcher->SetMaxSift((int)cb.Size());

        P::Array<float> desc1(cb.Size()*128);
        P::Array<float> desc2(keys.Size()*128);

        for (unsigned i=0; i<cb.Size(); i++)
            CopyVec(cb[i]->model->vec, &(desc1[i*128]), cb[i]->model->GetSize());
        for (unsigned i=0; i<keys.Size(); i++)
            CopyVec(keys[i]->vec, &(desc2[i*128]), keys[i]->GetSize());

        matcher->SetDescriptors(0, (int)cb.Size(), &desc1[0]); //codebook
        matcher->SetDescriptors(1, (int)keys.Size(), &desc2[0]); //keys

        num = matcher->GetSiftMatch(keys.Size(), match_buf);

        for (unsigned i=0; i<(unsigned)num; i++)
            matches.PushBack(new KeyClusterPair(keys[match_buf[i][1]], cb[match_buf[i][0]], 0));

        delete[] match_buf;
    }


    /**
     * GetRandIdx
     */
    void ODetect3D::GetRandIdx(unsigned size, unsigned num, P::Array<unsigned> &idx)
    {
        unsigned temp;
        idx.Clear();
        for (unsigned i=0; i<num; i++)
        {
            do{
                temp = rand()%size;
            }while(idx.Contains(temp));
            idx.PushBack(temp);
        }
    }

    /**
     * Count inlier
     */
    void ODetect3D::GetInlier(Array<KeyClusterPair*> &matches, PoseCv &pose, int &inl)
    {
        inl=0;
        Vector2 p;
        CodebookEntry *cbe;
        KeypointDescriptor *k;
        CvMat *pos = cvCreateMat( 3, 1, CV_32F );

        bool stop=false;
        for (unsigned i=0; i<matches.Size() && !stop; i++)
        {
            cbe = matches[i]->c;
            k = matches[i]->k;

            for (unsigned j=0; j<cbe->Size(); j++)
            {
                cvMatMulAdd( pose.R, cbe->occurrences[j]->pos, pose.t, pos );
                ProjectPoint2Image(pos->data.fl[0], pos->data.fl[1], pos->data.fl[2], cameraMatrix, p.x, p.y);

                if (Distance(p,k->p) < Def::DO_RANSAC_INL_DIST)
                {
                    if (pos->data.fl[2]<0)
                    {
                        inl=0;
                        stop=true;
                    }
                    else
                    {
                        inl++;
                    }
                    break;
                }
            }
        }

        cvReleaseMat(&pos);
    }


    /**
     * Verify object model
     */
    void ODetect3D::FitModelRANSAC(Array<KeyClusterPair*> &matches,
                                   PoseCv &pose, unsigned &numInl)
    {
        if (matches.Size()<n_points_to_match)
        {
            ROS_WARN("matches.Size(): %d while n_points_to_match is %d",
                     matches.Size(), n_points_to_match);
            return;
        }

        double eps = n_points_to_match/(double)matches.Size();
        int inl, inls = 0;
        Array<unsigned> idx, idx2;
        CvMat *modelPoints = cvCreateMat( n_points_to_match, 3, CV_32F );
        CvMat *imgPoints = cvCreateMat( n_points_to_match, 2, CV_32F );
        srand(time(NULL));
        KeyClusterPair* kp;
        PoseCv tempPose;
        CvMat *rod = cvCreateMat(3,1,CV_32F);
        //double ticksBefore = cv::getTickCount();
        int k;
        for(k=0; (pow(1. - pow(eps,n_points_to_match),k) >= Def::DO_RANSAC_ETA0 &&
                  k<Def::DO_MAX_RANSAC_TRIALS); ++k)
        {
            GetRandIdx(matches.Size(), n_points_to_match, idx);

            for (unsigned i=0; i<n_points_to_match; i++)
            {
                kp = matches[idx[i]];
                CvMat *pos = kp->c->occurrences[rand()%kp->c->Size()]->pos;
                cvmSet( modelPoints, i, 0, pos->data.fl[0] );
                cvmSet( modelPoints, i, 1, pos->data.fl[1] );
                cvmSet( modelPoints, i, 2, pos->data.fl[2] );
                cvmSet( imgPoints, i, 0, kp->k->p.x);
                cvmSet( imgPoints, i, 1, kp->k->p.y);
            }
            //DEBUG
            //            ROS_INFO("%f", pow(1. - pow(eps,n_points_to_match),k));
            //            ROS_INFO("fitmodelransac run: %f ms",
            //                     1000*(cv::getTickCount() - ticksBefore)/cv::getTickFrequency());

            cvFindExtrinsicCameraParams2(modelPoints, imgPoints,
                                         cameraMatrix, distCoeffs, rod, tempPose.t);
            cvRodrigues2(rod,tempPose.R);
            GetInlier(matches,tempPose,inl);

            if (inl > inls
                //                && cvmGet(tempPose.R, 0, 2) < 0.4 && cvmGet(tempPose.R, 0, 2) > -0.4 &&
                //                cvmGet(tempPose.R, 1, 2) < -0.5 && cvmGet(tempPose.R, 1, 2) > -0.99 &&
                //                cvmGet(tempPose.R, 2, 2) < -0.2 && cvmGet(tempPose.R, 2, 2) > -0.8
                )
            {
                //DEBUG
                //                mat3 cR;
                //                vec3 ct;
                //                cR[0] = cvmGet(tempPose.R,0,0); cR[3] = cvmGet(tempPose.R,0,1); cR[6] = cvmGet(tempPose.R,0,2);
                //                cR[1] = cvmGet(tempPose.R,1,0); cR[4] = cvmGet(tempPose.R,1,1); cR[7] = cvmGet(tempPose.R,1,2);
                //                cR[2] = cvmGet(tempPose.R,2,0); cR[5] = cvmGet(tempPose.R,2,1); cR[8] = cvmGet(tempPose.R,2,2);
                //
                //                ct.x = cvmGet(tempPose.t,0,0);
                //                ct.y = cvmGet(tempPose.t,1,0);
                //                ct.z = cvmGet(tempPose.t,2,0);
                //
                //                ROS_INFO("ct and cR in inl > inls: [%f %f %f] \n [%f %f %f] \n [%f %f %f] \n [%f %f %f]",
                //                         ct.x, ct.y, ct.z, cR[0], cR[3], cR[6], cR[1], cR[4], cR[7], cR[2], cR[5], cR[8]);

                inls = inl;
                eps = (double)inls / (double)matches.Size();
                CopyPoseCv(tempPose, pose);
            }
        }
        //DEBUG
        //        ROS_INFO("fitmodelransac run: %f ms, iterations: %d, eps: %f, escape cond: %f",
        //                 1000*(cv::getTickCount() - ticksBefore)/cv::getTickFrequency(),
        //                 k, eps, pow(1. - pow(eps,n_points_to_match),k));
        //        ROS_INFO("inliers: %d, total #points: %d", inls, matches.Size());


        numInl=inls;

        cvReleaseMat(&modelPoints);
        cvReleaseMat(&imgPoints);
        cvReleaseMat(&rod);
    }

    /**
     * Get best supporting corner of a matching codebook entry
     */
    bool ODetect3D::GetBestCorner(PoseCv &pose, KeypointDescriptor *k, CodebookEntry *cbe, Point3D &pos, double &minDist)
    {
        Vector2 p;

        double dist;
        minDist = DBL_MAX;
        CvMat *tempPos = cvCreateMat( 3, 1, CV_32F );


        for (unsigned j=0; j<cbe->Size(); j++)
        {
            cvMatMulAdd( pose.R, cbe->occurrences[j]->pos, pose.t, tempPos );
            ProjectPoint2Image(tempPos->data.fl[0], tempPos->data.fl[1], tempPos->data.fl[2], cameraMatrix, p.x, p.y);

            dist = Distance(p,k->p);
            if (dist < minDist)
            {
                minDist = dist;
                pos.x = cbe->occurrences[j]->pos->data.fl[0];
                pos.y = cbe->occurrences[j]->pos->data.fl[1];
                pos.z = cbe->occurrences[j]->pos->data.fl[2];
            }
        }

        cvReleaseMat(&tempPos);

        if (minDist < Def::DO_RANSAC_INL_DIST)
            return true;

        return false;
    }

    /**
     * Compute least squares pose
     */
    void ODetect3D::RefinePoseLS(Array<KeyClusterPair*> &matches, PoseCv &pose, unsigned &inl, double &err)
    {
        mat3 cR;
        vec3 ct;
        cR[0] = cvmGet(pose.R,0,0); cR[3] = cvmGet(pose.R,0,1); cR[6] = cvmGet(pose.R,0,2);
        cR[1] = cvmGet(pose.R,1,0); cR[4] = cvmGet(pose.R,1,1); cR[7] = cvmGet(pose.R,1,2);
        cR[2] = cvmGet(pose.R,2,0); cR[5] = cvmGet(pose.R,2,1); cR[8] = cvmGet(pose.R,2,2);

        ct.x = cvmGet(pose.t,0,0);
        ct.y = cvmGet(pose.t,1,0);
        ct.z = cvmGet(pose.t,2,0);

        ROS_INFO("ct and cR in RefinePoseLS: [%f %f %f] \n [%f %f %f] \n [%f %f %f] \n [%f %f %f]",
                 ct.x, ct.y, ct.z, cR[0], cR[3], cR[6], cR[1], cR[4], cR[7], cR[2], cR[5], cR[8]);


        double dist, minz=DBL_MAX, maxz=DBL_MIN;
        Array<Point3D> modelPoints;
        Array<Point2D> imgPoints;
        CvMat modelPointsCv;
        CvMat imgPointsCv;
        CvMat *rod;
        err=0;
        PoseCv tmpPose;
        Vector3 t, p;

        inlier.Clear();

        modelPoints.PushBack(Point3D());
        imgPoints.PushBack(Point2D());
        for (unsigned i=0; i<matches.Size(); i++)
        {
            if (GetBestCorner(pose, matches[i]->k, matches[i]->c, modelPoints.Last(), dist))
            {
                err+=dist;
                imgPoints.Last().x = (float)matches[i]->k->p.x;
                imgPoints.Last().y = (float)matches[i]->k->p.y;

                if (modelPoints.Last().z>maxz)
                    maxz=modelPoints.Last().z;
                else if (modelPoints.Last().z<minz)
                    minz=modelPoints.Last().z;

                modelPoints.PushBack(Point3D());
                imgPoints.PushBack(Point2D());
                inlier.PushBack(matches[i]->k);      //just store pointers to inlier for drawing....
            }
        }
        modelPoints.EraseLast();
        imgPoints.EraseLast();

        CopyPoseCv(pose,tmpPose);
        if (modelPoints.Size()>4)
        {
            err/=(double)modelPoints.Size();
            rod = cvCreateMat(3,1,CV_32F);
            cvInitMatHeader( &modelPointsCv, modelPoints.Size(), 3, CV_32F, &modelPoints[0].x );
            cvInitMatHeader( &imgPointsCv, imgPoints.Size(), 2, CV_32F, &imgPoints[0].x );

            cvFindExtrinsicCameraParams2(&modelPointsCv, &imgPointsCv, cameraMatrix, distCoeffs, rod, pose.t);
            cvRodrigues2(rod,pose.R);

            cvReleaseMat(&rod);
        }

        //up to now we have no keypoint normals, i.e. no visibility
        //so for now if least squares fails just copy back the RANSAC pose!!!
        p = Vector3(cvmGet(pose.t,0,0),cvmGet(pose.t,1,0),cvmGet(pose.t,2,0));
        t = Vector3(cvmGet(tmpPose.t,0,0),cvmGet(tmpPose.t,1,0),cvmGet(tmpPose.t,2,0));
        if (Distance(p,t) < (maxz-minz) * Def::DISTANCE_RATIO)
        {
            inl = modelPoints.Size();
        }else
        {
            CopyPoseCv(tmpPose,pose);
        }
    }

    /**
     * ComputeConfidence
     */
    void ODetect3D::ComputeConfidence(Array<KeypointDescriptor *> &keys, unsigned &numInl, Object3D &object)
    {
        Vector2 p;
        Array<Vector2> cs;
        CodebookEntry *cbe;

        if (numInl>=4)
        {
            unsigned numKeys=0;
            CvMat *pos = cvCreateMat( 3, 1, CV_32F );
            for (unsigned i=0; i<object.codebook.Size(); i++)
            {
                cbe=object.codebook[i];
                for (unsigned j=0; j<cbe->occurrences.Size(); j++)
                {
                    cvMatMulAdd( object.pose.R, cbe->occurrences[j]->pos, object.pose.t, pos );
                    ProjectPoint2Image(pos->data.fl[0],pos->data.fl[1],pos->data.fl[2], cameraMatrix, p.x, p.y);
                    cs.PushBack(p);
                }
            }
            cvReleaseMat(&pos);
            ConvexHull(cs, object.contour.v);

            for (unsigned i=0; i<keys.Size(); i++)
                if (object.contour.Inside(keys[i]->p))
                    numKeys++;

            if (numKeys>4)
            {
                object.conf = (double)numInl / (double)numKeys;
                return;
            }
        }

        object.conf = 0.;
    }



    /******************************** PUBLIC **************************/
    /**
     * Simple RANSAC based 3d object detector
     */
    bool ODetect3D::Detect(Array<KeypointDescriptor *> &keys, Object3D &object)
    {
#ifdef DEBUG
        struct timespec start0, end0, start1, end1, start2, end2, start3, end3;
        clock_gettime(CLOCK_REALTIME, &start1);
        clock_gettime(CLOCK_REALTIME, &start0);
#endif

        unsigned numInl=0;
        DeletePairs(matches); // make sure the vector is empty, we'll keep them for later to draw

        if (Def::DO_MATCHER==2)              // match keypoints with codebook  gpu
            MatchKeypointsGPU(keys, object.codebook, matches);
        else if (Def::DO_MATCHER==1)         // threshold matches
            MatchKeypoints(keys, object.codebook, matches);
        else                                 // use second nearest neighbour
            MatchKeypoints2(keys, object.codebook, matches);

        for (unsigned i=0; i<matches.Size(); i++)
            KeypointDescriptor::Draw(dbg,*matches[i]->k,CV_RGB(255,255,0));

#ifdef DEBUG
        clock_gettime(CLOCK_REALTIME, &end0);
        if (dbg!=0)
            for (unsigned i=0; i<matches.Size(); i++)
                KeypointDescriptor::Draw(dbg,*matches[i]->k,CV_RGB(255,0,0));
        clock_gettime(CLOCK_REALTIME, &start2);
#endif

        FitModelRANSAC(matches, object.pose, numInl); // ransac pose

#ifdef DEBUG
        clock_gettime(CLOCK_REALTIME, &end2);
        clock_gettime(CLOCK_REALTIME, &start3);
#endif

        RefinePoseLS(matches, object.pose, numInl, object.err);     // least squares pose
        ComputeConfidence(keys, numInl, object);


        //DEBUG
//        mat3 cR;
//        vec3 ct;
//        cR[0] = cvmGet(object.pose.R,0,0); cR[3] = cvmGet(object.pose.R,0,1); cR[6] = cvmGet(object.pose.R,0,2);
//        cR[1] = cvmGet(object.pose.R,1,0); cR[4] = cvmGet(object.pose.R,1,1); cR[7] = cvmGet(object.pose.R,1,2);
//        cR[2] = cvmGet(object.pose.R,2,0); cR[5] = cvmGet(object.pose.R,2,1); cR[8] = cvmGet(object.pose.R,2,2);
//
//        ct.x = cvmGet(object.pose.t,0,0);
//        ct.y = cvmGet(object.pose.t,1,0);
//        ct.z = cvmGet(object.pose.t,2,0);
//
//        ROS_INFO("ct and cR in DETECT: [%f %f %f] \n [%f %f %f] \n [%f %f %f] \n [%f %f %f]",
//                 ct.x, ct.y, ct.z, cR[0], cR[3], cR[6], cR[1], cR[4], cR[7], cR[2], cR[5], cR[8]);


#ifdef DEBUG
        clock_gettime(CLOCK_REALTIME, &end3);
        clock_gettime(CLOCK_REALTIME, &end1);
        cout<<"id="<<object.id<<", conf="<<object.conf<<", err="<<object.err<<", numInl="<<numInl
                <<", cbSize="<<object.codebook.Size()<<endl;
        cout<<"Pose translation: ["<<cvmGet(object.pose.t,0,0)<<" "<<cvmGet(object.pose.t,1,0)<<" "<<cvmGet(object.pose.t,2,0)<<"]"<<endl;
        cout<<"Time for matching keypoints [s]: "<<P::timespec_diff(&end0, &start0)<<endl;
        cout<<"Time for RANSAC [s]: "<<P::timespec_diff(&end2, &start2)<<endl;
        cout<<"Time for least squares pose [s]: "<<P::timespec_diff(&end3, &start3)<<endl;
        cout<<"Time for object detection (matching and RANSAC) [s]: "<<P::timespec_diff(&end1, &start1)<<endl;
#endif

        if (numInl>=4)
            return true;

        object.conf=0.;
        object.err=DBL_MAX;

        return false;
    }

    /**
     * Set camera intrinsic parameter
     * We assume undistort images!!!
     * Keypoints must be undistort!!!!
     * @param _C Camera intrinsic matrix of undistort images
     */
    void ODetect3D::SetCameraParameter(CvMat *C)
    {
        cvCopy(C,cameraMatrix);
    }

    /**
     * Draw inlier stored during recognition.
     * Attention: Pointers are stored!
     *            DrawInlier only works as long as image keypoints are valid!
     */
    void ODetect3D::DrawInlier(IplImage *img, CvScalar col)
    {
        for (unsigned i=0; i<inlier.Size(); i++)
            KeypointDescriptor::Draw(img,*inlier[i], col);
    }

}

