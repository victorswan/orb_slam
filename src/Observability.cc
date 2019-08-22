/**
* This file is part of GF-ORB-SLAM2.
*
* Copyright (C) 2019 Yipu Zhao <yipu dot zhao at gatech dot edu> 
* (Georgia Institute of Technology)
* For more information see 
* <https://sites.google.com/site/zhaoyipu/good-feature-visual-slam>
*
* GF-ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* GF-ORB-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with GF-ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Observability.h"
#include <sstream>
#include <iomanip>
#include <iostream>
#include <iosfwd>

//#define TIME_TMPRUPDATE_SORE
std::mutex mtx;

namespace ORB_SLAM2 {

void Observability::compute_SOM_In_Segment(const size_t seg_idx, const arma::mat Y, const arma::mat Z,
                                           arma::mat & curObsMat) {
    // Compute the H matrix
    arma::mat H13, H47, H_proj;
    float res_u = 0, res_v = 0;
    compute_H_subblock_complete(this->kinematic[seg_idx].Xv, Y, Z, H13, H47, H_proj, res_u, res_v);

    //    std::cout << "H13 = " << H13 << "; H47 = " << H47 << std::endl;

    //std::cout << "[OBS_COMPUTOR]  LinObsMat" << std::endl;
    curObsMat = arma::zeros<arma::mat>(26, 13);
    // Copy first 3 columns
    curObsMat.cols(0, 2) = arma::repmat(H13, 13, 1);
    // Columns 7~9
    curObsMat.cols(7, 9) = curObsMat.cols(0, 2) % this->H13_mul_fac;
    if(fabs(this->kinematic[seg_idx].dt_inSeg - 1.0) > 1e-6) {
        curObsMat.cols(7,9) = curObsMat.cols(7,9) * this->kinematic[seg_idx].dt_inSeg;
    }
    // First segment in linObsMat: just H:
    curObsMat(arma::span(0,1), arma::span(3,6))= H47;

    // Segment 2 to 13
    arma::mat rollingQAdd= arma::eye<arma::mat>(4, 4);  // (Q^(n-1) + Q^(n-2) +... + I)
    arma::mat rollingQFac = this->kinematic[seg_idx].F_Q_inSeg;  // Q^n
    for (int j = 1; j < 13; j++) {
        // [0,1,2,   3,4,5,6,   7,8,9,   10,11,12 ]

        // 2nd block:  (H47 * Q^n)
        curObsMat( arma::span(j*2, ((j+1)*2-1)), arma::span(3,6)) = H47 * rollingQFac;

        // 4th block: Q^(n-1) + Q^(n-2) +... + I) * Omega
        curObsMat( arma::span(j*2, ((j+1)*2-1)), arma::span(10,12)) = H47 * (rollingQAdd * this->kinematic[seg_idx].F_Omg_inSeg);

        // Update
        rollingQAdd = rollingQAdd + rollingQFac;
        rollingQFac = rollingQFac * this->kinematic[seg_idx].F_Q_inSeg;
    }
}

void Observability::batchStripObsMat_Frame(const int start_idx, const int end_idx) {

    assert(start_idx < end_idx);
    //
    for(int i = start_idx; i < end_idx; i ++)  {
        //        std::cout << "lmk " << i << std::endl;
        MapPoint* pMP = pFrame->mvpMapPoints[i];
        if(pMP) {
            if (this->pFrame->mvbOutlier[i] == false &&
                    this->pFrame->mvbCandidate[i] == true) {

                // skip if the Jacobian has already been built
                if (this->pFrame->mvbJacobBuilt[i] == true)
                    continue ;

                // Feature position
                cv::Mat Pw = pMP->GetWorldPos();
                arma::rowvec Y = arma::zeros<arma::rowvec>(3);
                Y[0] = Pw.at<float>(0);
                Y[1] = Pw.at<float>(1);
                Y[2] = Pw.at<float>(2);

                arma::mat TOM;
                arma::mat F_fac(13, 13);
                F_fac.eye();
                //
                size_t j;
                for (j=0; j<this->kinematic.size(); ++j) {
                    // Get the position of measurement
                    arma::rowvec Z = arma::zeros<arma::rowvec>(2);
                    if (j == 0) {
                        // use the actual measurement
                        cv::KeyPoint kpUn = pFrame->mvKeysUn[i];
                        Z[0] = kpUn.pt.x;
                        Z[1] = kpUn.pt.y;
                    }
                    else {
                        // predict the measurement
                        //
                        float u, v;
                        arma::mat proj_jacob;
                        project_Point_To_Frame(Pw, this->kinematic[j].Tcw, u, v, proj_jacob);
                        Z[0] = u;
                        Z[1] = v;
                    }
                    //                    std::cout << Z << std::endl;

                    // terminate if went out of bound
                    if (Z[0] < 0 || Z[1] < 0) {
                        break ;
                    }
                    //                    std::cout << Z << std::endl;

                    // Compute the In-segment Observability Matrix
                    arma::mat curSOM;
                    compute_SOM_In_Segment(j, Y, Z, curSOM);

                    // Multiply with full-seg system matrix, so as to
                    // obtain total obs matrix
                    TOM = arma::join_vert(TOM, curSOM * F_fac);
                    F_fac = this->kinematic[j].F * F_fac;
                }

                if (TOM.n_rows > 0) {
                    pMP->ObsMat  = TOM;
                    //                pMP->ObsRank = arma::rank(TOM);
                    pMP->ObsScore = 1.0;
                }
                else {
                    pMP->ObsScore = -1.0;
                }

                //
                this->pFrame->mvbJacobBuilt[i] = true;

            } //Perform regular temporal obs update!
        }
    } // For: all pMP
}

void Observability::batchHybridMat_Frame(const size_t start_idx, const size_t end_idx, const double time_for_build) {

    if (this->mKineIdx >= this->kinematic.size())
        return ;

    arma::mat H13, H47, H_meas, H_proj, H_rw;
    float res_u = 0, res_v = 0, u, v;
    arma::mat ZMat;
    ZMat.zeros(2, 6);

    double timeCap = 0;
    arma::wall_clock timer;
    timer.tic();

    for(size_t i = start_idx; i < end_idx; i++)  {
        //
        if (i >= this->pFrame->mvpMapPoints.size())
            break ;

        // apply cap on time cost
        timeCap = timer.toc();
        if (timeCap > time_for_build) {
            std::cout << "func batchHybridMat_Frame: reach max time cap " << timeCap << std::endl;
            break ;
        }

        //        std::cout << i << std::endl;
        MapPoint* pMP = this->pFrame->mvpMapPoints[i];
        // If the current Map point is not matched:
        // Reset the Obs information
        if(pMP) {
            if (this->pFrame->mvbOutlier[i] == false &&
                    this->pFrame->mvbCandidate[i] == true) {

                // skip if the Jacobian has already been built
                if (this->pFrame->mvbJacobBuilt[i] == true)
                    continue ;

                // Feature position
                arma::rowvec Y = arma::zeros<arma::rowvec>(4);
                cv::Mat Pw = pMP->GetWorldPos();
                Y[0] = Pw.at<float>(0);
                Y[1] = Pw.at<float>(1);
                Y[2] = Pw.at<float>(2);
                Y[3] = 1;

                // Measurement
                arma::rowvec Z = arma::zeros<arma::rowvec>(2);
                cv::KeyPoint kpUn = this->pFrame->mvKeysUn[i];
                Z[0] = kpUn.pt.x;
                Z[1] = kpUn.pt.y;

#if defined LOCAL_JACOBIAN

                //                std::cout << "Tcw_prev = " << this->Tcw_prev << std::endl;
                //                std::cout << "Twc = " << this->Twc << std::endl;

                // convert Y into local version
                // Y_rel
                arma::mat Y_rel = this->Tcw_prev * Y;
                //                std::cout << "Y_rel = " << Y_rel << std::endl;
                //
                compute_H_subblock_fast(this->Xv_rel, Y_rel.cols(0, 2), Z, H13, H47, res_u, res_v);
                //                std::cout << "H13 = " << H13 << std::endl;
                //                std::cout << "H47 = " << H47 << std::endl;
#else
                //            timer.tic();
                // Compute the H matrix
                //            compute_H_subblock(this->Xv, Y, Z, H13, H47);
                // compute_H_subblock_fast(this->kinematic[0].Xv, Y.cols(0, 2), Z, H13, H47);
                //                compute_H_subblock_complete(this->kinematic[this->mKineIdx].Xv, Y.subvec(0, 2), Z, H13, H47, H_proj, res_u, res_v);
                compute_H_subblock_simplied(this->kinematic[this->mKineIdx].Xv, Y.subvec(0, 2), H13, H47, H_proj, false, u, v);
                res_u = Z[0] - u;
                res_v = Z[1] - v;
                //            time_H += timer.toc();

                //                std::cout << H13 << " " << H47 << std::endl;
#endif

                // assemble into H matrix
                H_meas = arma::join_horiz(H13, H47);
                H_meas = arma::join_horiz(H_meas, ZMat);

                reWeightInfoMat( this->pFrame, i, pMP, H_meas, res_u, res_v, H_proj, H_rw );

                arma::mat Hyb = arma::join_vert(H_rw, H_rw * this->kinematic[this->mKineIdx].F);
                //                arma::mat HF = H_rw * this->kinematic[this->mKineIdx].F;
                //                arma::mat HF_2 = HF * this->kinematic[this->mKineIdx].F;
                //                arma::mat HF_3 = HF_2 * this->kinematic[this->mKineIdx].F;
                //                //
                //                arma::mat Hyb = arma::join_vert(H_rw, HF, HF_2, HF_3);
                arma::mat infMat = Hyb.t() * Hyb;
                //                std::cout << "H_rw = " << H_rw << std::endl;
                //                std::cout << "F = " << this->kinematic[this->mKineIdx].F << std::endl;
                //                std::cout << "Hyb = " << Hyb << std::endl;

#ifdef MULTI_THREAD_LOCK_ON
                mtx.lock();
#endif

                pMP->u_proj = u;
                pMP->v_proj = v;
                pMP->H_meas = H_meas;
                pMP->H_proj = H_proj;
                pMP->ObsMat = infMat;
                pMP->updateAtFrameId = this->mnFrameId;

                // do nothing on the obs matrix; set the score to 1 for up-coming subset selection
                //                pMP->ObsScore = arma::norm(pMP->ObsMat, 2);
                //                pMP->ObsScore = arma::norm(pMP->ObsMat, "fro");
                //                pMP->ObsScore = 1.0;
                pMP->ObsScore = this->pFrame->mvpMatchScore[i];
                //
                this->pFrame->mvbJacobBuilt[i] = true;

#ifdef MULTI_THREAD_LOCK_ON
                mtx.unlock();
#endif
            }
            else {
#ifdef MULTI_THREAD_LOCK_ON
                mtx.lock();
#endif

                pMP->ObsScore = -1.0;

#ifdef MULTI_THREAD_LOCK_ON
                mtx.unlock();
#endif
            }
        } //Perform regular temporal obs update!
    } // For: all pMP
}

void Observability::batchHybridMat_Map(const size_t start_idx, const size_t end_idx, const double time_for_build, const bool check_viz) {

    if (this->mKineIdx >= this->kinematic.size())
        return ;

    arma::mat H13, H47, H_meas, H_proj, H_rw;
    float u, v;
    arma::mat ZMat;
    ZMat.zeros(2, 6);

    double timeCap = 0;
    arma::wall_clock timer;
    timer.tic();

    for(size_t i = start_idx; i < end_idx; i++)  {
        //
        if (i >= this->mMapPoints->size())
            break ;

        // apply cap on time cost
        timeCap = timer.toc();
        if (timeCap > time_for_build) {
            std::cout << "func batchHybridMat_Map: reach max time cap " << timeCap << std::endl;
            break ;
        }

        //        std::cout << i << std::endl;
        MapPoint* pMP = this->mMapPoints->at(i);
        // If the current Map point is not matched:
        // Reset the Obs information
        if(pMP) {

            if (pMP->updateAtFrameId == this->mnFrameId)
                continue ;

            if (check_viz == false && pMP->mbTrackInView == false)
                continue ;

            // Feature position
            arma::rowvec Y = arma::zeros<arma::rowvec>(3);
            cv::Mat featPos = pMP->GetWorldPos();
            Y[0] = featPos.at<float>(0);
            Y[1] = featPos.at<float>(1);
            Y[2] = featPos.at<float>(2);

            bool flag = compute_H_subblock_simplied(this->kinematic[this->mKineIdx].Xv, Y, H13, H47, H_proj, check_viz, u, v);
            if (flag == false) {
#ifdef MULTI_THREAD_LOCK_ON
                mtx.lock();
#endif

                pMP->ObsScore = -1.0;
                //                pMP->updateAtFrameId = this->mnFrameId;

#ifdef MULTI_THREAD_LOCK_ON
                mtx.unlock();
#endif
                continue ;
            }

            //            std::cout << "find visible lmk " << i << std::endl;

            // assemble into H matrix
            H_meas = arma::join_horiz(H13, H47);
            H_meas = arma::join_horiz(H_meas, ZMat);

            reWeightInfoMat( NULL, -1, pMP, H_meas, 0, 0, H_proj, H_rw );

            arma::mat Hyb = arma::join_vert(H_rw, H_rw * this->kinematic[this->mKineIdx].F);
            //            arma::mat HF = H_rw * this->kinematic[this->mKineIdx].F;
            //            arma::mat HF_2 = HF * this->kinematic[this->mKineIdx].F;
            //            arma::mat HF_3 = HF_2 * this->kinematic[this->mKineIdx].F;
            //            //
            //            arma::mat Hyb = arma::join_vert(H_rw, HF, HF_2, HF_3);
            arma::mat infMat = Hyb.t() * Hyb;

#ifdef MULTI_THREAD_LOCK_ON
            mtx.lock();
#endif

            pMP->u_proj = u;
            pMP->v_proj = v;
            pMP->H_meas = H_meas;
            pMP->H_proj = H_proj;
            pMP->ObsMat = infMat;
            // do nothing on the obs matrix; set the score to 1 for up-coming subset selection
            pMP->ObsScore = 1.0;
            pMP->updateAtFrameId = this->mnFrameId;
            //            lmkSelectPool.push_back( tmpLmk );

#ifdef MULTI_THREAD_LOCK_ON
            mtx.unlock();
#endif
        }
    } //Perform regular temporal obs update!
}

void Observability::batchInfoMat_Frame(const size_t start_idx, const size_t end_idx, const double time_for_build) {

    if (this->mKineIdx >= this->kinematic.size())
        return ;

    arma::mat H13, H47, H_meas, H_proj, H_disp, H_rw;
    float res_u = 0, res_v = 0, u, v;

    double timeCap = 0;
    arma::wall_clock timer;
    timer.tic();

    for(size_t i = start_idx; i < end_idx; i++)  {
        //
        if (i >= this->pFrame->mvpMapPoints.size())
            break ;

        // apply cap on time cost
        timeCap = timer.toc();
        if (timeCap > time_for_build) {
            std::cout << "func batchInfoMat_Frame: reach max time cap " << timeCap << std::endl;
            break ;
        }

        //        std::cout << i << std::endl;
        MapPoint* pMP = this->pFrame->mvpMapPoints[i];
        // If the current Map point is not matched:
        // Reset the Obs information
        if(pMP) {
            if (this->pFrame->mvbOutlier[i] == false &&
                    this->pFrame->mvbCandidate[i] == true) {

                // skip if the Jacobian has already been built
//                if (this->pFrame->mvbJacobBuilt[i] == true)
//                    continue ;
                if (pMP->updateAtFrameId == this->mnFrameId)
                    continue ;

                // Feature position
                arma::rowvec Y = arma::zeros<arma::rowvec>(4);
                cv::Mat Pw = pMP->GetWorldPos();
                Y[0] = Pw.at<float>(0);
                Y[1] = Pw.at<float>(1);
                Y[2] = Pw.at<float>(2);
                Y[3] = 1;

                // Measurement
                arma::rowvec Z = arma::zeros<arma::rowvec>(2);
                cv::KeyPoint kpUn = this->pFrame->mvKeysUn[i];
                Z[0] = kpUn.pt.x;
                Z[1] = kpUn.pt.y;

#if defined LOCAL_JACOBIAN

                //                std::cout << "Tcw_prev = " << this->Tcw_prev << std::endl;
                //                std::cout << "Twc = " << this->Twc << std::endl;

                // convert Y into local version
                // Y_rel
                arma::mat Y_rel = this->Tcw_prev * Y;
                //                std::cout << "Y_rel = " << Y_rel << std::endl;
                //
                compute_H_subblock_fast(this->Xv_rel, Y_rel.cols(0, 2), Z, H13, H47, res_u, res_v);
                //                std::cout << "H13 = " << H13 << std::endl;
                //                std::cout << "H47 = " << H47 << std::endl;
#else
                //            timer.tic();
                // Compute the H matrix
                //            compute_H_subblock(this->Xv, Y, Z, H13, H47);
                // compute_H_subblock_fast(this->kinematic[0].Xv, Y.cols(0, 2), Z, H13, H47);
                //                compute_H_subblock_complete(this->kinematic[this->mKineIdx].Xv, Y.subvec(0, 2), Z, H13, H47, H_proj, res_u, res_v);
                compute_H_subblock_simplied(this->kinematic[this->mKineIdx].Xv, Y.subvec(0, 2), H13, H47, H_proj, false, u, v);
                res_u = Z[0] - u;
                res_v = Z[1] - v;
                //            time_H += timer.toc();

                //                std::cout << H13 << " " << H47 << std::endl;
#endif

                // assemble into H matrix
                H_meas = arma::join_horiz(H13, H47);

                //#ifdef DELAYED_STEREO_MATCHING
                if (mSensor == 1 || mSensor == 2) {
                    if (this->pFrame->mvuRight[i] >= 0) {
                        // add the disparity info term to curMat
                        compute_H_disparity_col (this->kinematic[this->mKineIdx].Xv, Y.subvec(0, 2), H_disp);
                        H_meas = arma::join_vert(H_meas, H_disp);
                    }
                }
                //                cout << H_meas << endl << endl;
                //#endif

                reWeightInfoMat( this->pFrame, i, pMP, H_meas, res_u, res_v, H_proj, H_rw );
                arma::mat infMat = H_rw.t() * H_rw;
                //                GoodPoint tmpLmk(static_cast<size_t>(i), this->pFrame->mvpMatchScore[i], infMat);

#ifdef MULTI_THREAD_LOCK_ON
                mtx.lock();
#endif
                //                // Update the total information matrix
                //                int num_row_obsMat = static_cast<int>(pMP->ObsMat.n_rows);
                //                arma::mat ObsMat_tmp = pMP->ObsMat;

                //                if (num_row_obsMat >= 2*OBS_SEGMENT_NUM) {
                //                    // remove the 2 rows in the bottom
                //                    ObsMat_tmp.shed_rows(2*OBS_SEGMENT_NUM-2, num_row_obsMat-1);
                //                }

                //                // multiply the rest blocks by F matrix
                //                if (ObsMat_tmp.n_rows > 0) {
                //                    ObsMat_tmp = ObsMat_tmp * this->kinematic[0].F;
                //                }

                //                // add new block to the head of SOM
                //                pMP->ObsMat  = arma::join_vert(H, ObsMat_tmp);
                //                //                std::cout << pMP->ObsMat << std::endl;
                pMP->u_proj = u;
                pMP->v_proj = v;
                pMP->H_meas = H_meas;
                pMP->H_proj = H_proj;
                pMP->ObsMat = infMat;

                // TODO
                // for some reason, adding all frame-by-frame matches into the prior info matrix hurts the performance of good feature;
                // simply ignore all of them also leads to degeneracy.
                // the best performance is obtained when only the prior information from previous map is used while the current frame-by-frame ones are ignored
                // later we need to dig into this issue.
                pMP->updateAtFrameId = this->mnFrameId;

                // compute observability score
#if defined OBS_MIN_SVD
                arma::vec s = arma::svd(H);
                pMP->ObsScore = s(12);
#elif defined OBS_MAX_VOL || defined OBS_MAXVOL_TOP_SV
                double s;
                arma::colvec u;
                arma::rowvec v;
                solveMaxSVD(H, u, s, v);
                pMP->ObsVector = arma::vec(DIMENSION_OF_STATE_MODEL);
                for (size_t dn=0; dn < DIMENSION_OF_STATE_MODEL; ++ dn) {
                    pMP->ObsVector[dn] = v(dn);
                }
                pMP->ObsScore = s;
#elif defined OBS_MAXVOL_FULL || defined OBS_MAXVOL_Greedy
                // do nothing on the obs matrix; set the score to 1 for up-coming subset selection
                //                pMP->ObsScore = arma::norm(pMP->ObsMat, 2);
                //                pMP->ObsScore = arma::norm(pMP->ObsMat, "fro");
                //                pMP->ObsScore = 1.0;
                pMP->ObsScore = this->pFrame->mvpMatchScore[i];
#endif
                //
//                this->pFrame->mvbJacobBuilt[i] = true;

#ifdef MULTI_THREAD_LOCK_ON
                mtx.unlock();
#endif
            }
            else {
#ifdef MULTI_THREAD_LOCK_ON
                mtx.lock();
#endif

                pMP->ObsScore = -1.0;

#ifdef MULTI_THREAD_LOCK_ON
                mtx.unlock();
#endif
            }
        } //Perform regular temporal obs update!
    } // For: all pMP

    //#ifdef OBS_DEBUG_VERBOSE
    //    std::cout << "func getObsMatrix: Time cost of H submatrix = " << time_H << std::endl;
    //    std::cout << "func getObsMatrix: Time cost of Build Inst SOM = " << time_BuildMat << std::endl;
    //    std::cout << "func getObsMatrix: Time cost of Update SOM = " << time_UpdateMat << std::endl;
    //    std::cout << "func getObsMatrix: Time cost of arma::svd = " << time_SVD_1 << std::endl;
    //    std::cout << "func getObsMatrix: Time cost of max-vector svd = " << time_SVD_2 << std::endl;
    //#endif
}

void Observability::batchInfoMat_Map(const size_t start_idx, const size_t end_idx, const double time_for_build, const bool check_viz) {

    if (this->mKineIdx >= this->kinematic.size())
        return ;

    arma::mat H13, H47, H_meas, H_disp, H_proj, H_rw;
    float u, v;

    double timeCap = 0;
    arma::wall_clock timer;
    timer.tic();

    for(size_t i = start_idx; i < end_idx; i++)  {
        //
        if (i >= this->mMapPoints->size())
            break ;

        // apply cap on time cost
        timeCap = timer.toc();
        if (timeCap > time_for_build) {
//            std::cout << "func batchInfoMat_Map: reach max time cap " << timeCap << std::endl;
            break ;
        }

        //        std::cout << i << std::endl;
        MapPoint* pMP = this->mMapPoints->at(i);
        // If the current Map point is not matched:
        // Reset the Obs information
        if(pMP) {

            if (pMP->updateAtFrameId == this->mnFrameId)
                continue ;

            if (check_viz == false && pMP->mbTrackInView == false)
                continue ;

            // Feature position
            arma::rowvec Y = arma::zeros<arma::rowvec>(3);
            cv::Mat featPos = pMP->GetWorldPos();
            Y[0] = featPos.at<float>(0);
            Y[1] = featPos.at<float>(1);
            Y[2] = featPos.at<float>(2);

            bool flag = compute_H_subblock_simplied(this->kinematic[this->mKineIdx].Xv, Y, H13, H47, H_proj, check_viz, u, v);
            if (flag == false) {
#ifdef MULTI_THREAD_LOCK_ON
                mtx.lock();
#endif

                pMP->ObsScore = -1.0;
                //                pMP->updateAtFrameId = this->mnFrameId;

#ifdef MULTI_THREAD_LOCK_ON
                mtx.unlock();
#endif
                continue ;
            }

            //            std::cout << "find visible lmk " << i << std::endl;

            // assemble into H matrix
            H_meas = arma::join_horiz(H13, H47);

            // NOTE
            // for RGBD data, assume the depth / disparity will be measured
            // therefore including the addition row in measurement Jacobian
            // meanwhile for mono & stereo setup, assuming only the left measurement being avaible
            if (mSensor == 2) {
                // add the disparity info term to curMat
                compute_H_disparity_col (this->kinematic[this->mKineIdx].Xv, Y.subvec(0, 2), H_disp);
                H_meas = arma::join_vert(H_meas, H_disp);
            }

            arma::mat infMat = H_meas.t() * H_meas;
            //            reWeightInfoMat( NULL, -1, pMP, H_meas, 0, 0, H_proj, H_rw );
            //            arma::mat infMat = H_rw.t() * H_rw;

#ifdef MULTI_THREAD_LOCK_ON
            mtx.lock();
#endif

            pMP->u_proj = u;
            pMP->v_proj = v;
            pMP->H_meas = H_meas;
            pMP->H_proj = H_proj;
            pMP->ObsMat = infMat;
            // do nothing on the obs matrix; set the score to 1 for up-coming subset selection
            pMP->ObsScore = 1.0;

            // TO DO
            pMP->updateAtFrameId = this->mnFrameId;
            //            lmkSelectPool.push_back( tmpLmk );

#ifdef MULTI_THREAD_LOCK_ON
            mtx.unlock();
#endif
        }
    } //Perform regular temporal obs update!
}

bool Observability::runMatrixBuilding(const size_t mat_type, const double time_for_build,
                                      const bool with_multi_thread, const bool check_viz) {

    int N = 0;

    switch (mat_type) {
    case ORB_SLAM2::MAP_INFO_MATRIX:
        //
        if (this->mMapPoints == NULL)
            return false;

        N = this->mMapPoints->size();

        if (with_multi_thread) {

            int grainSize = static_cast<double>(N)/static_cast<double>(mNumThreads);

            if (mNumThreads > 1)
                mThreads = new std::thread [mNumThreads-1];
            for (size_t i = 0; i < mNumThreads-1; i++) {
                mThreads[i] = std::thread(&Observability::batchInfoMat_Map, this, i*grainSize, (i+1)*grainSize, time_for_build, check_viz);
            }
            //    mThreads[mNumThreads-1] = std::thread(&Observability::batchBuildInstInfoMat, this, (mNumThreads-1)*grainSize, N);
            this->batchInfoMat_Map((mNumThreads-1)*grainSize, N, time_for_build, check_viz);

            for (size_t i = 0; i < mNumThreads-1; i++) {
                mThreads[i].join();
            }

            delete [] mThreads;

        }
        else {

            // single-thread matrix construction
            this->batchInfoMat_Map(0, N, time_for_build, check_viz);

        }

        break;

    case ORB_SLAM2::FRAME_INFO_MATRIX:
        //
        if (this->pFrame == NULL)
            return false;

        N = this->pFrame->mvpMapPoints.size();

        if (with_multi_thread) {

            // original solution, utilizing c++ 11 std thread
            int grainSize = static_cast<double>(N)/static_cast<double>(mNumThreads);

            if (mNumThreads > 1)
                mThreads = new std::thread [mNumThreads-1];
            for (size_t i = 0; i < mNumThreads-1; i++) {
                mThreads[i] = std::thread(&Observability::batchInfoMat_Frame, this, i*grainSize, (i+1)*grainSize, time_for_build);
            }
            //    mThreads[mNumThreads-1] = std::thread(&Observability::batchBuildInstInfoMat, this, (mNumThreads-1)*grainSize, N);
            this->batchInfoMat_Frame((mNumThreads-1)*grainSize, N, time_for_build);

            for (size_t i = 0; i < mNumThreads-1; i++) {
                mThreads[i].join();
            }
            delete [] mThreads;
            //        cout << "done with info mat multi thread!" << endl;

        }
        else {

            // single-thread matrix construction
            this->batchInfoMat_Frame(0, N, time_for_build);

        }

        break;

    case ORB_SLAM2::MAP_HYBRID_MATRIX:
        //
        if (this->mMapPoints == NULL)
            return false;

        N = this->mMapPoints->size();

        if (with_multi_thread) {

            int grainSize = static_cast<double>(N)/static_cast<double>(mNumThreads);

            if (mNumThreads > 1)
                mThreads = new std::thread [mNumThreads-1];
            for (size_t i = 0; i < mNumThreads-1; i++) {
                mThreads[i] = std::thread(&Observability::batchHybridMat_Map, this, i*grainSize, (i+1)*grainSize, time_for_build, check_viz);
            }
            //    mThreads[mNumThreads-1] = std::thread(&Observability::batchBuildInstInfoMat, this, (mNumThreads-1)*grainSize, N);
            this->batchHybridMat_Map((mNumThreads-1)*grainSize, N, time_for_build, check_viz);

            for (size_t i = 0; i < mNumThreads-1; i++) {
                mThreads[i].join();
            }

            delete [] mThreads;

        }
        else {

            // single-thread matrix construction
            this->batchHybridMat_Map(0, N, time_for_build, check_viz);

        }

        break;

    case ORB_SLAM2::FRAME_HYBRID_MATRIX:
        //
        if (this->pFrame == NULL)
            return false;

        N = this->pFrame->mvpMapPoints.size();

        if (with_multi_thread) {

            // original solution, utilizing c++ 11 std thread
            int grainSize = static_cast<double>(N)/static_cast<double>(mNumThreads);

            if (mNumThreads > 1)
                mThreads = new std::thread [mNumThreads-1];
            for (size_t i = 0; i < mNumThreads-1; i++) {
                mThreads[i] = std::thread(&Observability::batchHybridMat_Frame, this, i*grainSize, (i+1)*grainSize, time_for_build);
            }
            //    mThreads[mNumThreads-1] = std::thread(&Observability::batchBuildInstInfoMat, this, (mNumThreads-1)*grainSize, N);
            this->batchHybridMat_Frame((mNumThreads-1)*grainSize, N, time_for_build);

            for (size_t i = 0; i < mNumThreads-1; i++) {
                mThreads[i].join();
            }
            delete [] mThreads;
            //        cout << "done with info mat multi thread!" << endl;

        }
        else {

            // single-thread matrix construction
            this->batchHybridMat_Frame(0, N, time_for_build);

        }

        break;

    case ORB_SLAM2::FRAME_STRIP_OBS_MATRIX:
        // TODO
        break;

    default:
        // do nothing
        cerr << "unknown matrix type!" << endl;
    }

    return true;
}

// ==================================================================================

bool Observability::setSelction_Number(const size_t num_good_inlier,
                                       const int greedy_mtd,
                                       const double time_for_select,
                                       const double error_bound,
                                       Frame *pFrame, vector<GoodPoint> *mpVec) {

    if (pFrame == NULL || mpVec == NULL || time_for_select <= 0) {
        time_Selection = 0;
        return false;
    }

    const int N = pFrame->mvpMapPoints.size();

    this->pFrame = pFrame;
    //    this->segNum = OBS_SEGMENT_NUM;
    //    this->kinematic[0].dt = dt;
    // double random_sample_scale = 6.0; // 16.0; // 20.0; // 12.0; // 10.0; // 4.0; //
    //    double errorBound = 0.1; // 0.05; // 0.01; //
    double random_sample_scale = static_cast<size_t>( float(N) / float(num_good_inlier) * log(1.0 / error_bound) );

    //-----------------------------------------------------------------------------
    // Matrix construction for each feature (independently, therefore in parallel)

    arma::wall_clock timer;
    timer.tic();

    runMatrixBuilding(ORB_SLAM2::FRAME_INFO_MATRIX, time_for_select/2, true);

    lmkSelectPool.clear();
    // Collect the lmkSelectPool
    for(int i=0; i<N; i++)  {
        MapPoint* pMP = pFrame->mvpMapPoints[i];
        if(pMP) {
            if (pFrame->mvbOutlier[i] == false && pFrame->mvbCandidate[i] == true && pFrame->mvbJacobBuilt[i] == true) {
                assert(pMP->ObsScore >= 0);
                //
                GoodPoint tmpLmk(static_cast<size_t>(i), pMP->ObsScore, pMP->ObsMat);
                lmkSelectPool.push_back( tmpLmk );
            }
        }
    }
    //    std::cout << "func setSelction_Number: number of inliers for subset selection = " << lmkSelectPool.size() << std::endl;

    time_MatBuild = timer.toc();
#ifdef OBS_TIMECOST_VERBOSE
    std::cout << "func setSelction_Number: time cost of multi-thread matrix construction = " << time_MatBuild << std::endl;
#endif

    //-----------------------------------------------------------------------------

    // The sorting process seem to take as much time as the parrallel obs computation
    // Next we will replace the sort part with Max-Basis-Volume approach (O(Nlog(N)) -> O(M2), where M is the number of lmk to use per frame)
    timer.tic();

#if defined OBS_MIN_SVD
    mpVec->clear();
    // Collect the mpSorted
    for(int i= 0; i<N; i++)  {
        MapPoint* pMP = pFrame->mvpMapPoints[i];
        if(pMP) {
            if (pMP->ObsScore > 0) {
                GoodPoint tmpLmk(static_cast<size_t>(i), pMP->ObsScore);
                mpVec->push_back( tmpLmk );
            }
        }
    }
    // Sort features according to the tmprObs score
    std::sort(mpVec->begin(), mpVec->end(), obsLmkScoreDescend);
    // Erase features that are ranked after ratio_good_inlier
    size_t pivotPos  = static_cast<size_t>(num_good_inlier);
    if( mpVec->size() > pivotPos) {
        mpVec->erase(mpVec->begin() + pivotPos, mpVec->end());
    }
    time_Subset = timer.toc();
#ifdef OBS_TIMECOST_VERBOSE
    //
    std::cout << "func getObsScore_mt: time cost of collecting obs info & sorting = " << time_Subset << std::endl;
    //    std::cout << "func getObsScore_mt: threshold of obs score = " << mpVec->at(pivotPos).obs_score << std::endl;
    std::cout << "func setSelction_Number: cut the number of GF from " << mpVec->size() << " to " << pivotPos << "     " << std::endl;
#endif
    //#ifdef OBS_DEBUG_VERBOSE
    //    for (int i=0; i<std::min(size_t(10), mpVec->size()); ++i)
    //        std::cout << "Observability score = " << mpVec->at(i).obs_score << std::endl;
    //#endif

#elif defined OBS_MAXVOL_TOP_SV
    vector<GoodPoint> tmpVec;
    // Collect the mpSorted
    for(int i= 0; i<N; i++)  {
        MapPoint* pMP = pFrame->mvpMapPoints[i];
        if(pMP) {
            if (pMP->ObsScore > 0 && pMP->ObsMat.n_rows > 0) {
                GoodPoint tmpLmk(static_cast<size_t>(i), pMP->ObsScore, pMP->ObsVector);
                tmpVec.push_back( tmpLmk );
            }
        }
    }
    //
    std::cout << "func setSelction_Number: number of lmks feed into max-vol = " << tmpVec.size() << std::endl;

    //    // For Debug only
    //    // unnecessary in real task
    //    std::sort(tmpVec.begin(), tmpVec.end(), obsLmk_comparator);

    // Construct minimal-cardinal-maximum-volume subset
    mpVec->clear();
    // Erase features that are ranked after ratio_good_inlier
    //    size_t pivotPos  = std::max(min_inlier_num, static_cast<size_t>(static_cast<double>(tmpVec.size()) * ratio_good_inlier));
    //    size_t pivotPos  = static_cast<size_t>(num_good_inlier);
    maxVolSubset_Orig(&tmpVec, mpVec, num_good_inlier);

#elif defined OBS_MAXVOL_FULL

    vector<GoodPoint> tmpVec;
    // Collect the mpSorted
    for(int i= 0; i<N; i++)  {
        MapPoint* pMP = pFrame->mvpMapPoints[i];
        if(pMP) {
            if (pMP->ObsScore > 0 && pMP->ObsMat.n_rows > 0) {
                GoodPoint tmpLmk(static_cast<size_t>(i), pMP->ObsScore, pMP->ObsMat);
                tmpVec.push_back( tmpLmk );
            }
        }
    }
    //
    std::cout << "func setSelction_Number: number of lmks feed into max-vol = " << tmpVec.size() << std::endl;

    // Construct minimal-cardinal-maximum-volume subset
    mpVec->clear();
    //    size_t pivotPos  = static_cast<size_t>(static_cast<double>(tmpVec.size()) * ratio_good_inlier);
    //  size_t pivotPos = static_cast<size_t>(num_good_inlier);
    maxVolSubset_Full(&tmpVec, mpVec, num_good_inlier);

#elif defined OBS_MAXVOL_Greedy

    mpVec->clear();

    if (greedy_mtd == 1)
        maxVolSelection_BaselineGreedy(mpVec, num_good_inlier);
    else if (greedy_mtd == 2) {
        bool flagSucc = maxVolSelection_LazierGreedy(0, lmkSelectPool.size(), mpVec, num_good_inlier, random_sample_scale, time_for_select - time_MatBuild);
        if (flagSucc == false) {
            //            lmkSelectPool.clear();
            // update the good feature flag
            for (size_t i=0; i<mpVec->size(); ++i)
                pFrame->mvbGoodFeature[mpVec->at(i).idx] = true;
            time_Selection = timer.toc();
#ifdef OBS_TIMECOST_VERBOSE
            std::cout << "func setSelction_Number: time cost of finding max-vol subset = " << time_Selection << std::endl;
#endif
            return false;
        }
    }
    else if (greedy_mtd == 3){
        // simply call lazier greedy
        bool flagSucc = maxVolAutomatic_LazierGreedy(0, lmkSelectPool.size(), mpVec, num_good_inlier, random_sample_scale, time_for_select - time_MatBuild);
        if (flagSucc == false) {
            //                // fill the rest of mpVec with random lmks
            //                for (size_t i=0; i<N; ++i) {
            //                    //
            //                    if (mpVec->size() >= num_good_inlier)
            //                        break ;
            //                    if (lmkSelectPool[i].selected == false) {
            //                        lmkSelectPool[i].selected = true;
            //                        mpVec->push_back(lmkSelectPool[i]);
            //                    }
            //                }
            //                lmkSelectPool.clear();

            // update the good feature flag
            for (size_t i=0; i<mpVec->size(); ++i)
                pFrame->mvbGoodFeature[mpVec->at(i).idx] = true;
            time_Selection = timer.toc();
#ifdef OBS_TIMECOST_VERBOSE
            std::cout << "func setSelction_Number: time cost of finding max-vol subset = " << time_Selection << std::endl;
#endif
            return false;
        }
    }
    else if (greedy_mtd == 4) {
        //
        bool flagSucc = maxVolAutomatic_GroupedGreedy(0, lmkSelectPool.size(), mpVec, num_good_inlier, random_sample_scale, time_for_select - time_MatBuild);
        if (flagSucc == false) {
            //            lmkSelectPool.clear();
            // update the good feature flag
            for (size_t i=0; i<mpVec->size(); ++i)
                pFrame->mvbGoodFeature[mpVec->at(i).idx] = true;
            time_Selection = timer.toc();
#ifdef OBS_TIMECOST_VERBOSE
            std::cout << "func getObsScore_Number: time cost of finding max-vol subset = " << time_Selection << std::endl;
#endif
            return false;
        }
    }
    else
        std::cout << "func setSelction_Number: unknown greedy method being called!" << std::endl;
    //    std::cout << "func setSelction_Number: number of lmks extracted with max-vol = " << mpVec->size() << std::endl;

#endif

    // update the good feature flag
    for (size_t i=0; i<mpVec->size(); ++i)
        pFrame->mvbGoodFeature[mpVec->at(i).idx] = true;

    time_Selection = timer.toc();
#ifdef OBS_TIMECOST_VERBOSE
    std::cout << "func setSelction_Number: time cost of finding max-vol subset = " << time_Selection << std::endl;
#endif

    return true;
}


bool Observability::setSelction_Number(const size_t num_good_inlier,
                                       const int greedy_mtd,
                                       const double time_for_select,
                                       std::vector<MapPoint*> * mapPoints, vector<GoodPoint> *mpVec) {

    if (mapPoints == NULL || mpVec == NULL || time_for_select <= 0)
        return false;

    arma::wall_clock timer;

    mKineIdx = 1;
    mMapPoints = mapPoints;
    const int N = mMapPoints->size();
    //    double time_for_select = 0.020; // 999; //
    double random_sample_scale = 6.0; // 20.0; // 12.0; // 8.0; // 4.0; //

    //-----------------------------------------------------------------------------
    // Matrix construction for each feature (independently, therefore in parallel)
    runMatrixBuilding(ORB_SLAM2::MAP_INFO_MATRIX, time_for_select/2.0, true, true);

    //
    // TODO
    // move the info matrix part into parallel code block?
    //
    // Collect the mpSorted
    lmkSelectPool.clear();
    for(int i=0; i<N; i++)  {
        MapPoint* pMP = mMapPoints->at(i);
        if(pMP) {
            //            std::cout << pMP->ObsScore << std::endl;
            if (pMP->ObsScore >= 0 && pMP->ObsMat.n_rows > 0) {
                //
                GoodPoint tmpLmk(static_cast<size_t>(i), pMP->ObsScore, pMP->ObsMat);
                lmkSelectPool.push_back( tmpLmk );
            }
            //
        }
    }
    //    std::cout << "func setSelction_Number: number of local map points after visibility check = " << lmkSelectPool.size() << std::endl;

    time_MatBuild = timer.toc();
#ifdef OBS_TIMECOST_VERBOSE
    std::cout << "func setSelction_Number: time cost of multi-thread matrix construction = " << time_MatBuild << std::endl;
#endif

    //-----------------------------------------------------------------------------

    // The sorting process seem to take as much time as the parrallel obs computation
    // Next we will replace the sort part with Max-Basis-Volume approach (O(Nlog(N)) -> O(M2), where M is the number of lmk to use per frame)
    timer.tic();

    mpVec->clear();
    if (greedy_mtd == 1) {
        maxVolSelection_BaselineGreedy(mpVec, num_good_inlier);
    }
    else if (greedy_mtd == 2) {
        bool flagSucc = maxVolSelection_LazierGreedy(0, lmkSelectPool.size(), mpVec,
                                                     num_good_inlier, random_sample_scale, time_for_select - time_MatBuild);
        if (flagSucc == false) {
            //            lmkSelectPool.clear();
            time_Selection = timer.toc();
            return false;
        }
    }
    else if (greedy_mtd == 3){
        // instead of calling one thread only, here it automatically cut the load into multiple thread
        // per our simulation, lazier greedy performs consistent with a pool of around 100 features
        // therefore the intuition being, cut the tmpVec into multiple pool of 100, and summ up later on
        size_t threadNeeded = 1;
        if ( lmkSelectPool.size() - Greedy_Paral_RelFac * float(num_good_inlier) <= 10 ||
             lmkSelectPool.size() < 2 * Greedy_Paral_TrigSz)
            threadNeeded = 1;
        else {
            threadNeeded = static_cast<size_t>(round( float(lmkSelectPool.size()) / float(Greedy_Paral_TrigSz) ));
            if (threadNeeded > mNumThreads)
                threadNeeded = mNumThreads;
        }

        //        std::cout << "func getObsScore_Number: number of threads called for dist. greedy = " << threadNeeded << std::endl;
        //
        if (threadNeeded == 1) {
            // simply call lazier greedy
            bool flagSucc = maxVolAutomatic_LazierGreedy(0, lmkSelectPool.size(), mpVec,
                                                         num_good_inlier, random_sample_scale, time_for_select - time_MatBuild);
            if (flagSucc == false) {
                time_Selection = timer.toc();
#ifdef OBS_TIMECOST_VERBOSE
                std::cout << "func setSelction_Number: time cost of finding max-vol subset = " << time_Selection << std::endl;
#endif
                return false;
            }
        }
        else {
            //            arma::wall_clock timer_1;
            //            timer_1.tic();
            // multi-thread lazier greedy
            size_t numSubsetParal = static_cast<size_t>(ceil(float(num_good_inlier) / float(threadNeeded) * Greedy_Paral_RelFac));
            size_t numPoolParal = static_cast<size_t>(ceil(float(lmkSelectPool.size()) / float(threadNeeded)));
            //            cout << "numSubsetParal=" << numSubsetParal << "; numPoolParal=" << numPoolParal << endl;
            //            mThreads = new std::thread [threadNeeded-1];
            vector<GoodPoint> parVec[threadNeeded];
            for (size_t i = 0; i < threadNeeded; i++)
                parVec[i].clear();
            //
            mThreads = new std::thread [threadNeeded-1];
            for (size_t i = 0; i < threadNeeded-1; i++) {
                mThreads[i] = std::thread(&Observability::maxVolAutomatic_LazierGreedy, this,
                                          i*numPoolParal, (i+1)*numPoolParal,
                                          &(parVec[i]), numSubsetParal,
                                          random_sample_scale, (time_for_select - time_MatBuild) / 2);
            }
            //            threadArr[threadNeeded-1] = std::thread(&Observability::maxVolSelection_LazierGreedy, this,
            //                                                    &tmpVec, (threadNeeded-1)*numPoolParal, tmpVec.size(),
            //                                                    &(parVec[threadNeeded-1]), numSubsetParal);
            maxVolAutomatic_LazierGreedy((threadNeeded-1)*numPoolParal, lmkSelectPool.size(),
                                         &(parVec[threadNeeded-1]), numSubsetParal,
                    random_sample_scale, (time_for_select - time_MatBuild) / 2);

            for (size_t i = 0; i < threadNeeded-1; i++) {
                mThreads[i].join();
            }

            delete [] mThreads;
            lmkSelectPool.clear();
            for (size_t i = 0; i < threadNeeded; i++) {
                for (size_t j=0; j<(parVec[i]).size(); ++j) {
                    (parVec[i])[j].selected = false;
                    lmkSelectPool.push_back( (parVec[i])[j] );
                }
            }

            float time_so_far = timer.toc();
            //            std::cout << "func getObsScore_Number: time left for summary max-vol = " << time_for_select - time_MatBuild - time_so_far << std::endl;
            bool flagSucc = maxVolAutomatic_LazierGreedy(0, lmkSelectPool.size(), mpVec,
                                                         num_good_inlier, random_sample_scale, time_for_select - time_MatBuild - time_so_far);
            if (flagSucc == false) {
                time_Selection = timer.toc();
#ifdef OBS_TIMECOST_VERBOSE
                std::cout << "func setSelction_Number: time cost of finding max-vol subset = " << time_Selection << std::endl;
#endif
                return false;
            }
        }
    }
    else if (greedy_mtd == 4) {
        //
        size_t threadNeeded = 1;
        if ( lmkSelectPool.size() - Greedy_Paral_RelFac * float(num_good_inlier) <= 10 ||
             lmkSelectPool.size() < 2 * Greedy_Paral_TrigSz)
            threadNeeded = 1;
        else {
            threadNeeded = static_cast<size_t>(round( float(lmkSelectPool.size()) / float(Greedy_Paral_TrigSz) ));
            if (threadNeeded > mNumThreads)
                threadNeeded = mNumThreads;
        }

        //        std::cout << "func getObsScore_Number: number of threads called for dist. greedy = " << threadNeeded << std::endl;
        //
        if (threadNeeded == 1) {
            // simply call lazier greedy
            bool flagSucc = maxVolAutomatic_GroupedGreedy(0, lmkSelectPool.size(), mpVec,
                                                          num_good_inlier, random_sample_scale, time_for_select - time_MatBuild);
            if (flagSucc == false) {
                time_Selection = timer.toc();
                return false;
            }
        }
        else {
            //            arma::wall_clock timer_1;
            //            timer_1.tic();
            // multi-thread lazier greedy
            size_t numSubsetParal = static_cast<size_t>(ceil(float(num_good_inlier) / float(threadNeeded) * Greedy_Paral_RelFac));
            size_t numPoolParal = static_cast<size_t>(ceil(float(lmkSelectPool.size()) / float(threadNeeded)));
            //            cout << "numSubsetParal=" << numSubsetParal << "; numPoolParal=" << numPoolParal << endl;
            //            mThreads = new std::thread [threadNeeded-1];
            vector<GoodPoint> parVec[threadNeeded];
            for (size_t i = 0; i < threadNeeded; i++)
                parVec[i].clear();
            //
            mThreads = new std::thread [threadNeeded-1];
            for (size_t i = 0; i < threadNeeded-1; i++) {
                mThreads[i] = std::thread(&Observability::maxVolAutomatic_GroupedGreedy, this,
                                          i*numPoolParal, (i+1)*numPoolParal,
                                          &(parVec[i]), numSubsetParal,
                                          random_sample_scale, (time_for_select - time_MatBuild) / 2);
            }
            //            threadArr[threadNeeded-1] = std::thread(&Observability::maxVolSelection_LazierGreedy, this,
            //                                                    &tmpVec, (threadNeeded-1)*numPoolParal, tmpVec.size(),
            //                                                    &(parVec[threadNeeded-1]), numSubsetParal);
            maxVolAutomatic_GroupedGreedy((threadNeeded-1)*numPoolParal, lmkSelectPool.size(),
                                          &(parVec[threadNeeded-1]), numSubsetParal,
                    random_sample_scale, (time_for_select - time_MatBuild) / 2);

            for (size_t i = 0; i < threadNeeded-1; i++) {
                mThreads[i].join();
            }

            delete [] mThreads;
            lmkSelectPool.clear();
            for (size_t i = 0; i < threadNeeded; i++) {
                for (size_t j=0; j<(parVec[i]).size(); ++j) {
                    (parVec[i])[j].selected = false;
                    lmkSelectPool.push_back( (parVec[i])[j] );
                }
            }

            bool flagSucc = maxVolAutomatic_GroupedGreedy(0, lmkSelectPool.size(), mpVec,
                                                          num_good_inlier, random_sample_scale, (time_for_select - time_MatBuild) / 2);
            if (flagSucc == false) {
                time_Selection = timer.toc();
                return false;
            }
        }
    }
    else
        std::cout << "func setSelction_Number: unknown greedy method being called!" << std::endl;

    // lmkSelectPool.clear();
    //    std::cout << "func getObsScore_Number: number of lmks extracted with max-vol = " << mpVec->size() << std::endl;

    time_Selection = timer.toc();
#ifdef OBS_TIMECOST_VERBOSE
    std::cout << "func setSelction_Number: time cost of finding max-vol subset = " << time_Selection << std::endl;
#endif

    return true;
}

int Observability::runActiveMapMatching(Frame *pFrame,
                                        const size_t mat_type,
                                        const arma::mat &mBaseInfoMat,
                                        const float th,
                                        ORBmatcher &mORBMatcher,
                                        const int num_to_match,
                                        const double time_for_match ) {

    //    mLeftMapPoints.clear();
    if (pFrame == NULL || this->mMapPoints == NULL)
        return 0;
    if (mMapPoints->size() == 0 || num_to_match <= 0 || time_for_match <= 0) {
        //
        for (size_t i=0; i<mMapPoints->size(); ++i) {
            if (mMapPoints->at(i) == NULL)
                continue ;
            if (mMapPoints->at(i)->isBad())
                continue ;
            if (mMapPoints->at(i)->mbTrackInView == false)
                continue;
            //
            mLeftMapPoints.push_back(mMapPoints->at(i));
        }
        return 0;
    }

    double timeCap = 0;
    arma::wall_clock timer;
    timer.tic();

    // iteratively search for the most informative lmk
    arma::mat curMat = mBaseInfoMat;
    arma::mat H_disp;
    int thStereo = (ORBmatcher::TH_HIGH+ORBmatcher::TH_LOW)/2;
    double curCost = 0;

    // create a query index for fast insert and reject of entries
    arma::mat lmkIdx = arma::mat(1, mMapPoints->size());
    // create a flag array to avoid duplicate visit
    arma::mat lmkVisited = arma::mat(1, mMapPoints->size());
    size_t N = 0;
    for (size_t i=0; i<mMapPoints->size(); ++i) {
        if (mMapPoints->at(i) == NULL)
            continue ;
        if (mMapPoints->at(i)->isBad())
            continue ;
        if (mMapPoints->at(i)->mbTrackInView == false)
            continue;
        if (mMapPoints->at(i)->updateAtFrameId != pFrame->mnId)
            continue ;

        //
#ifdef INFORMATION_EFFICIENCY_SCORE
        mORBMatcher.GetCandidates(*pFrame, mMapPoints->at(i), th);
        if (mMapPoints->at(i)->mvMatchCandidates.empty())
            continue ;
#endif

        lmkIdx.at(0, N) = i;
        lmkVisited.at(0, N) = -1;
        ++ N;
    }
    lmkIdx.resize(1, N);
    lmkVisited.resize(1, N);
    //    std::cout << "func runActiveMapMatching: start active matching " << num_to_match
    //              << " lmks from " << N << " visible map points!" << std::endl;

    // define the size of random subset
    // in practice, multiplication factor 1.0 works better on many sequences
    // size_t szLazierSubset = static_cast<size_t>( float(N) / float(num_to_match) * 1.0 );
    // in theory, multiplication factor 2.3 is equivalant to decay factor of 0.1
    size_t szLazierSubset = static_cast<size_t>( float(N) / float(num_to_match) * 2.3 );

    int nMatched = 0;
    //
    while (nMatched < num_to_match) {
        //    for (size_t i=0; i<num_to_match; ++i) {
        //        int maxLmk = -1;
        //        double maxDet = -DBL_MAX;
        std::priority_queue<SimplePoint> heapSubset;
        std::vector<size_t> removeIdx;
#ifdef OBS_DEBUG_VERBOSE
        std::cout << "func matchMapPointActive: matching round " << nMatched << std::endl;
#endif
        size_t numHit = 0, numRndQue = 0, szActualSubset;
        szActualSubset = szLazierSubset;
        if (lmkIdx.n_cols < szActualSubset)
            szActualSubset = lmkIdx.n_cols;
        //
        //        std::srand(std::time(nullptr));
        while (numHit < szActualSubset) {
            // generate random query index
#ifdef OBS_DEBUG_VERBOSE
            cout << "random query round " << numHit << endl;
#endif
            size_t j;
            numRndQue = 0;
            while (numRndQue < MAX_RANDOM_QUERY_TIME) {
                j = ( std::rand() % lmkIdx.n_cols );
                //                cout << j << " ";
                // check if visited
                if (lmkVisited.at(0, j) < nMatched) {
                    lmkVisited.at(0, j) = nMatched;
                    break ;
                }
                ++ numRndQue;
            }
            if (numRndQue >= MAX_RANDOM_QUERY_TIME) {
                //                cout << "Failed to find a valid random map point to start with!" << endl;
                break ;
            }

            int queIdx = lmkIdx.at(0, j);
            ++ numHit;

            // apply cap on time cost
            timeCap = timer.toc();
            if (timeCap > time_for_match) {
                cout << "reach max time cap for active matching!" << timeCap << endl;
                return nMatched;
            }

            // else, the queried map point is updated in time, and being assessed with potential info gain
            double curDet = logDet( curMat + mMapPoints->at(queIdx)->ObsMat );
#ifdef INFORMATION_EFFICIENCY_SCORE
            heapSubset.push(SimplePoint(queIdx, curDet / double(curCost + mMapPoints->at(queIdx)->mvMatchCandidates.size())));
#else
            heapSubset.push(SimplePoint(queIdx, curDet));
#endif

            if (numHit >= szActualSubset) {
                // once filled the heap with random samples
                // start matching the one with highest logdet / info gain
                SimplePoint heapTop = heapSubset.top();
#ifdef OBS_DEBUG_VERBOSE
                cout << "heapTop logDet = " << heapTop.score << endl;
#endif

#ifdef INFORMATION_EFFICIENCY_SCORE
                int bestIdx = mORBMatcher.MatchCandidates(*pFrame, mMapPoints->at(heapTop.idx));
                curCost += mMapPoints->at(heapTop.idx)->mvMatchCandidates.size();
#else
                int bestIdx = mORBMatcher.SearchByProjection_OnePoint(*pFrame, mMapPoints->at(heapTop.idx), th);
#endif
                if (bestIdx >= 0) {
                    //                    std::cout << "Found match " << heapSubset.top().idx << std::endl;
                    // match succeed, move on to next batch random eval
                    //                    curMat = curMat + mMapPoints->at(heapTop.idx)->ObsMat;

                    if (mSensor == 1) {
#ifdef DELAYED_STEREO_MATCHING
                        //
                        cv::Mat Pw = mMapPoints->at(heapTop.idx)->GetWorldPos(), Pc;
                        //                        cout << "Pw = " << Pw.at<float>(0) << ", " << Pw.at<float>(1) << ", " << Pw.at<float>(2) << endl;
                        if (pFrame->WorldToCameraPoint(Pw, Pc) == true) {
                            //                            cout << "Pc = " << Pc.at<float>(0) << ", " << Pc.at<float>(1) << ", " << Pc.at<float>(2) << endl;
                            // check the range of disparity
                            float disp = float(pFrame->mbf) / Pc.at<float>(2);
                            float disp_min = std::max(disp - float(DISPARITY_THRES), 0.0f),
                                    disp_max = std::min(disp + float(DISPARITY_THRES), float(pFrame->mbf)/float(pFrame->mb));

                            // stereo matching with the narrowed disparity range
                            if (pFrame->ComputeStereoMatch_OnePoint(bestIdx, thStereo, disp_min, disp_max) == true) {
                                // grab the info term from right cam stereo match
                                // feature position
                                arma::rowvec Y = arma::zeros<arma::rowvec>(3);
                                cv::Mat featPos = mMapPoints->at(heapTop.idx)->GetWorldPos();
                                Y[0] = featPos.at<float>(0);
                                Y[1] = featPos.at<float>(1);
                                Y[2] = featPos.at<float>(2);
                                // add the disparity info term to curMat
                                compute_H_disparity_col (this->kinematic[this->mKineIdx].Xv, Y, H_disp);
                                //                        curMat = curMat + H_disp.t() * H_disp;
                                mMapPoints->at(heapTop.idx)->H_meas = arma::join_vert(mMapPoints->at(heapTop.idx)->H_meas, H_disp);

                                nMatched += 1;
                            }
                        }
#else
                        // grab the info term from right cam stereo match feature position
                        if (pFrame->mvDepth[bestIdx] >= 0) {
                            arma::rowvec Y = arma::zeros<arma::rowvec>(3);
                            cv::Mat featPos = mMapPoints->at(heapTop.idx)->GetWorldPos();
                            Y[0] = featPos.at<float>(0);
                            Y[1] = featPos.at<float>(1);
                            Y[2] = featPos.at<float>(2);
                            // add the disparity info term to curMat
                            compute_H_disparity_col (this->kinematic[this->mKineIdx].Xv, Y, H_disp);
                            mMapPoints->at(heapTop.idx)->H_meas = arma::join_vert(mMapPoints->at(heapTop.idx)->H_meas, H_disp);

                            nMatched += 1;
                        }
#endif
                    }
                    else if (mSensor == 2) {
                        // grab the info term from depth cam
                        if (pFrame->mvDepth[bestIdx] >= 0) {
                            arma::rowvec Y = arma::zeros<arma::rowvec>(3);
                            cv::Mat featPos = mMapPoints->at(heapTop.idx)->GetWorldPos();
                            Y[0] = featPos.at<float>(0);
                            Y[1] = featPos.at<float>(1);
                            Y[2] = featPos.at<float>(2);
                            // add the disparity info term to curMat
                            compute_H_disparity_col (this->kinematic[this->mKineIdx].Xv, Y, H_disp);
                            mMapPoints->at(heapTop.idx)->H_meas = arma::join_vert(mMapPoints->at(heapTop.idx)->H_meas, H_disp);

                            nMatched += 1;
                        }
                    }

                    // TODO
                    // update the info matrix with measurement info, e.g. ocl level, residual
                    float res_u = pFrame->mvKeysUn[bestIdx].pt.x - mMapPoints->at(heapTop.idx)->u_proj,
                            res_v = pFrame->mvKeysUn[bestIdx].pt.y - mMapPoints->at(heapTop.idx)->v_proj;
                    //                    pFrame->getProjectError(pFrame->mvpMapPoints[bestIdx], &(pFrame->mvKeysUn[bestIdx]), res_u, res_v);

                    arma::mat H_rw;
                    reWeightInfoMat( pFrame, bestIdx, mMapPoints->at(heapTop.idx),
                                     mMapPoints->at(heapTop.idx)->H_meas, res_u, res_v,
                                     mMapPoints->at(heapTop.idx)->H_proj, H_rw );
                    //                    std::cout << "H_rw = "  << H_rw << std::endl;

                    if (mat_type == ORB_SLAM2::FRAME_HYBRID_MATRIX || mat_type == ORB_SLAM2::MAP_HYBRID_MATRIX) {
                        //
                        arma::mat Hyb = arma::join_vert(H_rw, H_rw * this->kinematic[this->mKineIdx].F);
                        curMat = curMat + Hyb.t() * Hyb;
                    }
                    else if (mat_type == ORB_SLAM2::FRAME_INFO_MATRIX || mat_type == ORB_SLAM2::MAP_INFO_MATRIX) {
                        //
                        curMat = curMat + H_rw.t() * H_rw;
                    }
                    else {
                        // TODO
                    }

                    removeIdx.push_back(heapTop.idx);
                    nMatched += 2;
                    break ;
                }
                else {
                    // otherwise, remove the top one from map points, and re-sample a map points
                    //                    std::cout << "Failed to match " << heapSubset.top().idx << std::endl;
                    removeIdx.push_back(heapTop.idx);
                    heapSubset.pop();
                    -- numHit;
                }
            }
        }

        if (numRndQue >= MAX_RANDOM_QUERY_TIME || heapSubset.size() == 0 || removeIdx.size() == 0) {
            std::cout << "func runActiveMapMatching: early termination!" << std::endl;
            break ;
        }

        //        cout << "heap check: ";
        //        for (int j=0; j<heapSubset.size(); ++j) {
        //            cout << heapSubset.top().score << " ";
        //            heapSubset.pop();
        //        }
        //        cout << endl;

        if (lmkIdx.n_cols == removeIdx.size()) {
            std::cout << "func runActiveMapMatching: went through all map points!" << std::endl;
            break ;
        }

        // set up the index for columns that are not selected yet
        std::sort(removeIdx.begin(), removeIdx.end());
        arma::uvec restCol = arma::uvec(lmkIdx.n_cols-removeIdx.size());

#ifdef OBS_DEBUG_VERBOSE
        cout << "before clean entries!" << endl;
        cout << "removeIdx: " << endl;
        for (int j=0; j<removeIdx.size(); ++j)
            cout << removeIdx[j] << " " ;
        cout << endl;
        //        cout << "lmkIdx: " << endl;
        //        for (int j=0; j<lmkIdx.n_cols; ++j)
        //            cout << lmkIdx.at(0,j) << " " ;
        //        cout << endl;
        cout << "restCol: " << endl;
        cout << "size = " << lmkIdx.n_cols-removeIdx.size() << endl;
#endif

        size_t j = 0, k = 0, l = 0;
        while (j < lmkIdx.n_cols) {
            if (k >= removeIdx.size()) {
                restCol[l] = j;
                ++l;
                ++j;
            }
            else {
                if (lmkIdx.at(0,j) < removeIdx[k]) {
                    restCol[l] = j;
                    ++l;
                    ++j;
                }
                else {
                    if (lmkIdx.at(0,j) == removeIdx[k] ) {
                        ++j;
                    }
                    ++k;
                }
            }
        }
        lmkIdx = lmkIdx.cols(restCol);
        lmkVisited = lmkVisited.cols(restCol);

#ifdef OBS_DEBUG_VERBOSE
        cout << "after clean entries!" << endl;
        cout << "lmkIdx.n_cols = " << lmkIdx.n_cols << endl;
#endif
        //        // apply cap on time cost
        //        timeCap = timer.toc();
        //        if (timeCap > MAX_TIMECOST_SELECT) {
        //            //            cout << "reach max time cap for current greedy!" << timeCap << endl;
        //            //            break ;
        //            return false;
        //        }
    }

    //    std::cout << "func cutMaxVolSubset: time cost of assemble X" << time_Assemb
    //              << ", column norm = " << time_CNorm << ", append result = " << time_Append
    //              << ", set new cindex = " << time_Index << ", project column (appr) = " << time_Proj
    //              << ", remove column = " << time_Remov << std::endl;
    //#ifdef RANDOM_SHUFFLE_LAZIER_GREEDY
    //    delete rndEntry;
    //#endif
    //    mLeftMapPoints.clear();
    for (size_t i=0; i<lmkIdx.n_cols; ++i) {
        mLeftMapPoints.push_back(mMapPoints->at(lmkIdx.at(0, i)));
    }

    //    std::cout << "func matchMapPointActive: done with " << num_to_match << " iterations!" << std::endl;
    return nMatched;

}



int Observability::runBaselineMapMatching(Frame *pFrame,
                                          const size_t base_mtd,
                                          const float th,
                                          ORBmatcher &mORBMatcher,
                                          const int num_to_match,
                                          const double time_for_match ) {

    //    mLeftMapPoints.clear();
    if (pFrame == NULL || this->mMapPoints == NULL)
        return 0;
    if (mMapPoints->size() == 0 || num_to_match <= 0 || time_for_match <= 0) {
        //
        for (size_t i=0; i<mMapPoints->size(); ++i) {
            if (mMapPoints->at(i) == NULL)
                continue ;
            if (mMapPoints->at(i)->isBad())
                continue ;
            if (mMapPoints->at(i)->mbTrackInView == false)
                continue;
            //
            mLeftMapPoints.push_back(mMapPoints->at(i));
        }
        return 0;
    }

    double timeCap = 0;
    arma::wall_clock timer;
    timer.tic();

    int nMatched = 0;
    vector<GoodPoint> vldPoints;

    for (size_t i=0; i<mMapPoints->size(); ++i) {
        if (mMapPoints->at(i) == NULL)
            continue ;
        if (mMapPoints->at(i)->isBad())
            continue ;
        if (mMapPoints->at(i)->mbTrackInView == false)
            continue;
        //
        vldPoints.push_back(GoodPoint(i, mMapPoints->at(i)->mnVisible));
    }
    std::cout << "func runBaselineMapMatching: start active matching " << num_to_match
              << " lmks from " << vldPoints.size() << " visible map points!" << std::endl;

    //
    if (base_mtd == ORB_SLAM2::BASELINE_RANDOM) {
        // random shuffle
        std::random_shuffle ( vldPoints.begin(), vldPoints.end() );
    }
    else if (base_mtd == ORB_SLAM2::BASELINE_LONGLIVE) {
        // sort by life
        std::sort(vldPoints.begin(), vldPoints.end(), GoodPoint::rankObsScore_descend);
    }
    else {
        //
        std::cerr << "unknown baseline method being called!" << std::endl;
    }

    //
    int thStereo = (ORBmatcher::TH_HIGH+ORBmatcher::TH_LOW)/2;
    size_t i;
    for (i=0; i<vldPoints.size(); ++i) {

        // apply cap on time cost
        timeCap = timer.toc();
        if (timeCap > time_for_match) {
            cout << "reach max time cap for active matching!" << timeCap << endl;
            return nMatched;
        }

        if (nMatched >= num_to_match)
            break ;

        int bestIdx = mORBMatcher.SearchByProjection_OnePoint(*pFrame, mMapPoints->at(vldPoints[i].idx), th);
        if (bestIdx >= 0) {
            nMatched += 2;

            if (mSensor == 1) {
#ifdef DELAYED_STEREO_MATCHING
                //
                cv::Mat Pw = mMapPoints->at(vldPoints[i].idx)->GetWorldPos(), Pc;
                if (pFrame->WorldToCameraPoint(Pw, Pc) == true) {
                    // check the range of disparity
                    float disp = float(pFrame->mbf) / Pc.at<float>(2);
                    float disp_min = std::max(disp - float(DISPARITY_THRES), 0.0f),
                            disp_max = std::min(disp + float(DISPARITY_THRES), float(pFrame->mbf)/float(pFrame->mb));
                    // stereo matching with the narrowed disparity range
                    pFrame->ComputeStereoMatch_OnePoint(bestIdx, thStereo, disp_min, disp_max);
                }
#endif
            }

            if (pFrame->mvDepth[bestIdx] >= 0)
                nMatched += 1;
        }
    }

    //    mLeftMapPoints.clear();
    while (i<vldPoints.size()) {
        mLeftMapPoints.push_back(mMapPoints->at(vldPoints[i].idx));
        ++i;
    }

    //    std::cout << "func matchMapPointActive: done with " << num_to_match << " iterations!" << std::endl;
    return nMatched;

}


//
void Observability::maxVolSelection_Greedy_mt(vector<GoodPoint> *mpVec, vector<GoodPoint> *subVec, const size_t mpLimit) {
    //
    if (mpVec == nullptr || mpVec->size() == 0 || subVec == nullptr) {
        return ;
    }

    subVec->clear();

    if (mpLimit >= mpVec->size()) {
        // simply fill in all lmks as selected
        for (size_t i=0; i<mpVec->size(); ++i) {
            subVec->push_back(mpVec->at(i));
            mpVec->at(i).selected = true;
        }
        std::cout << "func maxVolSelection_Greedy_mt: subset limit higher than input lmk number!" << std::endl;
        return ;
    }

    // define the size of random subset
    size_t szLazierSubset = static_cast<size_t>( double(mpVec->size()) * 0.3 );
    baseInfoMat = arma::eye( size(mpVec->at(0).obs_block) ) * 0.00001;
    // create a query index for fast insert and reject of entries
    queryLmkIdx = arma::mat(1, mpVec->size());
    for (size_t i=0; i<mpVec->size(); ++i) {
        queryLmkIdx.at(0, i) = i;
    }
    //    std::cout << "get basic var ready for multi thread max vol!" << std::endl;
    //
    for (size_t i=0; i<mpLimit; ++i) {
        int maxLmk = -1;
        double maxDet = -DBL_MAX;
        //         std::cout << "iter " << i << std::endl;

        // generate a random permutation of query index pool
        rndEntryIdx.clear();
        for (size_t j=0; j<queryLmkIdx.n_cols; ++j)
            rndEntryIdx.push_back(j);
        std::random_shuffle ( rndEntryIdx.begin(), rndEntryIdx.end() );

        for (size_t j = 0; j < mNumThreads; j++) {
            maxEntryVal[j] = -DBL_MAX;
            maxEntryIdx[j] = -1;
        }

        int grainSize = static_cast<double>(szLazierSubset)/static_cast<double>(mNumThreads);
        if (mNumThreads > 1)
            mThreads = new std::thread [mNumThreads-1];
        for (size_t j = 0; j < mNumThreads-1; j++) {
            mThreads[j] = std::thread( &Observability::batchSearchMaxEntry, this,
                                       mpVec, j*grainSize, (j+1)*grainSize, j );
        }
        this->batchSearchMaxEntry( mpVec, (mNumThreads-1)*grainSize, szLazierSubset, mNumThreads-1 );

        for (size_t j = 0; j < mNumThreads-1; j++) {
            mThreads[j].join();
        }

        delete [] mThreads;

        //        std::cout << "finish multi thread part! " << std::endl;

        // collect the max entry from all threads
        for (size_t j = 0; j < mNumThreads; j++) {
            if (maxEntryVal[j] > maxDet) {
                maxDet = maxEntryVal[j];
                maxLmk = maxEntryIdx[j];
            }
        }

        if (maxLmk == -1) {
            std::cout << "func maxVolSubset_Greedy_mt: early termination!" << std::endl;
            break ;
        }

        // set up the index for columns that are not selected yet
        arma::uvec restCol = arma::uvec(queryLmkIdx.n_cols-1);
        size_t k = 0;
        for (size_t j=0; j<queryLmkIdx.n_cols; ++j) {
            if (queryLmkIdx.at(0,j) != maxLmk) {
                restCol[k] = j;
                k ++;
            }
        }

        // take the best lmk incrementaly
        baseInfoMat = baseInfoMat + mpVec->at(maxLmk).obs_block;
        subVec->push_back(mpVec->at(maxLmk));
        mpVec->at(maxLmk).selected = true;
        queryLmkIdx = queryLmkIdx.cols(restCol);
    }

    return ;
}

void Observability::batchSearchMaxEntry( const vector<GoodPoint> * lmkVec,
                                         const size_t stIdx, const size_t edIdx,
                                         const size_t resIdx ) {
    //
    for (size_t j = stIdx; j < edIdx; ++ j) {

        if (j >= queryLmkIdx.n_cols)
            break ;
        int queIdx = queryLmkIdx.at(0, rndEntryIdx[j]);

        if (lmkVec->at(queIdx).selected == true || lmkVec->at(queIdx).obs_score < 0) {
            std::cout << "It does not supposed to happen!" << std::endl;
            continue ;
        }
        //
#ifdef ARMA_LOGDET
        arma::cx_double curDet = arma::log_det( baseInfoMat + lmkVec->at(queIdx).obs_block );
        //#endif
        if (curDet.real() > maxEntryVal[resIdx]) {
            maxEntryVal[resIdx] = curDet.real();
            maxEntryIdx[resIdx] = queIdx;
        }
#else
        double curDet = logDet( baseInfoMat + lmkVec->at(queIdx).obs_block );
        if (curDet > maxEntryVal[resIdx]) {
            maxEntryVal[resIdx] = curDet;
            maxEntryIdx[resIdx] = queIdx;
        }
#endif
    }
}

bool Observability::maxVolDeletion_GroupedGreedy(const size_t stIdx, const size_t edIdx,
                                                 vector<GoodPoint> *subVec, const size_t mpLimit,
                                                 const double sampleScale,
                                                 const double max_time_for_select) {
    //
    if (lmkSelectPool.size() == 0 || subVec == nullptr) {
        return false;
    }
    if (stIdx >= edIdx || edIdx > lmkSelectPool.size()) {
        return false;
    }

    subVec->clear();

    if (mpLimit >= edIdx - stIdx) {
        // simply fill in all lmks as selected
        for (size_t i=stIdx; i<edIdx; ++i) {
#ifdef MULTI_THREAD_LOCK_ON
            mtx.lock();
#endif
            lmkSelectPool[i].selected = true;
            subVec->push_back(lmkSelectPool[i]);
#ifdef MULTI_THREAD_LOCK_ON
            mtx.unlock();
#endif
        }
        // std::cout << "func maxVolDeletion_LazierGreedy: subset limit higher than input lmk number!" << std::endl;
        return true;
    }

    //    std::cout << "Calling lazier greedy!" << std::endl;
    double timeCap = 0;
    arma::wall_clock timer;
    timer.tic();

    // define the size of random subset
    size_t szLazierSubset = static_cast<size_t>( double(edIdx - stIdx) / double(mpLimit) * sampleScale );
    // iteratively search for the most informative lmk
    arma::mat curMat = arma::eye( size(lmkSelectPool[0].obs_block) ) * 0.00001;

    size_t szGroup = 3; // 5; // 1; //
    size_t numGroup = std::ceil( float(edIdx - stIdx) / float(szGroup) );

    // create a query index for fast insert and reject of entries
    arma::mat groupIdx = arma::mat(szGroup, numGroup);
    // create a flag array to avoid duplicate visit
    arma::mat groupVisited = arma::mat(1, numGroup);

    // create random group of triplets
    std::vector<size_t> rndEntry;
    for (size_t j=stIdx; j<edIdx; ++j)
        rndEntry.push_back(j);
    std::random_shuffle ( rndEntry.begin(), rndEntry.end() );

    size_t l = 0;
    for (size_t i=0; i<rndEntry.size(); i+=szGroup) {
        //
        for (size_t j=0; j<szGroup; ++j) {
            if (i + j < rndEntry.size()) {
                groupIdx.at(j, l) = rndEntry[i+j];
#ifdef MULTI_THREAD_LOCK_ON
                mtx.lock();
#endif
                curMat = curMat + lmkSelectPool[rndEntry[i+j]].obs_block;
                lmkSelectPool[rndEntry[i+j]].selected = true;
#ifdef MULTI_THREAD_LOCK_ON
                mtx.unlock();
#endif
            }
            else
                groupIdx.at(j, l) = -1;
        }
        //
        groupVisited.at(0, l) = -1;
        ++ l;
    }

    //
    size_t mDelLim = edIdx - stIdx - mpLimit;
    for (size_t i = 0; i < std::floor(float(mDelLim)/float(szGroup)); ++ i) {
        arma::vec maxLmk = arma::ones(szGroup, 1) * (-1);
        double maxDet = -DBL_MAX;
        //        std::cout << "func maxVolSubset_Greedy: finding subset idx " << i << std::endl;

        size_t numHit = 0, numRndQue = 0, szActualSubset;
        szActualSubset = szLazierSubset;
        if (groupIdx.n_cols < szActualSubset)
            szActualSubset = groupIdx.n_cols;
        //
        while (numHit < szActualSubset) {
            // generate random query index
            //            cout << "random query round " << numHit << endl;
            size_t j;
            numRndQue = 0;
            while (numRndQue < MAX_RANDOM_QUERY_TIME) {
                j = ( std::rand() % groupIdx.n_cols );
                //                cout << j << " ";
                // check if visited
                if (groupVisited.at(0, j) < i) {
                    groupVisited.at(0, j) = i;
                    break ;
                }
                ++ numRndQue;
            }
            if (numRndQue >= MAX_RANDOM_QUERY_TIME)
                break ;

            //            cout << endl;
            arma::vec queIdx = groupIdx.col(j);
            ++ numHit;

            // apply cap on time cost
            timeCap = timer.toc();
            if (timeCap > max_time_for_select) {
                //            cout << "reach max time cap for current greedy!" << timeCap << endl;
                //            break ;
                // collect the valid lmk left
                for (size_t k=stIdx; k<edIdx; ++k) {
                    if (lmkSelectPool[k].selected == true) {
                        //
                        subVec->push_back(lmkSelectPool[k]);
                    }
                }

                return false;
            }

            arma::mat groupMat = arma::zeros( size(lmkSelectPool[0].obs_block) );
            for (size_t m=0; m<szGroup; ++m) {
                if (queIdx(m) >= 0) {
                    if (lmkSelectPool[queIdx(m)].selected == false || lmkSelectPool[queIdx(m)].obs_score < 0) {
                        // std::cout << "It does not supposed to happen!" << std::endl;
                        -- numHit;
                        continue ;
                    }

                    groupMat += lmkSelectPool[queIdx(m)].obs_block;
                }
            }
            //

#ifdef ARMA_LOGDET
            arma::cx_double curDet = arma::log_det( curMat - groupMat );
            if (curDet.real() > maxDet) {
                maxDet = curDet.real();
                maxLmk = queIdx;
                //                std::cout << "current determinant = " << curDet << std::endl;
            }
#else
            double curDet = logDet( curMat - groupMat );
            if (curDet > maxDet) {
                maxDet = curDet;
                maxLmk = queIdx;
            }
#endif
        }

        if (maxLmk(0) == -1) {
            std::cout << "func maxVolDeletion_GroupedGreedy: early termination!" << std::endl;
            break ;
        }

        // set up the index for columns that are not selected yet
        arma::uvec restCol = arma::uvec(groupIdx.n_cols-1);
        size_t k = 0;
        for (size_t j=0; j<groupIdx.n_cols; ++j) {
            if (groupIdx.at(0,j) != maxLmk(0)) {
                restCol[k] = j;
                k ++;
            }
        }

        // take the best lmk incrementaly
#ifdef MULTI_THREAD_LOCK_ON
        mtx.lock();
#endif
        for (size_t m=0; m<szGroup; ++m) {
            if (maxLmk(m) >= 0) {
                curMat = curMat - lmkSelectPool[maxLmk(m)].obs_block;
                lmkSelectPool[maxLmk(m)].selected = false;
            }
        }
#ifdef MULTI_THREAD_LOCK_ON
        mtx.unlock();
#endif
        //
        groupIdx = groupIdx.cols(restCol);
        groupVisited = groupVisited.cols(restCol);
    }

    // collect the valid lmk left
    for (size_t i=stIdx; i<edIdx; ++i) {
        if (lmkSelectPool[i].selected == true) {
            //
            subVec->push_back(lmkSelectPool[i]);
        }
    }

    return true;
}

bool Observability::maxVolDeletion_LazierGreedy(const size_t stIdx, const size_t edIdx,
                                                vector<GoodPoint> *subVec, const size_t mpLimit,
                                                const double sampleScale,
                                                const double max_time_for_select) {
    //    cout << "start lazier greedy from " << stIdx << " to " << edIdx << endl;
    //
    if (lmkSelectPool.size() == 0 || subVec == nullptr) {
        return false;
    }
    if (stIdx >= edIdx || edIdx > lmkSelectPool.size()) {
        return false;
    }

    subVec->clear();

    if (mpLimit >= edIdx - stIdx) {
        // simply fill in all lmks as selected
        for (size_t i=stIdx; i<edIdx; ++i) {
#ifdef MULTI_THREAD_LOCK_ON
            mtx.lock();
#endif
            lmkSelectPool[i].selected = true;
            subVec->push_back(lmkSelectPool[i]);
#ifdef MULTI_THREAD_LOCK_ON
            mtx.unlock();
#endif
        }
        // std::cout << "func maxVolDeletion_LazierGreedy: subset limit higher than input lmk number!" << std::endl;
        return true;
    }

    //    std::cout << "Calling lazier greedy!" << std::endl;
    double timeCap = 0;
    arma::wall_clock timer;
    timer.tic();

    // define the size of random subset
    //    double errorBound = 0.001; // 0.01; //
    size_t szLazierSubset = static_cast<size_t>( double(edIdx - stIdx) / double(mpLimit) * sampleScale );
    //
    // NOTE
    // To obtain stable subset results with random deletion, the size of random pool has to be increased;
    // as a consequence, it actually takes more time than the selection method.
    //
    //    size_t szLazierSubset = static_cast<size_t>( double(mpVec->size()) / double(mpLimit) * log(1.0 / errorBound) * 2);
    arma::mat curMat = arma::eye( size(lmkSelectPool[0].obs_block) ) * 0.00001;

    // iteratively search for the least informative lmk
    // create a query index for fast insert and reject of entries
    arma::mat lmkIdx = arma::mat(1, edIdx - stIdx);
    // create a flag array to avoid duplicate visit
    arma::mat lmkVisited = arma::mat(1, edIdx - stIdx);
    size_t l = 0;
    for (size_t i=stIdx; i<edIdx; ++i) {

#ifdef MULTI_THREAD_LOCK_ON
        mtx.lock();
#endif

        curMat = curMat + lmkSelectPool[i].obs_block;
        lmkSelectPool[i].selected = true;

#ifdef MULTI_THREAD_LOCK_ON
        mtx.unlock();
#endif
        //
        lmkIdx.at(0, l) = i;
        lmkVisited.at(0, l) = -1;
        //
        ++ l;
    }

    //
    size_t mDelLim = edIdx - stIdx - mpLimit;
    for (size_t i=0; i<mDelLim; ++i) {
        int maxLmk = -1;
        double maxDet = -DBL_MAX;
        //        std::cout << "func maxVolSubset_Greedy: finding subset idx " << i << std::endl;

#ifdef RANDOM_SHUFFLE_LAZIER_GREEDY
        // generate a random permutation of query index pool
        //        std::uniform_int_distribution<size_t> rndDistr(0, lmkIdx.n_cols-1);
        //        std::generate( rndEntry.begin(), rndEntry.end(), [&](){ return rndDistr(rndEngine); });
        std::vector<size_t> rndEntry;
        for (size_t j=0; j<lmkIdx.n_cols; ++j)
            rndEntry.push_back(j);
        std::random_shuffle ( rndEntry.begin(), rndEntry.end() );

        for (size_t j=0; j<szLazierSubset; ++j) {
            if (j >= lmkIdx.n_cols)
                break ;
            int queIdx = lmkIdx.at(0, rndEntry[j]);

#elif defined RANDOM_ACCESS_LAZIER_GREEDY

        size_t numHit = 0, numRndQue = 0, szActualSubset;
        szActualSubset = szLazierSubset;
        if (lmkIdx.n_cols < szActualSubset)
            szActualSubset = lmkIdx.n_cols;
        //
        while (numHit < szActualSubset) {
            // generate random query index
            //            cout << "random query round " << numHit << endl;
            size_t j;
            numRndQue = 0;
            while (numRndQue < MAX_RANDOM_QUERY_TIME) {
                j = ( std::rand() % lmkIdx.n_cols );
                //                cout << j << " ";
                // check if visited
                if (lmkVisited.at(0, j) < i) {
                    lmkVisited.at(0, j) = i;
                    break ;
                }
                ++ numRndQue;
            }
            if (numRndQue >= MAX_RANDOM_QUERY_TIME)
                break ;

            //            cout << endl;
            int queIdx = lmkIdx.at(0, j);
            ++ numHit;

#endif

            // apply cap on time cost
            timeCap = timer.toc();
            if (timeCap > max_time_for_select) {
                //            cout << "reach max time cap for current greedy!" << timeCap << endl;
                //            break ;
                // collect the valid lmk left
                for (size_t i=stIdx; i<edIdx; ++i) {
                    if (lmkSelectPool[i].selected == true) {
                        //
                        subVec->push_back(lmkSelectPool[i]);
                    }
                }

                return false;
            }

            if (lmkSelectPool[queIdx].selected == false || lmkSelectPool[queIdx].obs_score < 0) {
                // std::cout << "It does not supposed to happen!" << std::endl;
#if defined RANDOM_ACCESS_LAZIER_GREEDY
                -- numHit;
#endif
                continue ;
            }
            //
#ifdef ARMA_LOGDET
            arma::cx_double curDet = arma::log_det( curMat - lmkSelectPool[queIdx].obs_block );
            if (curDet.real() > maxDet) {
                maxDet = curDet.real();
                maxLmk = queIdx;
                //                std::cout << "current determinant = " << curDet << std::endl;
            }
#else
            double curDet = logDet( curMat - lmkSelectPool[queIdx].obs_block );
            if (curDet > maxDet) {
                maxDet = curDet;
                maxLmk = queIdx;
            }
#endif
        }

        if (maxLmk == -1) {
            std::cout << "func maxVolDeletion_LazierGreedy: early termination!" << std::endl;
            break ;
        }

        // set up the index for columns that are not selected yet
        arma::uvec restCol = arma::uvec(lmkIdx.n_cols-1);
        size_t k = 0;
        for (size_t j=0; j<lmkIdx.n_cols; ++j) {
            if (lmkIdx.at(0,j) != maxLmk) {
                restCol[k] = j;
                k ++;
            }
        }

        // take the best lmk incrementaly
#ifdef MULTI_THREAD_LOCK_ON
        mtx.lock();
#endif
        curMat = curMat - lmkSelectPool[maxLmk].obs_block;
        lmkSelectPool[maxLmk].selected = false;
#ifdef MULTI_THREAD_LOCK_ON
        mtx.unlock();
#endif
        //
        lmkIdx = lmkIdx.cols(restCol);
        lmkVisited = lmkVisited.cols(restCol);

    }

    // collect the valid lmk left
    for (size_t i=stIdx; i<edIdx; ++i) {
        if (lmkSelectPool[i].selected == true) {
            //
            subVec->push_back(lmkSelectPool[i]);
        }
    }

    return true;
}

void Observability::maxVolDeletion_BaselineGreedy(vector<GoodPoint> *mpVec, vector<GoodPoint> *subVec, const size_t mpLimit) {
    //
    if (mpVec == nullptr || mpVec->size() == 0 || subVec == nullptr) {
        return ;
    }

    //
    subVec->clear();
    if (mpLimit >= mpVec->size()) {
        // simply fill in all lmks as selected
        for (size_t i=0; i<mpVec->size(); ++i) {
            subVec->push_back(mpVec->at(i));
            mpVec->at(i).selected = true;
            pFrame->mvbGoodFeature[mpVec->at(i).idx] = true;
        }
        std::cout << "func maxVolDeletion_Greedy: subset limit higher than input lmk number!" << std::endl;
        return ;
    }

    // define the size of random subset
    // iteratively search for the least informative lmk
    // create a query index for fast insert and reject of entries
    arma::mat lmkIdx = arma::mat(1, mpVec->size());
    arma::mat curMat = arma::zeros( size(mpVec->at(0).obs_block) );
    for (size_t i=0; i<mpVec->size(); ++i) {
        curMat = curMat + mpVec->at(i).obs_block;
        lmkIdx.at(0, i) = i;
        mpVec->at(i).selected = true;
    }
    //
    size_t mDelLim = mpVec->size() - mpLimit;
    for (size_t i=0; i<mDelLim; ++i) {
        int maxLmk = -1;
        double maxDet = -DBL_MAX;

        //        for (int j=0; j<mpVec->size(); ++j) {
        for (size_t j=0; j<lmkIdx.n_cols; ++j) {
            int queIdx = lmkIdx.at(0, j);

            if (mpVec->at(queIdx).selected == false || mpVec->at(queIdx).obs_score < 0) {
                std::cout << "It does not supposed to happen!" << std::endl;
                continue ;
            }
            //
#ifdef ARMA_LOGDET
            arma::cx_double curDet = arma::log_det( curMat - mpVec->at(queIdx).obs_block );
            if (curDet.real() > maxDet) {
                maxDet = curDet.real();
                maxLmk = queIdx;
                //                std::cout << "current determinant = " << curDet << std::endl;
            }
#else
            double curDet = logDet( curMat - mpVec->at(queIdx).obs_block );
            if (curDet > maxDet) {
                maxDet = curDet;
                maxLmk = queIdx;
                //                std::cout << "current determinant = " << curDet << std::endl;
            }
#endif
        }

        if (maxLmk == -1) {
            std::cout << "func maxVolDeletion_Greedy: early termination!" << std::endl;
            break ;
        }

        // set up the index for columns that are not selected yet
        arma::uvec restCol = arma::uvec(lmkIdx.n_cols-1);
        size_t k = 0;
        for (size_t j=0; j<lmkIdx.n_cols; ++j) {
            if (lmkIdx.at(0,j) != maxLmk) {
                restCol[k] = j;
                k ++;
            }
        }

        // take the best lmk incrementaly
        curMat = curMat - mpVec->at(maxLmk).obs_block;
        mpVec->at(maxLmk).selected = false;
        lmkIdx = lmkIdx.cols(restCol);
    }

    // collect the valid lmk left
    for (size_t i=0; i<mpVec->size(); ++i) {
        if (mpVec->at(i).selected == true) {
            //
            subVec->push_back(mpVec->at(i));
        }
    }

    return ;
}


bool Observability::maxVolSelection_GroupedGreedy(const size_t stIdx, const size_t edIdx,
                                                  vector<GoodPoint> *subVec, const size_t mpLimit,
                                                  const double sampleScale,
                                                  const double max_time_for_select) {
    //
    if (lmkSelectPool.size() == 0 || subVec == nullptr) {
        return false;
    }
    if (stIdx >= edIdx || edIdx > lmkSelectPool.size() || max_time_for_select < 0) {
        return false;
    }

    subVec->clear();

    if (mpLimit >= edIdx - stIdx) {
        // simply fill in all lmks as selected
        for (size_t i=stIdx; i<edIdx; ++i) {
#ifdef MULTI_THREAD_LOCK_ON
            mtx.lock();
#endif
            lmkSelectPool[i].selected = true;
            subVec->push_back(lmkSelectPool[i]);
#ifdef MULTI_THREAD_LOCK_ON
            mtx.unlock();
#endif
        }
        //        std::cout << "func maxVolSelection_GroupedGreedy: subset limit higher than input lmk number!" << std::endl;
        return true;
    }

    //    std::cout << "Calling lazier greedy!" << std::endl;
    double timeCap = 0;
    arma::wall_clock timer;
    timer.tic();

    // define the size of random subset
    size_t szLazierSubset = static_cast<size_t>( double(edIdx - stIdx) / double(mpLimit) * sampleScale );
    // iteratively search for the most informative lmk
    arma::mat curMat = arma::eye( size(lmkSelectPool[0].obs_block) ) * 0.00001;

    size_t szGroup = 3; // 5; // 1; //
    size_t numGroup = std::ceil( float(edIdx - stIdx) / float(szGroup) );

    // create a query index for fast insert and reject of entries
    arma::mat groupIdx = arma::mat(szGroup, numGroup);
    // create a flag array to avoid duplicate visit
    arma::mat groupVisited = arma::mat(1, numGroup);

    // create random group of triplets
    std::vector<size_t> rndEntry;
    for (size_t j=stIdx; j<edIdx; ++j)
        rndEntry.push_back(j);
    std::random_shuffle ( rndEntry.begin(), rndEntry.end() );

    size_t l = 0;
    for (size_t i=0; i<rndEntry.size(); i+=szGroup) {
        //
        for (size_t j=0; j<szGroup; ++j) {
            if (i + j < rndEntry.size())
                groupIdx.at(j, l) = rndEntry[i+j];
            else
                groupIdx.at(j, l) = -1;
        }
        //
        groupVisited.at(0, l) = -1;
        ++ l;
    }

    //
    for (size_t i = 0; i < std::ceil(float(mpLimit)/float(szGroup)); ++ i) {
        arma::vec maxLmk = arma::ones(szGroup, 1) * (-1);
        double maxDet = -DBL_MAX;
        //        std::cout << "func maxVolSubset_Greedy: finding subset idx " << i << std::endl;

        size_t numHit = 0, numRndQue = 0, szActualSubset;
        szActualSubset = szLazierSubset;
        if (groupIdx.n_cols < szActualSubset)
            szActualSubset = groupIdx.n_cols;
        //
        //        std::srand(std::time(nullptr));
        while (numHit < szActualSubset) {
            // generate random query index
            //            cout << "random query round " << numHit << endl;
            size_t j;
            numRndQue = 0;
            while (numRndQue < MAX_RANDOM_QUERY_TIME) {
                j = ( std::rand() % groupIdx.n_cols );
                //                cout << j << " ";
                // check if visited
                if (groupVisited.at(0, j) < i) {
                    groupVisited.at(0, j) = i;
                    break ;
                }
                ++ numRndQue;
            }
            if (numRndQue >= MAX_RANDOM_QUERY_TIME)
                break ;

            //            cout << endl;
            arma::vec queIdx = groupIdx.col(j);
            ++ numHit;

            // apply cap on time cost
            timeCap = timer.toc();
            if (timeCap > max_time_for_select) {
                //            cout << "reach max time cap for current greedy!" << timeCap << endl;
                //            break ;
                return false;
            }

            arma::mat groupMat = arma::zeros( size(lmkSelectPool[0].obs_block) );
            for (size_t m=0; m<szGroup; ++m) {
                if (queIdx(m) >= 0) {
                    if (lmkSelectPool[queIdx(m)].selected == true || lmkSelectPool[queIdx(m)].obs_score < 0) {
                        // std::cout << "It does not supposed to happen!" << std::endl;
                        -- numHit;
                        continue ;
                    }

                    groupMat += lmkSelectPool[queIdx(m)].obs_block;
                }
            }
            //

#ifdef ARMA_LOGDET
            arma::cx_double curDet = arma::log_det( curMat + groupMat );
            if (curDet.real() > maxDet) {
                maxDet = curDet.real();
                maxLmk = queIdx;
                //                std::cout << "current determinant = " << curDet << std::endl;
            }
#else
            double curDet = logDet( curMat + groupMat );
            if (curDet > maxDet) {
                maxDet = curDet;
                maxLmk = queIdx;
            }
#endif
        }

        if (maxLmk(0) == -1) {
            std::cout << "func maxVolSelection_GroupedGreedy: early termination!" << std::endl;
            break ;
        }

        // set up the index for columns that are not selected yet
        arma::uvec restCol = arma::uvec(groupIdx.n_cols-1);
        size_t k = 0;
        for (size_t j=0; j<groupIdx.n_cols; ++j) {
            if (groupIdx.at(0,j) != maxLmk(0)) {
                restCol[k] = j;
                k ++;
            }
        }

        // take the best lmk incrementaly
#ifdef MULTI_THREAD_LOCK_ON
        mtx.lock();
#endif
        for (size_t m=0; m<szGroup; ++m) {
            if (maxLmk(m) >= 0) {
                curMat = curMat + lmkSelectPool[maxLmk(m)].obs_block;
                lmkSelectPool[maxLmk(m)].selected = true;
                subVec->push_back(lmkSelectPool[maxLmk(m)]);
            }
        }
#ifdef MULTI_THREAD_LOCK_ON
        mtx.unlock();
#endif
        //
        groupIdx = groupIdx.cols(restCol);
        groupVisited = groupVisited.cols(restCol);
    }

    return true;
}

bool Observability::maxVolSelection_LazierGreedy(const size_t stIdx, const size_t edIdx,
                                                 vector<GoodPoint> *subVec, const size_t mpLimit,
                                                 const double sampleScale,
                                                 const double max_time_for_select) {
    //        cout << "start lazier greedy from " << stIdx << " to " << edIdx << endl;
    //
    if (lmkSelectPool.size() == 0 || subVec == nullptr) {
        return false;
    }
    if (stIdx >= edIdx || edIdx > lmkSelectPool.size() || max_time_for_select < 0) {
        return false;
    }

    subVec->clear();

    if (mpLimit >= edIdx - stIdx) {
        // simply fill in all lmks as selected
        for (size_t i=stIdx; i<edIdx; ++i) {
#ifdef MULTI_THREAD_LOCK_ON
            mtx.lock();
#endif
            lmkSelectPool[i].selected = true;
            subVec->push_back(lmkSelectPool[i]);
#ifdef MULTI_THREAD_LOCK_ON
            mtx.unlock();
#endif
        }
        //        std::cout << "func maxVolSelection_LazierGreedy: subset limit higher than input lmk number!" << std::endl;
        return true;
    }

    //    std::cout << "Calling lazier greedy!" << std::endl;
    double timeCap = 0;
    arma::wall_clock timer;
    timer.tic();

    // define the size of random subset
    //    double errorBound = 0.001; // 0.005; // 0.01; // 0.0005; //
    size_t szLazierSubset = static_cast<size_t>( double(edIdx - stIdx) / double(mpLimit) * sampleScale );
    //        std::cout << "lazier subset size = " << szLazierSubset << std::endl;
    //    size_t szLazierSubset = static_cast<size_t>( double(mpVec->size()) * 0.3 );
    // for random permutation
    //    std::default_random_engine rndEngine; // or other engine as std::mt19937
    //    std::vector<size_t> rndEntry (szLazierSubset);

    // iteratively search for the most informative lmk
    arma::mat curMat = arma::eye( size(lmkSelectPool[0].obs_block) ) * 0.00001;

    // create a query index for fast insert and reject of entries
    arma::mat lmkIdx = arma::mat(1, edIdx - stIdx);
    // create a flag array to avoid duplicate visit
    arma::mat lmkVisited = arma::mat(1, edIdx - stIdx);
    size_t l = 0;
    for (size_t i=stIdx; i<edIdx; ++i) {
        lmkIdx.at(0, l) = i;
        lmkVisited.at(0, l) = -1;
        ++ l;
    }
    //
    for (size_t i=0; i<mpLimit; ++i) {
        int maxLmk = -1;
        double maxDet = -DBL_MAX;
        //        std::cout << "func maxVolSelection_LazierGreedy: finding subset idx " << i << std::endl;

#ifdef RANDOM_SHUFFLE_LAZIER_GREEDY
        // generate a random permutation of query index pool
        //        std::uniform_int_distribution<size_t> rndDistr(0, lmkIdx.n_cols-1);
        //        std::generate( rndEntry.begin(), rndEntry.end(), [&](){ return rndDistr(rndEngine); });
        std::vector<size_t> rndEntry;
        for (size_t j=0; j<lmkIdx.n_cols; ++j)
            rndEntry.push_back(j);
        std::random_shuffle ( rndEntry.begin(), rndEntry.end() );
        //        std::cout << "query index for lazier: ";
        //        for (size_t j=0; j<szLazierSubset; ++j)
        //            std::cout << rndEntry[j] << " ";
        //        std::cout << std::endl;

        for (size_t j=0; j<szLazierSubset; ++j) {
            if (j >= lmkIdx.n_cols)
                break ;
            int queIdx = lmkIdx.at(0, rndEntry[j]);

#elif defined RANDOM_ACCESS_LAZIER_GREEDY

        size_t numHit = 0, numRndQue = 0, szActualSubset;
        szActualSubset = szLazierSubset;
        if (lmkIdx.n_cols < szActualSubset)
            szActualSubset = lmkIdx.n_cols;
        //
        //        std::srand(std::time(nullptr));
        while (numHit < szActualSubset) {
            // generate random query index
            //            cout << "random query round " << numHit << endl;
            size_t j;
            numRndQue = 0;
            while (numRndQue < MAX_RANDOM_QUERY_TIME) {
                j = ( std::rand() % lmkIdx.n_cols );
                //                cout << j << " ";
                // check if visited
                if (lmkVisited.at(0, j) < i) {
                    lmkVisited.at(0, j) = i;
                    break ;
                }
                ++ numRndQue;
            }
            if (numRndQue >= MAX_RANDOM_QUERY_TIME)
                break ;

            //            cout << endl;
            int queIdx = lmkIdx.at(0, j);
            ++ numHit;

#else
        //        for (int j=0; j<mpVec->size(); ++j) {
        for (size_t j=0; j<lmkIdx.n_cols; ++j) {
            int queIdx = lmkIdx.at(0, j);
#endif


            // apply cap on time cost
            timeCap = timer.toc();
            if (timeCap > max_time_for_select) {
                //                cout << "reach max time cap for current greedy!" << timeCap << endl;
                //            break ;
                return false;
            }


            if (lmkSelectPool[queIdx].selected == true || lmkSelectPool[queIdx].obs_score < 0) {
                //                std::cout << "It does not supposed to happen: " << lmkSelectPool[queIdx].selected << "; "
                //                          <<  lmkSelectPool[queIdx].obs_score << std::endl;
#if defined RANDOM_ACCESS_LAZIER_GREEDY
                -- numHit;
#endif
                continue ;
            }
            //
            //#ifdef LAZY_GREEDY
            //            if (mpVec->at(queIdx).upper_bound < maxDet) {
            //                // early termination
            //                std::cout << "upper bound too small: " << mpVec->at(queIdx).upper_bound << " vs. " << maxDet << std::endl;
            //                break ;
            //            }
            //            arma::cx_double curDet = arma::log_det( mpVec->at(queIdx).sum_mat );
            //#else
            //            assert(curMat.size() == mpVec->at(j).obs_block.size());
            //            std::cout << "current obs_block = " << mpVec->at(j).obs_block << std::endl;
            //            double curDet = log10( arma::det( mpVec->at(j).sum_mat ) );
            //
            //            arma::cx_double curDet = arma::log_det( mpVec->at(queIdx).sum_mat );
#ifdef ARMA_LOGDET
            arma::cx_double curDet = arma::log_det( curMat + lmkSelectPool[queIdx].obs_block );
            //#endif
            if (curDet.real() > maxDet) {
                maxDet = curDet.real();
                maxLmk = queIdx;
                //                std::cout << "current determinant = " << curDet << std::endl;
            }
#else
            double curDet = logDet( curMat + lmkSelectPool[queIdx].obs_block );
            if (curDet > maxDet) {
                maxDet = curDet;
                maxLmk = queIdx;
            }
#endif
        }

        if (maxLmk == -1) {
            std::cout << "func maxVolSelection_LazierGreedy: early termination!" << std::endl;
            break ;
        }

        // set up the index for columns that are not selected yet
        arma::uvec restCol = arma::uvec(lmkIdx.n_cols-1);
        size_t k = 0;
        for (size_t j=0; j<lmkIdx.n_cols; ++j) {
            if (lmkIdx.at(0,j) != maxLmk) {
                restCol[k] = j;
                k ++;
            }
        }

        // take the best lmk incrementaly
#ifdef MULTI_THREAD_LOCK_ON
        mtx.lock();
#endif
        curMat = curMat + lmkSelectPool[maxLmk].obs_block;
        lmkSelectPool[maxLmk].selected = true;
        subVec->push_back(lmkSelectPool[maxLmk]);
#ifdef MULTI_THREAD_LOCK_ON
        mtx.unlock();
#endif
        //
        lmkIdx = lmkIdx.cols(restCol);
        lmkVisited = lmkVisited.cols(restCol);

        //        // apply cap on time cost
        //        timeCap = timer.toc();
        //        if (timeCap > MAX_TIMECOST_SELECT) {
        //            //            cout << "reach max time cap for current greedy!" << timeCap << endl;
        //            //            break ;
        //            return false;
        //        }
    }

    //    std::cout << "func cutMaxVolSubset: time cost of assemble X" << time_Assemb
    //              << ", column norm = " << time_CNorm << ", append result = " << time_Append
    //              << ", set new cindex = " << time_Index << ", project column (appr) = " << time_Proj
    //              << ", remove column = " << time_Remov << std::endl;
    //#ifdef RANDOM_SHUFFLE_LAZIER_GREEDY
    //    delete rndEntry;
    //#endif

    return true;
}

void Observability::maxVolSelection_BaselineGreedy(vector<GoodPoint> *subVec, const size_t mpLimit) {
    //
    if (lmkSelectPool.size() == 0 || subVec == nullptr) {
        return ;
    }

    subVec->clear();

    if (mpLimit >= lmkSelectPool.size()) {
        // simply fill in all lmks as selected
        for (size_t i=0; i<lmkSelectPool.size(); ++i) {
            lmkSelectPool[i].selected = true;
            subVec->push_back(lmkSelectPool[i]);
        }
        std::cout << "func maxVolSelection_BaselineGreedy: subset limit higher than input lmk number!" << std::endl;
        return ;
    }

    //    std::cout << "Calling baseline greedy!" << std::endl;

    // iteratively search for the most informative lmk
    arma::mat curMat = arma::eye( size(lmkSelectPool[0].obs_block) ) * 0.00001;

    // create a query index for fast insert and reject of entries
    //    arma::mat lmkIdx = arma::mat(1, lmkSelectPool.size());
    //    for (size_t i=0; i<lmkSelectPool.size(); ++i) {
    //        lmkIdx.at(0, i) = i;
    //    }
    //
    for (size_t i=0; i<mpLimit; ++i) {
        int maxLmk = -1;
        double maxDet = -DBL_MAX;
        //        std::cout << "func maxVolSubset_Greedy: finding subset idx " << i << std::endl;

        // estimate the upper bound of each lmk in mpVec
        for (size_t j=0; j<lmkSelectPool.size(); ++j) {
            if (lmkSelectPool[j].selected == true || lmkSelectPool[j].obs_score < 0) {
                lmkSelectPool[j].upper_bound = -DBL_MAX;
                continue ;
            }
            //
            lmkSelectPool[j].sum_mat = curMat + lmkSelectPool[j].obs_block;
            lmkSelectPool[j].upper_bound = arma::sum( arma::log( lmkSelectPool[j].sum_mat.diag() ) );
            //            std::cout << "current upper_bound = " << lmkSelectPool[j].upper_bound << std::endl;
        }

        // sort mpVec based on the upper bound of determinant
        std::sort(lmkSelectPool.begin(), lmkSelectPool.end(), GoodPoint::rankUpperBound_descend);

        for (int j=0; j<lmkSelectPool.size(); ++j) {
            int queIdx = j;
            //        for (size_t j=0; j<lmkIdx.n_cols; ++j) {
            //            int queIdx = lmkIdx.at(0, j);

            if (lmkSelectPool[queIdx].selected == true || lmkSelectPool[queIdx].obs_score < 0) {
                // std::cout << "It does not supposed to happen!" << std::endl;
                continue ;
            }
            //

            if (lmkSelectPool[queIdx].upper_bound < maxDet) {
                // early termination
                std::cout << "upper bound too small: " << lmkSelectPool[queIdx].upper_bound << " vs. " << maxDet << std::endl;
                break ;
            }

#ifdef ARMA_LOGDET
            arma::cx_double curDet = arma::log_det( lmkSelectPool[queIdx].sum_mat );
            //#endif
            if (curDet.real() > maxDet) {
                maxDet = curDet.real();
                maxLmk = queIdx;
                //                std::cout << "current determinant = " << curDet << std::endl;
            }
#else
            double curDet = logDet( lmkSelectPool[queIdx].sum_mat );
            if (curDet > maxDet) {
                maxDet = curDet;
                maxLmk = queIdx;
                //                std::cout << "current determinant = " << curDet << std::endl;
            }
#endif
        }

        if (maxLmk == -1) {
            std::cout << "func maxVolSelection_BaselineGreedy: early termination!" << std::endl;
            break ;
        }

        //        // set up the index for columns that are not selected yet
        //        arma::uvec restCol = arma::uvec(lmkIdx.n_cols-1);
        //        size_t k = 0;
        //        for (size_t j=0; j<lmkIdx.n_cols; ++j) {
        //            if (lmkIdx.at(0,j) != maxLmk) {
        //                restCol[k] = j;
        //                k ++;
        //            }
        //        }
        //        cout << maxLmk << "; " << lmkSelectPool[maxLmk].idx << endl;

        // take the best lmk incrementaly
        curMat = curMat + lmkSelectPool[maxLmk].obs_block;
        lmkSelectPool[maxLmk].selected = true;
        subVec->push_back(lmkSelectPool[maxLmk]);
        //        lmkIdx = lmkIdx.cols(restCol);
    }

    return ;
}

bool Observability::maxVolAutomatic_LazierGreedy(const size_t stIdx, const size_t edIdx,
                                                 vector<GoodPoint> *subVec, const size_t mpLimit,
                                                 const double sampleScale,
                                                 const double max_time_for_select) {
    if (mpLimit * 2 > edIdx - stIdx) {
        // deletion
        //        std::cout << "Deletion is chosen!" << std::endl;
        return maxVolDeletion_LazierGreedy(stIdx, edIdx, subVec, mpLimit, sampleScale, max_time_for_select);
    }
    else {
        // addition
        //        std::cout << "Selection is chosen!" << std::endl;
        return maxVolSelection_LazierGreedy(stIdx, edIdx, subVec, mpLimit, sampleScale, max_time_for_select);
    }
}

bool Observability::maxVolAutomatic_GroupedGreedy(const size_t stIdx, const size_t edIdx,
                                                  vector<GoodPoint> *subVec, const size_t mpLimit,
                                                  const double sampleScale,
                                                  const double max_time_for_select) {
    if (mpLimit * 2 > edIdx - stIdx) {
        // deletion
        //        std::cout << "Deletion is chosen!" << std::endl;
        return maxVolDeletion_GroupedGreedy(stIdx, edIdx, subVec, mpLimit, sampleScale, max_time_for_select);
    }
    else {
        // addition
        //        std::cout << "Selection is chosen!" << std::endl;
        return maxVolSelection_GroupedGreedy(stIdx, edIdx, subVec, mpLimit, sampleScale, max_time_for_select);
    }
}

void Observability::maxVolSubset_Orig(vector<GoodPoint> *mpVec, vector<GoodPoint> *subVec, const size_t mpLimit) {
    //
    if (mpVec == nullptr || mpVec->size() == 0 || subVec == nullptr) {
        return ;
    }

    //    double time_Assemb, time_CNorm = 0, time_Append = 0, time_Index = 0, time_Proj = 0, time_Remov = 0;
    //    arma::wall_clock timer;
    //    timer.tic();
    // assemble the matrix in which each column from one lmk
    arma::mat X = arma::mat(DIMENSION_OF_STATE_MODEL, mpVec->size());
    arma::mat lmkIdx = arma::mat(1, mpVec->size());
    for (size_t i=0; i<mpVec->size(); ++i) {
        X.col(i) = mpVec->at(i).obs_vector * mpVec->at(i).obs_score;
        lmkIdx.at(0, i) = i;
    }
    //    time_Assemb = timer.toc();
    //    std::cout << "func cutMaxVolSubset: X cols = " << X.n_cols << ", X rows = " << X.n_rows
    //              << ", lmkIdx cols = " << lmkIdx.n_cols  << ", lmkIdx rows = " << lmkIdx.n_rows << std::endl;

    vector<double> cNormVec;
    // iteratively search for the column with largest norm
    while (subVec->size() < mpLimit) {
        cNormVec.clear();
        //        timer.tic();
        // get the norm from each col
        double cnorm_max = -1;
        size_t idx_max = 0;
        for (size_t i=0; i<X.n_cols; ++i) {
            double cnorm_tmp = arma::norm(X.col(i), 2);
            if (cnorm_tmp > cnorm_max) {
                cnorm_max = cnorm_tmp;
                idx_max = i;
            }
            //
            cNormVec.push_back(cnorm_tmp);
        }
        if (cnorm_max < EPS) {
            // early terminate when norm of column become too small
            std::cout << "func maxVolSubset_Orig: early termination; column norm too small!" << std::endl;
            break ;
        }
        //        time_CNorm += timer.toc();
        //        std::cout << "func cutMaxVolSubset: max(X(:,i)) = " << X.col(idx_max) << " with i = " << idx_max << std::endl;

        //        arma::vec pi_Xv_norm = X.col(idx_max) / cnorm_max;

        //        timer.tic();
        // append selected vec into the subset
        size_t app_idx = lmkIdx.at(0, idx_max);
        subVec->push_back(mpVec->at(app_idx));
        mpVec->at(app_idx).selected = true;
        //        time_Append += timer.toc();

        // remove the projection of selected column from X
        if (X.n_cols > 1) {

            //            timer.tic();
            // set up the index for columns that are not selected yet
            arma::uvec restCol = arma::uvec(X.n_cols-1);
            size_t i = 0;
            for (size_t j=0; j<X.n_cols; ++j) {
                if (j != idx_max) {
                    restCol[i] = j;
                    i ++;
                }
            }
            //            time_Index += timer.toc();
            //            std::cout << "func cutMaxVolSubset: restCol = [" << restCol(0) << ", " << restCol(X.n_cols-2) << "]" << std::endl;

            //            timer.tic();
            //            // project the selected column into the space defined by unselected part of the matrix
            //            arma::vec pi_Xv = X.cols(restCol) * arma::pinv(X.cols(restCol)) * X.col(idx_max);
            //            time_Proj += timer.toc();

            //            timer.tic();
            // NOTE
            // the first 2 implementations in the following are WRONG!
            // when taking column/row from the pool, the PROJECTION of that column/row with on each single column/row rest
            // should be solved and deducted independently!
#if defined PINV_STANDARD
            // arma::vec pi_Xv = X.cols(restCol) * (X.cols(restCol).t() * arma::inv(X.cols(restCol) * X.cols(restCol).t())) * X.col(idx_max);
            arma::vec pi_Xv = X.cols(restCol) * arma::pinv(X.cols(restCol)) * X.col(idx_max);
            X.each_col() -= pi_Xv;
#elif defined PINV_FAST
            arma::vec pi_Xv = X.col(idx_max);
            X.each_col() -= pi_Xv;
#elif defined PINV_FAST_PERP
            //            X.each_col() -= arma::dot(X.each_col(), pi_Xv_norm) * pi_Xv_norm;
            for (size_t j=0; j<X.n_cols; ++j) {
                //                double cnorm_j = arma::norm(X.col(j), 2);
                //                arma::vec pj_Xv_norm = X.col(j) / cnorm_j;
                arma::vec pj_Xv_norm = X.col(j) / cNormVec[j];
                X.col(j) -= arma::dot(X.col(idx_max), pj_Xv_norm) * pj_Xv_norm;
                //                X.col(j) -= arma::dot(X.col(j), pi_Xv_norm) * pi_Xv_norm;
            }
#endif

            //            time_Proj += timer.toc();
            //            std::cout << "func cutMaxVolSubset: diff(pi_Xv) = " << arma::sum(arma::abs(pi_Xv - pi_Xv_1)) << std::endl;

            //            timer.tic();
            // remove the selected column from X & lmkIdx
            X = X.cols(restCol);
            lmkIdx = lmkIdx.cols(restCol);
            //            time_Remov += timer.toc();
            //            std::cout << "func cutMaxVolSubset: update size of X = " << X.n_cols << ", and size of lmkIdx = " << lmkIdx.n_cols << std::endl;
        }
        else {
            // terminate when the only column of X shall be removed here
            std::cout << "func maxVolSubset_Orig: early termination; went through all columns!" << std::endl;
            break ;
        }
    }

    //    std::cout << "func cutMaxVolSubset: time cost of assemble X" << time_Assemb
    //              << ", column norm = " << time_CNorm << ", append result = " << time_Append
    //              << ", set new cindex = " << time_Index << ", project column (appr) = " << time_Proj
    //              << ", remove column = " << time_Remov << std::endl;

    return ;
}

void Observability::maxVolSubset_Full(vector<GoodPoint> *mpVec, vector<GoodPoint> *subVec, const size_t mpLimit) {
    //
    if (mpVec == nullptr || mpVec->size() == 0 || subVec == nullptr) {
        return ;
    }

    // assemble the matrix in which each column from one lmk
    arma::mat X = arma::mat(DIMENSION_OF_STATE_MODEL, 2*mpVec->size());
    arma::mat lmkIdx = arma::mat(1, 2*mpVec->size());
    for (size_t i=0; i<mpVec->size(); ++i) {
        X.cols(2*i, 2*i+1) = mpVec->at(i).obs_block.t();
        lmkIdx.at(0, 2*i) = i;
        lmkIdx.at(0, 2*i+1) = i;
    }
    //    std::cout << "lmkIdx = " << lmkIdx << std::endl;

    // iteratively search for the column with largest norm
    while (subVec->size() < mpLimit) {
        //        timer.tic();
        // get the norm from each col
        double cnorm_max = -1;
        size_t idx_max = 0;
        for (size_t i=0; i<X.n_cols; ++i) {
            double cnorm_tmp = arma::norm(X.col(i), 2);
            if (cnorm_tmp > cnorm_max) {
                cnorm_max = cnorm_tmp;
                idx_max = i;
            }
        }
        if (cnorm_max < EPS) {
            // early terminate when norm of column become too small
            std::cout << "func maxVolSubset_Full: early termination; column norm too small!" << std::endl;
            break ;
        }
        //        arma::vec pi_Xv_norm = X.col(idx_max) / cnorm_max;

        // append selected vec into the subset
        size_t app_idx = lmkIdx.at(0, idx_max);
        subVec->push_back(mpVec->at(app_idx));
        mpVec->at(app_idx).selected = true;

        size_t idx_row1, idx_row2;
        if (idx_max%2 == 0) {
            idx_row1 = idx_max;
            idx_row2 = idx_max + 1;
        }
        else {
            idx_row1 = idx_max - 1;
            idx_row2 = idx_max;
        }
        //        std::cout << "idx_row1 = " << idx_row1 << std::endl;
        //        std::cout << "idx_row2 = " << idx_row2 << std::endl;

        // remove the projection of selected column from X
        if (X.n_cols > 1) {

            //            timer.tic();
            // set up the index for columns that are not selected yet
            arma::uvec restCol = arma::uvec(X.n_cols-2);
            size_t i = 0;
            for (size_t j=0; j<lmkIdx.n_cols; ++j) {
                if (lmkIdx.at(0, j) != app_idx) {
                    restCol[i] = j;
                    i ++;
                }
            }

            for (size_t j=0; j<X.n_cols; ++j) {
                arma::vec pj_Xv_norm = X.col(j) / arma::norm(X.col(j), 2);
                X.col(j) -= arma::dot(X.col(idx_row1), pj_Xv_norm) * pj_Xv_norm;
                //                if (j == idx_row1 || j == idx_row2)
                //                    std::cout << "norm of removed column = " << arma::norm(X.col(j), 2) << std::endl;
                //
                pj_Xv_norm = X.col(j) / arma::norm(X.col(j), 2);
                X.col(j) -= arma::dot(X.col(idx_row2), pj_Xv_norm) * pj_Xv_norm;
                //                if (j == idx_row1 || j == idx_row2)
                //                    std::cout << "norm of removed column = " << arma::norm(X.col(j), 2) << std::endl;
            }

            //            timer.tic();
            // remove the selected column from X & lmkIdx
            X = X.cols(restCol);
            lmkIdx = lmkIdx.cols(restCol);
        }
        else {
            // terminate when the only column of X shall be removed here
            std::cout << "func maxVolSubset_Full: early termination; went through all columns!" << std::endl;
            break ;
        }
    }
    return ;
}

}
