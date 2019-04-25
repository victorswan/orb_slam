#ifndef UTIL_HPP
#define UTIL_HPP

#include<vector>
#include<algorithm>

#define ARMA_NO_DEBUG
#include "armadillo"

#include <opencv2/core/core.hpp>
//
// for opencv 2
//
// #include <opencv2/nonfree/features2d.hpp>
// #include <opencv2/nonfree/nonfree.hpp>
//
// for opencv 3
//
#include <opencv2/features2d.hpp>

#define N_MAX_ITER_SVD                  72881	/* Rosser's matrix converges (tol=1e-16) */

namespace ORB_SLAM2 {

//-------------------------------
class SimplePoint {
public:
    SimplePoint(const int& idx_, const double& score_) {
        this->idx = idx_;
        this->score = score_;
    }

    bool operator<(const SimplePoint& right) const              //overloaded < operator
    {
        return this->score < right.score;
    }

    int idx;
    double score;
};


class GoodPoint {
public:
    // simplified version of construction func
    GoodPoint(const size_t& idx, const double& obs_score) {
        this->idx = idx;
        this->obs_score = obs_score;
        this->selected = false;
    }

    GoodPoint(const size_t& idx, const double& obs_score, const arma::vec& obs_vector) {
        this->idx = idx;
        this->obs_score = obs_score;
        this->obs_vector = obs_vector;
        this->selected = false;
    }


    GoodPoint(const size_t& idx, const double& obs_score, const arma::mat& obs_matrix) {
        this->idx = idx;
        this->obs_score = obs_score;
        this->obs_block = obs_matrix;
        this->selected = false;
        this->upper_bound = -DBL_MAX;
    }


    GoodPoint(const size_t& idx, const double& obs_score, const cv::Mat& wPt, const cv::KeyPoint& iPt) {
        this->idx = idx;
        this->obs_score = obs_score;
        //
        this->pW[0] = wPt.at<float>(0);
        this->pW[1] = wPt.at<float>(1);
        this->pW[2] = wPt.at<float>(2);
        //
        this->pI[0] = iPt.pt.x;
        this->pI[1] = iPt.pt.y;
    }

    //
    size_t idx;
    double obs_score;
    arma::vec obs_vector;
    arma::mat obs_block;
    // for acceleration
    arma::mat sum_mat;
    double upper_bound;
    //
    float pI[2];
    float pW[3];
    bool selected;


    static bool rankObsScore_descend(const GoodPoint & t1,  const GoodPoint & t2) {
        return t1.obs_score > t2.obs_score;
    }

    static bool rankObsScore_ascend(const GoodPoint & t1,  const GoodPoint & t2) {
        return t1.obs_score < t2.obs_score;
    }

    static bool rankUpperBound_descend(const GoodPoint & t1,  const GoodPoint & t2) {
        return t1.upper_bound > t2.upper_bound;
    }

};

class PinHoleCamera {
public:
    // TODO: change nRows and nCols to int?
    PinHoleCamera(double _f, size_t _nRows, size_t _nCols, double _Cx, double _Cy,
                  double _k1, double _k2, double _dx, double _dy):
        nRows(_nRows), nCols(_nCols), Cx(_Cx), Cy(_Cy), k1(_k1), k2(_k2), dx(_dx), dy(_dy)  {

        fu = _f / _dx;
        fv = _f / _dy;

        K  << fu << 0.0 << _Cx << arma::endr
           << 0.0 << fv << _Cy << arma::endr
           << 0.0 << 0.0 <<  1.0 << arma::endr;
    }


    PinHoleCamera(double _fu, double _fv, int _nRows, int _nCols, double _Cx, double _Cy,
                  double _k1, double _k2):
        fu(_fu), fv(_fv), nRows(_nRows), nCols(_nCols), Cx(_Cx), Cy(_Cy), k1(_k1), k2(_k2) {

        dx = 0.0112;
        dy = _fu * dx / _fv;

        K  << _fu << 0.0 << _Cx << arma::endr
           << 0.0 << _fv << _Cy << arma::endr
           << 0.0 << 0.0 <<  1.0 << arma::endr;
    }


    double fu, fv;
    int nRows, nCols;
    double Cx, Cy;
    // simplified version of radial distortion model
    double k1, k2;
    double dx, dy;
    arma::mat K;
    //
    double bf;
};

struct KineStruct
{
    double dt, dt_inSeg;
    arma::mat F_Q, F_Omg, F;
    arma::mat F_Q_inSeg, F_Omg_inSeg, F_inSeg;
    arma::rowvec Xv;
    cv::Mat Tcw;
    //
    arma::rowvec Xv_right;
    cv::Mat Tcw_right;
};

// Structure for reporting time cost per module
class TimeLog {
public:
    //
    void setZero() {
        frame_time_stamp = 0;
        time_rectification = 0;
        time_ORB_extraction = 0;
        time_track_motion = 0;
        time_track_frame = 0;
        time_track_map = 0;
        //
        time_stereo_motion = 0;
        time_stereo_frame = 0;
        time_stereo_map = 0;
        time_stereo_post = 0;
        //
        time_match = 0;
        time_select = 0;
        time_select_Mat = 0;
        time_select_Sel = 0;
        time_optim = 0;
        time_re_optim = 0;
        time_mat_online = 0;
        //
        time_create_kf = 0;
        time_update_motion = 0;
        time_post_proc = 0;
        //
        lmk_num_motion = 0;
        lmk_num_frame = 0;
        lmk_num_map = 0;
        lmk_num_BA = 0;
        //
        log_det_frame = 0;
        // obselet
        lmk_num_good = 0;
        lmk_num_inlier_good = 0;
    }

    double frame_time_stamp;
    // stereo rectification
    double time_rectification;
    // feature extraction
    double time_ORB_extraction;
    // track motion
    double time_track_motion;
    double time_track_frame;
    // track map
    double time_track_map;
    double time_match;
    double time_select;
    // for stereo only
    double time_stereo_motion;
    double time_stereo_frame;
    double time_stereo_map;
    double time_stereo_post;
    //    double time_select_SVD;
    //    double time_select_Subset;
    double time_select_Mat;
    double time_select_Sel;
    double time_optim;
    double time_re_optim;
    //
    // matrix building
    double time_mat_online;
    //
    double time_create_kf;
    double time_update_motion;
    double time_post_proc;
    //
    double lmk_num_motion;
    double lmk_num_frame;
    double lmk_num_map;
    double lmk_num_BA;
    //
    double log_det_frame;
    // obselet
    double lmk_num_good;
    double lmk_num_inlier_good;
};

class FramePose {
public:
    FramePose(const double &time_stamp, const cv::Mat &homm) {
        //        cout << "create frame pose obj" << endl;
        this->time_stamp = time_stamp;
        //        cout << "input pose matrix(0,3) " << homm.at<float>(0,3) << endl;
        homm.copyTo(this->homm);
        //        cout << "finish frame pose creation" << endl;
    }

    //
    double time_stamp;
    cv::Mat homm;
};

// Lmk life log
class LmkLog {
public:
  LmkLog(const int &id_, const int &life_) {
    id = id_;
    life = life_;
  }

  int id;
  int life;
};

//
// inlier functions that support quaternion manipulation
//
inline arma::rowvec qconj(arma::rowvec q) {

    q = -1.0 * q;
    q[0] = -q[0];
    return q;
}

inline arma::mat q2r(const arma::rowvec& q) {

    double x, y, z, r;
    x = q[1];
    y = q[2];
    z = q[3];
    r = q[0];

    arma::mat R;
    R << r*r+x*x-y*y-z*z << 2.0*(x*y -r*z)  <<  2.0*(z*x+r*y) << arma::endr
      << 2.0*(x*y+r*z)   << r*r-x*x+y*y-z*z <<  2.0*(y*z-r*x) << arma::endr
      << 2.0*(z*x-r*y)   << 2.0*(y*z+r*x)   <<  r*r-x*x-y*y+z*z << arma::endr;

    return R;
}

inline arma::rowvec v2q(const arma::rowvec& v) {

    //    double a = sqrt(dot(v, v));
    double a = arma::norm(v, 2);

    arma::rowvec u;
    if (a > 0.0000001) {
        u = v / a;
    }
    else {
        a = 0;
        u << 0 << 0 << 0 << arma::endr;
    }

    arma::rowvec q;
    q << cos(a/2) << u[0] * sin(a/2) << u[1] * sin(a/2) << u[2] * sin(a/2) << arma::endr;

    return q;
}

inline arma::rowvec qProd(const arma::rowvec& q1, const arma::rowvec& q2) {

    double a, b, c, d;
    a = q1[0];
    b = q1[1];
    c = q1[2];
    d = q1[3];

    double w, x, y, z;
    w = q2[0];
    x = q2[1];
    y = q2[2];
    z = q2[3];

    arma::rowvec qr;
    qr << a*w - b*x - c*y - d*z
       << a*x + b*w + c*z - d*y
       << a*y - b*z + c*w + d*x
       << a*z + b*y - c*x + d*w << arma::endr;

    return qr;
}

inline arma::rowvec qNormalize(const arma::rowvec& q) {

    double qNorm = arma::norm(q, 2);
    arma::rowvec qn = q / qNorm;
    return qn;
}

inline double dq0_by_domegaA(const double & omegaA, const double & omega, const double & delta_t){

    return (-delta_t / 2.0) * (omegaA / omega) * std::sin(omega * delta_t / 2.0);
}

inline double dqA_by_domegaA(const double & omegaA, const double & omega, const double & delta_t) {

    double result = (delta_t / 2.0) * omegaA * omegaA / (omega * omega)
            * std::cos(omega * delta_t / 2.0)
            + (1.0 / omega) * (1.0 - omegaA * omegaA / (omega * omega))
            * std::sin(omega * delta_t / 2.0);
    return result;
}

inline double dqA_by_domegaB(const double & omegaA, const double & omegaB,
                             const double & omega,  const double & delta_t) {

    double result = (omegaA * omegaB / (omega * omega)) *
            ( (delta_t / 2.0) * std::cos(omega * delta_t / 2.0)
              - (1.0 / omega) * std::sin(omega * delta_t / 2.0) );
    return result;
}

inline arma::mat dRq_times_a_by_dq(const arma::rowvec & q,
                                   const arma::rowvec & aMat) {
#ifndef ARMA_NO_DEBUG
    //    assert(aMat.n_rows == 3 && aMat.n_cols == 1);
    assert(aMat.n_cols == 3);
#endif

    double q0 = q[0];
    double qx = q[1];
    double qy = q[2];
    double qz = q[3];

    arma::mat dR_by_dq0(3,3), dR_by_dqx(3,3), dR_by_dqy(3,3), dR_by_dqz(3,3);
    //    dR_by_dq0 << 2.0*q0 << -2.0*qz << 2.0*qy << arma::endr
    //              << 2.0*qz << 2.0*q0  << -2.0*qx << arma::endr
    //              << -2.0*qy << 2.0*qx << 2.0*q0 << arma::endr;
    dR_by_dq0 = { {2.0*q0, -2.0*qz, 2.0*qy},
                  {2.0*qz, 2.0*q0, -2.0*qx},
                  {-2.0*qy, 2.0*qx, 2.0*q0} };

    //    dR_by_dqx << 2.0*qx << 2.0*qy << 2.0*qz << arma::endr
    //              << 2.0*qy << -2.0*qx << -2.0*q0 << arma::endr
    //              << 2.0*qz << 2.0*q0 << -2.0*qx << arma::endr;
    dR_by_dqx = { {2.0*qx, 2.0*qy, 2.0*qz},
                  {2.0*qy, -2.0*qx, -2.0*q0},
                  {2.0*qz, 2.0*q0, -2.0*qx} };

    //    dR_by_dqy << -2.0*qy << 2.0*qx << 2.0*q0 << arma::endr
    //              << 2.0*qx << 2.0*qy  << 2.0*qz << arma::endr
    //              << -2.0*q0 << 2.0*qz << -2.0*qy << arma::endr;
    dR_by_dqy = { {-2.0*qy, 2.0*qx, 2.0*q0},
                  {2.0*qx, 2.0*qy, 2.0*qz},
                  {-2.0*q0, 2.0*qz, -2.0*qy} };

    //    dR_by_dqz << -2.0*qz << -2.0*q0 << 2.0*qx << arma::endr
    //              << 2.0*q0 << -2.0*qz  << 2.0*qy << arma::endr
    //              << 2.0*qx << 2.0*qy << 2.0*qz << arma::endr;
    dR_by_dqz = { {-2.0*qz, -2.0*q0, 2.0*qx},
                  {2.0*q0, -2.0*qz, 2.0*qy},
                  {2.0*qx, 2.0*qy, 2.0*qz} };

    arma::mat RES = arma::zeros<arma::mat>(3,4);
    RES(arma::span(0,2), arma::span(0,0)) = dR_by_dq0 * aMat.t();
    RES(arma::span(0,2), arma::span(1,1)) = dR_by_dqx * aMat.t();
    RES(arma::span(0,2), arma::span(2,2)) = dR_by_dqy * aMat.t();
    RES(arma::span(0,2), arma::span(3,3)) = dR_by_dqz * aMat.t();

    return RES;
}

inline arma::mat dqomegadt_by_domega(const arma::rowvec & omega, const double & delta_t) {

    arma::mat RES = arma::zeros<arma::mat>(4,3);
    double omegamod = arma::norm(omega, 2);
    if(fabs(omegamod) < 1e-8) {
        //        return arma::zeros<arma::mat>(4,3);
        //        RES(1,0) = RES(2,1) = RES(3,2) = delta_t / 2;
        //        return RES;
        //        RES << 0 << 0 << 0 << arma::endr
        //            << delta_t / 2 << 0 << 0 << arma::endr
        //            << 0 << delta_t / 2 << 0 << arma::endr
        //            << 0 << 0 << delta_t / 2 << arma::endr;
        RES = { {0, 0, 0}, {delta_t / 2, 0, 0}, {0, delta_t / 2, 0}, {0, 0, delta_t / 2} };
    }
    else {
        //        RES << dq0_by_domegaA(omega(0,0), omegamod, delta_t)
        //            << dq0_by_domegaA(omega(1,0), omegamod, delta_t)
        //            << dq0_by_domegaA(omega(2,0), omegamod, delta_t)
        //            << arma::endr

        //            << dqA_by_domegaA(omega(0,0), omegamod, delta_t)
        //            << dqA_by_domegaB(omega(0,0), omega(1,0), omegamod, delta_t)
        //            << dqA_by_domegaB(omega(0,0), omega(2,0), omegamod, delta_t)
        //            << arma::endr

        //            << dqA_by_domegaB(omega(1,0), omega(0,0), omegamod, delta_t)
        //            << dqA_by_domegaA(omega(1,0), omegamod, delta_t)
        //            << dqA_by_domegaB(omega(1,0), omega(2,0), omegamod, delta_t)
        //            << arma::endr

        //            << dqA_by_domegaB(omega(2,0), omega(0,0), omegamod, delta_t)
        //            << dqA_by_domegaB(omega(2,0), omega(1,0), omegamod, delta_t)
        //            << dqA_by_domegaA(omega(2,0), omegamod, delta_t)
        //            << arma::endr;
        RES = { {
                    dq0_by_domegaA(omega[0], omegamod, delta_t),
                    dq0_by_domegaA(omega[1], omegamod, delta_t),
                    dq0_by_domegaA(omega[2], omegamod, delta_t)
                },
                {
                    dqA_by_domegaA(omega[0], omegamod, delta_t),
                    dqA_by_domegaB(omega[0], omega[1], omegamod, delta_t),
                    dqA_by_domegaB(omega[0], omega[2], omegamod, delta_t)
                },
                {
                    dqA_by_domegaB(omega[1], omega[0], omegamod, delta_t),
                    dqA_by_domegaA(omega[1], omegamod, delta_t),
                    dqA_by_domegaB(omega[1], omega[2], omegamod, delta_t)
                },
                {
                    dqA_by_domegaB(omega[2], omega[0], omegamod, delta_t),
                    dqA_by_domegaB(omega[2], omega[1], omegamod, delta_t),
                    dqA_by_domegaA(omega[2], omegamod, delta_t)
                } };
    }
    return RES;
}

// ==================================================================================
//inline arma::mat DCM2QUAT(const cv::Mat& a){

//    arma::mat q = arma::zeros<arma::mat>(4, 1);

//    double trace = a.at<double>(0,0) + a.at<double>(1,1) + a.at<double>(2,2);
//    if( trace > 0 ) {// I changed M_EPSILON to 0
//        //std::cout<<"Condition 1\n";
//        double s = 0.5f / sqrtf(trace+ 1.0f);
//        q(0,0) = 0.25f / s;
//        q(1,0) = ( a.at<double>(2,1) - a.at<double>(1,2) ) * s;
//        q(2,0) = ( a.at<double>(0,2) - a.at<double>(2,0) ) * s;
//        q(3,0) = ( a.at<double>(1,0) - a.at<double>(0,1) ) * s;
//    } else {
//        if ( a.at<double>(0,0) > a.at<double>(1,1) && a.at<double>(0,0) > a.at<double>(2,2) ) {
//            //std::cout<<"Condition 2\n";
//            double s = 2.0f * sqrtf( 1.0f + a.at<double>(0,0) - a.at<double>(1,1) - a.at<double>(2,2));
//            q(0,0) = (a.at<double>(2,1) - a.at<double>(1,2) ) / s;
//            q(1,0) = 0.25f * s;
//            q(2,0) = (a.at<double>(0,1) + a.at<double>(1,0) ) / s;
//            q(3,0) = (a.at<double>(0,2) + a.at<double>(2,0) ) / s;
//        } else if (a.at<double>(1,1) > a.at<double>(2,2)) {
//            //std::cout<<"Condition 3\n";
//            double s = 2.0f * sqrtf( 1.0f + a.at<double>(1,1) - a.at<double>(0,0) - a.at<double>(2,2));
//            q(0,0) = (a.at<double>(0,2) - a.at<double>(2,0) ) / s;
//            q(1,0) = (a.at<double>(0,1) + a.at<double>(1,0) ) / s;
//            q(2,0) = 0.25f * s;
//            q(3,0) = (a.at<double>(1,2) + a.at<double>(2,1) ) / s;
//        } else {
//            //std::cout<<"Condition 4\n";
//            double s = 2.0f * sqrtf( 1.0f + a.at<double>(2,2) - a.at<double>(0,0) - a.at<double>(1,1) );
//            q(0,0) = (a.at<double>(1,0) - a.at<double>(0,1) ) / s;
//            q(1,0) = (a.at<double>(0,2) + a.at<double>(2,0) ) / s;
//            q(2,0) = (a.at<double>(1,2) + a.at<double>(2,1) ) / s;
//            q(3,0) = 0.25f * s;
//        }
//    }

//    if(q(0,0)<0)
//        q=-1.0*q;

//    return q;

//}

inline arma::rowvec DCM2QUAT_float(const cv::Mat& a){

    arma::mat q = arma::zeros<arma::rowvec>(4);

    float trace = a.at<float>(0,0) + a.at<float>(1,1) + a.at<float>(2,2);
    if( trace > 0 ) {// I changed M_EPSILON to 0
        //std::cout<<"Condition 1\n";
        float s = sqrtf(trace + 1.0f) * 2;
        q[0] = 0.25f * s;
        q[1] = ( a.at<float>(2,1) - a.at<float>(1,2) ) / s;
        q[2] = ( a.at<float>(0,2) - a.at<float>(2,0) ) / s;
        q[3] = ( a.at<float>(1,0) - a.at<float>(0,1) ) / s;
    }
    else {
        if ( a.at<float>(0,0) > a.at<float>(1,1) && a.at<float>(0,0) > a.at<float>(2,2) ) {
            //std::cout<<"Condition 2\n";
            float s = 2.0f * sqrtf( 1.0f + a.at<float>(0,0) - a.at<float>(1,1) - a.at<float>(2,2));
            q[0] = (a.at<float>(2,1) - a.at<float>(1,2) ) / s;
            q[1] = 0.25f * s;
            q[2] = (a.at<float>(0,1) + a.at<float>(1,0) ) / s;
            q[3] = (a.at<float>(0,2) + a.at<float>(2,0) ) / s;
        }
        else if (a.at<float>(1,1) > a.at<float>(2,2)) {
            //std::cout<<"Condition 3\n";
            float s = 2.0f * sqrtf( 1.0f + a.at<float>(1,1) - a.at<float>(0,0) - a.at<float>(2,2));
            q[0] = (a.at<float>(0,2) - a.at<float>(2,0) ) / s;
            q[1] = (a.at<float>(0,1) + a.at<float>(1,0) ) / s;
            q[2] = 0.25f * s;
            q[3] = (a.at<float>(1,2) + a.at<float>(2,1) ) / s;
        }
        else {
            //std::cout<<"Condition 4\n";
            float s = 2.0f * sqrtf( 1.0f + a.at<float>(2,2) - a.at<float>(0,0) - a.at<float>(1,1) );
            q[0] = (a.at<float>(1,0) - a.at<float>(0,1) ) / s;
            q[1] = (a.at<float>(0,2) + a.at<float>(2,0) ) / s;
            q[2] = (a.at<float>(1,2) + a.at<float>(2,1) ) / s;
            q[3] = 0.25f * s;
        }
    }

    if(q[0]<0)
        q=-1.0*q;

    return q;

}

inline arma::rowvec DCM2QUAT_float(const arma::mat& a){

    arma::rowvec q = arma::zeros<arma::rowvec>(4);

    float trace = a(0,0) + a(1,1) + a(2,2);
    if( trace > 0 ) {// I changed M_EPSILON to 0
        //std::cout<<"Condition 1\n";
        float s = sqrtf(trace + 1.0f) * 2;
        q[0] = 0.25f * s;
        q[1] = ( a(2,1) - a(1,2) ) / s;
        q[2] = ( a(0,2) - a(2,0) ) / s;
        q[3] = ( a(1,0) - a(0,1) ) / s;
    }
    else {
        if ( a(0,0) > a(1,1) && a(0,0) > a(2,2) ) {
            //std::cout<<"Condition 2\n";
            float s = 2.0f * sqrtf( 1.0f + a(0,0) - a(1,1) - a(2,2));
            q[0] = (a(2,1) - a(1,2) ) / s;
            q[1] = 0.25f * s;
            q[2] = (a(0,1) + a(1,0) ) / s;
            q[3] = (a(0,2) + a(2,0) ) / s;
        }
        else if (a(1,1) > a(2,2)) {
            //std::cout<<"Condition 3\n";
            float s = 2.0f * sqrtf( 1.0f + a(1,1) - a(0,0) - a(2,2));
            q[0] = (a(0,2) - a(2,0) ) / s;
            q[1] = (a(0,1) + a(1,0) ) / s;
            q[2] = 0.25f * s;
            q[3] = (a(1,2) + a(2,1) ) / s;
        }
        else {
            //std::cout<<"Condition 4\n";
            float s = 2.0f * sqrtf( 1.0f + a(2,2) - a(0,0) - a(1,1) );
            q[0] = (a(1,0) - a(0,1) ) / s;
            q[1] = (a(0,2) + a(2,0) ) / s;
            q[2] = (a(1,2) + a(2,1) ) / s;
            q[3] = 0.25f * s;
        }
    }

    if(q[0]<0)
        q=-1.0*q;

    return q;

}

inline void QUAT2DCM_float(const arma::rowvec& q, cv::Mat & R) {

    double x, y, z, r;
    x = q[1];
    y = q[2];
    z = q[3];
    r = q[0];

    R.at<float>(0,0) = r*r+x*x-y*y-z*z;
    R.at<float>(0,1) = 2.0*(x*y -r*z);
    R.at<float>(0,2) = 2.0*(z*x+r*y);
    //
    R.at<float>(1,0) = 2.0*(x*y+r*z);
    R.at<float>(1,1) = r*r-x*x+y*y-z*z;
    R.at<float>(1,2) = 2.0*(y*z-r*x);
    //
    R.at<float>(2,0) = 2.0*(z*x-r*y);
    R.at<float>(2,1) = 2.0*(y*z+r*x);
    R.at<float>(2,2) = r*r-x*x-y*y+z*z;

    return ;
}

// ==================================================================================

// Calculates rotation matrix to euler angles
// The result is the same as MATLAB except the order
// of the euler angles ( x and z are swapped ).
inline arma::mat rotationMatrixToEulerAngles(const cv::Mat& R) {

    float sy = sqrt(R.at<float>(0,0) * R.at<float>(0,0) +  R.at<float>(1,0) * R.at<float>(1,0) );
    bool singular = sy < 1e-6; // If

    arma::mat omega = arma::zeros<arma::mat>(3, 1);
    if (!singular) {
        omega(0,0) = std::atan2(R.at<float>(2,1) , R.at<float>(2,2));
        omega(1,0) = std::atan2(-R.at<float>(2,0), sy);
        omega(2,0) = std::atan2(R.at<float>(1,0), R.at<float>(0,0));
    }
    else {
        omega(0,0) = std::atan2(-R.at<float>(1,2), R.at<float>(1,1));
        omega(1,0) = std::atan2(-R.at<float>(2,0), sy);
        omega(2,0) = 0 ;
    }

    return omega;
}

inline arma::rowvec get_angular_velocity_with_H(const cv::Mat& Hrel){

    cv::Mat Rrel = Hrel(cv::Range(0,3), cv::Range(0,3));
    cv::Mat Omega_x = Rrel;
    Omega_x = (Omega_x - Omega_x.t());

    arma::rowvec omega = arma::zeros<arma::rowvec>(3);
    omega[0] = 0.5 * Omega_x.at<float>(2,1);
    omega[1] = 0.5 * Omega_x.at<float>(0,2);
    omega[2] = 0.5 * Omega_x.at<float>(1,0);

    return omega;

}

inline arma::mat get_angular_velocity_with_R(const cv::Mat& Rprev, const cv::Mat& Rcur){

    cv::Mat Omega_x = (Rcur - Rprev) * Rprev.t();
    Omega_x = (Omega_x - Omega_x.t());

    arma::mat omega = arma::zeros<arma::mat>(3, 1);
    omega(0,0) = 0.5 * Omega_x.at<double>(2,1);
    omega(1,0) = 0.5 * Omega_x.at<double>(0,2);
    omega(2,0) = 0.5 * Omega_x.at<double>(1,0);

    return omega;

}

inline double logDet(arma::mat M) {

#ifndef ARMA_NO_DEBUG
    assert(M.rows == M.cols);
#endif

    arma::mat R;
    if (arma::chol(R, M, "lower") == true) {
        //        arma::vec dv = R.diag();
        //        double min_dv = arma::min(dv);
        //        return 2 * std::log( arma::prod( dv ) / min_dv );
        return 2 * std::log( arma::prod( R.diag() ) );
    }
    else {
        //        arma::vec dv;
        //        arma::eig_sym( dv, M );
        //        double min_dv = arma::min(dv);
        //        return 2 * std::log( arma::prod( dv ) / min_dv );
        arma::cx_double ld_ = arma::log_det(M);
        return ld_.real();
    }
}

//
//
// A simple power method from
// https://www.mathworks.com/matlabcentral/newsreader/view_thread/50542
// Suppose to solve the maximum singular value & vector only,
// when the inital vector is sharing component with maximum singular vector
//
inline void solveMaxSVD(const arma::mat & SOM, arma::colvec & u, double & s, arma::rowvec & v) {
    //
    double s0 = 0.0, tol = 1.e-5; // 1.e-12;
    v = arma::sum(arma::abs(SOM), 0);
    s = arma::norm(v, 2);

    // handle zero input matrix
    if (std::fabs(s) < 1e-6)
    {
        u.zeros(SOM.n_rows, 1);
        v.zeros(1, SOM.n_cols);
        return ;
    }

    // set initial singular vector
    v = v / s;

    // iterative sol
    int cnt = 0;
    while ( std::fabs(s-s0) > tol * s )
    {
        if ( cnt >= N_MAX_ITER_SVD )
        {
            //#ifdef OBS_DEBUG_VERBOSE
            //                        std::cout << "func solveMaxSVD: no convergence after " << cnt << " iterations" << std::endl;
            //#endif
            return ;
        }
        //
        s0 = s;
        u = SOM * v.t();

        s = arma::norm(u, 2);
        u = u / s;
        v = u.t() * SOM;

        s = arma::norm(v, 2);
        v = v / s;
        cnt = cnt + 1;
    }
    //#ifdef OBS_DEBUG_VERBOSE
    //        std::cout << "func solveMaxSVD: converge after " << cnt << " iterations" << std::endl;
    //#endif
}



inline void compute_F_subblock(const arma::rowvec & Xv, const double & dt, arma::mat & F_Q, arma::mat & F_Omg) {
    //-----------------------------------------------------------------------------
    // Compute F
    //std::cout << "[OBS_COMPUTOR]  Get F" << std::endl;
    arma::rowvec omegaOld = Xv.subvec(10, 12);
    arma::rowvec qOld = Xv.subvec(3, 6);
    arma::rowvec v = omegaOld * dt;
    double theta = arma::norm(v, 2);
    arma::rowvec q;
    //    cout << "v = " << v << endl;

    if(theta < 1e-6) {
        //        q << 1 << 0 << 0 << 0 << arma::endr;
        q = {1, 0, 0, 0};
    } else {
        arma::rowvec v_n = v / theta;
        //        cout << "v_n = " << v_n << endl;

        arma::rowvec v_n_reshape =  std::sin(theta / 2.0) * (v_n / arma::norm(v_n, 2));
        //        cout << "v_n_reshape = " << v_n_reshape << endl;

        //        q << std::cos(theta / 2.0) << v_n_reshape(0,0) << v_n_reshape(0,1) << v_n_reshape(0,2);
        q = { std::cos(theta / 2.0), v_n_reshape[0], v_n_reshape[1], v_n_reshape[2] };
    }

    //    cout << "q = " << q << endl;

    // F matrix subblock:  F_Q
    double R = q[0], X = q[1], Y = q[2], Z = q[3];
    //    F_Q  << R << -X << -Y << -Z << arma::endr
    //         << X << R  <<  Z << -Y << arma::endr
    //         << Y << -Z <<  R <<  X << arma::endr
    //         << Z <<  Y << -X <<  R << arma::endr;
    F_Q = { {R, -X, -Y, -Z}, {X, R, Z, -Y}, {Y, -Z, R, X}, {Z, Y, -X, R} };

    //    cout << "F_Q = " << F_Q << endl;

    // F matrix subblock:   F_Omg
    R = qOld[0], X = qOld[1], Y = qOld[2], Z = qOld[3];
    arma::mat  dq3_by_dq1;
    //    dq3_by_dq1 << R << -X << -Y << -Z << arma::endr
    //               << X <<  R << -Z <<  Y << arma::endr
    //               << Y <<  Z <<  R << -X << arma::endr
    //               << Z << -Y <<  X <<  R << arma::endr;
    dq3_by_dq1 = { {R, -X, -Y, -Z}, {X, R, -Z, Y}, {Y, Z, R, -X}, {Z, -Y, X, R} };

    //    cout << "dq3_by_dq1 = " << dq3_by_dq1 << endl;
    //    cout << "omegaOld = " << omegaOld << endl;
    //    cout << "dqomegadt_by_domega = " << dqomegadt_by_domega(omegaOld, dt) << endl;

    F_Omg = dq3_by_dq1 * dqomegadt_by_domega(omegaOld, dt);

    //    cout << "F_Omg = " << F_Omg << endl;

    //    // Update the quaternion
    //    arma::mat omegaOld = Xv.rows(10, 12);
    //    arma::mat qOld = Xv.rows(3, 6);
    //    arma::mat v = omegaOld * dt;
    //    double theta = arma::norm(v, 2);
    //    arma::mat q;
    //    if(theta < 1e-6) {
    //        q << 1 << 0 << 0 << 0 << arma::endr;
    //    } else {
    //        arma::mat v_n = v / theta;
    //        arma::mat v_n_reshape =  std::sin(theta / 2.0) * (arma::reshape(v_n, 1,3) / arma::norm(v_n, 2));
    //        q << std::cos(theta / 2.0) << v_n_reshape(0,0) << v_n_reshape(0,1) << v_n_reshape(0,2);
    //    }

    //    // dq3_by_dq2
    //    double R = q(0,0), X = q(0,1), Y = q(0,2), Z = q(0,3);

    //    arma::mat dq3_by_dq2;
    //    dq3_by_dq2 << R << -X << -Y << -Z << arma::endr
    //               << X << R  <<  Z << -Y << arma::endr
    //               << Y << -Z <<  R <<  X << arma::endr
    //               << Z <<  Y << -X <<  R << arma::endr;

    //    // dq3_by_dq1
    //    R = qOld(0,0), X = qOld(1,0), Y = qOld(2,0), Z = qOld(3,0);

    //    arma::mat dq3_by_dq1;

    //    dq3_by_dq1 << R << -X << -Y << -Z << arma::endr
    //               << X <<  R << -Z <<  Y << arma::endr
    //               << Y <<  Z <<  R << -X << arma::endr
    //               << Z << -Y <<  X <<  R << arma::endr;

    //    // df_dxv
    //    df_dxv = arma::eye<arma::mat>(13, 13);
    //    df_dxv.submat(0, 7, 2, 9) = arma::eye(3,3) * dt;
    //    df_dxv.submat(3, 3, 6, 6) = dq3_by_dq2;
    //    df_dxv.submat(3, 10, 6, 12) = dq3_by_dq1 * dqomegadt_by_domega(omegaOld,dt);

    return;
}

inline void convert_PWLS_Vec_To_Homo(const arma::rowvec & pwlsVec, cv::Mat & Tcw) {

    cv::Mat Rwc(3, 3, CV_32F);
    cv::Mat twc(3, 1, CV_32F);
    //    cout << pwlsVec << endl;

    // translation
    twc.at<float>(0, 0) = pwlsVec[0];
    twc.at<float>(1, 0) = pwlsVec[1];
    twc.at<float>(2, 0) = pwlsVec[2];
    //    cout << twc << endl;

    // quaternion
    QUAT2DCM_float(pwlsVec.subvec(3, 6), Rwc);
    //    cout << Rwc << endl;

    //
    Tcw = cv::Mat::eye(4, 4, CV_32F);
    Tcw.rowRange(0,3).colRange(0,3) = Rwc.t();
    Tcw.rowRange(0,3).col(3) = -Rwc.t() * twc;

    //    std::cout << "func convert_PWLS_Vec_To_Homo: Tcw = " << Tcw << std::endl;
}

inline void convert_Homo_Pair_To_PWLS_Vec(const double t_0, const cv::Mat& Tcw_0,
                                          const double t_1, const cv::Mat& Twc_1,
                                          arma::rowvec & pwlsVec) {
    //
    pwlsVec = arma::zeros<arma::rowvec>(13);

    // fill in the pose info with current Twc
    // translation
    pwlsVec[0] = Twc_1.at<float>(0, 3);
    pwlsVec[1] = Twc_1.at<float>(1, 3);
    pwlsVec[2] = Twc_1.at<float>(2, 3);
    // quaternion
    pwlsVec.subvec(3, 6) = DCM2QUAT_float(Twc_1.rowRange(0,3).colRange(0,3));

    // fill in the velocity info with Tcw_prev * Twc_cur
    //
    // NOTE
    // don't call opencv inv function to get the inverse of homogenous matrix!
    // try to do it manually instead, as shown in pose update of tracking module!
    //
    cv::Mat T_rel = Tcw_0 * Twc_1;
    //    std::cout << "T_rel = " << T_rel << std::endl;
    //    std::cout << "Tcw_prev = " << Tcw_prev << std::endl;
    //    std::cout << "Twc_cur = " << Twc_cur << std::endl;

    // velocity
    pwlsVec[7] = T_rel.at<float>(0, 3) / float(t_1 - t_0);
    pwlsVec[8] = T_rel.at<float>(1, 3) / float(t_1 - t_0);
    pwlsVec[9] = T_rel.at<float>(2, 3) / float(t_1 - t_0);

    // omega
    //
    // NOTE
    // the func here hasn't been tested throughly;
    // ideally it should provide the body-framed angular rate (rather than the world-framed one)
    //
    pwlsVec.subvec(10, 12) = get_angular_velocity_with_H(T_rel) / float(t_1 - t_0);
    //    pwlsVec.rows(10, 12) =
    //            rotationMatrixToEulerAngles(T_rel.rowRange(0,3).colRange(0,3))
    //            / float(t_1 - t_0);
    //    std::cout << "orig angular vel = "
    //              << get_angular_velocity_with_H(T_rel) / float(time_cur - time_prev)
    //              << std::endl;
    //    std::cout << "curr angular vel = "
    //              << this->Xv.rows(10, 12)
    //              << std::endl;
}

inline void assemble_F_matrix(const double dt, const arma::mat & F_Q, const arma::mat & F_Omg,
                              arma::mat & F) {

    F = arma::eye(13, 13);
    F.at(0, 7) = dt;
    F.at(1, 8) = dt;
    F.at(2, 9) = dt;
    F.submat(arma::span(3, 6), arma::span(3, 6))   = F_Q;
    F.submat(arma::span(3, 6), arma::span(10, 12)) = F_Omg;

}

inline void propagate_PWLS(const KineStruct & curState,
                           arma::rowvec & nextPWLSVec) {

    //    std::cout << "start PWLS propagation" << std::endl;
    //    std::cout << "s1. " << curState.Xv << std::endl;
    nextPWLSVec = curState.Xv;

    // linear propagation for input PWLS state vector
    // r   = r + v*dt;
    nextPWLSVec[0] = curState.Xv[0] + curState.Xv[7] * curState.dt;
    nextPWLSVec[1] = curState.Xv[1] + curState.Xv[8] * curState.dt;
    nextPWLSVec[2] = curState.Xv[2] + curState.Xv[9] * curState.dt;

    //    std::cout << "s2. "  << nextPWLSVec << std::endl;

    // q1 = qProd(q, v2q(w*dt));
    // q1 = quatnormalize(q1);
    //
    arma::rowvec qCur = curState.Xv.subvec(3, 6);
    arma::rowvec wCur = curState.Xv.subvec(10, 12);
    arma::rowvec qMove = v2q( wCur * curState.dt );
    arma::rowvec qPred = qProd( qCur, qMove);

    //        std::cout << qPred << std::endl;

    arma::rowvec qn = qNormalize( qPred );

    //        std::cout << qn << std::endl;

    //    nextPWLSVec.rows(3, 6) = qn;
    nextPWLSVec[3] = qn[0];
    nextPWLSVec[4] = qn[1];
    nextPWLSVec[5] = qn[2];
    nextPWLSVec[6] = qn[3];

    //    std::cout << "s3. "  << nextPWLSVec << std::endl;

}


inline void compute_Huber_loss (const float residual_, float & loss_) {

    float delta_ = sqrt(5.991);

    if (fabs(residual_) < delta_) {
        loss_ = pow(residual_, 2);
    }
    else {
        loss_ = 2 * delta_ * fabs(residual_) - pow(delta_, 2);
    }
    return ;
}

inline void compute_Huber_weight (const float residual_, float & weight_) {
    if (fabs(residual_) > 0.001) {
        float loss_;
        compute_Huber_loss(residual_, loss_);
        weight_ = sqrt( loss_ ) / residual_;
    }
    else
        weight_ = 1.0;
    return ;
}

}

#endif
