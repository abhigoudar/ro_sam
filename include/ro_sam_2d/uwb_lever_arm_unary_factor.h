#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose2.h>
#include <cmath>

namespace sam
{
    class UWBRangeLeverArmFactor2D : public gtsam::NoiseModelFactor2<gtsam::Pose2, gtsam::Point2>
    {
        public:
        UWBRangeLeverArmFactor2D(gtsam::Key i, gtsam::Key j, const gtsam::SharedNoiseModel& model,
            gtsam::Vector3 _anc_pos, double range) :
            gtsam::NoiseModelFactor2<gtsam::Pose2, gtsam::Point2>(model, i, j), 
            anc_pos(_anc_pos), meas_range(range){}

        gtsam::Vector evaluateError(const gtsam::Pose2& q, const gtsam::Point2& l_arm,
            boost::optional<gtsam::Matrix&> H1 = boost::none, 
            boost::optional<gtsam::Matrix&> H2 = boost::none) const
        {
            gtsam::Vector1 residual;
            gtsam::Vector2 diff;
            Point2 anc(anc_pos.x(), anc_pos.y());

            Point2 delta = anc - q.rotation()*l_arm - q.translation();
            double pred_range = delta.norm();
            // calculate residual
            // note that the order is different compared to
            // convention residual defn: measured - predicted
            residual[0] = pred_range - meas_range;
            // calculate jacobian
            if(H1)
            {
                double dh_dx, dh_dy, dh_dtheta;
                // calculate jacobian w.r.t position
                dh_dx = -delta[0]/pred_range;
                dh_dy = -delta[1]/pred_range;
                // calculate jacobian w.r.t theta
                dh_dtheta = delta[0]*(q.r().s()*l_arm[0]+q.r().c()*l_arm[1]) + 
                            delta[1]*(-q.r().c()*l_arm[0]+q.r().s()*l_arm[1]);
                dh_dtheta = dh_dtheta/pred_range;
                (*H1) = (gtsam::Matrix(1,3) << dh_dx, dh_dy,
                                              dh_dtheta).finished();
            }

            if(H2)
            {
                double dh_dlx, dh_dly;
                // calculate jcobian w.r.t lever-arm
                dh_dlx = delta[0]*-q.r().c() + delta[1]*-q.r().s();
                dh_dlx = dh_dlx/pred_range;
                dh_dly = delta[0]*q.r().s() + delta[1]*-q.r().c();
                dh_dly = dh_dly/pred_range;
                (*H2) = (gtsam::Matrix(1,2) << dh_dlx, dh_dly).finished();
            }
            // printf(" anchor:x:[%f] y:[%f] mobile: x:[%f] y:[%f] range:[%f] pred:[%f] res:[%f]\n", anc_pos[0],
            //         anc_pos[1], q.x(), q.y(), meas_range, pred_range, residual[0]);
            return residual;
        }

        private:
        // position of anchor
        gtsam::Vector3 anc_pos;
        // measured range
        double meas_range;
    };
}