#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose2.h>
#include <cmath>

namespace sam
{
    class UWBRangeFactor2D : public gtsam::NoiseModelFactor1<gtsam::Pose2>
    {
        public:
        UWBRangeFactor2D(gtsam::Key j, const gtsam::SharedNoiseModel& model,
            gtsam::Vector3 _anc_pos, double range) :
            gtsam::NoiseModelFactor1<gtsam::Pose2>(model, j), 
            anc_pos(_anc_pos), meas_range(range){
                // printf("Adding key:%d\n", j);
                // printf(" anchor:x:[%f] y:[%f]\n", anc_pos[0],
                //     anc_pos[1]);
                // printf("range:[%f]\n", range);
            }

        gtsam::Vector evaluateError(const gtsam::Pose2& q, 
            boost::optional<gtsam::Matrix&> H = boost::none) const
        {
            gtsam::Vector1 residual;
            gtsam::Vector2 diff;
            diff[0] = q.x() - anc_pos[0];
            diff[1] = q.y() - anc_pos[1];
            double pred_range = diff.norm();
            // calculate residual
            // note that the order is different compared to
            // convention residual defn: measured - predicted
            residual[0] = pred_range - meas_range;
            // calculate jacobian
            if(H)
            {
                (*H) = (gtsam::Matrix(1,3) << diff[0]/pred_range,
                                              diff[1]/pred_range,
                                              0).finished();
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