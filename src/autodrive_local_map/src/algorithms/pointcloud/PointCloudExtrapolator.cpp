#include "algorithms/pointcloud/PointCloudExtrapolator.h"
#include "algorithms/MovementModel.h"
#include "local_map/Frames.h"

namespace AutoDrive::Algorithms {

    std::vector<std::shared_ptr<DataModels::PointCloudBatch>> PointCloudExtrapolator::splitPointCloudToBatches(
            std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> scan,
            DataModels::LocalPosition startPose,
            DataModels::LocalPosition poseDiff,
            rtl::Transformation3D<double> sensorOffsetTf) {


        std::vector<std::shared_ptr<DataModels::PointCloudBatch>> output;

        auto singleBatchSize = static_cast<size_t>(std::ceil(static_cast<float>(scan->width) / noOfBatches_));
        auto timeOffset = startPose.getTimestamp();
        auto duration = poseDiff.getTimestamp();


        std::cout << " ------- " << std::endl;
        size_t pointCnt = 0;
        for(size_t i = 0 ; i < noOfBatches_ ; i++) {

            double ratio = static_cast<double>(i) / noOfBatches_;
            auto pose = AutoDrive::DataModels::LocalPosition {
                    {poseDiff.getPosition().x() * ratio, poseDiff.getPosition().y() * ratio, poseDiff.getPosition().z() * ratio},
                    {poseDiff.getOrientation().slerp({},1 - ratio)},
                    uint64_t(duration * ratio)
            };
            std::cout << "  tf: " << pose.getPosition().x() << "  " << pose.getPosition().y() <<  " " << pose.getPosition().z() << std::endl;


            rtl::Transformation3D<double> movementCompensationTF{pose.getOrientation(), pose.getPosition()};
            uint64_t ts = timeOffset + static_cast<uint64_t>(ratio * duration);

            pcl::PointCloud<pcl::PointXYZ> points;
            points.reserve(singleBatchSize);

            for(size_t j = 0 ; j < singleBatchSize ; j++) {
                if(pointCnt < scan->width) {
                    points.push_back(scan->at(pointCnt++));
                }
            }

            rtl::Transformation3D<double> toGlobelFrameTf{startPose.getOrientation().inverted(), -startPose.getPosition()};

            auto finalTF = toGlobelFrameTf(movementCompensationTF(sensorOffsetTf));

            std::cout << "  final tf: " << finalTF.trX() << "  " << finalTF.trY() <<  " " << finalTF.trZ() << std::endl;
            auto batch = std::make_shared<DataModels::PointCloudBatch>(ts, points, LocalMap::Frames::kOrigin, finalTF);
            output.push_back(batch);
        }


        return output;
    }



    DataModels::LocalPosition PointCloudExtrapolator::interpolateLocalPosition(
            DataModels::LocalPosition& begin,
            DataModels::LocalPosition& end,
            float ratio) {

        float r = static_cast<float>(std::min(std::max((double)ratio, 0.0), 1.0));

        return MovementModel::interpolateLocalPosition(begin, end, r);
    }

}
