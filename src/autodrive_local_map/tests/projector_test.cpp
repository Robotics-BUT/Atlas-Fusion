#include <gtest/gtest.h>
#include <rtl/Frustum.h>
#include "algorithms/Projector.h"
#include "algorithms/DetectionsProcessor.h"

const cv::Mat getIntrinsic() {
    return (cv::Mat_<double> (3,3) <<
            1.3763974496214614e+03, 0.0000000000000000e+00, 9.5923661859185984e+02,
            0.0000000000000000e+00, 1.3771721669205588e+03, 5.7674337567054886e+02,
            0.0000000000000000e+00, 0.0000000000000000e+00, 1.0000000000000000e+00
    );
}

const cv::Mat getDistortion() {
    return (cv::Mat_<double> (1,5) <<
       -1.7245433170404437e-01, 1.1239016157509915e-01, -1.5253720111333031e-04, 3.8487261087130094e-04, -2.0770262391598841e-02
    );
}

const cv::Mat getDistortionNull() {
    return (cv::Mat_<double> (1,5) <<
       0,0,0,0,0
    );
}

rtl::Transformation3D<double> getTF(){
    return rtl::Transformation3D<double>{};
}

TEST(projector_test, initialization) {

    auto tf = getTF();
    AutoDrive::Algorithms::Projector projector(getIntrinsic(), getDistortion(), tf);
}

TEST(projector_test, simple_projection) {

    auto tf = getTF();
    AutoDrive::Algorithms::Projector projector(getIntrinsic(), getDistortion(), tf);

    std::vector<cv::Point2f> points2D;
    std::vector<cv::Point3f> points3D{
            {0,0,10},
            {1,0,10},
            {0,1,10},
            {1,1,10},
    };

    projector.projectPoints(points3D, points2D);
    for (const auto& point : points2D) {
        std::cout << point.x << " " << point.y << std::endl;
    }
}


TEST(projector_test, reverse_projection) {

    auto tf = getTF();
    AutoDrive::Algorithms::Projector projector(getIntrinsic(), getDistortion(), tf);

    float distance = 10;
    std::vector<cv::Point3f> points3D{
            {0,0,distance},
            {1,1,distance},
    };

    std::vector<cv::Point2f> points2D;
    projector.projectPoints(points3D, points2D);
    for (const auto& point : points2D) {
        std::cout << point.x << " " << point.y << std::endl;
    }

    std::vector<cv::Point3f> pointDir;
    projector.reverseProjection(points2D, pointDir);
    for (const auto& point : pointDir) {
        std::cout << point.x << " " << point.y << " " << point.z << std::endl;
    }


    assert(points3D.size() == pointDir.size());
    for(size_t i = 0 ; i < points3D.size() ; i++){
        double ratio = distance / pointDir.at(i).z;
        EXPECT_NEAR(points3D.at(i).x, pointDir.at(i).x * ratio, 0.0001);
        EXPECT_NEAR(points3D.at(i).y, pointDir.at(i).y * ratio, 0.0001);
        EXPECT_NEAR(points3D.at(i).z, pointDir.at(i).z * ratio, 0.0001);
    }
}



TEST(projector_test, visualization) {

    auto tf = getTF();
    AutoDrive::Algorithms::Projector projector(getIntrinsic(), getDistortion(), tf);

    cv::Mat img(1080, 1920, CV_8UC3);
    std::vector<cv::Point2f> points2D{
            {100,500},
            {300,500},
            {100,600},
            {300,600}
    };

    for(const auto& p : points2D) {
        cv::circle(img, p, 3, {255,0,0,}, 1);
    }

    std::vector<cv::Point3f> points3D;
    projector.reverseProjection(points2D, points3D);

    std::vector<cv::Point2f> newPoints2D;
    projector.projectPoints(points3D, newPoints2D);


    for(const auto& p : newPoints2D) {
        cv::circle(img, p, 5, {0,0,255}, 1);
    }

    auto f = rtl::Frustum<double>(
            {0,0,0},
            {points3D[0].x, points3D[0].y, points3D[0].z},
            {points3D[1].x, points3D[1].y, points3D[1].z},
            {points3D[2].x, points3D[2].y, points3D[2].z},
            {points3D[3].x, points3D[3].y, points3D[3].z},
            10);

    std::vector<cv::Point3f> frustumConers;
    frustumConers.push_back(cv::Point3f{(float)f.getNearMidPoint().x(), (float)f.getNearMidPoint().y(), (float)f.getNearMidPoint().z()});
    frustumConers.push_back(cv::Point3f{(float)f.getNearTopLeft().x(), (float)f.getNearTopLeft().y(), (float)f.getNearTopLeft().z()});
    frustumConers.push_back(cv::Point3f{(float)f.getNearTopRight().x(), (float)f.getNearTopRight().y(), (float)f.getNearTopRight().z()});
    frustumConers.push_back(cv::Point3f{(float)f.getNearBottomLeft().x(), (float)f.getNearBottomLeft().y(), (float)f.getNearBottomLeft().z()});
    frustumConers.push_back(cv::Point3f{(float)f.getNearBottomRight().x(), (float)f.getNearBottomRight().y(), (float)f.getNearBottomRight().z()});


    std::vector<cv::Point2f> frustum2D;
    projector.projectPoints(frustumConers, frustum2D);

    for(const auto& p : frustum2D) {
        cv::circle(img, p, 7, {0,255,0}, 1);
    }
    //cv::imshow("bublebum", img);
    //cv::waitKey(0);
}



TEST(projector_test, circular_pattern_projection) {

    auto tf = getTF();
    AutoDrive::Algorithms::Projector projector(getIntrinsic(), getDistortion(), tf);

    std::vector<cv::Point2f> points2D;
    std::vector<cv::Point3f> points3D;

    for(int i = 0 ; i < 360 ; i+=10) {
        float angle = i * M_PI / 180;
        points3D.emplace_back(cv::Point3f{sin(angle), 0,cos(angle)});
    }

    projector.projectPoints(points3D, points2D);

    cv::Mat img(1080,1920,CV_8UC3);
    size_t cnt = 1;
    for(const auto& p : points2D) {
        cv::circle(img, p, cnt++, {0, 255, 0}, 1);
    }
    cv::imshow("bublebum", img);
    cv::waitKey(0);

}


int main(int argc, char **argv){

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}