#pragma once

#include <rtl/Vector3D.h>
#include <rtl/Quaternion.h>
#include <rtl/Transformation3D.h>

namespace AutoDrive::DataModels {

    /**
     * Local Position represents metric position and orientation in the 3D space
     */
    class LocalPosition {

    public:

        /**
         * Construcotr
         * @param positon 3D metric position
         * @param orientation Quaternion orientation
         * @param timestamp timestamp, the position has been estimated
         */
        explicit LocalPosition(rtl::Vector3D<double> positon, rtl::Quaternion<double> orientation, uint64_t timestamp)
        : position_(std::move(positon))
        , orientation_(std::move(orientation))
        , timestamp_{timestamp} {

        }

        /**
         * 3D vector position getter
         * @return 3D metric position
         */
        rtl::Vector3D<double> getPosition() const { return position_; };

        /**
         * 3D space orientation represetned by quaternion getter
         * @return quatermion represented orientation
         */
        rtl::Quaternion<double> getOrientation() const { return orientation_;};

        /**
         * set X component of 3D position
         * @param x axis position
         */
        void setX(double x) {position_.setX(x);};

        /**
         * set Y component of 3D position
         * @param y axis position
         */
        void setY(double y) {position_.setY(y);};

        /**
         * set Z component of 3D position
         * @param z axis position
         */
        void setZ(double z) {position_.setZ(z);};

        /**
         * Setter for full 3D position
         * @param pose 3D metric position
         */
        void setPositon(rtl::Vector3D<double> pose) { position_ = pose;};

        /**
         * Setter for orientation
         * @param orientation quaternion represented 3D orientation
         */
        void setOrientation(rtl::Quaternion<double> orientation) { orientation_ = orientation; };

        /**
         * Converts 3D orientation nad translation into the 3D transformation
         * @return
         */
        rtl::Transformation3D<double> toTf() const;

        /**
         * Nanosecond timestamp getter
         * @return timesamp of the position estimation time
         */
        uint64_t getTimestamp() const {return timestamp_;};

        /**
         * Operator combines two local poses into one. Translation are added and quatermions are multiplied
         * @param other second position
         * @return Returns newly created local position combined by this and the given one
         */
        LocalPosition operator+(LocalPosition& other);

        /**
         * Inverse operator to operator+. A + B - B = A
         * @param other B component
         * @return Result of substracting two local poses
         */
        LocalPosition operator-(LocalPosition& other);

        /**
         * Method generates short string form of the Local Position
         * @return string expression of the local pose
         */
        std::string toString();

    private:

        rtl::Vector3D<double> position_;
        rtl::Quaternion<double> orientation_;
        uint64_t timestamp_;

    };

}

