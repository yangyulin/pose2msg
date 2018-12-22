/*
 * record.cpp
 *
 *  Created on: Jan 15, 2013
 *      Author: cforster
 *  Edited on: Feb 16, 2017
 *      Author: pgeneva
 */

#include <stdio.h>
#include <string>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/package.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include "GPSConversion.h"

using namespace std;

class Recorder {
public:
    std::string name;
    std::ofstream ofs_;
    int n_msgs_received_;
    bool invert_pose_;
    tf::TransformListener tf_listener_;
    Eigen::Quaterniond q_;
    Eigen::Vector3d p_;
    Eigen::Vector3d Pp_;
    Eigen::Vector3d Pr_;
    Eigen::Matrix<double,6,6> Cov_;
    double stamp_;

    // GPS data
    bool gps_ref_got;
    bool noref_update_;
    Eigen::Vector3d gps_ref_;

    Recorder(std::string topic_name, std::string filename, bool invert_pose, bool noref_update) :
            name(topic_name),
            n_msgs_received_(0),
            invert_pose_(invert_pose),
            noref_update_(noref_update),
            gps_ref_got(false),
            tf_listener_(ros::Duration(100)) {
        // Set defaults to zero
        Pp_ = Eigen::Vector3d::Zero();
        Pr_ = Eigen::Vector3d::Zero();
        Cov_ = Eigen::MatrixXd::Zero(6,6);
        // Setup file
        ofs_.open(filename.c_str());
        if (ofs_.fail())
            throw std::runtime_error("Could not create tracefile. Does folder exist?");
        ofs_ << "# format: timestamp tx ty tz qx qy qz qw Ptx Pty Ptz Prx Pry Prz" << std::endl;
    }

    ~Recorder() {}

    void write() {
        if (invert_pose_) {
            Eigen::Matrix3d R = q_.toRotationMatrix().transpose();
            p_ = -R * p_;
            q_ = Eigen::Quaterniond(R);
        }

        ofs_.precision(15);
        ofs_.setf(std::ios::fixed, std::ios::floatfield);
        ofs_ << stamp_ << " ";
        ofs_.precision(10);
        ofs_ << p_.x() << " " << p_.y() << " " << p_.z() << " "
             << q_.x() << " " << q_.y() << " " << q_.z() << " " << q_.w() << " "
                << Pp_.x() << " " << Pp_.y() << " " << Pp_.z() << " "
                << Pr_.x() << " " << Pr_.y() << " " << Pr_.z() << " " << std::endl;

        if (++n_msgs_received_ % 50 == 0)
            printf("[%s]: Received %i pose messages\n", name.c_str(), n_msgs_received_);
    }


    void write_cov() {
        if (invert_pose_) {
            Eigen::Matrix3d R = q_.toRotationMatrix().transpose();
            p_ = -R * p_;
            q_ = Eigen::Quaterniond(R);
        }

        ofs_.precision(15);
        ofs_.setf(std::ios::fixed, std::ios::floatfield);
        ofs_ << stamp_ << " ";
        ofs_.precision(15);
        ofs_ << p_.x() << " " << p_.y() << " " << p_.z() << " "
             << q_.x() << " " << q_.y() << " " << q_.z() << " " << q_.w() << " "
             << Cov_(0,0) << " " << Cov_(0,1) << " " << Cov_(0,2) << " " << Cov_(0,3) << " " << Cov_(0,4) << " " << Cov_(0,5) << " "
             << Cov_(1,0) << " " << Cov_(1,1) << " " << Cov_(1,2) << " " << Cov_(1,3) << " " << Cov_(1,4) << " " << Cov_(1,5) << " "
             << Cov_(2,0) << " " << Cov_(2,1) << " " << Cov_(2,2) << " " << Cov_(2,3) << " " << Cov_(2,4) << " " << Cov_(2,5) << " "
             << Cov_(3,0) << " " << Cov_(3,1) << " " << Cov_(3,2) << " " << Cov_(3,3) << " " << Cov_(3,4) << " " << Cov_(3,5) << " "
             << Cov_(4,0) << " " << Cov_(4,1) << " " << Cov_(4,2) << " " << Cov_(4,3) << " " << Cov_(4,4) << " " << Cov_(4,5) << " "
             << Cov_(5,0) << " " << Cov_(5,1) << " " << Cov_(5,2) << " " << Cov_(5,3) << " " << Cov_(5,4) << " " << Cov_(5,5) << " "
             << std::endl;

        if (++n_msgs_received_ % 50 == 0)
            printf("[%s]: Received %i pose messages\n", name.c_str(), n_msgs_received_);
    }

    void odomCallback(const nav_msgs::OdometryPtr &msg) {
        q_ = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                                msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
        p_ = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        Pp_ = Eigen::Vector3d(msg->pose.covariance.at(0),msg->pose.covariance.at(7),msg->pose.covariance.at(14));
        Pr_ = Eigen::Vector3d(msg->pose.covariance.at(21),msg->pose.covariance.at(28),msg->pose.covariance.at(35));
        stamp_ = msg->header.stamp.toSec();
        write();
    }

    void poseCallback(const geometry_msgs::PoseStampedPtr &msg) {
        q_ = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x,
                                msg->pose.orientation.y, msg->pose.orientation.z);
        p_ = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        stamp_ = msg->header.stamp.toSec();
        write();
    }

    void poseCovCallback(const geometry_msgs::PoseWithCovarianceStampedPtr &msg) {
        q_ = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                                msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
        p_ = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        Pp_ = Eigen::Vector3d(msg->pose.covariance.at(0),msg->pose.covariance.at(7),msg->pose.covariance.at(14));
        Pr_ = Eigen::Vector3d(msg->pose.covariance.at(21),msg->pose.covariance.at(28),msg->pose.covariance.at(35));
        Eigen::Matrix<double,6,6> temp_cov;

        temp_cov << msg->pose.covariance.at(0),msg->pose.covariance.at(1),msg->pose.covariance.at(2),
                msg->pose.covariance.at(3),msg->pose.covariance.at(4),msg->pose.covariance.at(5),
                msg->pose.covariance.at(6),msg->pose.covariance.at(7),msg->pose.covariance.at(8),
                msg->pose.covariance.at(9),msg->pose.covariance.at(10),msg->pose.covariance.at(11),
                msg->pose.covariance.at(12),msg->pose.covariance.at(13),msg->pose.covariance.at(14),
                msg->pose.covariance.at(15),msg->pose.covariance.at(16),msg->pose.covariance.at(17),
                msg->pose.covariance.at(18),msg->pose.covariance.at(19),msg->pose.covariance.at(20),
                msg->pose.covariance.at(21),msg->pose.covariance.at(22),msg->pose.covariance.at(23),
                msg->pose.covariance.at(24),msg->pose.covariance.at(25),msg->pose.covariance.at(26),
                msg->pose.covariance.at(27),msg->pose.covariance.at(28),msg->pose.covariance.at(29),
                msg->pose.covariance.at(30),msg->pose.covariance.at(31),msg->pose.covariance.at(32),
                msg->pose.covariance.at(33),msg->pose.covariance.at(34),msg->pose.covariance.at(35);

        Cov_ = temp_cov;
        // fill in the covariance matrix
//        for(int i=0; i<6; i++){
//            for(int j=0;j<6; j++){
//                Cov_ << (double)msg->pose.covariance[6*i+j];
//            }
//        }
        std::cout<<Cov_<<std::endl;
        stamp_ = msg->header.stamp.toSec();
        write_cov();
    }

    void tfStampedCallback(const geometry_msgs::TransformStampedPtr &msg) {
        q_ = Eigen::Quaterniond(msg->transform.rotation.w, msg->transform.rotation.x,
                                msg->transform.rotation.y, msg->transform.rotation.z);
        p_ = Eigen::Vector3d(msg->transform.translation.x, msg->transform.translation.y,
                             msg->transform.translation.z);
        stamp_ = msg->header.stamp.toSec();
        write();
    }

    void tfCallback(const std::string &topic, const std::string &topic_ref) {
        tf::StampedTransform tf_transform;
        ros::Time now(ros::Time::now());
        try {
            tf_listener_.waitForTransform(topic, topic_ref, now, ros::Duration(2.0));
            tf_listener_.lookupTransform(topic, topic_ref, now, tf_transform);
        }
        catch (tf::TransformException ex) {
            ROS_WARN("tfCallback: %s", ex.what());
        }

        Eigen::Affine3d eigen_transform;
        tf::transformTFToEigen(tf_transform, eigen_transform);
        q_ = Eigen::Quaterniond(eigen_transform.rotation());
        p_ = eigen_transform.translation();
        stamp_ = now.toSec();
        write();
    }

    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr &msg) {
        // Return if we do not have a reference yet
        if(!gps_ref_got)
            return;
        // Check that is a "good" gps measurement
        //if(msg->status.status < sensor_msgs::NavSatStatus::STATUS_GBAS_FIX)
        //    return;
        // Convert into ENU frame from the Lat, Lon frame
        double xEast, yNorth, zUp;
        GPSConversion::GeodeticToEnu(msg->latitude, msg->longitude, msg->altitude, gps_ref_(0), gps_ref_(1), gps_ref_(2), xEast, yNorth, zUp);
        // Set our values
        q_ = Eigen::Quaterniond(0,0,0,1);
        p_ = Eigen::Vector3d(xEast, yNorth, zUp);
        stamp_ = msg->header.stamp.toSec();
        write();
    }

    void gpsRefCallback(const sensor_msgs::NavSatFix::ConstPtr &msg) {
        // If we do not want to do update and we have already
        // Gotten our gps reference then just return
        if(noref_update_ && gps_ref_got)
            return;
        // Else update our ref
        gps_ref_ = Eigen::Vector3d(msg->latitude, msg->longitude, msg->altitude);
        gps_ref_got = true;
    }

};

int main(int argc, char **argv) {
    // create ros node
    ros::init(argc, argv, "trajectory_recorder");
    ros::NodeHandle nh("~");

    // get parameters to subscribe
    std::string topic;
    nh.getParam("topic", topic);
    std::string topic_type;
    nh.getParam("topic_type", topic_type);
    std::string topic_ref;
    nh.getParam("topic_ref", topic_ref);
    bool invert_pose;
    nh.getParam("invert_pose", invert_pose);
    bool noref_update;
    nh.getParam("noref_update", noref_update);

    // Get current time/date
    // http://stackoverflow.com/a/997803
    char buf[16];
    snprintf(buf, 16, "%lu", time(NULL));

    // Generate filename
    std::string topic_time(buf);
    std::string topic_name(topic);
    std::replace(topic_name.begin(), topic_name.end(), '/', '_');
    std::string filename(ros::package::getPath("posemsg_to_file") + "/logs/" + topic_time + topic_name + ".txt");

    // Debug
    cout << "Done reading config values" << endl;
    cout << " - topic = " << topic << endl;
    cout << " - topic_type = " << topic_type << endl;
    cout << " - topic_ref = " << topic_ref << endl;
    cout << " - invert_pose = " << invert_pose << endl;
    cout << " - noref_update = " << noref_update << endl;
    cout << " - file = " << topic_time+topic_name << ".txt" << endl;

    // start recorder
    Recorder recorder(topic_name, filename, invert_pose, noref_update);

    // subscribe to topic
    ros::Subscriber sub, sub_ref;
    if (topic_type == std::string("PoseWithCovarianceStamped")) {
        sub = nh.subscribe(topic, 10, &Recorder::poseCovCallback, &recorder);
    } else if (topic_type == std::string("PoseStamped")) {
        sub = nh.subscribe(topic, 10, &Recorder::poseCallback, &recorder);
    } else if (topic_type == std::string("TransformStamped")) {
        sub = nh.subscribe(topic, 10, &Recorder::tfStampedCallback, &recorder);
    } else if (topic_type == std::string("tf")) {
        if (topic_ref.empty())
            throw std::runtime_error("no tf reference topic specified.");
    } else if(topic_type == std::string("NavSatFix")) {
        // Check that we have a gps datum
        if (topic_ref.empty())
            throw std::runtime_error("no GPS reference topic specified.");
        // Sub to the two message topics
        sub = nh.subscribe(topic, 10, &Recorder::gpsCallback, &recorder);
        sub_ref = nh.subscribe(topic_ref, 10, &Recorder::gpsRefCallback, &recorder);
    } else if(topic_type == std::string("Odometry")) {
        sub = nh.subscribe(topic, 10, &Recorder::odomCallback, &recorder);
    } else {
        throw std::runtime_error("specified topic_type is not supported.");
    }

    ros::spin();

    // spin
//    ros::Rate r(500);
//    while (ros::ok()) {
//        ros::spinOnce();
//        if (topic_type == std::string("tf"))
//            recorder.tfCallback(topic, topic_ref);
//        r.sleep();
//    }
    return 0;
}
