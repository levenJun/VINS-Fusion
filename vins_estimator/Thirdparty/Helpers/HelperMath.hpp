#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace MyHelpers{

class HelperMath
{
private:
    /* data */
public:
    HelperMath(/* args */){};
    ~HelperMath(){};

public:

    static bool getToEularXZY(const Eigen::Quaternionf& q, Eigen::Vector3f& outEular){

        double cp_sr = 2*(q.w()*q.y() + q.x()*q.z());
        double cp_cr = 1 - 2*(q.y()*q.y() + q.z()*q.z());
        outEular(2) = std::atan2(cp_sr, cp_cr);

        double sp = -2*(q.x()*q.y() - q.w()*q.z());
        if (std::abs(sp) >= 1)
            outEular(1) = std::copysign(M_PI / 2, sp); // use 90 degrees if out of range
        else
            outEular(1) = std::asin(sp);
        
        double cp_sy = 2*(q.w()*q.x() + q.y()*q.z());
        double cp_cy = 1 - 2*(q.x()*q.x() + q.z()*q.z());
        outEular(0) = std::atan2(cp_sy, cp_cy);
        
        return true;
    };

    static Eigen::Matrix3d Skew(const Eigen::Vector3d& v3){
        Eigen::Matrix3d result;
        result <<  0.0,   -v3(2),  v3(1),
                    v3(2) , 0.0,   -v3(0),
                    -v3(1) ,  v3(0), 0.0;
        return result;
    }
    static Eigen::Matrix3f Skew(const Eigen::Vector3f& v3){
        Eigen::Matrix3f result;
        result <<  0.0,   -v3(2),  v3(1),
                    v3(2) , 0.0,   -v3(0),
                    -v3(1) ,  v3(0), 0.0;
        return result;
    }    

    //未对输入q作check,也没有作归一化操作，请提前保证
    static void TransQtoM4Right(const Eigen::Vector4f& q, Eigen::Matrix4f& qRight){
        qRight.setIdentity();
        const float& s = q(0);
        const Eigen::Vector3f& w = q.segment(1, 3);

        for(int i = 0; i < 4; i++) qRight(i,i) = s;
        qRight.block<1,3>(0,1) = -w.transpose();
        qRight.block<3,1>(1,0) = w;
        qRight.block<3,3>(1,1) -= Skew(w);
    }
};
}