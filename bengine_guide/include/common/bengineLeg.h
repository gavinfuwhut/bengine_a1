#ifndef BENGINELEG_H
#define BENGINELEG_H

#include "common/mathTypes.h"
#include "common/enumClass.h"

class QuadrupedLeg{
public:
    QuadrupedLeg(int legID, float abadLinkLength, float hipLinkLength, 
                 float kneeLinkLength, Vec3 pHip2B);
    ~QuadrupedLeg(){}
    Vec3 calcPEe2H(Vec3 q);                                         // Forward Kinematics(参考坐标系为侧摆关节)
    Vec3 calcPEe2B(Vec3 q);                                         // Forward Kinematics(参考坐标系为身体坐标系)
    Vec3 calcVEe(Vec3 q, Vec3 qd);                                  //速度正向运动学
    Vec3 calcQ(Vec3 pEe, FrameType frame);                          // Inverse Kinematics
    // Vec3 calcQd(Vec3 q, Vec3 vEe);
    // Vec3 calcQd(Vec3 pEe, Vec3 vEe, FrameType frame);
    // Vec3 calcTau(Vec3 q, Vec3 force);
    Mat3 calcJaco(Vec3 q);                                          //雅可比矩阵
    // Vec3 getHip2B(){return _pHip2B;}
protected:
    float q1_ik(float py, float pz, float b2y);
    float q3_ik(float b3z, float b4z, float b);
    float q2_ik(float q1, float q3, float px, 
                float py, float pz, float b3z, float b4z);
    float _sideSign;
    const float _abadLinkLength,            //侧摆连杆长度(从侧摆关节轴心到髋关节轴心的距离)
                _hipLinkLength,             //大腿长度（从髋关节到膝关节）
                _kneeLinkLength;            //小腿长度（从膝关节到足端）
    const Vec3 _pHip2B;                     //从髋关节参考点到机身坐标系的偏移向量((x, y, z) 在机身坐标系中的位置)
};

// class A1Leg : public QuadrupedLeg{
// public:
    // A1Leg(const int legID, const Vec3 pHip2B):
    //     QuadrupedLeg(legID, 0.0838, 0.2, 0.2, pHip2B){}
    // ~A1Leg(){}
// };

class BengineLeg : public QuadrupedLeg{
public:
    BengineLeg(const int legID, const Vec3 pHip2B):
        QuadrupedLeg(legID, 0.1205, 0.3, 0.35, pHip2B){}
    ~BengineLeg(){}
};

#endif  // BENGINELEG_H