#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include "utility.h"

using namespace Eigen;
using namespace std;
int main()
{

    Eigen::Quaterniond corrected_delta_q = Eigen::Quaterniond(10,1,1,1).normalized();
    Eigen::Quaterniond Qi=Eigen::Quaterniond(1,2,3,4).normalized();
    Eigen::Quaterniond Qj=Eigen::Quaterniond(1,4,5,6).normalized();
    Eigen::Quaterniond dq=Eigen::Quaterniond(1000,1,1,1).normalized();//小角
    cout<<"q:"<<endl<<corrected_delta_q.matrix()<<endl;
    cout<<"Qi:"<<endl<<Qi.matrix()<<endl;
    cout<<"Qj:"<<endl<<Qj.matrix()<<endl;
    cout<<"dq:"<<endl<<dq.matrix()<<endl;
    

    /*
    检验原本代码中的雅可比
    Eigen::Vector3d residual = 2 * (corrected_delta_q.inverse() * (Qi.inverse() * Qj)).vec();
    
    Eigen::Matrix3d J_Qi = -(Utility::Qleft(Qj.inverse() * Qi) * Utility::Qright(corrected_delta_q)).bottomRightCorner<3, 3>();
    //Eigen::Matrix3d J_Qj=   Utility::Qleft(corrected_delta_q.inverse() * Qi.inverse() * Qj).bottomRightCorner<3, 3>();

    //对Qi加一个扰动
    Eigen::Vector3d residual_after=2 * (corrected_delta_q.inverse() * ((Qi*dq).inverse() * Qj)).vec();
    Eigen::Vector3d residual_predict=residual+J_Qi*(2*dq.vec());
    cout<<"residual:"<<endl<<residual<<endl;
    cout<<"J_Qi:"<<endl<<J_Qi<<endl;
    cout<<"residual_after:"<<endl<<residual_after<<endl;
    cout<<"residual_predict:"<<endl<<residual_predict<<endl;
    */

   //检验gps观测方程中的雅可比
   Eigen::Quaterniond q_w_e=Eigen::Quaterniond(1,2,3,4).normalized();
   Eigen::Quaterniond q_c_w=Eigen::Quaterniond(5,4,3,2).normalized();
   Eigen::Vector3d p_w_c=Eigen::Vector3d(3,4,5);
   Eigen::Vector3d p_else=Eigen::Vector3d(1,2,3);
   Eigen::Vector3d p_e_w=Eigen::Vector3d(0,0,0);

   //残差的表达式
   Eigen::Vector3d residual=p_e_w+q_w_e.toRotationMatrix()*p_w_c+q_w_e.toRotationMatrix()*q_c_w.toRotationMatrix()*p_else;

    cout<<"residual:"<<endl<<residual<<endl;

    //雅可比
    Eigen::Matrix3d J_q_c_w=-q_w_e.toRotationMatrix()*q_c_w.toRotationMatrix()*Utility::skewSymmetric(p_else);
    Eigen::Matrix3d J_q_w_e=-q_w_e.toRotationMatrix()*Utility::skewSymmetric(p_w_c)
                                                -q_w_e.toRotationMatrix()*Utility::skewSymmetric(q_c_w.toRotationMatrix()*p_else);

    //对q_c_w加一个dq的扰动
    Eigen::Vector3d residual_after_q_c_w=p_e_w+q_w_e.toRotationMatrix()*p_w_c+q_w_e.toRotationMatrix()*(q_c_w*dq).toRotationMatrix()*p_else;
    cout<<"residual_after q_c_w:"<<endl<<residual_after_q_c_w<<endl;
    Eigen::Vector3d residual_predict_q_c_w=residual+ J_q_c_w*(2*dq.vec());
    cout<<"residual_predict q_c_w:"<<endl<<residual_predict_q_c_w<<endl;
    cout<<endl;

    //对q_w_e加一个dq的扰动
    Eigen::Vector3d residual_after_q_w_e=p_e_w+(q_w_e*dq).toRotationMatrix()*p_w_c+(q_w_e*dq).toRotationMatrix()*q_c_w.toRotationMatrix()*p_else;
    cout<<"residual_after q_w_e:"<<endl<<residual_after_q_w_e<<endl;
    Eigen::Vector3d residual_predict_q_w_e=residual+ J_q_w_e*(2*dq.vec());
    cout<<"residual_predict q_w_e:"<<endl<<residual_predict_q_w_e<<endl;





    return 0;
}