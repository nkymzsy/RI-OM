
#ifndef KALMAN_H
#define KALMAN_H


#include <eigen3/Eigen/Dense>
using Eigen::MatrixXd;
using Eigen::VectorXd;


//X,Y,Z同时加滤波
class kalman_filter_3D
{
public:
	kalman_filter_3D()
    {
        delta_time=0;
        X=MatrixXd(6, 1);
        F=MatrixXd(6, 6);
        H=MatrixXd(3, 6);
        Q=MatrixXd(6, 6);
        R=MatrixXd(3, 3);
        P=MatrixXd(6, 6);
        K=MatrixXd(6, 3);
        X.setZero();
        F<<1,0,0,delta_time,0,0,
        0,1,0,0,delta_time,0,
        0,0,1,0,0,delta_time,
        0,0,0,1,0,0,
        0,0,0,0,1,0,
        0,0,0,0,0,1;
        H<<1,0,0,0,0,0,
        0,1,0,0,0,0,
        0,0,1,0,0,0;

        Q<< 0.01, 0,0,0,0,0,
        0,0.01,0,0,0,0,
        0,0,0.01,0,0,0,
        0,0,0,0.01,0,0,
        0,0,0,0,0.01,0,
        0,0,0,0,0,0.01;
        R<< 0.1,0,0,
        0,0.1,0,
        0,0,0.1;

        // R.setOnes();
        // Q=Q/1000.0;
        //  R=R/1000.0;
        P.setZero();
        K.setZero();
    }
	~kalman_filter_3D(){}
	Vector3d input(MatrixXd z, double time=0.1)
    {
        this->delta_time=time;
        F<<1,0,0,delta_time,0,0,
        0,1,0,0,delta_time,0,
        0,0,1,0,0,delta_time,
        0,0,0,1,0,0,
        0,0,0,0,1,0,
        0,0,0,0,0,1;
        /* Predict */
        X=F*X;
        P=F*P*F.transpose() + Q;
        /*update*/
        K = P*H.transpose()*(H*P*H.transpose()+R).inverse();
        X = X+K*(z-H*X);
        P = P - K*H*P;

        Vector3d ans;
        ans<<X(0,0),X(1,0),X(2,0);
        return ans;
    }

	MatrixXd X;  /* state */
	MatrixXd F;  /* x(n)=F*x(n-1)+B*u(n),u(n)~N(0,q) */
	MatrixXd B;  /*B为内部控制量*/
	MatrixXd H;  /* z(n)=H*x(n)+w(n),w(n)~N(0,r)   */
	MatrixXd Q;  /* process(predict) noise convariance */
	MatrixXd R;  /* measure noise convariance */
	MatrixXd P;  /* estimated error convariance */
	MatrixXd K;  /*kalman 增益*/
    double delta_time=0.1;
};
#endif

