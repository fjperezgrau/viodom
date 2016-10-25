/**
 * @file imufilter.hpp
 * @brief Filter for obtaining roll/pitch/yaw data from the IMU on Skybotix VI-Sensor
 * @author Fernando Caballero, fcaballero@us.es
 * @author Francisco J Perez-Grau, fjperez@catec.aero
 * @date October 2016
 *
Copyright (c) 2016, fcaballero, fjperezgrau
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __IMUFILTER_H__
#define __IMUFILTER_H__

#include <Eigen/Core>
#include <Eigen/Dense>
#include <sensor_msgs/Imu.h>
#include <vector>

// Convenient constants
#define RAD2DEG(X) ( (X) * 57.2957795131)
#define DEG2RAD(X) ( (X) * 0.01745329251)
#define LIN2GRAV(X) ( (X) * 0.10197162)

/**
 * @class ImuFilter
 * @brief Estimates IMU angles from acceleration and gyro data
 */
class ImuFilter
{
public:
	
    /** @brief Constructor
     * @param T Prediction period (seconds)
     * @param imuTopic Name of the IMU data topic
     * @param calibTime Initial calibration time (seconds)
     */
    ImuFilter(double T, std::string imuTopic, double calibTime = 5.0)
	{
        init_ = false;
        T_ = T;
        T2_ = T*T;
        calibTime_ = calibTime;

		// IMU EKF parameters
        accDev_ = 0.006;//0.001
        gyrDev_ = 0.005;//0.01;
        magDev_ = 0;
        magXs_ = 1.0;
        magYs_ = 1.0;
        magZs_ = 1.0;
        magXo_ = 0.0;
        magYo_ = 0.0;
        magZo_ = 0.0;
        biaDev_ = 0.00001;//0.000001;
        biaTh_ = 0.005; //0.001;
		
        // Setup IMU data subscriber
        sub_ = nh_.subscribe(imuTopic, 10, &ImuFilter::imuDataCallback, this);
	}

	
    /** @brief Initialize EKF
     * Input: vector of sensor_msgs::Imu
     * The user must continue calling this function with new gyro data until it returns true
     */
	bool initialize(void)
	{
		double gx_m, gy_m, gz_m, gx_m2, gy_m2, gz_m2, gx_d, gy_d, gz_d; 
		
		// Compute mean value and mean square
		gx_m = gy_m = gz_m = gx_m2 = gy_m2 = gz_m2 = 0.0;
        for(int i = 0; i < (int)calibData_.size(); i++)
		{
            gx_m += calibData_[i].angular_velocity.x;
            gy_m += calibData_[i].angular_velocity.y;
            gz_m += calibData_[i].angular_velocity.z;
            gx_m2 += calibData_[i].angular_velocity.x*calibData_[i].angular_velocity.x;
            gy_m2 += calibData_[i].angular_velocity.y*calibData_[i].angular_velocity.y;
            gz_m2 += calibData_[i].angular_velocity.z*calibData_[i].angular_velocity.z;
		}
        gx_m = gx_m/(double)calibData_.size();
        gy_m = gy_m/(double)calibData_.size();
        gz_m = gz_m/(double)calibData_.size();
        gx_m2 = gx_m2/(double)calibData_.size();
        gy_m2 = gy_m2/(double)calibData_.size();
        gz_m2 = gz_m2/(double)calibData_.size();
		//std::cout << "gxM: " << gx_m << ", gyM: " << gy_m << ", gzM: " << gz_m << std::endl;
		
		// Compute standar deviation of gyros
		gx_d = sqrt(gx_m2-gx_m*gx_m);
		gy_d = sqrt(gy_m2-gy_m*gy_m);
		gz_d = sqrt(gz_m2-gz_m*gz_m);
		//std::cout << "gxDev: " << gx_d << ", gyDev: " << gy_d << ", gzDev: " << gz_d << std::endl;
		
		// Initalize compass calibration
        magCal_[0] = magXs_; magCal_[1] = magYs_; magCal_[2] = magZs_;
        magCal_[3] = magXo_; magCal_[4] = magYo_; magCal_[5] = magZo_;
		
		// Initialize sensor variances
        accVar_[0] = accDev_*accDev_; 	accVar_[1] = accDev_*accDev_; 	accVar_[2] = accDev_*accDev_;		// Variance in g
        gyrVar_[0] = gyrDev_*gyrDev_; gyrVar_[1] = gyrDev_*gyrDev_; gyrVar_[2] = gyrDev_*gyrDev_;	// Variance in rad/s
        magVar_[0] = magDev_*magDev_; 	magVar_[1] = magDev_*magDev_; 	magVar_[2] = magDev_*magDev_;	// Variance in mGaus wth data normalized to 1
        biaVar_[0] = biaDev_*biaDev_; biaVar_[1] = biaDev_*biaDev_; biaVar_[2] = biaDev_*biaDev_;	// Variance in rad/s
		
		// Initialize accelerometer threshold
        accTh_ = sqrt(accVar_[0]+accVar_[1]+accVar_[2]);
		
		// Initialize state vector x = [rx, ry, rz, gbx, gby, gbz]
        rx_ = ry_ = rz_ = 0.0;
        gbx_ = gx_m;
        gby_ = gy_m;
        gbz_ = gz_m;
		
		// Initialize covariance matrix
        P_.setIdentity(6, 6);
        P_(0,0) = M_PI_2;
        P_(1,1) = M_PI_2;
        P_(2,2) = M_PI_2;
        P_(3,3) = 0.01*0.01;
        P_(4,4) = 0.01*0.01;
        P_(5,5) = 0.01*0.01;
        /*P_(3,3) = biaVar_[0];
        P_(4,4) = biaVar_[1];
        P_(5,5) = biaVar_[2];*/
		
        if(gx_d < biaTh_ && gy_d < biaTh_ && gz_d < biaTh_)
		{
            init_ = true;
            ROS_INFO("IMU filter initialized");
		
			return true;
		}
		else
			return false;
	}	
	
    /** EKF prediction stage based on gyro information
     * @param[in] gx Raw X gyro data (rad/s)
     * @param[in] gy Raw Y gyro data (rad/s)
     * @param[in] gz Raw Z gyro data (rad/s)
     */
	bool predict(double gx, double gy, double gz)
	{
		// Check initialization 
        if(!init_)
			return false;
		
		// Compute matrix F 
		Eigen::Matrix<double, 6, 6> F;
        F(0,0) = 1, F(0,1) = 0, F(0,2) = 0, F(0,3) = -T_, F(0,4) = 0,   F(0,5) = 0;
        F(1,0) = 0, F(1,1) = 1, F(1,2) = 0, F(1,3) = 0,  F(1,4) = -T_,  F(1,5) = 0;
        F(2,0) = 0, F(2,1) = 0, F(2,2) = 1, F(2,3) = 0,  F(2,4) = 0,   F(2,5) = -T_;
		F(3,0) = 0, F(3,1) = 0, F(3,2) = 0, F(3,3) = 1,  F(3,4) = 0,   F(3,5) = 0;
		F(4,0) = 0, F(4,1) = 0, F(4,2) = 0, F(4,3) = 0,  F(4,4) = 1,   F(4,5) = 0;
		F(5,0) = 0, F(5,1) = 0, F(5,2) = 0, F(5,3) = 0,  F(5,4) = 0,   F(5,5) = 1;

		// Update covariance matrix
        P_ = F*P_*F.transpose();
        P_(0,0) += gyrVar_[0]*T2_;
        P_(1,1) += gyrVar_[1]*T2_;
        P_(2,2) += gyrVar_[2]*T2_;
        P_(3,3) += biaVar_[0]*T_;
        P_(4,4) += biaVar_[1]*T_;
        P_(5,5) += biaVar_[2]*T_;
		
		// Update state vector
        rx_ += T_*(gx - gbx_);
        ry_ += T_*(gy - gby_);
        rz_ += T_*(gz - gbz_);
														
		return true; 
	}
	
    /** EKF update stage based on accelerometer information
     * @param[in] ax Raw X accelerometer data (g)
     * @param[in] ay Raw Y accelerometer data (g)
     * @param[in] az Raw Z accelerometer data (g)
     */
	bool update(double ax, double ay, double az)
	{
		double crx, srx, cry, sry, mod, y[3];
		
		// Check initialization 
        if(!init_)
			return false;
		
		// Pre-compute constants
        crx = cos(rx_);
        cry = cos(ry_);
        srx = sin(rx_);
        sry = sin(ry_);
		
		// Create measurement jacobian H
		Eigen::Matrix<double, 3, 6> H;
		H(0,0) = 0;			H(0,1) = cry;		H(0,2) = 0; H(0,3) = 0; H(0,4) = 0; H(0,5) = 0;
		H(1,0) = -crx*cry; 	H(1,1) = srx*sry; 	H(1,2) = 0; H(1,3) = 0; H(1,4) = 0; H(1,5) = 0;
		H(2,0) = cry*srx;	H(2,1) = crx*sry;	H(2,2) = 0; H(2,3) = 0; H(2,4) = 0; H(2,5) = 0;
		
		// Compute measurement noise jacoban R
		Eigen::Matrix<double, 3, 3> R;
		mod = fabs(sqrt(ax*ax + ay*ay + az*az)-1);
		R.setZero(3, 3);
        R(0,0) = accVar_[0];
        R(1,1) = accVar_[1];
        R(2,2) = accVar_[2];
		/*if(mod > accTh)
		{
			R(0,0) += 1.5*mod*mod;
			R(1,1) += 1.5*mod*mod;
			R(2,2) += 1.5*mod*mod;
		}*/
		
		// Compute innovation matrix
		Eigen::Matrix<double, 3, 3> S;
        S = H*P_*H.transpose()+R;
		
		// Compute kalman gain
		Eigen::Matrix<double, 6, 3> K;
        K = P_*H.transpose()*S.inverse();
		
		// Compute mean error
		y[0] = ax-sry;
		y[1] = ay+cry*srx;
		y[2] = az+crx*cry;
		
		// Compute new state vector
        rx_ += K(0,0)*y[0]+K(0,1)*y[1]+K(0,2)*y[2];
        ry_ += K(1,0)*y[0]+K(1,1)*y[1]+K(1,2)*y[2];
        rz_ += K(2,0)*y[0]+K(2,1)*y[1]+K(2,2)*y[2];
        gbx_ += K(3,0)*y[0]+K(3,1)*y[1]+K(3,2)*y[2];
        gby_ += K(4,0)*y[0]+K(4,1)*y[1]+K(4,2)*y[2];
        gbz_ += K(5,0)*y[0]+K(5,1)*y[1]+K(5,2)*y[2];
		
		// Compute new covariance matrix
		Eigen::Matrix<double, 6, 6> I;
		I.setIdentity(6, 6);
        P_ = (I-K*H)*P_;
		
		return true;
	}

    /** EKF update stage based on accelerometer information
     * @param[in] ax Raw X accelerometer data (g)
     * @param[in] ay Raw Y accelerometer data (g)
     * @param[in] az Raw Z accelerometer data (g)
     * @param[in] mx Raw X magnetometer data ()
     * @param[in] my Raw Y magnetometer data ()
     * @param[in] mz Raw Z magnetometer data ()
     */
	bool update(double ax, double ay, double az, double mx, double my, double mz)
	{
		double crx, srx, cry, sry, crz, srz, mod, y[5], hx, hy;
		
		// Check initialization 
        if(!init_)
			return false;
		
		// Remove distortion and normalize magnetometer
        mx = (mx - magCal_[3])/magCal_[0];
        my = (my - magCal_[4])/magCal_[1];
        mz = (mz - magCal_[5])/magCal_[2];
		mod = sqrt(mx*mx + my*my + mz*mz);
		mx /= mod;
		my /= mod;
		mz /= mod;
		
		// Pre-compute constants
        crx = cos(rx_);
        cry = cos(ry_);
        crz = cos(rz_);
        srx = sin(rx_);
        sry = sin(ry_);
        srz = sin(rz_);
		
		// Create measurement jacobian H
		Eigen::Matrix<double, 5, 6> H;
		H(0,0) = 0;			H(0,1) = cry;		H(0,2) = 0; 	H(0,3) = 0; H(0,4) = 0; H(0,5) = 0;
		H(1,0) = -crx*cry; 	H(1,1) = srx*sry; 	H(1,2) = 0; 	H(1,3) = 0; H(1,4) = 0; H(1,5) = 0;
		H(2,0) = cry*srx;	H(2,1) = crx*sry;	H(2,2) = 0; 	H(2,3) = 0; H(2,4) = 0; H(2,5) = 0;
		H(3,0) = 0;			H(3,1) = 0;			H(3,2) = -srz; 	H(3,3) = 0; H(3,4) = 0; H(3,5) = 0;
		H(4,0) = 0;			H(4,1) = 0;			H(4,2) = -crz; 	H(4,3) = 0; H(4,4) = 0; H(4,5) = 0;
		
        // Compute measurement noise jacobian R
		Eigen::Matrix<double, 5, 5> R;
		mod = fabs(sqrt(ax*ax + ay*ay + az*az)-1);
		R.setZero(5, 5);
        R(0,0) = accVar_[0];
        R(1,1) = accVar_[1];
        R(2,2) = accVar_[2];
        R(3,3) = magVar_[0];
        R(4,4) = magVar_[1];
        if(mod > accTh_)
		{
			R(0,0) += 1.5*mod*mod;
			R(1,1) += 1.5*mod*mod;
			R(2,2) += 1.5*mod*mod;
		}
		
		// Compute innovation matrix
		Eigen::Matrix<double, 5, 5> S;
        S = H*P_*H.transpose()+R;
		
        // Compute Kalman gain
		Eigen::Matrix<double, 6, 5> K; 
        K = P_*H.transpose()*S.inverse();
		
		// Compute mean error
		hx = mx*cry + mz*crx*sry + my*srx*sry;
		hy = my*crx - mz*srx;
		mod = sqrt(hx*hx+hy*hy);
		y[0] = ax-sry;
		y[1] = ay+cry*srx;
		y[2] = az+crx*cry;
		y[3] = hx/mod-crz;
		y[4] = hy/mod+srz;
		
		// Compute new state vector
        rx_ += K(0,0)*y[0]+K(0,1)*y[1]+K(0,2)*y[2]+K(0,3)*y[3]+K(0,4)*y[4];
        ry_ += K(1,0)*y[0]+K(1,1)*y[1]+K(1,2)*y[2]+K(1,3)*y[3]+K(1,4)*y[4];
        rz_ += K(2,0)*y[0]+K(2,1)*y[1]+K(2,2)*y[2]+K(2,3)*y[3]+K(2,4)*y[4];
        gbx_ += K(3,0)*y[0]+K(3,1)*y[1]+K(3,2)*y[2]+K(3,3)*y[3]+K(3,4)*y[4];
        gby_ += K(4,0)*y[0]+K(4,1)*y[1]+K(4,2)*y[2]+K(4,3)*y[3]+K(4,4)*y[4];
        gbz_ += K(5,0)*y[0]+K(5,1)*y[1]+K(5,2)*y[2]+K(5,3)*y[3]+K(5,4)*y[4];
		
		// Compute new covariance matrix
		Eigen::Matrix<double, 6, 6> I;
		I.setIdentity(6, 6);
        P_ = (I-K*H)*P_;
		
		return true;
	}
	
    /** @brief Get estimated Euler angles in radians interval [-PI,PI] rad
     * @param[out] rx Estimated roll (rad)
     * @param[out] ry Estimated pitch (rad)
     * @param[out] rz Estimated yaw (rad)
     */
    bool getAngles(double &rx, double &ry, double &rz)
	{
        rx = Pi2PiRange(rx_);
        ry = Pi2PiRange(ry_);
        rz = Pi2PiRange(rz_);
		return true;
	}

    /** @brief Get estimated integrated Euler angles since last reset
     * @param[out] rx Estimated integrated roll (rad)
     * @param[out] ry Estimated integrated pitch (rad)
     * @param[out] rz Estimated integrated yaw (rad)
     */
    bool getAngleIntegration(double &irx, double &iry, double &irz)
    {
        irx = irx_;
        iry = iry_;
        irz = irz_;
        return true;
    }

    /** @brief Reset integrated angles
     */
    bool resetAngleIntegration(void)
    {
        irx_ = iry_ = irz_ = 0.0;
        return true;
    }

    /** @brief Get estimated gyro biases in rad/s
     * @param[out] gbx Estimated X gyro bias (rad/s)
     * @param[out] gby Estimated Y gyro bias (rad/s)
     * @param[out] gbz Estimated Z gyro bias (rad/s)
     */
    bool getBIAS(double &gbx, double &gby, double &gbz)
	{
        gbx = gbx_;
        gby = gby_;
        gbz = gbz_;
		
		return true;
	}
	
    /** @brief IMU initialization return function
     * @return Returns true if IMU initialized
     */
	bool isInit(void)
	{
        return init_;
	}

    /** IMU sensor data callback
     * @param[in] msg IMU data message
     */
	void imuDataCallback(const sensor_msgs::Imu::ConstPtr& msg)
	{
		// Check for IMU initialization
        if(!init_)
		{
            calibData_.push_back(*msg);
            if(calibData_.size() > calibTime_/T_)
				initialize();
			
			return;
		}	
		
		// Process sensor data
		predict(msg->angular_velocity.z, msg->angular_velocity.x, msg->angular_velocity.y);
		update(LIN2GRAV(msg->linear_acceleration.z), LIN2GRAV(msg->linear_acceleration.x), LIN2GRAV(msg->linear_acceleration.y));

        // Integrate angle rates
        irx_ += (msg->angular_velocity.z-gbx_)*T_;
        iry_ += (msg->angular_velocity.x-gby_)*T_;
        irz_ += (msg->angular_velocity.y-gbz_)*T_;
    }
	
protected:

    /** @brief Round down absolute function
     * @param[in] value Input value
     * @return Input value floored
     */
    double floorAbsolute( double value )
	{
	  if (value < 0.0)
		return ceil( value );
	  else
		return floor( value );
	}

    /** @brief Convert angles into interval [-PI,PI] rad
     * @param[in] contAngle Input angle (rad)
     * @return Input angle in interval [-PI,PI] rad
     */
    double Pi2PiRange(double contAngle)
	{
        double boundAngle = 0.0;
        if(fabs(contAngle)<=M_PI)
            boundAngle= contAngle;
		else
		{
            if(contAngle > M_PI)
                boundAngle = (contAngle-2*M_PI) - 2*M_PI*floorAbsolute((contAngle-M_PI)/(2*M_PI));
			
            if(contAngle < - M_PI)
                boundAngle = (contAngle+2*M_PI) - 2*M_PI*floorAbsolute((contAngle+M_PI)/(2*M_PI));
		}
		
        return boundAngle;
	}
	

    double calibTime_;      /**< IMU calibration time*/

    double T_;                                  /**< IMU KF prediction period*/
    double T2_;                                 /**< Squared IMU KF prediction period*/
    double rx_, ry_, rz_, gbx_, gby_, gbz_;     /**< IMU KF state vector x = [rx, ry, rz, gbx, gby, gbz]*/
    Eigen::MatrixXd P_;                         /**< IMU KF matrix*/
	
    double magCal_[6];      /**< Sensor calib info [gainX, gainY, gainZ, offsetX, offsetY, offsetZ]*/
	
    double accVar_[3];      /**< Accelerometers variances [varX, varY, varZ]*/
    double magVar_[3];      /**< Magnetometers variances [varX, varY, varZ]*/
    double gyrVar_[3];      /**< Gyroscopes variances [varX, varY, varZ]*/
    double biaVar_[3];      /**< Bias variances [varX, varY, varZ]*/
	
    double accTh_;          /**< Accelerometer threshold for filter updating*/
	
    bool init_;             /**< Flag indicating if IMU has been initialized*/
	
	// EKF Parameters
    double accDev_;
    double gyrDev_;
    double magDev_;
    double biaDev_;
    double biaTh_;
    double magXs_;
    double magYs_;
    double magZs_;
    double magXo_;
    double magYo_;
    double magZo_;
	
    std::vector<sensor_msgs::Imu> calibData_;   /**< IMU data for initialization*/
	
    ros::NodeHandle nh_;        /**< ROS node handler*/
    ros::Subscriber sub_;       /**< IMU data subscriber*/

    double irx_, iry_, irz_;    /**< Integrated angles since last reset*/
};

#endif








