#define _USE_MATH_DEFINES
 
#include <cmath>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#define RAD2DEG (180.0/M_PI)
#define DEG2RAD (M_PI/180.0)

/* Program 4: Quaternion investigation */

//! This is the quaternion to euler conversion we've used when dealing with NED coordinates:
void quat2euler(const Eigen::Quaterniond& q, double& phi, double& theta, double& psi)
{
    phi = atan2( 2.0*(q.w()*q.x()+q.y()*q.z()), 1.0 - 2.0*(q.x()*q.x() + q.y()*q.y()));

    double temp = 2.0*(q.w()*q.y() - q.z()*q.x());
    if (temp > 1.0) {
        temp = 1.0;
    } else if (temp < -1.0) {
        temp = -1.0;
    }

    theta = asin(temp);
    psi = atan2( 2.0*(q.w()*q.z() + q.x()*q.y()), 1.0 - 2.0*(q.y()*q.y() + q.z()*q.z()));
}



int main(void)
{
	//! The constructor expects w, x, y, z order (where w is the scalar)
	Eigen::Quaterniond q(1, 1, 0, 0);
	std::cout << "Quaternion - scalar: " << q.w() << "\n vector:\n" << q.vec() << "\n\n"; 
	
	//! Normalize: 
	q.normalize();
	std::cout << "Normalized:\n" << q.w() << "\n" << q.vec() << "\nNorm: " << q.norm() << "\n\n";
	
	//! Rotate a vector by a quaternion:
	Eigen::Vector3d v(0, 1, 0);
	
	Eigen::Quaterniond p;
	p.w() = 0;
	p.vec() = v;
	
	Eigen::Quaterniond rotatedP = q * p * q.inverse();
	Eigen::Vector3d rotatedV = rotatedP.vec();
	
	std::cout << "Before rotating: Vector v = \n" << v << "\n\n";
	std::cout << "After rotating: q*[0 v]'*q'=\n" << rotatedV << "\n\n";
	
	//! Rotate by converting to a rotation matrix first:
	Eigen::Matrix3d R = q.toRotationMatrix(); 
	std::cout << "Rotation Matrix from q: \n" << R << "\n\n";
	std::cout << "R*v:\n" << R*v << "\n\n";
	
	/** Specify two vectors (like true gravity and measured gravity 
	 *  during a zupt).  Figure out the quaternion from this:
	 * 
	 *  Gravity vector = [0, 0, -1]' when using ros-style coordinates (forward, left, up)
	 * 
	 *  If we measure gravity using the accelerometer, we can find the quaternion that 
	 *  rotates from the world frame to the body frame of the accelerometer.
	 *  This will give us our attitude vector (minus yaw)
	 */	 
	double theta = -23.2*DEG2RAD;		//! Note: Using this coordinate system, positive pitch is pitch down!
	double phi = 75.3*DEG2RAD;
	
	Eigen::Vector3d g_world(0, 0, -1);	//! Using this coordinate system, gravity is negative in world frame!
	Eigen::Matrix3d Rtheta = Eigen::Matrix3d::Identity();
	Rtheta(0,0) = cos(theta);
	Rtheta(0,2) = -sin(theta);
	Rtheta(2,0) = sin(theta);
	Rtheta(2,2) = cos(theta);
	Eigen::Matrix3d Rphi = Eigen::Matrix3d::Identity();
	Rphi(1,1) = cos(phi);
	Rphi(1,2) = sin(phi);
	Rphi(2,1) = -sin(phi);
	Rphi(2,2) = cos(phi);
	
	Eigen::Vector3d g_meas = Rphi*Rtheta*g_world;
	
	//Eigen::Vector3d g_meas(sin(theta), 0, -cos(theta));
	
	std::cout << "Gravity in inertial frame: \n" << g_world << "\n\n";
	std::cout << "Gravity measurement: \n" << g_meas << "\n\n";	
	
	/** We want to find the quaternion that would take a vector in the 
	 *  body frame and align it with a vector in the world frame, so we 
	 *  pass in the measured vector first, and then the expected vector
	 *  in the world frame. 
	 */
	Eigen::Quaterniond a = Eigen::Quaterniond::FromTwoVectors(g_meas, g_world);
	
	std::cout << "Quaternion from pitch of " << theta*RAD2DEG << " degrees:\n" 
		<< a.w() << "\n" << a.vec() << "\n\n";
		
	//! Recover 3-2-1 Euler angles (phi and theta only):
	double roll, pitch, yaw;
	quat2euler(a, roll, pitch, yaw);
	
	std::cout << "  Original euler rotations - phi: " << phi*RAD2DEG << ", theta: " << theta*RAD2DEG << "\n";
	std::cout << "  Recovered rotations      - phi: " << roll*RAD2DEG << ", theta: " << pitch*RAD2DEG << "\n";
	
	
	
	return 0;
}


/* Program 3 */
/*/

using namespace Eigen;
using namespace std;

int main()
{
  Matrix3d m = Matrix3d::Random();
  m = (m + Matrix3d::Constant(1.2)) * 50;
  cout << "m =" << endl << m << endl;
  Vector3d v(1,2,3);
  
  cout << "m * v =" << endl << m * v << endl;
}
//*/


/* Program 2 */
/*/

using namespace Eigen;
using namespace std;

int main()
{
	MatrixXd m = MatrixXd::Random(3,3);
	m = (m + MatrixXd::Constant(3,3,1.2)) * 50;
	cout << "m = " << endl << m << endl;
	
	VectorXd v(3);
	v << 1, 2, 3;
	cout << "m * v =" << endl << m * v << endl;
}

//*/


/*
   Program 1:
 
 
using Eigen::MatrixXd;

int main(void)
{
	MatrixXd m(2,2);
	m(0,0) = 3;
	m(1,0) = 2.5;
	m(0,1) = -1;
	m(1,1) = m(1,0) + m(0,1);
	std::cout << m << "\n";
	
	return 0;
}*/
