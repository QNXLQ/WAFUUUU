/*
input: rotation matrix, type cv::Mat(4, 4, CV_32FC1) or Eigen::MatrixXf(4, 4)

output: joint values: [q1 q2 q3 q4 q5 q6]
*/
#include<opencv/opencv.hpp>
#include<eigen3/Eigen>


#include<iostream>
#include<cmath>
/*
DH parameters:
theta: rotación respecto a eje z
d: traslación respecto a eje z
alpha: rotación respecto a eje x
a: traslación respecto a eje x
*/

/*
Modified DH:
0      0       0.0985  q1+pi;
0      -pi/2   0.1215  q2-pi/2;
0.408  pi      0       q3;
0.376  pi      0       q4-pi/2;
0      -pi/2   0.1025  q5;
0      pi/2    0.094   q6
*/

/*
Simplified modified DH:
0   	0   	0.0985 	  Q1;
0	    pi/2    0.1215    Q2;
0       -pi/2   0.8865    Q5;
0       pi/2    0.094     Q6
where:
Q1 = q1 + pi
Q2 = q2 + q3 + q4
Q5 = q5
Q6 = q6
*/

//Simplified modified DH parameters:
const float D1 = 0.0985, D2 = 0.1215, D3 = 0.8865, D4 = 0.094;
const float ALPHA1 = 0, ALPHA2 = pi/2, ALPHA3 = -pi/2, ALPHA4 = pi/2;
//Modified DH parameters:
const float d1 = 0.0985, d2 = 0.1215, a3 = 0.408, a4 = 0.376, d5 = 0.1025, d6 = 0.094;

double sign(double number)
{
	if (number > 0)
		return 1.0;
	else if (number < 0)
		return -1.0;
	else
		return 0.0;
}

//Homogenous transformation matrix for modified DH parameters
cv::Mat HTM_MDH(float a = 0.0, float alpha = 0.0, float d = 0.0, float theta = 0.0)
{
	cv::Mat homo_matrix= cv::Mat::eye(4, 4, CV_32FC1);
	if (a == 0.0 && alpha == 0.0 && d == 0.0 && theta == 0.0)
		return homo_matrix;
	else
	{
		homo_matrix.at<float>(0, 0) = cos(theta);
		homo_matrix.at<float>(0, 1) = -sin(theta);
		homo_matrix.at<float>(0, 3) = a;
		homo_matrix.at<float>(1, 0) = cos(alpha) * sin(theta);
		homo_matrix.at<float>(1, 1) = cos(alpha) * cos(theta);
		homo_matrix.at<float>(1, 2) = -sin(alpha);
		homo_matrix.at<float>(1, 3) = -d * sin(alpha);
		homo_matrix.at<float>(2, 0) = sin(alpha) * sin(theta);
		homo_matrix.at<float>(2, 1) = sin(alpha) * cos(theta);
		homo_matrix.at<float>(2, 2) = cos(alpha);
		homo_matrix.at<float>(2, 3) = d * cos(alpha);
		return homo_matrix;
	}
}

//From DH parameters create homogenous transformation matrix of the end effector
cv::Mat HTM_ee(std::vector<float> mdhparams)
{
	//dhparams format: 6x4 matrix, rows: J1 to J2, cols: a, alpha, d, theta.
	//the result can be write in this way:[n, o, a, p;0, 0, 0, 1]
	cv::Mat Joint1 = HTM_MDH(mdhparams(0, 0), mdhparams(0, 1), mdhparams(0, 2), mdhparams(0, 3));
	cv::Mat Joint2 = HTM_MDH(mdhparams(1, 0), mdhparams(1, 1), mdhparams(1, 2), mdhparams(1, 3));
	cv::Mat Joint3 = HTM_MDH(mdhparams(2, 0), mdhparams(2, 1), mdhparams(2, 2), mdhparams(2, 3));
	cv::Mat Joint4 = HTM_MDH(mdhparams(3, 0), mdhparams(3, 1), mdhparams(3, 2), mdhparams(3, 3));
	cv::Mat Joint5 = HTM_MDH(mdhparams(4, 0), mdhparams(4, 1), mdhparams(4, 2), mdhparams(4, 3));
	cv::Mat Joint6 = HTM_MDH(mdhparams(5, 0), mdhparams(5, 1), mdhparams(5, 2), mdhparams(5, 3));
	return Joint1 * Joint2 * Joint3 * Joint4 * Joint5 * Joint6;
}

struct Pose
{
	float x, y, z;
	std::vector<float> quaternion(4, 0);
};

cv::Mat mth_from_quaternion(Pose pose)
{
	float qx, qy, qz, qw = pose.quaternion(0), pose.quaternion(1), pose.quaternion(2), pose.quaternion(3);
	cv::Mat mth = cv::Mat::eye(4, 4, CV_32FC1);
	mth.at<float>(0, 0) = 1 - 2 * pow(qy, 2) - 2 * pow(qz, 2);
	mth.at<float>(0, 1) = 2 * qx * qy - 2 * qw * qz;
    mth.at<float>(0, 2) = 2 * qx * qz + 2 * qw * qy;
	mth.at<float>(0, 3) = pose.x;
    mth.at<float>(1, 0) = 2 * qx * qy + 2 * qw * qz;
	mth.at<float>(1, 1) = 1 - 2 * pow(qx, 2) - 2 * pow(qz, 2);
    mth.at<float>(1, 2) = 2 * qy * qz - 2 * qw * qx;
	mth.at<float>(1, 3) = pose.y;
    mth.at<flaot>(2, 0) = 2 * qx * qz - 2 * qw * qy;
    mth.at<float>(2, 1) = 2 * qy * qz + 2 * qw * qx;
    mth.at<float>(2, 2) = 1 - 2 * (qx ** 2) - 2 * (qy ** 2);
	mth.at<float>(2, 3) = pose.z;
	return mth;
}

float sin_solution(float a11, float a12, float a21, float a22, float u1, float u2)
{
	/*
	a11 * sin(theta) + a12 * cos(theta) = u1
	a21 * sin(theta) + a22 * cos(theta) = u2
	*/
	if ((a11 * a22) != (a12 * a21))
		return (u1 - a12 * u2) / (a11 * a22 - a12 * a21);
	else
		return -2.0;
}

float cos_solution(float a11, float a12, float a21, float a22, float u1, float u2)
{
	/*
	a11 * sin(theta) + a12 * cos(theta) = u1
	a21 * sin(theta) + a22 * cos(theta) = u2
	*/
	if ((a11 * a22) != (a12 * a21))
		return (u2 - a21 * u1) / (a11 * a22 - a12 * a21);
	else
		return -2.0;
}

void situation1(float Q2, float Q5, cv::Mat ee_mth, float Q6, float Q1)
{
	Q6 = asin(sin_solution(cos(Q2), cos(Q5) * sin(Q2), -cos(Q5) * sin (Q2), cos(Q2), ee_mth.at<float>(2, 0), ee_mth.at<flaot>(2, 1)));
		if (cos(Q6) != cos_solution(cos(Q2), cos(Q5) * sin(Q2), -cos(Q5) * sin (Q2), cos(Q2), ee_mth.at<float>(2, 0), ee_mth.at<flaot>(2, 1)))
			Q6 = pi - Q6;
		
		if (sin(Q5) != 0)
		{
			Q1 = asin(sin_solution(-cos(Q6) * sin(Q5), (cos(Q2) * cos(Q5) * cos(Q6) - sin(Q2) * sin(Q6)), (cos(Q2) * cos(Q5) * cos(Q6) - sin(Q2) * sin(Q6)), cos(Q6) * sin(Q5), ee_mth.at<float>(0, 0), ee_mth.at<flaot>(0, 1)));
			if (cos(Q1) != cos_solution(-cos(Q6) * sin(Q5), (cos(Q2) * cos(Q5) * cos(Q6) - sin(Q2) * sin(Q6)), (cos(Q2) * cos(Q5) * cos(Q6) - sin(Q2) * sin(Q6)), cos(Q6) * sin(Q5), ee_mth.at<float>(0, 0), ee_mth.at<flaot>(0, 1)))
				Q1 = pi - Q1;
		}
		else
		{
			Q1 = asin(ee_mth.at<flaot>(0, 2) / cos(Q5));
			if (cos(Q1) != (-ee_mth.at<flaot>(0, 2) / cos(Q5)))
				Q1 = pi - Q1;
		}
}

void situation2(float Q2, float Q5, cv::Mat ee_mth, float Q6, float Q1)
{
	Q6 = asin(ee_mth.at<flaot>(2, 1) / ( -sin(Q2) * cos(Q5)));
	if (cos(Q6) != ee_mth.at<flaot>(2, 0) / (sin(Q2) * cos(Q5)))
		Q6 = pi - Q6;
	if (sin(Q5) * sin(Q6) != 0)
	{
		Q1 = asin(sin_solution(-cos(Q6) * sin(Q5), (cos(Q2) * cos(Q5) * cos(Q6) - sin(Q2) * sin(Q6)), (cos(Q2) * cos(Q5) * cos(Q6) - sin(Q2) * sin(Q6)), cos(Q6) * sin(Q5), ee_mth.at<float>(0, 0), ee_mth.at<flaot>(0, 1)));
		if (cos(Q1) != cos_solution(-cos(Q6) * sin(Q5), (cos(Q2) * cos(Q5) * cos(Q6) - sin(Q2) * sin(Q6)), (cos(Q2) * cos(Q5) * cos(Q6) - sin(Q2) * sin(Q6)), cos(Q6) * sin(Q5), ee_mth.at<float>(0, 0), ee_mth.at<flaot>(0, 1)))
			Q1 = pi - Q1;
	}
	else if (sin(Q5) != 0)
	{
		Q1 = asin(-ee_mth.at<float>(0, 0) / (sin(Q5) * cos(Q6)));
		if (cos(Q1) != ee_mth.at<float>(1, 0) / (cos(Q6) * sin(Q5)))
			Q1 = pi - Q1;
	}
	else
	{
		Q1 = asin(-ee_mth.at<float>(1, 1) / (sin(Q2) * cos(Q6)));
		if (cos(Q1) != (-ee_mth.at<float>(0, 1) / (sin(Q2) * cos(Q6))))
			Q1 = pi - Q1;
	}
}

void descouple(cv::Mat mat_constante, float q3, float q2, float q4)
{
	q2 = asin(sin_solution(a4 * sin(q3), a4 * cos(q3) + a3, -(a4 * cos(q3) + a3), a4 * sin(q3), mat_constante.at<flaot>(0, 3), mat_constante.at<float>(2, 3))); //must a3 != a4 !!!! and the result is q2-pi/2
	if (cos(q2) != (cos_solution(a4 * sin(q3), a4 * cos(q3) + a3, -(a4 * cos(q3) + a3), a4 * sin(q3), mat_constante.at<flaot>(0, 3), mat_constante.at<float>(2, 3)))
		q2 = pi - q2;
	if( (q2 - q3) mod (pi/4) != 0)
	{
		q4 = asin(sin_solution(-cos(q2-q3), -sin(q2-q3), sin(q2-q3), -cos(q2-q3), mat_constante.at<float>(0, 1), mat_constante.at<flaot>(2, 1)));
		if (cos(q4) != cos_solution(-cos(q2-q3), -sin(q2-q3), sin(q2-q3), -cos(q2-q3), mat_constante.at<float>(0, 1), mat_constante.at<flaot>(2, 1)))
			q4 = pi - q1;
	}
	else if (cos(q2-q3) == sin(q2-q3))
	{
		q4 = asin((mat_constante.at<float>(2, 1) - mat_constante.at<float>(0, 1)) /(2 * cos(q2 - q3)));
		if (cos(q4) != ((-mat_constante.at<float>(2, 1) - mat_constante.at<float>(0, 1)) /(2 * cos(q2 - q3))))
			q4 = pi - q4;
	}
	else
	{
		q4 = asin((-mat_constante.at<float>(2, 1) - mat_constante.at<float>(0, 1)) /(2 * cos(q2 - q3)));
		if (cos(q4) != (mat_constante.at<float>(0, 1) - mat_constante.at<float>(2, 1)) / (2 * cos(q2 - q3)))
			q4 = pi - q4;
	}	
	q2 += pi/2;
	q4 += pi/2;
}



std::vector<std::vector<float>> inverse_kinematics(Pose target)
{
	/*
	target: target of end effector, format is
	x in m
	y in m
	z in m
	quaternion: qx, qy, qz, qw
	*/
	//2-D vector for save all solutions
	std::vector<std::vector<float>> joint_solutions;
	std::vector<std::vector<float>> simplified_joint_solutions;
	
	//from target Pose get the homogenous transformation matrix
	cv::Mat ee_mth = mth_from_quaternion(target);
	

	
	//solve angles from simplified modified DH parameters
	float Q1, Q2, Q5, Q6；
	Q2 = acos((ee_mth.at<float>(2, 3) - D1 - D4 * ee_mth.at<float>(2, 2)) / D3); 	// !!! cos(Q2) =cos(-Q2)
	if ((Q2 mod (pi/2)) != 0)
	{
		Q5 = asin(ee_mth.at<float>(2, 2) / sin(Q2));									// !!! Q2 has 2 Q5(Q5 && pi-Q5) solution, -Q2 too !!!
		situation1(Q2, Q5, ee_mth, Q6, Q1);
		simplified_joint_solutions.push_back({Q1, Q2, Q5, Q6});
		situation1(Q2, pi-Q5, ee_mth, Q6, Q1);
		simplified_joint_solutions.push_back({Q1, Q2, pi-Q5, Q6});
		situation1(-Q2, Q5, ee_mth, Q6, Q1);
		simplified_joint_solutions.push_back({Q1, -Q2, Q5, Q6});
		situation1(-Q2, pi-Q5, ee_mth, Q6, Q1);
		simplified_joint_solutions.push_back({Q1, -Q2, pi-Q5, Q6});
	}
	else
	{
		if (sin(Q2) == 0)
		{
			Q6 = asin(ee_mth.at<float>(2, 0)/cos(Q2));
			if (cos(Q6) != ee_mth.at<float>(2, 1)/cos(Q2))
				Q6 = pi - Q6;
			Q1 = asin((ee_mth.at<float>(0, 3) - D4 * ee_mth.at<flaot>(0, 2)) / D2);
			if (cos(Q1) != ((-ee_mth.at<float>(1, 3) + D4 * ee_mth.at<float>(1, 2)) / D2) )
				Q1 = pi - Q1;
			Q5 = asin(sin_solution(cos(Q1) * cos(Q2), sin(Q1), cos(Q2) * sin(Q1), -cos(Q1), ee_mth.at<float>(0, 2), ee_mth.at<flaot>(1, 2)));
			if (cos(Q5) != cos_solution(cos(Q1) * cos(Q2), sin(Q1), cos(Q2) * sin(Q1), -cos(Q1), ee_mth.at<float>(0, 2), ee_mth.at<flaot>(1, 2)))
				Q5 = pi - Q5;
			simplified_joint_solutions.push_back({Q1, Q2, Q5, Q6});
			simplified_joint_solutions.push_back({Q1, Q2 - sign(Q2) * 2 * pi, Q5, Q6});
		}
		else
		{
			Q5 = asin(ee_mth.at<float>(2, 2) / sin(Q2));
			situation2(Q2, Q5, ee_mth, Q6, Q1);
			simplified_joint_solutions.push_back({Q1, Q2, Q5, Q6});
			situation2(Q2, pi-Q5, ee_mth, Q6, Q1);
			simplified_joint_solutions.push_back({Q1, Q2, pi-Q5, Q6});
			situation2(-Q2, Q5, ee_mth, Q6, Q1);
			simplified_joint_solutions.push_back({Q1, -Q2, Q5, Q6});
			situation2(-Q2, pi-Q5, ee_mth, Q6, Q1);
			simplified_joint_solutions.push_back({Q1, -Q2, pi-Q5, Q6});
		}
	}
	
	//descouple Q2
	float q2, q3, q4;
	
	for (size_t i = 0; i < simplified_joint_solutions.size(); i++)
	{
		cv::Mat A1 = HTM_MDH(0, 0, D1, simplified_joint_solutions[i][0]);
		cv::Mat A5 = HTM_MDH(0, -pi/2, D3, simplified_joint_solutions[i][2]);
		cv::Mat A6 = HTM_MDH(0,pi/2, D4, simplified_joint_solutions[i][3]);
		cv::Mat mat_constante = A1.inv() * ee_mth * A6.inv() * A5.inv();
		
		q3 = acos((pow(mat_constante.at<float>(0, 3), 2) + pow(mat_constante.at<flaot>(2, 3), 2) - pow(a4, 2) -pow(a3, 2)) / (2 * a4 * a3));// or -q3
		descouple(mat_constante, q3, q2, q4)
		joint_solutions.push_back({simplified_joint_solutions[i][0], q2, q3, q4, simplified_joint_solutions[i][2], simplified_joint_solutions[i][3]});
		descouple(mat_constante, -q3, q2, q4)
		joint_solutions.push_back({simplified_joint_solutions[i][0], q2, -q3, q4, simplified_joint_solutions[i][2], simplified_joint_solutions[i][3]});
	}
	return joint_solutions;
}


int main()
{
	Pose target;
	
	std::vector<std::vector<float>> solutions = inverse_kinematics(target);
	
	std::cout<< solutions.size() << std::endl;
	if (solutions.size())
		for (size_t i = 0;i < solutions.size();i++)
		{
			for (size_t j = 0; j < solutions[i].size();j++)
				std::cout << solutions[i][j] << "  ";
			std::cout << std::endl;
		}
		
	return 0;
}
