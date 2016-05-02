 /**
 *  This file is part of tum_ardrone.
 *
 *  Copyright 2012 Jakob Engel <jajuengel@gmail.com> (Technical University of Munich)
 *  For more information see <https://vision.in.tum.de/data/software/tum_ardrone>.
 *
 *  tum_ardrone is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  tum_ardrone is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with tum_ardrone.  If not, see <http://www.gnu.org/licenses/>.
 */
 
#include "TooN/TooN.h"
#include "TooN/so3.h"
#include "TooN/se3.h"

inline static TooN::SO3<float> rpy2rodToon(float roll, float pitch, float yaw)
{
	TooN::Matrix<3,3> mat;

	pitch /= 180/3.14159265;
	roll /= 180/3.14159265;
	yaw /= -180/3.14159265;

	float sa = sin(yaw);	// a is yaw = psi
	float ca = cos(yaw);
	float sb = sin(roll);	// b is roll = phi
	float cb = cos(roll);
	float sg = sin(pitch);	// g is pitch = theta
	float cg = cos(pitch);

	mat(0,0) = ca*cb;
	mat(0,1) = sa*cb;
	mat(0,2) = -sb;

	mat(1,0) = ca*sb*sg-sa*cg;
	mat(1,1) = sa*sb*sg+ca*cg;
	mat(1,2) = cb*sg;

	mat(2,0) = ca*sb*cg+sa*sg;
	mat(2,1) = sa*sb*cg-ca*sg;
	mat(2,2) = cb*cg;

	//mat = mat.T();

	TooN::SO3<float> res = mat;
	return res.inverse();
}


inline static void rod2rpyToon(TooN::SO3<float> trans, float* roll, float* pitch, float* yaw)
{
	TooN::Matrix<3,3> mat = trans.inverse().get_matrix();//.T();

	*roll = atan2(-mat(0,2),sqrt(mat(0,0)*mat(0,0) + mat(0,1)*mat(0,1)));
	*yaw = atan2(mat(0,1)/cos(*roll),mat(0,0)/cos(*roll));
	*pitch = atan2(mat(1,2)/cos(*roll),mat(2,2)/cos(*roll));

	*pitch *= 180/3.14159265;
	*roll *= 180/3.14159265;
	*yaw *= -180/3.14159265;


	while(*pitch > 180) *pitch -= 360;
	while(*pitch < -180) *pitch += 360;
	while(*roll > 180) *roll -= 360;
	while(*roll < -180) *roll += 360;
	while(*yaw > 180) *yaw -= 360;
	while(*yaw < -180) *yaw += 360;
}

// input in rpy
TooN::SE3<float> getCurrentFrontToGlobalTransformation(float newX, float newY, float newZ, float newRoll, float newPitch, float newYaw)
{	

	TooN::SE3<float> globaltoDrone;
	TooN::SE3<float> globalToFront;
	TooN::SE3<float> frontToGlobal;
	TooN::SE3<float> droneToGlobal;
	//0.025 -0.2
	const TooN::SE3<float> droneToFront = TooN::SE3<float>(TooN::SO3<float>(TooN::makeVector(3.14159265/2,0,0)),TooN::makeVector(0,0,-0.0));
	const TooN::SE3<float> frontToDrone = droneToFront.inverse();

	// set se3
	droneToGlobal.get_translation()[0] = newX;
	droneToGlobal.get_translation()[1] = newY;
	droneToGlobal.get_translation()[2] = newZ;
	droneToGlobal.get_rotation() = rpy2rodToon(newRoll,newPitch,newYaw);

	globaltoDrone = droneToGlobal.inverse();

	// set rest
	globalToFront = droneToFront * globaltoDrone;
	frontToGlobal = globalToFront.inverse();
	
	return frontToGlobal;
}



