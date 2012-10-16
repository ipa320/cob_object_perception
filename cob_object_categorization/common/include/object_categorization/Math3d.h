// *****************************************************************
// This file is part of the IVT - Integrating Vision Toolkit.
//
// Copyright (C) 2004-2008 Pedram Azad, Chair Prof. Dillmann (IAIM),
// Institute for Computer Science and Engineering,
// University of Karlsruhe. All rights reserved.
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public
// License along with this program; if not, write to the Free
// Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
// Boston, MA 02110-1301, USA.
// *****************************************************************

// *****************************************************************
// Filename:  Math3d.h
// Copyright: Pedram Azad, Chair Prof. Dillmann (IAIM),
//            Institute for Computer Science and Engineering (CSE),
//            University of Karlsruhe. All rights reserved.
// Author:    Pedram Azad
// Date:      2004
// *****************************************************************


#ifndef _MATH_3D_H_
#define _MATH_3D_H_

/** \defgroup Math Mathematic Routines (directory Math)
 */
// *****************************************************************
// necessary includes
// *****************************************************************

#include <vector>


// *****************************************************************
// structs and typedefs
// *****************************************************************

struct Vec3d
{
	float x, y, z;
};

struct Mat3d
{
	float r1, r2, r3, r4, r5, r6, r7, r8, r9;
};

struct Transformation3d
{
	Mat3d rotation;
	Vec3d translation;
};

struct Quaternion
{
	Vec3d v;
	float w;
};

typedef std::vector<Vec3d> Vec3dList;



// *****************************************************************
// Math3d
// *****************************************************************
/** \ingroup Math
 *  \brief Data structure and operations for efficiently calculating with vectors and matrices in 3D
 */
namespace Math3d
{
	void SetVec(Vec3d &vec, float x, float y, float z);
	void SetVec(Vec3d &vec, const Vec3d &sourceVector);
	void SetMat(Mat3d &matrix, float r1, float r2, float r3, float r4, float r5, float r6, float r7, float r8, float r9);
	void SetMat(Mat3d &matrix, const Mat3d &sourceMatrix);
	void SetRotationMat(Mat3d &matrix, const Vec3d &axis, float theta);
	void SetRotationMat(Mat3d &matrix, float alpha, float beta, float gamma);
	void SetRotationMat(Mat3d &matrix, const Vec3d &rotation);
	void SetRotationMatYZX(Mat3d &matrix, const Vec3d &rotation);
	void SetRotationMatX(Mat3d &matrix, float theta);
	void SetRotationMatY(Mat3d &matrix, float theta);
	void SetRotationMatZ(Mat3d &matrix, float theta);
	void SetRotationMatAxis(Mat3d &matrix, const Vec3d &axis, float theta);

	void MulMatVec(const Mat3d &matrix, const Vec3d &vec, Vec3d &result);
	void MulMatVec(const Mat3d &matrix, const Vec3d &vector1, const Vec3d &vector2, Vec3d &result);
	void MulMatMat(const Mat3d &matrix1, const Mat3d &matrix2, Mat3d &result);

	void MulVecTransposedVec(const Vec3d &vector1, const Vec3d &vector2, Mat3d &result);
	
	void RotateVec(const Vec3d &vec, const Vec3d &rotation, Vec3d &result);
	void RotateVecYZX(const Vec3d &vec, const Vec3d &rotation, Vec3d &result);
	void TransformVec(const Vec3d &vec, const Vec3d &rotation, const Vec3d &translation, Vec3d &result);
	void TransformVecYZX(const Vec3d &vec, const Vec3d &rotation, const Vec3d &translation, Vec3d &result);
	
	void MulVecScalar(const Vec3d &vec, float scalar, Vec3d &result);
	void MulMatScalar(const Mat3d &matrix, float scalar, Mat3d &result);

	void AddMatMat(const Mat3d &matrix1, const Mat3d &matrix2, Mat3d &matrix);
	void AddToMat(Mat3d &matrix, const Mat3d &matrixToAdd);
	void SubtractMatMat(const Mat3d &matrix1, const Mat3d &matrix2, Mat3d &result);
		
	void AddVecVec(const Vec3d &vector1, const Vec3d &vector2, Vec3d &result);
	void SubtractVecVec(const Vec3d &vector1, const Vec3d &vector2, Vec3d &result);
	void AddToVec(Vec3d &vec, const Vec3d &vectorToAdd);
	void SubtractFromVec(Vec3d &vec, const Vec3d &vectorToSubtract);
	
	void CrossProduct(const Vec3d &vector1, const Vec3d &vector2, Vec3d &result);
	float ScalarProduct(const Vec3d &vector1, const Vec3d &vector2);
	float SquaredLength(const Vec3d &vec);
	float Length(const Vec3d &vec);
	float Distance(const Vec3d &vector1, const Vec3d &vector2);
	float SquaredDistance(const Vec3d &vector1, const Vec3d &vector2);
	float Angle(const Vec3d &vector1, const Vec3d &vector2);
	float Angle(const Vec3d &vector1, const Vec3d &vector2, const Vec3d &axis);
	float EvaluateForm(const Vec3d &matrix1, const Mat3d &matrix2); // matrix1^T * matrix2 * matrix1
	
	void NormalizeVec(Vec3d &vec);
	
	void Transpose(const Mat3d &matrix, Mat3d &result);
	void Invert(const Mat3d &matrix, Mat3d &result);
	float Det(const Mat3d &matrix);

	void SetTransformation(Transformation3d &transformation, const Vec3d &rotation, const Vec3d &translation);
	void SetTransformation(Transformation3d &transformation, const Transformation3d &sourceTransformation);
	void Invert(const Transformation3d &input, Transformation3d &result);
	void MulTransTrans(const Transformation3d &transformation1, const Transformation3d &transformation2, Transformation3d &result);
	void MulTransVec(const Transformation3d &transformation, const Vec3d &vec, Vec3d &result);

	void MulQuatQuat(const Quaternion &quat1, const Quaternion &quat2, Quaternion &result);
	void RotateVecQuaternion(const Vec3d &vec, const Vec3d &axis, float theta, Vec3d &result);
	void RotateVecAngleAxis(const Vec3d &vec, const Vec3d &axis, float theta, Vec3d &result);
	void GetAxisAndAngle(const Mat3d &R, Vec3d &axis, float &angle);

	extern Vec3d zero_vec;
	extern Mat3d unit_mat;
	extern Mat3d zero_mat;
}



#endif /* _MATH_3D_H_ */
