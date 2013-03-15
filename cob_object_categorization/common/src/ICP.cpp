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
// -----------------------------------------------------------------
// ICP.cpp - Implementation of CICP
// -----------------------------------------------------------------
// *****************************************************************

// *****************************************************************
// Author:  Pedram Azad
// Date:    06.09.2003
// *****************************************************************


// *****************************************************************
// includes
// *****************************************************************

#include "object_categorization/ICP.h"
#include "object_categorization/Math3d.h"

#include <stdio.h>
#include <math.h>



// *****************************************************************
// internal functions
// *****************************************************************

#define JACOBI_ROTATE(a, i, j, k, l) g = a[i][j]; h = a[k][l]; a[i][j] = g - s * (h + g * tau); a[k][l] = h + s * (g - h * tau)
#define JACOBI_MAX_ROTATIONS		20

static void Jacobi4(double **a, double *w, double **v)
{
	int i, j, k, l;
	double b[4], z[4];

	// initialize
	for (i = 0; i < 4; i++) 
	{
		for (j = 0; j < 4; j++)
			v[i][j] = 0.0;

		v[i][i] = 1.0;
		z[i] = 0.0;
		b[i] = w[i] = a[i][i];
	}
  
	// begin rotation sequence
	for (i = 0; i < JACOBI_MAX_ROTATIONS; i++) 
	{
		double sum = 0.0;
		
		for (j = 0; j < 3; j++) 
		{
			for (k = j + 1; k < 4; k++)
				sum += fabs(a[j][k]);
		}
    
		if (sum == 0.0)
			break;

		const double tresh = (i < 3) ? 0.2 * sum / 16 : 0;

		for (j = 0; j < 3; j++)
		{
			for (k = j + 1; k < 4; k++) 
			{
				double g = 100.0 * fabs(a[j][k]);
				double theta, t;

				// after for runs
				if (i > 3 && (fabs(w[j]) + g) == fabs(w[j]) && (fabs(w[k]) + g) == fabs(w[k]))
				{
					a[j][k] = 0.0;
				}
				else if (fabs(a[j][k]) > tresh) 
				{
					double h = w[k] - w[j];
					
					if ((fabs(h) + g) == fabs(h))
					{
						t = (a[j][k]) / h;
					}
					else 
					{
						theta = 0.5 * h / a[j][k];
						t = 1.0 / (fabs(theta) + sqrt(1.0 + theta * theta));
            
						if (theta < 0.0)
							t = -t;
					}

					double c = 1.0 / sqrt(1.0 + t * t);
					double s = t * c;
					double tau = s / (1.0 + c);
					h = t * a[j][k];
					z[j] -= h;
					z[k] += h;
					w[j] -= h;
					w[k] += h;
					a[j][k] = 0.0;

					// j already shifted left by 1 unit
					for (l = 0; l < j; l++) 
					{
						JACOBI_ROTATE(a, l, j, l, k);
					}
          
					// j and k already shifted left by 1 unit
					for (l = j + 1; l < k; l++) 
					{
						JACOBI_ROTATE(a, j, l, l, k);
					}
          
					// k already shifted left by 1 unit
					for (l = k + 1; l < 4; l++) 
					{
						JACOBI_ROTATE(a, j, l, k, l);
					}
			
					for (l = 0; l < 4; l++) 
					{
						JACOBI_ROTATE(v, l, j, l, k);
					}
				}
			}
		}

		for (j = 0; j < 4; j++)
		{
			b[j] += z[j];
			w[j] = b[j];
			z[j] = 0.0;
		}
	}

	// sort eigenfunctions
	for (j = 0; j < 3; j++)
	{
		int k = j;
		
		double tmp = w[k];
    
		for (i = j + 1; i < 4; i++)
		{
			if (w[i] > tmp)
			{
				k = i;
				tmp = w[k];
			}
		}
    
		if (k != j) 
		{
			w[k] = w[j];
			w[j] = tmp;
		
			for (i = 0; i < 4; i++) 
			{
				tmp = v[i][j];
				v[i][j] = v[i][k];
				v[i][k] = tmp;
			}
		}
	}

	// Ensure eigenvector consistency (Jacobi can compute vectors that
	// are negative of one another). Compute most positive eigenvector.
	const int nCeilHalf = (4 >> 1) + (4 & 1);

	for (j = 0; j < 4; j++)
	{
		double sum = 0;

		for (i = 0; i < 4; i++)
		{
			if (v[i][j] >= 0.0)
				sum++;
		}

		if (sum < nCeilHalf)
		{
			for(i = 0; i < 4; i++)
				v[i][j] *= -1.0;
		}
	}
}

#undef JACOBI_ROTATE
#undef JACOBI_MAX_ROTATIONS


static void Perpendiculars(const double x[3], double y[3], double z[3], double theta)
{
	const double x2 = x[0] * x[0];
	const double y2 = x[1] * x[1];
	const double z2 = x[2] * x[2];
	const double r = sqrt(x2 + y2 + z2);
	int dx,	dy,	dz;

	// transpose the vector to avoid division-by-zero
	if (x2 > y2 && x2 > z2)
	{
		dx = 0;
		dy = 1;
		dz = 2;
	}
	else if (y2 > z2) 
	{
		dx = 1;
		dy = 2;
		dz = 0;
	}
	else 
	{
		dx = 2;
		dy = 0;
		dz = 1;
	}

	const double a = x[dx] / r;
	const double b = x[dy] / r;
	const double c = x[dz] / r;
	const double tmp = sqrt(a * a + c * c);

	if (theta != 0)
    {
		const double sintheta = sin(theta);
		const double costheta = cos(theta);

		if (y)
		{
			y[dx] = (c * costheta - a * b * sintheta) / tmp;
			y[dy] = sintheta * tmp;
			y[dz] = (- a * costheta - b * c * sintheta) / tmp;
		}

		if (z)
		{
			z[dx] = (-c * sintheta - a * b * costheta) / tmp;
			z[dy] = costheta * tmp;
			z[dz] = (a * sintheta - b * c * costheta) / tmp;
		}
	}
	else
	{
		if (y)
		{
			y[dx] = c / tmp;
			y[dy] = 0;
			y[dz] = - a / tmp;
		}

		if (z)
		{
			z[dx] = - a * b / tmp;
			z[dy] = tmp;
			z[dz] = - b * c / tmp;
		}
	}      
}


// *****************************************************************
// CalculateOptimalTransformation
// *****************************************************************

bool CICP::CalculateOptimalTransformation(const Vec3d *pSourcePoints, const Vec3d *pTargetPoints, int nPoints, Mat3d &rotation, Vec3d &translation)
{
	if (nPoints < 2)
	{
		printf("error: CICP::CalculateOptimalTransformation needs at least two point pairs");
		Math3d::SetMat(rotation, Math3d::unit_mat);
		Math3d::SetVec(translation, Math3d::zero_vec);
		return false;
	}

	// The solution is based on
	// Berthold K. P. Horn (1987),
	// "Closed-form solution of absolute orientation using unit quaternions,"
	// Journal of the Optical Society of America A, pp. 629-642
	// Original python implementation by David G. Gobbi.
	
	// find the centroid of each set
	Vec3d source_centroid = { 0.0f, 0.0f, 0.0f };
	Vec3d target_centroid = { 0.0f, 0.0f, 0.0f };
	
	int i;
  
	for (i = 0; i < nPoints; i++)
	{
		Math3d::AddToVec(source_centroid, pSourcePoints[i]);
		Math3d::AddToVec(target_centroid, pTargetPoints[i]);
	}

	Math3d::MulVecScalar(source_centroid, 1.0f / nPoints, source_centroid);
	Math3d::MulVecScalar(target_centroid, 1.0f / nPoints, target_centroid);
  
	// build the 3x3 matrix M
	Mat3d M = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
	
	for (i = 0; i < nPoints; i++)
	{
		Vec3d a, b;
		Mat3d matrix;

		Math3d::SubtractVecVec(pSourcePoints[i], source_centroid, a);
		Math3d::SubtractVecVec(pTargetPoints[i], target_centroid, b);
		    
		// accumulate the products a * b^T into the matrix M
		Math3d::MulVecTransposedVec(a, b, matrix);
		Math3d::AddToMat(M, matrix);
	}

	// build the 4x4 matrix N
	double N0[4], N1[4], N2[4], N3[4];
	double *N[4] = { N0, N1, N2, N3 };

	for (i = 0; i < 4; i++)
	{
		// fill N with zeros
		N[i][0] = 0.0f;
		N[i][1] = 0.0f;
		N[i][2] = 0.0f;
		N[i][3] = 0.0f;
	}

	// on-diagonal elements
	N[0][0] =  M.r1 + M.r5 + M.r9;
	N[1][1] =  M.r1 - M.r5 - M.r9;
	N[2][2] = -M.r1 + M.r5 - M.r9;
	N[3][3] = -M.r1 - M.r5 + M.r9;

	// off-diagonal elements
	N[0][1] = N[1][0] = M.r6 - M.r8;
	N[0][2] = N[2][0] = M.r7 - M.r3;
	N[0][3] = N[3][0] = M.r2 - M.r4;

	N[1][2] = N[2][1] = M.r2 + M.r4;
	N[1][3] = N[3][1] = M.r7 + M.r3;
	N[2][3] = N[3][2] = M.r6 + M.r8;

	double eigenvalues[4];
	double eigenvectors0[4], eigenvectors1[4], eigenvectors2[4], eigenvectors3[4];
	double *eigenvectors[4] = { eigenvectors0, eigenvectors1, eigenvectors2, eigenvectors3 };

	// eigen-decompose N (is symmetric)
	Jacobi4(N, eigenvalues, eigenvectors);

	// the eigenvector with the largest eigenvalue is the quaternion we want
	// (they are sorted in decreasing order for us by JacobiN)
	float w, x, y, z;

	// first: if points are collinear, choose the quaternion that 
	// results in the smallest rotation.
	if (eigenvalues[0] == eigenvalues[1] || nPoints == 2)
	{
		Vec3d s0, t0, s1, t1;
		Math3d::SetVec(s0, pSourcePoints[0]);
		Math3d::SetVec(t0, pTargetPoints[0]);
		Math3d::SetVec(s1, pSourcePoints[1]);
		Math3d::SetVec(t1, pTargetPoints[1]);

		Vec3d ds, dt;
		Math3d::SubtractVecVec(s1, s0, ds);
		Math3d::SubtractVecVec(t1, t0, dt);
		Math3d::NormalizeVec(ds);
		Math3d::NormalizeVec(dt);
  
		// take dot & cross product
		w = ds.x * dt.x + ds.y * dt.y + ds.z * dt.z;
		x = ds.y * dt.z - ds.z * dt.y;
		y = ds.z * dt.x - ds.x * dt.z;
		z = ds.x * dt.y - ds.y * dt.x;

		float r = sqrtf(x * x + y * y + z * z);
		const float theta = atan2f(r, w);

		// construct quaternion
		w = cosf(0.5f * theta);
  
		if (r != 0)
		{
			r = sinf(0.5f * theta) / r;
			x = x * r;
			y = y * r;
			z = z * r;
		}
		else // rotation by 180 degrees: special case
		{
			// rotate around a vector perpendicular to ds
			double ds_[3] = { ds.x, ds.y, ds.z };
			double dt_[3];
						
			Perpendiculars(ds_, dt_, 0, 0);
			
			r = sinf(0.5f * theta);
			x = float(dt_[0] * r);
			y = float(dt_[1] * r);
			z = float(dt_[2] * r);
		}
	}
	else
	{
		// points are not collinear
		w = (float) eigenvectors[0][0];
		x = (float) eigenvectors[1][0];
		y = (float) eigenvectors[2][0];
		z = (float) eigenvectors[3][0];
	}

	// convert quaternion to a rotation matrix
	const float ww = w * w;
	const float wx = w * x;
	const float wy = w * y;
	const float wz = w * z;

	const float xx = x * x;
	const float yy = y * y;
	const float zz = z * z;

	const float xy = x * y;
	const float xz = x * z;
	const float yz = y * z;

	rotation.r1 = ww + xx - yy - zz; 
	rotation.r4 = 2.0f * (wz + xy);
	rotation.r7 = 2.0f * (-wy + xz);

	rotation.r2 = 2.0f * (-wz + xy);  
	rotation.r5 = ww - xx + yy - zz;
	rotation.r8 = 2.0f * (wx + yz);

	rotation.r3 = 2.0f * (wy + xz);
	rotation.r6 = 2.0f * (-wx + yz);
	rotation.r9 = ww - xx - yy + zz;

	// the translation is given by the difference of the transformed
	// source centroid and the target centroid
	Vec3d temp;
	Math3d::MulMatVec(rotation, source_centroid, temp);
	Math3d::SubtractVecVec(target_centroid, temp, translation);
	
	return true;
}
