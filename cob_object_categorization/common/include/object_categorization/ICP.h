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
// ICP.h - Declaration of CICP
// -----------------------------------------------------------------
// *****************************************************************

// *****************************************************************
// Author:  Pedram Azad
// Date:    6.9.2003
// *****************************************************************


// *****************************************************************
// double-include protetction
// *****************************************************************

#ifndef __I_C_P_H__
#define __I_C_P_H__


// *****************************************************************
// forward declarations
// *****************************************************************

struct Vec3d;
struct Mat3d;


// *****************************************************************
// defines
// *****************************************************************



// *****************************************************************
// CICP
// *****************************************************************

class CICP
{
public:
	// public methods
	static bool CalculateOptimalTransformation(const Vec3d *pSourcePoints, const Vec3d *pTargetPoints, int nPoints, Mat3d &rotation, Vec3d &translation);	
};



#endif /* __I_C_P_H__ */
