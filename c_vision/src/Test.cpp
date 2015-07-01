/*
 * c_vision,
 *
 *
 * Copyright (C) 2015 Davide Tateo
 * Versione 1.0
 *
 * This file is part of c_vision.
 *
 * c_vision is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * c_vision is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with c_vision.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "FHOMT/FuzzyMorphologicalOperators.h"

using namespace cv;

int main()
{
	Mat_<double> A(8, 4);
	A << 0, 0, 0, 0, //
	0, 0, 0, 0,      //
	0, 1, 0, 0,      //
	0, 1, 0, 0,      //
	0, 1, 0, 0,      //
	0, 1, 0, 0,      //
	0, 0, 0, 0,      //
	0, 0, 0, 0;      //

	/*Structuring Elements*/
	Mat_<double> Bc(3, 3);
	Bc << 0, 1, 0, //
	0, 1, 0, //
	0, 1, 0; //

	Mat_<double> Bf(3, 3);
	Bf << 0, 0.2, 0, //
	0, 1, 0, //
	0, 0.2, 0; //

	Mat_<double> B(3, 3);
	B << -1, 0.2, -1, //
	-0.2, 1, -0.2, //
	-1, 0.2, -1; //

	/* Fuzzy Operators */
	LukasiewiczConjunction T;
	LukasiewiczImplication I;

	/* Test erosion and dilation */
	Mat Ec = MorphOp::f_erode(A, Bc, I);
	Mat Ef = MorphOp::f_erode(A, Bf, I);

	Mat Dc = MorphOp::f_dilate(A, Bc, T);
	Mat Df = MorphOp::f_dilate(A, Bf, T);

	/* Test fuzzy HoMT */

	Mat R = MorphOp::f_homt(A, B, T, I);

	/* output data */

	std::cout << "A: "  << std::endl << A << std::endl;
	std::cout << "Ec: " << std::endl << Ec << std::endl;
	std::cout << "Ef: " << std::endl << Ef << std::endl;
	std::cout << "Dc: " << std::endl << Dc << std::endl;
	std::cout << "Df: " << std::endl << Df << std::endl;
	std::cout << "R: "  << std::endl << R << std::endl;
}

