/*
 * c_fuzzy,
 *
 *
 * Copyright (C) 2FUZZY_MIN13 Davide Tateo
 * Versione FUZZY_MAX
 *
 * This file is part of c_fuzzy.
 *
 * c_fuzzy is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * c_fuzzy is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with c_fuzzy.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "FuzzyMF.h"

double TolMF::evaluate()
{
	if (input <= top)
		return FUZZY_MAX_V;
	else if (input >= bottom)
		return FUZZY_MIN_V;
	else
		return line(top, FUZZY_MAX_V, bottom, FUZZY_MIN_V, input);
}

double TolMF::defuzzify(double level)
{
	//TODO implementare!
	return 0;
}

double TorMF::evaluate()
{
	if (input >= top)
		return FUZZY_MAX_V;
	else if (input <= bottom)
		return FUZZY_MIN_V;
	else
		return line(bottom, FUZZY_MIN_V, top, FUZZY_MAX_V, input);
}

double TorMF::defuzzify(double level)
{
	//TODO implementare!
	return 0;
}

double TriMF::evaluate()
{
	if (input <= left || input >= right)
		return FUZZY_MIN_V;
	else if (input < center)
		return line(left, FUZZY_MIN_V, right, FUZZY_MAX_V, input);
	else if (input > center)
		return line(left, FUZZY_MAX_V, right, FUZZY_MIN_V, input);
	else
		return FUZZY_MAX_V;
}

double TriMF::defuzzify(double level)
{
	//TODO implementare!
	return 0;
}

double TraMF::evaluate()
{
	if (input <= bottomLeft || input >= bottomRight)
		return FUZZY_MIN_V;
	else if (input > bottomLeft && input < topLeft)
		return line(bottomLeft, FUZZY_MIN_V, topLeft, FUZZY_MAX_V, input);
	else if (input > topRight && input < bottomRight)
		return line(topRight, FUZZY_MAX_V, bottomRight, FUZZY_MIN_V, input);
	else
		return FUZZY_MAX_V;
}

double TraMF::defuzzify(double level)
{
	//TODO implementare!
	return 0;
}

double IntMF::evaluate()
{
	if (input >= left && input <= right)
		return FUZZY_MAX_V;
	else
		return FUZZY_MIN_V;
}

double IntMF::defuzzify(double level)
{
	//TODO implementare!
	return 0;
}

double SgtMF::evaluate()
{
	if (input == value)
		return FUZZY_MAX_V;
	else
		return FUZZY_MIN_V;
}

double SgtMF::defuzzify(double level)
{
	return value * level;
}
