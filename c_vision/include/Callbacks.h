/*
 * c_vision,
 *
 *
 * Copyright (C) 2013 Davide Tateo
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

#ifndef CALLBACKS_H_
#define CALLBACKS_H_

///FAST threshold
void thresholdCorner(int value, void* data);

///Cluster max distance
void maxDistanceCluster(int maxDistance, void* data);
///Cluster min points
void minPointsCluster(int minPoints, void* data);


///Canny min threshold trackbar callback
void minCanny(int value, void* data);
///Canny max threshold trackbar callback
void maxCanny(int value, void* data);
///HoughLineP threshold trackbar callback
void thresholdHoughP(int value, void* data);
///HoughLineP minimum line length
void minLineLengthHoughP(int value, void* data);
///HoughLineP max line gap trackbar callback
void maxLineGapHoughP(int value, void* data);


#endif /* CALLBACKS_H_ */
