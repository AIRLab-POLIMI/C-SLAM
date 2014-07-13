/*
 * Copyright (c) 2014, delmottea
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the {organization} nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <CMT.h>
#include <stdio.h>
#include <iostream>

#define DEBUG_MODE

using namespace std;

void num2str(char *str, int length, int num)
{
	for (int i = 0; i < length - 1; i++)
	{
		str[length - i - 2] = '0' + num % 10;
		num /= 10;
	}
	str[length - 1] = 0;
}

int main(int argc, char *argv[])
{
	const char *path = "/home/dave/CMT/Dataset/ball/";
	const char *ext = "png";
	int numLength = 8;
	char numString[numLength + 1];
	char filename[255];
	int start = 1;
	int end = 603;
	cv::Point2f initTopLeft(195, 108);
	cv::Point2f initBottomDown(195+54, 108+60);

	CMT cmt;
	//cmt.estimateRotation = false;
	for (int i = start; i <= end; i++)
	{
		num2str(numString, numLength + 1, i);
		sprintf(filename, "%s%s.%s", path, numString, ext);

#ifdef DEBUG_MODE
		cout << filename << endl;
#endif

		cv::Mat img = cv::imread(filename);
		cv::Mat im_gray;
		cv::cvtColor(img, im_gray, CV_RGB2GRAY);

		if (i == start)
			cmt.initialize(im_gray, initTopLeft, initBottomDown);
		cmt.processFrame(im_gray);

		for (int i = 0; i < cmt.trackedKeypoints.size(); i++)
			cv::circle(img, cmt.trackedKeypoints[i].first.pt, 3,
						cv::Scalar(255, 255, 255));
		cv::line(img, cmt.topLeft, cmt.topRight, cv::Scalar(255, 255, 255));
		cv::line(img, cmt.topRight, cmt.bottomRight, cv::Scalar(255, 255, 255));
		cv::line(img, cmt.bottomRight, cmt.bottomLeft,
					cv::Scalar(255, 255, 255));
		cv::line(img, cmt.bottomLeft, cmt.topLeft, cv::Scalar(255, 255, 255));

#ifdef DEBUG_MODE
		cout << "trackedKeypoints";
		for(int i = 0; i<cmt.trackedKeypoints.size(); i++)
			cout << cmt.trackedKeypoints[i].first.pt.x << cmt.trackedKeypoints[i].first.pt.x << cmt.trackedKeypoints[i].second << endl;
		cout << "box" << endl;
		cout << cmt.topLeft.x << cmt.topLeft.y << endl;
		cout << cmt.topRight.x << cmt.topRight.y << endl;
		cout << cmt.bottomRight.x << cmt.bottomRight.y << endl;
		cout << cmt.bottomLeft.x << cmt.bottomLeft.y << endl;
#endif

		imshow("frame", img);
		cv::waitKey(1);
	}
	return 0;
}
