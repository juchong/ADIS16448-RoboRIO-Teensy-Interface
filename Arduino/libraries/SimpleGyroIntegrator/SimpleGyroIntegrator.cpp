////////////////////////////////////////////////////////////////////////////////////////////////////////
//  January 2017
//  Author: Juan Jose Chong <juan.chong@analog.com>
////////////////////////////////////////////////////////////////////////////////////////////////////////
//  SimpleGyroIntegrator.cpp
////////////////////////////////////////////////////////////////////////////////////////////////////////
// 
//  This library integrates gyroscope outputs over time and provides a heading output.
//
//  Permission is hereby granted, free of charge, to any person obtaining
//  a copy of this software and associated documentation files (the
//  "Software"), to deal in the Software without restriction, including
//  without limitation the rights to use, copy, modify, merge, publish,
//  distribute, sublicense, and/or sell copies of the Software, and to
//  permit persons to whom the Software is furnished to do so, subject to
//  the following conditions:
//
//  The above copyright notice and this permission notice shall be
//  included in all copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
//  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
//  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
//  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
//  LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
//  WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "SimpleGyroIntegrator.h"

SimpleGyroIntegrator::SimpleGyroIntegrator() {
	intx = 0.0f;
	inty = 0.0f;
	intz = 0.0f;
}

void SimpleGyroIntegrator::update(float gx, float gy, float gz, unsigned long deltat, int resetGyroFlag) {
	invSampleFreq = deltat * 0.000001f;
	if (resetGyroFlag == 0) {
		intx += gx * invSampleFreq;
		inty += gy * invSampleFreq;
		intz += gz * invSampleFreq;
	}

	if (resetGyroFlag == 1) {
		intx = 0.0f;
		inty = 0.0f;
		intz = 0.0f;
	}

}