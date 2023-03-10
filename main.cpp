#include <iostream>
#include <stdlib.h>
#include "BitmapRawConverter.h"
#include <tbb/tick_count.h>
#include <tbb/task_group.h>
using namespace std;
using namespace tbb;

#define __ARG_NUM__				6
#define FILTER_SIZE				3
#define OFFSET					FILTER_SIZE / 2
#define THRESHOLD				128
#define CUTOFF					128
#define NEIGHBOUR_DEPTH         1

using namespace std;

// Prewitt operators
//int filterHor[FILTER_SIZE * FILTER_SIZE] = { 1, 1, 0, -1, -1, 1, 1, 0, -1, -1, 1, 1, 0, -1, -1, 1, 1, 0, -1, -1, 1, 1, 0, -1, -1 };
//int filterVer[FILTER_SIZE * FILTER_SIZE] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 };
int filterHor[FILTER_SIZE * FILTER_SIZE] = {-1, 0, 1, -1, 0, 1, -1, 0, 1};
int filterVer[FILTER_SIZE * FILTER_SIZE] = {-1, -1, -1, 0, 0, 0, 1, 1, 1};
//int filterHor[FILTER_SIZE * FILTER_SIZE] = { 1, 1, 1, 0, -1, -1, -1, 1, 1, 1, 0, -1, -1, -1, 1, 1, 1, 0, -1, -1, -1, 1, 1, 1, 0, -1, -1, -1 , 1, 1, 1, 0, -1, -1, -1 , 1, 1, 1, 0, -1, -1, -1 , 1, 1, 1, 0, -1, -1, -1 };
//int filterVer[FILTER_SIZE * FILTER_SIZE] = { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1 , 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };

/**
* @brief Serial version of edge detection algorithm implementation using Prewitt operator
* @param inBuffer buffer of input image
* @param outBuffer buffer of output image
* @param rowStart index of starting row
* @param rowEnd index of ending row
* @param colStart index of starting column
* @param colEnd index of ending column
* @param width image width
*/
void filter_serial_prewitt(int* inBuffer, int* outBuffer, int rowStart, int rowEnd, int colStart, int colEnd, int width) {
	for (int i = rowStart; i < rowEnd; i++) {
		for (int j = colStart; j < colEnd; j++) {
			int x_value = 0;
			int y_value = 0;
			int value = 0;
			for (int k = 0; k < FILTER_SIZE; k++) {
				for (int l = 0; l < FILTER_SIZE; l++) {
					x_value += inBuffer[(j - OFFSET + l) + (i - OFFSET + k) * width] * filterHor[l + k * FILTER_SIZE];
					y_value += inBuffer[(j - OFFSET + l) + (i - OFFSET + k) * width] * filterVer[l + k * FILTER_SIZE];
				}
			}
			value = abs(x_value) + abs(y_value);
			outBuffer[j+i*width] = (value < THRESHOLD) ? 0 : 255;
		}
	}
}
/**
* @brief Parallel version of edge detection algorithm implementation using Prewitt operator
* @param inBuffer buffer of input image
* @param outBuffer buffer of output image
* @param rowStart index of starting row
* @param rowEnd index of ending row
* @param colStart index of starting column
* @param colEnd index of ending column
* @param width image width
*
*/
void filter_parallel_prewitt(int *inBuffer, int *outBuffer, int rowStart, int rowEnd, int colStart, int colEnd, int width)
{
	if (rowEnd - rowStart <= CUTOFF) {
		filter_serial_prewitt(inBuffer, outBuffer, rowStart, rowEnd, colStart, colEnd, width);
	}
	else {
		task_group g;
		g.run([&] {filter_parallel_prewitt(inBuffer, outBuffer, rowStart, (rowStart+ rowEnd) / 2, colStart, (colStart+colEnd) / 2, width); });
		g.run([&] {filter_parallel_prewitt(inBuffer, outBuffer, rowStart, (rowStart + rowEnd) / 2, (colStart + colEnd) / 2, colEnd, width); });
		g.run([&] {filter_parallel_prewitt(inBuffer, outBuffer, (rowStart + rowEnd) / 2, rowEnd, colStart, (colStart + colEnd) / 2, width); });
		g.run([&] {filter_parallel_prewitt(inBuffer, outBuffer, (rowStart + rowEnd) / 2, rowEnd, (colStart + colEnd) / 2, colEnd, width); });
		g.wait();
	}
}

/**
* @brief Function for converting input image's pixels to BW 
* @param inBuffer buffer of input image
* @param rowStart index of starting row
* @param rowEnd index of ending row
* @param colStart index of starting column
* @param colEnd index of ending column
* @param width image width
*
*/
void prepare_for_edge_detection(int* inBuffer, int rowStart, int rowEnd, int colStart, int colEnd, int width) {
	for (int i = rowStart; i < rowEnd; i++) {
		for (int j = colStart; j < colEnd; j++) {
			int value = inBuffer[j + i * width];
			inBuffer[j + i * width] = (value < THRESHOLD) ? 0 : 1;
		}
	}
}

/**
* @brief Serial version of edge detection algorithm
* @param inBuffer buffer of input image
* @param outBuffer buffer of output image
* @param rowStart index of starting row
* @param rowEnd index of ending row
* @param colStart index of starting column
* @param colEnd index of ending column
* @param width image width
*/

void filter_serial_edge_detection(int* inBuffer, int* outBuffer, int rowStart, int rowEnd, int colStart, int colEnd, int width)
{
	for (int i = rowStart; i < rowEnd; i++) {
		for (int j = colStart; j < colEnd; j++) {
			int p_value = 0;
			int o_value = 1;
			for (int k = 0; k < 2*NEIGHBOUR_DEPTH+1; k++) {
				for (int l = 0; l < 2*NEIGHBOUR_DEPTH+1; l++) {
					if ((j - NEIGHBOUR_DEPTH + l) == j && (i - NEIGHBOUR_DEPTH + k) == i) continue;
					int pixel_value = (inBuffer[(j - NEIGHBOUR_DEPTH + l) + (i - NEIGHBOUR_DEPTH + k) * width]);
					p_value = p_value || pixel_value;
					o_value = o_value && pixel_value;
				}
			}
			int value = abs(p_value - o_value);
			outBuffer[j + i * width] = value * 255;
		}
	}
}

/**
* @brief Parallel version of edge detection algorithm
* 
* @param inBuffer buffer of input image
* @param outBuffer buffer of output image
* @param rowStart index of starting row
* @param rowEnd index of ending row
* @param colStart index of starting column
* @param colEnd index of ending column
* @param width image width
*/
void filter_parallel_edge_detection(int* inBuffer, int* outBuffer, int rowStart, int rowEnd, int colStart, int colEnd, int width)
{
	if (rowEnd - rowStart <= CUTOFF) {
		filter_serial_edge_detection(inBuffer, outBuffer, rowStart, rowEnd, colStart, colEnd, width);
	}
	else {
		task_group g;
		g.run([&] {filter_parallel_edge_detection(inBuffer, outBuffer, rowStart, (rowStart + rowEnd) / 2, colStart, (colStart + colEnd) / 2, width); });
		g.run([&] {filter_parallel_edge_detection(inBuffer, outBuffer, rowStart, (rowStart + rowEnd) / 2, (colStart + colEnd) / 2, colEnd, width); });
		g.run([&] {filter_parallel_edge_detection(inBuffer, outBuffer, (rowStart + rowEnd) / 2, rowEnd, colStart, (colStart + colEnd) / 2, width); });
		g.run([&] {filter_parallel_edge_detection(inBuffer, outBuffer, (rowStart + rowEnd) / 2, rowEnd, (colStart + colEnd) / 2, colEnd, width); });
		g.wait();
	}
}


/**
* @brief Function for running test.
*
* @param testNr test identification, 1: for serial version, 2: for parallel version
* @param ioFile input/output file, firstly it's holding buffer from input image and than to hold filtered data
* @param outFileName output file name
* @param outBuffer buffer of output image
* @param width image width
* @param height image height
*/


void run_test_nr(int testNr, BitmapRawConverter* ioFile, char* outFileName, int* outBuffer, unsigned int width, unsigned int height)
{
	if (testNr == 3 || testNr == 4)
		prepare_for_edge_detection(ioFile->getBuffer(), 0, height, 0, width, width);
	tick_count startTime = tick_count::now();
	switch (testNr)
	{
		case 1:
			cout << "Running serial version of edge detection using Prewitt operator" << endl;
			filter_serial_prewitt(ioFile->getBuffer(), outBuffer, OFFSET, height - OFFSET, OFFSET, width - OFFSET, width);
			break;
		case 2:
			cout << "Running parallel version of edge detection using Prewitt operator" << endl;
			filter_parallel_prewitt(ioFile->getBuffer(), outBuffer, OFFSET, height - OFFSET, OFFSET, width - OFFSET, width);
			break;
		case 3:
			cout << "Running serial version of edge detection" << endl;
			filter_serial_edge_detection(ioFile->getBuffer(), outBuffer, OFFSET, height - OFFSET, OFFSET, width - OFFSET, width);
			break;
		case 4:
			cout << "Running parallel version of edge detection" << endl;
			filter_parallel_edge_detection(ioFile->getBuffer(), outBuffer, OFFSET, height - OFFSET, OFFSET, width - OFFSET, width);
			break;
		default:
			cout << "ERROR: invalid test case, must be 1, 2, 3 or 4!";
			break;
	}
	tick_count endTime = tick_count::now();
	cout << "Time: " << (endTime - startTime).seconds() << "ms." << endl;
	ioFile->setBuffer(outBuffer);
	ioFile->pixelsToBitmap(outFileName);
}

/**
* @brief Print program usage.
*/
void usage()
{
	cout << "\n\ERROR: call program like: " << endl << endl; 
	cout << "ProjekatPP.exe";
	cout << " input.bmp";
	cout << " outputSerialPrewitt.bmp";
	cout << " outputParallelPrewitt.bmp";
	cout << " outputSerialEdge.bmp";
	cout << " outputParallelEdge.bmp" << endl << endl;
}

int main(int argc, char * argv[])
{

	if(argc != __ARG_NUM__)
	{
		usage();
		return 0;
	}

	BitmapRawConverter inputFile(argv[1]);
	BitmapRawConverter outputFileSerialPrewitt(argv[1]);
	BitmapRawConverter outputFileParallelPrewitt(argv[1]);
	BitmapRawConverter outputFileSerialEdge(argv[1]);
	BitmapRawConverter outputFileParallelEdge(argv[1]);

	unsigned int width, height;

	int test;
	
	width = inputFile.getWidth();
	height = inputFile.getHeight();

	int* outBufferSerialPrewitt = new int[width * height];
	int* outBufferParallelPrewitt = new int[width * height];

	memset(outBufferSerialPrewitt, 0x0, width * height * sizeof(int));
	memset(outBufferParallelPrewitt, 0x0, width * height * sizeof(int));

	int* outBufferSerialEdge = new int[width * height];
	int* outBufferParallelEdge = new int[width * height];

	memset(outBufferSerialEdge, 0x0, width * height * sizeof(int));
	memset(outBufferParallelEdge, 0x0, width * height * sizeof(int));

	// serial version Prewitt
	run_test_nr(1, &outputFileSerialPrewitt, argv[2], outBufferSerialPrewitt, width, height);

	// parallel version Prewitt
	run_test_nr(2, &outputFileParallelPrewitt, argv[3], outBufferParallelPrewitt, width, height);

	// serial version special
	run_test_nr(3, &outputFileSerialEdge, argv[4], outBufferSerialEdge, width, height);

	// parallel version special
	run_test_nr(4, &outputFileParallelEdge, argv[5], outBufferParallelEdge, width, height);

	// verification
	cout << "Verification: ";
	test = memcmp(outBufferSerialPrewitt, outBufferParallelPrewitt, width * height * sizeof(int));

	if(test != 0)
	{
		cout << "Prewitt FAIL!" << endl;
	}
	else
	{
		cout << "Prewitt PASS." << endl;
	}

	test = memcmp(outBufferSerialEdge, outBufferParallelEdge, width * height * sizeof(int));

	if(test != 0)
	{
		cout << "Edge detection FAIL!" << endl;
	}
	else
	{
		cout << "Edge detection PASS." << endl;
	}

	// clean up
	delete outBufferSerialPrewitt;
	delete outBufferParallelPrewitt;

	delete outBufferSerialEdge;
	delete outBufferParallelEdge;

	return 0;
} 