#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "windowdetector.h"

int WindowDetector::readImage(std::string filepath)
{
	try
	{
		inputIm = cv::imread(filepath, 1);
	}
	catch(...)
	{
		std::cerr << "Error in reading image." << std::endl;
		std::cerr << "Check filepath" << std::endl;
		return -1;
	}
	// Convert to float
	inputIm.convertTo(inputIm, cv::DataType<float>::type,
			1/255.0f);
	return 0;
}

void WindowDetector::detectEdges()
{
	// Initialize edges
	//edgeIm = cv::Mat(inputIm.size(), inputIm.type());
	seDetect->detectEdges(inputIm, edgeIm);

	edgeIm.convertTo(edgeIm, CV_8UC1, 255);
}

void WindowDetector::detectRectangles(std::vector<cv::Rect> &boundRectOut,
		bool debug)
{
	if(debug)
		cv::imwrite("edges.png", edgeIm);

	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> h;

	cv::Canny(edgeIm, edgeIm, 125, 255);
	cv::Mat elem_h = cv::Mat::ones(3, 1, CV_8U);
	cv::Mat elem_v = cv::Mat::ones(1, 3, CV_8U);
	cv::dilate(edgeIm, edgeIm, elem_h);
	cv::dilate(edgeIm, edgeIm, elem_v);
	if(debug)
		cv::imwrite("edges_new.png", edgeIm);
	cv::findContours(edgeIm, contours, h, CV_RETR_EXTERNAL, 
			CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

	std::vector<std::vector<cv::Point> > contours_poly(contours.size());
	std::vector<cv::Rect> boundRect(contours.size());
	std::vector<cv::Point2f> center(contours.size());
	
	if(debug)
		std::cout << "Contours size: " << contours.size() << std::endl;

	for(int i = 0; i < contours.size(); ++i)
	{
		cv::approxPolyDP(cv::Mat(contours[i]), contours_poly[i], 30, true);
		boundRect[i] = cv::boundingRect(cv::Mat(contours_poly[i]));
	}


	for(int i = 0; i < contours.size(); ++i)
	{
		float aspect_ratio = boundRect[i].height / (float )boundRect[i].width;
		float area = boundRect[i].width * boundRect[i].height;

		if((aspect_ratio > 0.2f) && (aspect_ratio < 1.5f) && (area > 2000.0f))
		{
			boundRectOut.push_back(boundRect[i]);
			cv::rectangle(edgeIm, boundRect[i].tl(), boundRect[i].br(), cv::Scalar(0, 255, 0), 2, 6, 0);
		}
	}
	
	//cv::drawContours(edgeIm, contours_poly, 0, cv::Scalar(0, 255, 0));

	if(debug)
		cv::imwrite("rectangles.png", edgeIm);
}

WindowDetector::~WindowDetector()
{
	delete seDetect;
}
