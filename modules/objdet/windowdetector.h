/** @file windowdetector.h */
#include <opencv2/core/utility.hpp>
#include <opencv2/ximgproc.hpp>
#include <iostream>

/**
 * @class WindowDetector
 * @brief Get windows/doors and rectangle-ish objects in the image
 *
 * The edges in the image are extracted using Structured Edge Detection
 * and converted in to connected contours. A bounding rectangle over
 * contours which meet certain criteria(min area, w/h ratio etc.) is 
 * drawn.
 * TODO: Use pixel coords of the rectangle to get real world coords
 * from the point cloud
 */
class WindowDetector
{
	private:
		cv::Mat inputIm; ///< input image
		cv::Mat edgeIm; ///< edge image
		cv::Ptr<cv::ximgproc::StructuredEdgeDetection> seDetect;
	
	public:
		/**
		 * Initializes the struct edge detector.
		 * @param model_file Path to the trained model file
		 */
		WindowDetector(std::string model_file)
			:seDetect(cv::ximgproc::createStructuredEdgeDetection(model_file))
		{}

		/**
		 * Reads image from file, loads into private variable 
		 * and converts to float datatype.
		 * @param filepath Path to input image.
		 * @return 0 on success, -1 on failure.
		 */
		int readImage(std::string filepath);

		/**
		 * Detects edges in image using structured edge detection
		 */
		void detectEdges();

		/**
		 * Finds contours in edge image and detect rectangles.
		 * @param debug Verbose mode and dump intermediate images
		 * for debugging
		 * TODO: Find a way to get rectangle coordinates.
		 */
		void detectRectangles(std::vector<cv::Rect> &boundRectOut, bool debug = false);

		/**
		 * Ain't nothing but a destructor.
		 */
		~WindowDetector();
};
