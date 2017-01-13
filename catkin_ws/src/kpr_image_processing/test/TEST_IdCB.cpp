#ifndef TESTING
	#define TESTING
#endif

#include "../src/IdCB/IdCB.cpp"
#include <gtest/gtest.h>

/**
 * Creates a disparity image for testing
 *
 * @param f The focal length in pixels
 * @param T The baseline in metres
 * @param step Amount of data at 1 row
 * @param width Amount of pixels at 1 row
 * @param vecLength Total datapoints
 * @param dataPoint Point in vector where a value can be found (must be the place where depth of cucmber postion will be measures)
 * param dataPointValue The value the dataPoint has
 *
 * @return Disparity image
 */
stereo_msgs::DisparityImage CreateDisparity(int f, float T, int step, int width, int vecLength, int dataPoint, float dataPointValue){
	stereo_msgs::DisparityImage d = stereo_msgs::DisparityImage();
	d.f = f;
	d.T = T;
	sensor_msgs::Image i = sensor_msgs::Image();
	i.step = step;
	i.width = width;
	std::vector<uint8_t> arr(vecLength);
	arr[dataPoint] = dataPointValue;
	i.data = arr;
	d.image = i;
	return d;
}

/**
 * Run all the test cases.
 */
int main (int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

/**
 * Tests for to3D() function.
 * to3D(cucumber_msgs::Cucumber in, int camera, stereo_msgs::DisparityImage disparity)
 * CucumberContainer c = CucumberContainer(x,y,width(cucumber),height,curve);
 */
TEST(to3D, GoodCucumber){
	CucumberContainer c = CucumberContainer(1,2,1,10,0);
	cucumber_msgs::Cucumber msg = c.toMessage();
	int camera = CAM_LEFT;
	stereo_msgs::DisparityImage d = CreateDisparity(100, 0.1, 1, 1, 100, 51, 12);
	ASSERT_TRUE(CucumberContainer(0.068333,-0.382233,0.913621 + .5*4.65e-6 ,4.65e-6,4.65e-5,0).equals(to3D(msg, camera,d)));
}

TEST(to3D, WrongCam){
	CucumberContainer c = CucumberContainer(1,2,1,100,0);
	cucumber_msgs::Cucumber msg = c.toMessage();
	int camera = CAM_RIGHT;
	stereo_msgs::DisparityImage d = CreateDisparity(100, 0.1, 1, 1, 100, 51, 12);
	CucumberContainer res = to3D(msg, camera,d);
	ASSERT_EQ(-1, res.getCurvature());
}

TEST(to3D, OutOfRange){
	CucumberContainer c = CucumberContainer(1,2,1,100,0);
	cucumber_msgs::Cucumber msg = c.toMessage();
	int camera = CAM_LEFT;
	stereo_msgs::DisparityImage d = CreateDisparity(100, 0.1, 1, 1, 40, 51, 12);
	CucumberContainer res = to3D(msg, camera,d);
	ASSERT_EQ(-1, res.getCurvature());
}
