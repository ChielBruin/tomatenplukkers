#ifndef TESTING
	#define TESTING
#endif

#include "../src/IdCB/IdCB.cpp"
#include <gtest/gtest.h>

/**
 * Run all the test cases.
 */
int main (int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

/**
 * Tests for to3D() function.
to3D(cucumber_msgs::Cucumber in, int camera, stereo_msgs::DisparityImage disparity)
CucumberContainer c = CucumberContainer(x,y,width(cucumber),height,curve);
 */
TEST(to3D, GoodCucumber){
	CucumberContainer c = CucumberContainer(1,2,1,100,0);
	cucumber_msgs::Cucumber msg = c.toMessage();
	int camera = CAM_LEFT;
	stereo_msgs::DisparityImage d = stereo_msgs::DisparityImage();
	d.f = 100;
	d.T = 0.1;
	sensor_msgs::Image i = sensor_msgs::Image();
	i.step = 1;
	i.width = 1;
	std::vector<uint8_t> arr(50);
	arr[47] = 12;
	i.data = arr;
	d.image = i;
	ASSERT_TRUE(CucumberContainer(1/120.0,1/60.0,5/6.0,2.325e-4, 4.65e-4, 0).equals(to3D(msg, camera,d)));
}

TEST(to3D, WrongCam){
	CucumberContainer c = CucumberContainer(30,2,50,100,0);
	cucumber_msgs::Cucumber msg = c.toMessage();
	int camera = CAM_RIGHT;
	stereo_msgs::DisparityImage d = stereo_msgs::DisparityImage();
	d.f = 100;
	d.T = 0.1;
	sensor_msgs::Image i = sensor_msgs::Image();
	i.step = 1;
	i.width = 1;
	std::vector<uint8_t> arr(50);
	arr[47] = 12;
	i.data = arr;
	d.image = i;
	ASSERT_FALSE(CucumberContainer(0.25,1/60.0,5/6.0,2.325e-4, 4.65e-4, 0).equals(to3D(c.toMessage(),camera,d)));
}

TEST(to3D, OutOfRange){
	CucumberContainer c = CucumberContainer(30,2,50,45,0);
	cucumber_msgs::Cucumber msg = c.toMessage();
	int camera = CAM_LEFT;
	stereo_msgs::DisparityImage d = stereo_msgs::DisparityImage();
	d.f = 100;
	d.T = 0.1;
	sensor_msgs::Image i = sensor_msgs::Image();
	i.step = 1;
	i.width = 1;
	std::vector<uint8_t> arr(50);
	arr[47] = 12;
	i.data = arr;
	d.image = i;
	ASSERT_FALSE(CucumberContainer(0.25,1/60.0,5/6.0,2.325e-4, 2.0925e-4, 0).equals(to3D(c.toMessage(),camera,d)));
}
