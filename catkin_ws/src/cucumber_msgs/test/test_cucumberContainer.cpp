#ifndef TESTING
	#define TESTING
#endif

#include "cucumber_msgs/CucumberContainer.h"
#include <gtest/gtest.h>

/**
 * Run all the test cases.
 */
int main (int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

TEST(testEquals, testPositionsOut) {
	CucumberContainer c1 = CucumberContainer(2, 3, 4, 10, 10, 0);
	float l = DISTANCE_THRESHOLD + .1;
	std::vector<float> d {sqrtf(.4) * l, sqrtf(.3) * l, sqrtf(.3) * l};
	CucumberContainer c2 = CucumberContainer(2 + d[0], 3 + d[1], 4 + d[2], 10, 10, 0);
	ASSERT_FALSE(c1.equals(c2)) << "The distance is greater than the maximum distance";
}

TEST(testEquals, testPositionsIn) {
	CucumberContainer c1 = CucumberContainer(2, 3, 4, 10, 10, 0);
	float l = DISTANCE_THRESHOLD - .1;
	std::vector<float> d {sqrtf(.4) * l, sqrtf(.3) * l, sqrtf(.3) * l};
	CucumberContainer c2 = CucumberContainer(2 + d[0], 3 + d[1], 4 + d[2], 10, 10, 0);
	ASSERT_TRUE(c1.equals(c2)) << "The distance is smaller than the maximum distance";
}

TEST(testEquals, testBigger) {
	CucumberContainer c1 = CucumberContainer(2, 3, 4, 10, 10, 0);
	float h = (c1.getWeight() + WEIGHT_THRESHOLD + 1) / (M_PI * (25));
	CucumberContainer c2 = CucumberContainer(2, 3, 4, 10, h, 0);
	ASSERT_FALSE(c1.equals(c2)) << "This cucumber is way too heavy";
}

TEST(testEquals, testSmaller) {
	CucumberContainer c1 = CucumberContainer(2, 3, 4, 10, 10, 0);
	float h = (c1.getWeight() - WEIGHT_THRESHOLD - 1) / (M_PI * (25));
	CucumberContainer c2 = CucumberContainer(2, 3, 4, 10, h, 0);
	ASSERT_FALSE(c1.equals(c2)) << "This cucumber is way too light";
}

TEST(testEquals, testSameSize) {
	CucumberContainer c1 = CucumberContainer(2, 3, 4, 10, 10, 0);
	float h = (c1.getWeight() + .5 * WEIGHT_THRESHOLD) / (M_PI * (25));
	CucumberContainer c2 = CucumberContainer(2, 3, 4, 10, h, 0);
	ASSERT_TRUE(c1.equals(c2)) << "This cucumber is the same size";
}

TEST(testEquals, testTooCurvey) {
	CucumberContainer c1 = CucumberContainer(2, 3, 4, 10, 10, 0);
	CucumberContainer c2 = CucumberContainer(2, 3, 4, 10, 10, CURVATURE_THRESHOLD + .1);
	ASSERT_FALSE(c1.equals(c2)) << "This cucumber is too curvey";
}

TEST(testEquals, testTooStraight) {
	CucumberContainer c1 = CucumberContainer(2, 3, 4, 10, 10, 0);
	CucumberContainer c2 = CucumberContainer(2, 3, 4, 10, 10, -CURVATURE_THRESHOLD - .1);
	ASSERT_FALSE(c1.equals(c2)) << "This cucumber is too straight";
}

TEST(testEquals, testCurvey) {
	CucumberContainer c1 = CucumberContainer(2, 3, 4, 10, 10, 0);
	CucumberContainer c2 = CucumberContainer(2, 3, 4, 10, 10, CURVATURE_THRESHOLD - .1);
	ASSERT_TRUE(c1.equals(c2)) << "This cucumber has the same curve";
}
