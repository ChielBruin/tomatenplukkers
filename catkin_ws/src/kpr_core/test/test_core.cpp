#ifndef TESTING
	#define TESTING
#endif

#include "../src/core/core.cpp"
#include <gtest/gtest.h>

CucumberContainer testCucumberContainer(float width, float height, float curvature) {
	return CucumberContainer(0, 0, 0, width, height, curvature);
}

void setSettings(float minWeight, float maxWeight, float maxCurvature) {
	std::map<std::string, std::string> settings = std::map<std::string, std::string>();
	settings.insert(std::pair<std::string, std::string>("minWeight", std::to_string(minWeight)));
	settings.insert(std::pair<std::string, std::string>("maxWeight", std::to_string(maxWeight)));
	settings.insert(std::pair<std::string, std::string>("maxCurvature", std::to_string(maxCurvature)));
	setSettings(settings);
}

/**
 * Run all the test cases.
 */
int main (int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

/*
 * Tests for checkHarvestable() method.
 */
TEST(testIsHarvestable, testTooSmall) {
	setSettings(100, 200, 0);
	CucumberContainer c = testCucumberContainer(1, 1, 0);
	ASSERT_FALSE(checkHarvestable(c)) << "This small cucumber should not be harvested";
}

TEST(testIsHarvestable, testTooBig) {
	setSettings(10, 20, 0);
	CucumberContainer c = testCucumberContainer(100, 100, 0);
	ASSERT_FALSE(checkHarvestable(c)) << "This huge cucumber should not be harvested";
}

TEST(testIsHarvestable, testGood) {
	setSettings(400, 900, 0);
	CucumberContainer c = testCucumberContainer(10, 10, 0);
	ASSERT_TRUE(checkHarvestable(c)) << "This perfect cucumber should be harvested";
}

TEST(testIsHarvestable, testCurvey) {
	setSettings(400, 900, 0);
	CucumberContainer c = testCucumberContainer(10, 10, 4);
	ASSERT_FALSE(checkHarvestable(c)) << "This curvey cucumber should not be harvested";
}

TEST(testIsHarvestable, testAllWrong) {
	setSettings(10, 10, 0);
	CucumberContainer c = testCucumberContainer(10, 10, 4);
	ASSERT_FALSE(checkHarvestable(c)) << "This cucumber should never be harvested";
}
