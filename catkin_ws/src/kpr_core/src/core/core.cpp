#include "ros/ros.h"
#include "cucumber_msgs/CucumberContainer.h"
#include "cucumber_msgs/HarvestStatus.h"

std::vector<CucumberContainer> queue;
std::map<std::string, std::string> settings;

/**
 * Check if the given cucumber meets the harvesting requirements.
 */
bool checkHarvestable(CucumberContainer c) {
	float w = c.getWeight();
	bool weightOK = w > stof(settings["minWeight"]) && w < stof(settings["maxWeight"]);
	bool curvatureOK = c.getCurvature() <= stof(settings["maxCurvature"]);
	return weightOK && curvatureOK;
}

/**
 * Check if the queue already contains a cucumber that is the same as the given cucumber.
 * @return True when the cucumber is already in the queue, false otherwise
 */
bool queueContains(CucumberContainer c) {
	for (CucumberContainer &cucumber : queue) {
		if (cucumber.equals(c)) return true;
	}
	return false;
}

/**
 * Add the given cucumber to the harvest queue.
 * This method also checks if the produce is of harvestable size 
 * and if it is not already contained in the queue.
 */
void push_back(CucumberContainer c) {
	if (!checkHarvestable(c)) return;
	if (queueContains(c)) return;		// prevent duplicates
	queue.push_back(c);
}

/**
 * Check if there are cucumbers detected that are ready to harvest.
 */
bool hasNext() {
	return !queue.empty();
}

/**
 * Gets the first cucumber that can be harvested and removed it from the queue.
 */
CucumberContainer pop() {
	CucumberContainer c = queue.front();
	queue.erase(queue.begin());				// pop
	return c;
}

/**
 * Set the current settings to the values specified by th given map.
 */
void setSettings(std::map<std::string, std::string> newSettings) {
	if(settings.size() != settings.size()) {
		ROS_WARN("The size of the old and the new settings is not the same, new values are omitted!");
		return;
	}
	settings = newSettings;
}

bool processResult(cucumber_msgs::HarvestStatus msg) {
	if(msg.success != cucumber_msgs::HarvestStatus::OK) {
		if(msg.success == cucumber_msgs::HarvestStatus::ERROR) {
			ROS_WARN("%s", msg.message.c_str());
			return true;
		} else {
			ROS_FATAL("%s", msg.message.c_str());
			return false;
		}
	}
	return true;
}
