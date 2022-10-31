#include <ros/ros.h>
#include <vital_sign_msgs/VitalSigns.h>
#include <iostream>
#include <fstream>
#include <cmath>


void chatterCallback(const vital_sign_msgs::VitalSigns::ConstPtr& msg)
{
    std::fstream vitalSignData;
    vitalSignData.open("/home/philipp/Uni/10_SoSe/BachelorSimulation/Data/Messages/complexWorld_drive.txt",
                       std::fstream::app | std::fstream::in | std::fstream::out);
    //code for one human in world
    /*if (msg->measurements.size() == 0) {
        vitalSignData << "0 0 0 0 0" << std::endl;
    } else {
        double distance = sqrt(pow(msg->measurements[0].location.x, 2.0) +
                               pow(msg->measurements[0].location.y, 2.0) +
                               pow(msg->measurements[0].location.z, 2.0));

        if (vitalSignData) {
            vitalSignData << "" << distance << " " << msg->measurements[0].heart_rate << " " <<
                   msg->measurements[0].heart_rate_variance << " " <<
                   msg->measurements[0].breathing_rate << " " <<
                   msg->measurements[0].breathing_rate_variance << std::endl;

            vitalSignData.close();
        } else {
            printf("File does not exist \n");
        }
    }
    */
    //code for complex World
    if(!vitalSignData) {
        printf("File does not exist \n");
    }
    int numHumans = 5;
    for (int i = 0; i < numHumans; i++) {
        if (i < msg->measurements.size()) {
            double distance = sqrt(pow(msg->measurements[i].location.x, 2.0) +
                                   pow(msg->measurements[i].location.y, 2.0) +
                                   pow(msg->measurements[i].location.z, 2.0));
            vitalSignData << "" << distance << " " << msg->measurements[i].heart_rate << " " <<
                          msg->measurements[i].heart_rate_variance << " " <<
                          msg->measurements[i].breathing_rate << " " <<
                          msg->measurements[i].breathing_rate_variance << " ";
        } else if (msg->measurements.size() == 0) {
            vitalSignData << "" << 0 << " " << 0 << " " << 0 << " " << 0 << " " << 0 << " "
                    << 0 << " " << 0 << " " << 0 << " " << 0 << " " << 0 << " "
                    << 0 << " " << 0 << " " << 0 << " " << 0 << " " << 0 << " "
                    << 0 << " " << 0 << " " << 0 << " " << 0 << " " << 0 << " "
                    << 0 << " " << 0 << " " << 0 << " " << 0 << " " << 0 << std::endl;
            break;
        } else {
            vitalSignData << " " << 0 << " " << 0 << " " << 0 << " " << 0 << " " << 0;
        }
    }
    vitalSignData << std::endl;
    vitalSignData.close();

}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("vitalRadar", 100, chatterCallback);

  ros::spin();
  return 0;
}