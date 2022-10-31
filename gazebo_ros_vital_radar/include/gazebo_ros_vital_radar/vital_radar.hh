#ifndef GAZEBO_PLUGINS_VITALRADAR_HH_
#define GAZEBO_PLUGINS_VITALRADAR_HH_

#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/util/system.hh>

#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <vital_sign_msgs/VitalSigns.h>

namespace gazebo {
    class GZ_PLUGIN_VISIBLE VitalRadar : public SensorPlugin {
    public:
        VitalRadar();

        virtual ~VitalRadar();

    protected:

        virtual void OnNewLaserScans();

        void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);


    private:
        sensors::RaySensorPtr sensor;
        physics::WorldPtr world;

        ros::NodeHandle* node_handle_;
        ros::Publisher publisher_;
        
        vital_sign_msgs::VitalSigns multipleVitalSignsMsg;

        std::string namespace_;
        std::string topic_;
        std::string frame_id_;

        event::ConnectionPtr newLaserScansConnection;

        //parameters changeable in sdf sensor plugin
        int penetrableObjects;
        double radarPower;
        double receivableSignalArea;
        double gain;
        double minDetectablePower;
        double maxQualityThreshold;
        //damping should be >=1
        double noisePower;
        double defaultDamping;
        double wallDamping;
    };
}
#endif