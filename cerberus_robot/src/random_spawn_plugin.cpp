#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <functional>

using namespace gazebo;

class RandomSpawnPlugin : public ModelPlugin
{
public:
    void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
        // Make sure the ROS node for Gazebo has already been initialized
        if (!ros::isInitialized())
        {
            ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                                     << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
            return;
        }

        // Store the pointer to the model
        this->model = _parent;

        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                std::bind(&RandomSpawnPlugin::OnUpdate, this));

        this->setRandomPosition();
    }

    // Called by the world update start event
    void OnUpdate()
    {

    }

private:
    // Pointer to the model
    physics::ModelPtr model;

    // Pointer to the update event connection
    event::ConnectionPtr updateConnection;

    void setRandomPosition()
    {
        float x_pos_rand = randomFloat(-0.7, 0.7);
        float y_pos_rand = randomFloat(-0.7, 0.7);

        // replace 0 and -0.4 with x_pos_rand and y_pos_rand for random positions
        ignition::math::Pose3d random_pose(ignition::math::Vector3d(0, -0.4, 1e-4 - 0.0025),
                                           ignition::math::Quaterniond(0.0, 0.0, 0.0));

        this->model->SetWorldPose(random_pose);
    }

    double randomFloat(double fMin, double fMax)
    {
        double f = (double)rand() / RAND_MAX;
        return f * (fMax - fMin) + fMin;
    }
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(RandomSpawnPlugin);
