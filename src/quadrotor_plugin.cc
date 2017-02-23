#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <stdio.h>

namespace gazebo
{
    class ModelPush : public ModelPlugin
    {
        public:
            void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
            {
                //Store the pointer to the model
                this->model = _model;

                // Listen to the update event. This event is broadcast every
                // simulation iteration
                this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                    boost::bind(&ModelPush::OnUpdate, this, _1));
            }

            // Called by the world update start event
            void OnUpdate(const common::UpdateInfo& /*_info*/)
            {
                // Apply a small linear velocity to the model
                this->model->SetLinearVel(math::Vector3(0.01, 0, 0));
            }

        private:
            // Pointer to the model
            physics::ModelPtr model;

            event::ConnectionPtr updateConnection;
    };

    GZ_REGISTER_MODEL_PLUGIN(ModelPush);
}

