#include"../include/tm_gazebo_plugin/tm_joint_plugin.hpp"
#include <cmath>

namespace gazebo_plugins
{
void TMGazeboPluginRos::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr /*_sdf*/){
    this->_model = model;
    std::cout<<"AAA"<<std::endl;
    
    this->counter  =0;
    this->_joints.resize(6);
    this->_joints[0] = this->_model->GetJoint("shoulder_1_joint");
    this->_joints[1] = this->_model->GetJoint("shoulder_2_joint");
    this->_joints[2] = this->_model->GetJoint("elbow_1_joint");
    this->_joints[3] = this->_model->GetJoint("wrist_1_joint");
    this->_joints[4] = this->_model->GetJoint("wrist_2_joint");
    this->_joints[5] = this->_model->GetJoint("wrist_3_joint");

    this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
          std::bind(&TMGazeboPluginRos::OnUpdate, this));
}

void TMGazeboPluginRos::OnUpdate(){
    //if(this->counter%1000 == 0){
    //    std::cout<<"tm joint plugin is launching !!!"<<std::endl;
    //}
    //this->counter++;
    this->counter += 0.00005;
    float joint_value = std::sin(this->counter);
    //std::cout<<"tm joint plugin joint_value is "<<joint_value<<std::endl;
    _joints[0]->SetPosition(0, joint_value, false);
    _joints[1]->SetPosition(0, joint_value, false);
    _joints[2]->SetPosition(0, joint_value, false);
    _joints[3]->SetPosition(0, joint_value, false);
    _joints[4]->SetPosition(0, joint_value, false);
    _joints[5]->SetPosition(0, joint_value, false);
}

void TMGazeboPluginRos::Reset(){
    this->counter =0;
}
GZ_REGISTER_MODEL_PLUGIN(TMGazeboPluginRos)
}
