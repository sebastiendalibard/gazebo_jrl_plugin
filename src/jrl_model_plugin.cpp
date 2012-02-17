#include <jrl/mal/matrixabstractlayer.hh>
#include "gazebo_jrl_plugin/jrl_model_plugin.h"

#include <gazebo/Global.hh>
#include <gazebo/XMLConfig.hh>
#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>
#include <gazebo/ControllerFactory.hh>
#include <gazebo/World.hh>
#include <gazebo/PhysicsEngine.hh>
#include <gazebo/XMLConfig.hh>
#include <gazebo/Model.hh>
#include <gazebo/HingeJoint.hh>
#include <gazebo/SliderJoint.hh>
#include <gazebo/Simulator.hh>
#include <gazebo/gazebo.h>
#include <angles/angles.h>
#include <gazebo/GazeboError.hh>
#include <gazebo/ControllerFactory.hh>

#include <jrl/dynamics/urdf/parser.hh>

namespace gazebo{

  GZ_REGISTER_DYNAMIC_CONTROLLER("gazebo_jrl_plugin", JrlModelPlugin);

  JrlModelPlugin::JrlModelPlugin(Entity *parent)
    : Controller(parent)
  {
    parent_model_ = dynamic_cast<Model*>(parent);

    if (!parent_model_)
      gzthrow("JrlModelPlugin controller requires a Model as its parent");

    Param::Begin(&parameters);
    robotParamP = new ParamT<std::string>("robotParam", "robot_description", 0);
    robotNamespaceP = new ParamT<std::string>("robotNamespace", "/", 0);
    Param::End();

  }

  JrlModelPlugin::~JrlModelPlugin()
  {
    if (rosnode_)
      delete rosnode_;
    if (cm_)
      delete cm_;
    if (jrl_dynamic_robot_)
      delete jrl_dynamic_robot_;
  }

  void JrlModelPlugin::LoadChild(XMLConfigNode *node)
  {
    // get parameter name
    robotParamP->Load(node);
    robotParam = robotParamP->GetValue();
    robotNamespaceP->Load(node);

    if (!ros::isInitialized())
      {
	int argc = 0;
	char** argv = NULL;
	ros::init(argc,
		  argv,
		  "gazebo",
		  ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
      }
    rosnode_ = new ros::NodeHandle(robotNamespace);
    ROS_INFO("starting gazebo_ros_controller_manager plugin in ns: %s",
	     robotNamespace.c_str());

    readUrdf(node);

    if(!jrl_dynamic_robot_){
      std::cerr << "jrl_dynamic_robot not initialized.\n";
      return;
    }

    cm_ = new jrl_controller_manager::ControllerManager(jrl_dynamic_robot_,
							*rosnode_);

    std::vector<CjrlJoint *> joint_vector = jrl_dynamic_robot_->getActuatedJoints();
    std::cout << "Number of DoFs: " << jrl_dynamic_robot_->numberDof()
	      << "\nNumber of joints: " << joint_vector.size()
	      << std::endl;

    for(unsigned int i = 0; i< joint_vector.size();++i)
      {
	std::string joint_name = joint_vector[i]->getName();
	gazebo::Joint *joint = parent_model_->GetJoint(joint_name);
	if (joint)
	  {
	    this->joints_.push_back(joint);
	  }
	else
	  {
	    std::cout << "A joint named "
		      << joint_name
		      << " is not part of Mechanism Controlled joints.\n";
	    this->joints_.push_back(NULL);
	  }
	
      }

  }

  void JrlModelPlugin::readUrdf(XMLConfigNode *node)
  {
    std::string urdf_param_name;
    std::string urdf_string;
    // search and wait for robot_description on param server
    while(urdf_string.empty())
      {
	ROS_DEBUG("gazebo jrl plugin is waiting for urdf: %s on the param server.", robotParam.c_str());
	if (rosnode_->searchParam(robotParam,urdf_param_name))
	  {
	    rosnode_->getParam(urdf_param_name,urdf_string);
	    ROS_DEBUG("found upstream\n%s\n------\n%s\n------\n%s",robotParam.c_str(),urdf_param_name.c_str(),urdf_string.c_str());
	  }
	else
	  {
	    rosnode_->getParam(robotParam,urdf_string);
	    ROS_DEBUG("found in node namespace\n%s\n------\n%s\n------\n%s",robotParam.c_str(),urdf_param_name.c_str(),urdf_string.c_str());
	  }
	usleep(100000);
      }

    ROS_DEBUG("gazebo jrl plugin got urdf file from param server, parsing it...");
    jrl::dynamics::urdf::Parser parser;
    jrl_dynamic_robot_ = parser.buildFromXmlString(urdf_string,"base_joint");
  }


  ////////////////////////////////////////////////////////////////////////////////
  // Initialize the controller
  void JrlModelPlugin::InitChild()
  {
    cm_->initialize();
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Update the controller
  void JrlModelPlugin::UpdateChild()
  {

    if (gazebo::Simulator::Instance()->IsPaused()) return;

    vectorN currentConfig = 
      jrl_dynamic_robot_->currentConfiguration();

    vectorN currentVelocity =
      jrl_dynamic_robot_->currentVelocity();

    unsigned int dof = 6; //Skip free flyer 
    for (unsigned int i = 0; i < joints_.size(); ++i)
      {
	Joint *current_joint = joints_[i];
	if (!current_joint)
	  continue;

	switch(current_joint->GetType())
	  //For now, only deal with 1 dof rotation joints. 
	  {
	  case Joint::HINGE: {
	    currentConfig[dof] += 
	      angles::shortest_angular_distance(currentConfig[dof],
						current_joint->GetAngle(0).GetAsRadian());
	    currentVelocity[dof] = current_joint->GetVelocity(0);
	    ++dof;
	    break;
	  }
	    //TODO: Other cases
	    /*
	      case Joint::SLIDER: { 
	      ++dof;
	      break;
	      }
	    */
	  default:
	    abort();
	  }
      }


    jrl_dynamic_robot_->currentConfiguration(currentConfig);
    jrl_dynamic_robot_->currentVelocity(currentVelocity);
    //Publish joint states and get command
    cm_->update();
  
    //Take-in command as joint efforts

    dof = 0;
    for (unsigned int i = 1; i < joints_.size(); ++i)
      {
	Joint *current_joint = joints_[i];
	if (!current_joint)
	  continue;

	double effort = cm_->command_[dof];
	current_joint->SetForce(0,effort);
	++dof;
      }

  }

  ////////////////////////////////////////////////////////////////////////////////
  // Finalize the controller
  void JrlModelPlugin::FiniChild()
  {
    delete cm_;
    cm_ = NULL;
    rosnode_->shutdown();
  }

}
