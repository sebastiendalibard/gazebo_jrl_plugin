#ifndef GAZEBO_JRL_MODEL_PLUGIN_HH
#define GAZEBO_JRL_MODEL_PLUGIN_HH

#include <vector>

#include <gazebo/Controller.hh>
#include <gazebo/Body.hh>
#include <gazebo/World.hh>
#include <gazebo/Param.hh>
#include <gazebo/Joint.hh>

#include <ros/ros.h>

#include <abstract-robot-dynamics/abstract-robot-dynamics.hh>

#include <jrl_controller_manager/controller_manager.h>

namespace gazebo
{
  class XMLConfigNode;
  class JrlModelPlugin : public Controller
  {
  public: 
    JrlModelPlugin(Entity *parent);

    virtual ~JrlModelPlugin();

    jrl_controller_manager::ControllerManager* cm_;

  protected: 
    virtual void LoadChild(XMLConfigNode *node);
    virtual void InitChild();
    virtual void UpdateChild();
    virtual void FiniChild();

  private:
    void readUrdf(XMLConfigNode *node);

    Model *parent_model_;

    std::vector<gazebo::Joint*> joints_;

    ros::NodeHandle* rosnode_;
    
    CjrlDynamicRobot *jrl_dynamic_robot_;

    /// \brief set topic name of robot description parameter
    ParamT<std::string> *robotParamP;
    ParamT<std::string> *robotNamespaceP;
    std::string robotParam;
    std::string robotNamespace;
  };

}
#endif
