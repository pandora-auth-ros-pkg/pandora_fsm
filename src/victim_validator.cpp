

#include <string>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <pandora_gui_msgs/ValidateVictimGUIAction.h>
#include <pandora_gui_msgs/ValidateVictimGUIActionGoal.h>


class VictimValidator
{
  protected:
    ros::NodeHandle nh_;

    std::string actionName_;
    std::string operatorResponse_;

    actionlib::SimpleActionServer<pandora_gui_msgs::ValidateVictimGUIAction> as_;
    pandora_gui_msgs::ValidateVictimGUIResult validationResult_;

  public:
    VictimValidator(std::string name) :
      as_(nh_, name, boost::bind(&VictimValidator::validateVictim, this, _1), false),
      actionName_(name)
    {
      as_.start();

      ROS_INFO("Waiting for validation requests.");
    }

    ~VictimValidator()
    {
    }

    void validateVictim(const pandora_gui_msgs::ValidateVictimGUIGoalConstPtr &goal)
    {
      ROS_INFO("********************************************************");
      ROS_INFO("Received validation request for victim with id: %d", goal->victimId);
      ROS_INFO("Found at (%.2f, %.2f)", goal->victimFoundx, goal->victimFoundy);
      ROS_INFO("Probability: %.2f", goal->probability);

      ROS_INFO("Identified from: ");

      for (int i = 0; i < goal->sensorIDsFound.size(); i++)
      {
        ROS_INFO("==> %s", goal->sensorIDsFound[i].c_str());
      }

      ROS_INFO("********************************************************");

      while (true)
      {
        std::cout << "Please validate this victim...(y/n) " << std::endl;
        std::cin >> operatorResponse_;

        if (operatorResponse_ == "y" || operatorResponse_ == "Y")
        {
          ROS_INFO("The victim is valid!");
          validationResult_.victimValid = true;
          as_.setSucceeded(validationResult_);
          break;
        }
        else if (operatorResponse_ == "n" || operatorResponse_ == "N")
        {
          ROS_INFO("The victim is not valid!");
          validationResult_.victimValid = false;
          as_.setSucceeded(validationResult_);
          break;
        }
        else
        {
          ROS_INFO("Your input is not valid.");
        }
      }
    }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "victim_validator");
  ROS_INFO("Starting victim validation node.");

  VictimValidator validator("/gui/validate_victim");
  ros::spin();

  return 0;
}
