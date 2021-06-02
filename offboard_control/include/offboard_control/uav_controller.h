#include <ros/ros.h>
#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/TwistStamped.h>
#include<mavros_msgs/State.h>
#include<mavros_msgs/SetMode.h>



class UavController
{
private:
    mavros_msgs::State current_state;
public:
    UavController(/* args */);
    ~UavController();
    void state_cb (const mavros_msgs::State::ConstPtr& msg);
    mavros_msgs::State getState(void);
};
