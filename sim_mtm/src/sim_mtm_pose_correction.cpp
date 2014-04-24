// Author: Adnan Munawar
// Email : amunawar@wpi.edu

#include<ros/ros.h>
#include<tf/tf.h>
#include<tf/transform_broadcaster.h>
#include<tf/transform_listener.h>
#include<geometry_msgs/PoseStamped.h>


// The purpose of this class is to implement the same cartesian pose as reported by the real MTMs. For this purpose, two extra frames have
// been placed. One at origin and one at the end effector, both of which are represented in the daVinci documentation.
class pose_correction{
public:
    pose_correction();
    void listen();
    void broadcast();
    tf::TransformListener listener;
    tf::TransformBroadcaster brodcaster;
    tf::StampedTransform tran;
    tf::StampedTransform base_tr;
    tf::StampedTransform ee_tr;
    ros::Publisher pub;
    ros::NodeHandle node;
};

pose_correction::pose_correction()
{
    this->base_tr.setOrigin(tf::Vector3(0.0,0.0,-0.190));
    this->base_tr.setRotation(tf::createQuaternionFromRPY(0.000, 0.000, 0.000));
    this->ee_tr.setOrigin(tf::Vector3(0.000, 0.000, 0.039));
    this->ee_tr.setRotation(tf::createQuaternionFromRPY(0.0,3.142,0.0));
    this->pub = this->node.advertise<geometry_msgs::PoseStamped>("/simulated_mtm/cartesian_pose_current",1);

    //Broadcast the frames in the constructor.
    this->broadcast();
}

void pose_correction::broadcast()
{
    this->brodcaster.sendTransform(tf::StampedTransform(
                                       this->base_tr,ros::Time::now(),"right_top_panel","base_frame"));
    this->brodcaster.sendTransform(tf::StampedTransform(
                                       this->ee_tr,ros::Time::now(),"right_wrist_roll_link","ee_frame"));
}

void pose_correction::listen()
{
    try{
        this->listener.lookupTransform("base_frame","ee_frame",ros::Time(),this->tran);
    }
    catch(tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
    }
}

int main(int argc, char ** argv)
{
    ros::init(argc,argv,"end_effector_pose_node");
    ros::NodeHandle node;
    ros::Rate rate(500);
    pose_correction cart_pos;
    cart_pos.listener.waitForTransform("base_frame","ee_frame",ros::Time(),ros::Duration(1.0));
    geometry_msgs::PoseStamped pose_stamped;
    while(ros::ok())
    {
        cart_pos.broadcast();
        cart_pos.listen();
        pose_stamped.header.stamp = cart_pos.tran.stamp_;
        pose_stamped.pose.position.x = cart_pos.tran.getOrigin().getX();
        pose_stamped.pose.position.y = cart_pos.tran.getOrigin().getY();
        pose_stamped.pose.position.z = cart_pos.tran.getOrigin().getZ();
        pose_stamped.pose.orientation.x = cart_pos.tran.getRotation().getX();
        pose_stamped.pose.orientation.y = cart_pos.tran.getRotation().getY();
        pose_stamped.pose.orientation.z = cart_pos.tran.getRotation().getZ();
        pose_stamped.pose.orientation.w = cart_pos.tran.getRotation().getW();

        cart_pos.pub.publish(pose_stamped);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
