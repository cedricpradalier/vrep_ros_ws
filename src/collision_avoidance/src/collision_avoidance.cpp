
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>


class CollisionAvoidance {
    protected:
        ros::Subscriber scanSub;
        ros::Subscriber cloudSub;
        ros::Subscriber velSub;
        ros::Publisher velPub;

        ros::NodeHandle nh;
        std::string base_frame;
        double safety_diameter;
        tf::TransformListener listener;

        pcl::PointCloud<pcl::PointXYZ> lastpc;

        void velocity_filter(const geometry_msgs::TwistConstPtr msg) {
            geometry_msgs::Twist filtered = findClosestAcceptableVelocity(*msg);
            velPub.publish(filtered);
        }

        void scan_callback(const sensor_msgs::LaserScanConstPtr msg) {
            lastpc.clear();
            for (size_t i=0;i<msg->ranges.size();i++) {
                double alpha = msg->angle_min + i*msg->angle_increment;
                pcl::PointXYZ P;
                P.x = msg->ranges[i] * cos(alpha);
                P.y = msg->ranges[i] * sin(alpha);
                P.z = 0;
                lastpc.push_back(P);
            }

            // unsigned int n = lastpc.size();
            // ROS_INFO("New point cloud: %d points",n);
            // for (unsigned int i=0;i<n;i++) {
            //     float x = lastpc[i].x;
            //     float y = lastpc[i].y;
            //     float z = lastpc[i].z;
            //     ROS_INFO("%d %.3f %.3f %.3f",i,x,y,z);
            // }
            // printf("\n\n\n");
        }

        void pc_callback(const sensor_msgs::PointCloud2ConstPtr msg) {
            pcl::PointCloud<pcl::PointXYZ> temp;
            pcl::fromROSMsg(*msg, temp);
            // Make sure the point cloud is in the base-frame
            listener.waitForTransform(base_frame,msg->header.frame_id,msg->header.stamp,ros::Duration(1.0));
            pcl_ros::transformPointCloud(base_frame,msg->header.stamp, temp, msg->header.frame_id, lastpc, listener);
            // unsigned int n = lastpc.size();
            // ROS_INFO("New point cloud: %d points",n);
            // for (unsigned int i=0;i<n;i++) {
            //     float x = lastpc[i].x;
            //     float y = lastpc[i].y;
            //     float z = lastpc[i].z;
            //     ROS_INFO("%d %.3f %.3f %.3f",i,x,y,z);
            // }
            // printf("\n\n\n");
        }

        geometry_msgs::Twist findClosestAcceptableVelocity(const geometry_msgs::Twist & desired) {
            geometry_msgs::Twist res = desired;
            unsigned int n = lastpc.size();
            double multiplier = 1;
            for (unsigned int i=0;i<n;i++) {
                float x = lastpc[i].x;
                float y = lastpc[i].y;
                float z = lastpc[i].z;

                // float z = *(float*)(&lastpc.data[lastpc.point_step*i + lastpc.fields[2].offset]);
                if (hypot(x,y) < 1e-2) {
                    // bogus point, the laser did not return
                    continue;
                }

                if (fabs(y) > safety_diameter) {
                    // too far on the side
                    continue;
                }
                if (x*res.linear.x < 0) {
                    // not going towards it
                    continue;
                }

                if (fabs(x) < safety_diameter) {
                    ROS_INFO("Too close! %.2f %.2f",x,y);
                    multiplier = 0;
                } else if (fabs(x) > 3 * safety_diameter) {
                    continue;
                } else {
                    multiplier = std::min(multiplier,desired.linear.x*(fabs(x)-safety_diameter)/(2*safety_diameter));
                }
                res.linear.x = std::min(multiplier, desired.linear.x);
            }
            ROS_INFO("Speed limiter: desired %.2f controlled %.2f",desired.linear.x,res.linear.x);

            return res;
        }

    public:
        CollisionAvoidance() : nh("~") {
            nh.param("base_frame",base_frame,std::string("/body"));
            nh.param("safety_diameter",safety_diameter,0.2);

            scanSub = nh.subscribe("scan",1,&CollisionAvoidance::scan_callback,this);
            cloudSub = nh.subscribe("cloud",1,&CollisionAvoidance::pc_callback,this);
            velSub = nh.subscribe("vel_input",1,&CollisionAvoidance::velocity_filter,this);
            velPub = nh.advertise<geometry_msgs::Twist>("vel_output",1);

        }

};

int main(int argc, char * argv[]) 
{
    ros::init(argc,argv,"collision_avoidance");

    CollisionAvoidance ca;

    ros::spin();
    // TODO: implement a security layer
}


