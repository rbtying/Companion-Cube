#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

#define CRUISING_SPEED 0.05

#define PI 3.141592654

using namespace sensor_msgs;

double turnRate;
double minDist;
double speed;

double threshold;
double turn_rate_const;
double turn_in_place_rate;
double cruise_speed;

/**
 * This function processes LaserScan messages and looks for nearby objects, 
 * then defines a path to get out of them.
 */
void laserCallback( const boost::shared_ptr<const LaserScan>& scan ) {
    // minimum and maximum angles are given by the laserscan message
    // the increment is givne as well.
    double MAX_ANG = scan->angle_max;
    double MIN_ANG = scan->angle_min;
    double inc = scan->angle_icrement;

    // we can only iterate through the number of scan indices given
    int bins = ( int ) ( ( MAX_ANG - MIN_ANG ) / inc );

    // useful variables
    double minAngle = 0.0;
    double maxAngle = 0.0;
    minDist = scan->range_max;
    double maxDist = 0.0;

    // iterate through everything to find min/max values of angle and distance.
    for ( int bin = 0; bin < bins; bin++ ) {
        double angle = MIN_ANGLE + bin * inc;
        double range = scan->ranges[bin];

        if ( range > scan->range_min && range < minDist ) {
            minDist = range;
            minAngle = angle;
        }

        if ( range < scan->range_max && range > maxDist ) {
            maxDist = range;
            maxAngle = angle;
        }
    }

    // if we're far enough away start to turn away from the obstacle
    if ( minDist > threshold ) {
        // depending on direction we should turn a different direction
        if ( minAngle < 0 ) {
            turnRate = minAngle + turn_rate_const;
        } else {
            turnRate = minAngle - turn_rate_const;
        }

        // normalize with pi.
        turnRate /= PI;

        // magic number calculation for speed =)
        speed = cruise_speed / ( 1.0 + 5.0 * abs ( turnRate ) );
    } else { // if we're close by we should stop and turn
        if ( maxDist > 1.0 ) {
            turnRate = maxAngle / PI;
        } else {
            turnRate = turn_in_place_rate;
        }
        // set speed to 0
        speed = 0.0;
    }
}

/**
 * Main execution loop
 */
int main ( int argc, char** argv ) {
    ros::init( argc, argv, "ObstacleAvoid" );
    ros::NodeHandle n;

    // allocate loop variables
    ros::Rate loop_rate( 5 );
    ros::Time current_time;

    // request publishers and subscribers
    ros::Publisher cmdVelPub = n.advertise<geometry_msgs::Twist>( "cmd_vel", 50 );
    ros::Subscriber laserSub = n.subscribe( "scan", 100, laserCallback );

    // get parameters from configuration files
    n.param( "threshold", threshold, 0.25 );
    n.param( "turn_rate", turn_rate_const, PI / 2.0 );
    n.param( "turn_rate_in_place", turn_in_place_rate, 0.2 );
    n.param( "speed", cruise_speed, CRUISING_SPEED );

    // loop while node is active
    while ( n.ok() ) {
        current_time = ros::Time::now();

        // check if autonomous is enabled
        n.param( "autoMode", automatic, 1 );

        // if enabled then send a geometry_msgs::Twist with the speed/turn values
        if ( automatic ) {
            geometry_msgs::Twist cmdVelMsg;
            cmdVelMsg.angular.x = 0.0;
            cmdVelMsg.angular.y = 0.0;
            cmdVelMsg.angular.z = turnRate;
            cmdVelMsg.linear.x = speed;
            cmdVelMsg.linear.y = 0.0;
            cmdVelMsg.linear.z = 0.0;

            cmdVelPub.publish( cmdVelMsg );
        }

        // spin until time to run next loop
        ros::spinOnce();
        loop_rate.sleep();
    }
}
