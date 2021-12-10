#include <offboard/offboard.h>

sensor_msgs::NavSatFix goalTransfer(double lat, double lon, double alt)
{
    sensor_msgs::NavSatFix goal;
    goal.latitude = lat;
    goal.longitude = lon;
    goal.altitude = alt;
    return goal;
}

geometry_msgs::PoseStamped targetTransfer(double x, double y, double z)
{
    geometry_msgs::PoseStamped target;
    target.pose.position.x = x;
    target.pose.position.y = y;
    target.pose.position.z = z;
    return target;
}

double avgBodyVelocity(geometry_msgs::TwistStamped vel)
{
    return sqrt((vel.twist.linear.x*vel.twist.linear.x)+(vel.twist.linear.y*vel.twist.linear.y)+(vel.twist.linear.z*vel.twist.linear.z));
}

bool checkPosition(double error, geometry_msgs::PoseStamped current, geometry_msgs::PoseStamped target)
{
    double xt = target.pose.position.x;
	double yt = target.pose.position.y;
	double zt = target.pose.position.z;
	double xc = current.pose.position.x;
	double yc = current.pose.position.y;
	double zc = current.pose.position.z;

	if(((xt - error) < xc) && (xc < (xt + error)) 
	&& ((yt - error) < yc) && (yc < (yt + error))
	&& ((zt - error) < zc) && (zc < (zt + error)))
	{
		return true;
	}
	else
	{
		return false;
	}
}
bool checkOrientation(double error, geometry_msgs::PoseStamped current, geometry_msgs::PoseStamped target)
{
    double currentx = current.pose.orientation.x;
	double currenty = current.pose.orientation.y;
	double currentz = current.pose.orientation.z;
	double currentw = current.pose.orientation.w;
	double target_x = target.pose.orientation.x;
	double target_y = target.pose.orientation.y;
	double target_z = target.pose.orientation.z;
	double target_w = target.pose.orientation.w;

	// tf Quaternion to RPY
	tf::Quaternion qc(currentx, currenty, currentz, currentw);
	tf::Matrix3x3 mc(qc);
	double rc, pc, yc;
	mc.getRPY(rc, pc, yc);

	tf::Quaternion qt(target_x, target_y, target_z, target_w);
	tf::Matrix3x3 mt(qt);
	double rt, pt, yt;
	mt.getRPY(rt, pt, yt);

	// check
	if((((degreeOf(rt)-1)<(degreeOf(rc)))&&(degreeOf(rc)<(degreeOf(rt)+1)))
	 &&(((degreeOf(pt)-1)<(degreeOf(pc)))&&(degreeOf(pc)<(degreeOf(pt)+1)))
	 &&(((degreeOf(yt)-1)<(degreeOf(yc)))&&(degreeOf(yc)<(degreeOf(yt)+1)))) 
	{
		return true;
	}
	else
	{
		return false;
	}
}

void inputTarget()
{
    char c;
    std::printf("[ INFO] How do you want to input target/goal?\n");
    std::printf("- Choose 1: Manual Input from keyboard\n");
    std::printf("- Choose 2: Load prepared parameters from launch file\n");
    std::printf("(1/2): ");
    std::cin >> c;
    if(c == '1')
    {
        std::printf("[ INFO] Choose the setpoint type\n");
        std::printf("- 3 for Local position\n");
        std::printf("- 4 for Global position\n");
        std::printf("(3/4): ");
        std::cin >> c;
        if(c == '3')
        {
            inputLocal();
            input_type_ = true;
        }
        else if(c == '4')
        {
            inputGlobal();
            input_type_ = false;
        }
        else inputTarget();
    }
    else if(c == '2')
    {
        std::printf("[ INFO] Choose the setpoint type\n");
        std::printf("- 3 for Local position\n");
        std::printf("- 4 for Global position\n");
        std::printf("(3/4): ");
        std::cin >> c;
        if(c == '3')
        {
            input_type_ = true;
            // if(!x_target_.empty() || !y_target_.empty() || !z_target_.empty())
            // {
            //     x_target_.clear();
            //     y_target_.clear();
            //     z_target_.clear();
            // }
            // ros::param::get("number_of_target", target_num_);
            // ros::param::get("x_pos", x_target_);
            // ros::param::get("y_pos", y_target_);
            // ros::param::get("z_pos", z_target_);
            // ros::param::get("target_error", check_error_);
            std::printf("[ INFO] Loaded parameters\n");
            for(int i = 0; i < target_num_; i++)
            {
                std::printf("- Target (%d): [%.1f, %.1f, %.1f]\n", i+1, x_target_[i], y_target_[i], z_target_[i]);
            }
            check_error_ = target_error_;
            std::printf("- Position error check: %.1f (m)\n", check_error_);
        }
        else if(c == '4')
        {
            input_type_ = false;
            // if(!lat_goal_.empty() || !lon_goal_.empty() || !alt_goal_.empty())
            // {
            //     lat_goal_.clear();
            //     lon_goal_.clear();
            //     alt_goal_.clear();
            // }
            // ros::param::get("number_of_goal", goal_num_);
            // ros::param::get("latitude", lat_goal_);
            // ros::param::get("longitude", lon_goal_);
            // ros::param::get("altitude", alt_goal_);
            // ros::param::get("goal_error", check_error_);
            std::printf("[ INFO] Loaded parameters\n");
            for(int i = 0; i < goal_num_; i++)
            {
                alt_goal_[i] += current_global_.altitude;
                std::printf("- Goal (%d): [%.8f, %.8f, %.3f]\n", i+1, lat_goal_[i], lon_goal_[i], alt_goal_[i]);
            }
            check_error_ = goal_error_;
            std::printf("- Position error check: %.1f (m)\n", check_error_);
        }
        else inputTarget();
    }
    else inputTarget();
}

void inputLocal()
{
    double x, y, z;
    std::printf("[ INFO] Input Local target position(s)\n");
    std::printf(" Number of target(s): "); 
    std::cin >> target_num_;
    if(!x_target_.empty() || !y_target_.empty() || !z_target_.empty())
    {
        x_target_.clear();
        y_target_.clear();
        z_target_.clear();
    }
    for(int i = 0; i < target_num_; i++)
    {
        std::printf(" Target (%d) postion (in meter):\n", i+1);
        std::printf("   x(%d): ", i+1);
        std::cin >> x; x_target_.push_back(x);
        std::printf("   y(%d): ", i+1);
        std::cin >> y; y_target_.push_back(y);
        std::printf("   z(%d): ", i+1);
        std::cin >> z; z_target_.push_back(z);
    }
    std::printf(" Position error check value (in meter): ");
    std::cin >> target_error_;
}

void inputGlobal()
{
    double lat, lon, alt;
    std::printf("[ INFO] Input Global goal position(s)\n");
    std::printf("  Number of goal(s): "); 
    std::cin >> goal_num_;
    if(!lat_goal_.empty() || !lon_goal_.empty() || !alt_goal_.empty())
    {
        lat_goal_.clear();
        lon_goal_.clear();
        alt_goal_.clear();
    }
    for(int i = 0; i < goal_num_; i++)
    {
        std::printf("  Goal (%d) postion:\n", i+1);
        std::printf("    Latitude (%d) (in degree): ", i+1);
        std::cin >> lat; lat_goal_.push_back(lat);
        std::printf("    Longitude (%d) (in degree): ", i+1);
        std::cin >> lon; lon_goal_.push_back(lon);
        std::printf("    Altitude (%d) (in meter.above ground): ", i+1);
        std::cin >> alt; alt += current_global_.altitude; 
        alt_goal_.push_back(alt);
    }
    std::printf("  Position error check value (in meter): ");
    std::cin >> goal_error_;
}

std::vector<double> velLimit(double v_desired, geometry_msgs::PoseStamped current, geometry_msgs::PoseStamped target)
{
    double xc = current.pose.position.x;
    double yc = current.pose.position.y;
    double zc = current.pose.position.z;

    double xt = target.pose.position.x;
    double yt = target.pose.position.y;
    double zt = target.pose.position.z;

    double dx = xt - xc;
    double dy = yt - yc;
    double dz = zt - zc;

    double d = sqrt(dx*dx + dy*dy + dz*dz);

    std::vector<double> vel;

    vel.push_back((dx/d) * v_desired);
    vel.push_back((dy/d) * v_desired);
    vel.push_back((dz/d) * v_desired);

    return vel;
}

void takeOff(geometry_msgs::PoseStamped takeoff_pose, ros::Rate rate)
{
    bool takeoff_reached = false;
    while(ros::ok() && !takeoff_reached)
    {
        takeoff_pose.header.stamp = ros::Time::now();
        local_pose_pub_.publish(takeoff_pose);

        takeoff_reached = checkPosition(0.05, current_pose_, takeoff_pose);
        if(takeoff_reached)
        {
            // ros::param::get("takeoff_hover_time", hover_time_);
            hover_time_ = takeoff_time_;
            std::printf("[ INFO] Hovering at %.1f (m) in %.1f (s)\n", takeoff_pose.pose.position.z, hover_time_);
            
            hoverAt(hover_time_, takeoff_pose, rate);
        }
        else
        {
            ros::spinOnce();
            rate.sleep();
        }
    }
}

void hoverAt(double hover_time, geometry_msgs::PoseStamped target, ros::Rate rate)
{
    ros::Time t_check;
    t_check = ros::Time::now();

    while ((ros::Time::now() - t_check) < ros::Duration(hover_time))
    {
        local_pose_pub_.publish(target);

        ros::spinOnce();
    	rate.sleep();
    }
}

void landingAt(geometry_msgs::PoseStamped setpoint, ros::Rate rate)
{
    bool land_reached = false;
    std::printf("[ INFO] Ready to Land\n");
    while(ros::ok() && !land_reached)
    {
        // ros::param::get("land_velocity", vel_desired_);
        vel_ = velLimit(land_vel_, current_pose_, targetTransfer(setpoint.pose.position.x, setpoint.pose.position.y, 0.5));

        target_pose_.pose.position.x = current_pose_.pose.position.x + vel_[0];
        target_pose_.pose.position.y = current_pose_.pose.position.y + vel_[1];
        target_pose_.pose.position.z = current_pose_.pose.position.z + vel_[2];

        target_pose_.header.stamp = ros::Time::now();
        local_pose_pub_.publish(target_pose_);
        // std::printf("[ DEBUG] Desired velocity: %.3f (m/s)\n", vel_desired_);
        // std::printf("[ DEBUG] Body velocity: %.3f (m/s)\n", avgBodyVelocity(current_vel_));
        std::printf("   Descending - %.1f (m)\n", current_pose_.pose.position.z);
        if(current_state_.system_status == 3)
        {
            land_reached = true;
        }
        else
        {
            // ros::param::get("land_error", land_error_);
            land_reached = checkPosition(land_error_, current_pose_, targetTransfer(setpoint.pose.position.x, setpoint.pose.position.y, 0.5));
        }

        if(land_reached)
        {
            std::printf("\n[ INFO] Unpacking\n");
            if(current_state_.system_status == 3)
            {
                hover_time_ = unpack_time_;
                hoverAt(hover_time_, targetTransfer(setpoint.pose.position.x, setpoint.pose.position.y, current_pose_.pose.position.z), rate);
            }
            else
            {
                // ros::param::get("unpack_time", hover_time_);
                hover_time_ = unpack_time_;
                hoverAt(hover_time_, targetTransfer(setpoint.pose.position.x, setpoint.pose.position.y, 0.5), rate);
            }
            
            std::printf("[ INFO] Done - Return setpoint [%.1f, %.1f, %.1f]\n", setpoint.pose.position.x, setpoint.pose.position.y, setpoint.pose.position.z);

            bool return_reached = false;
            while(ros::ok() && !return_reached)
            {
                // ros::param::get("return_velocity", vel_desired_);
                vel_ = velLimit(return_vel_, current_pose_, setpoint);

                target_pose_.pose.position.x = current_pose_.pose.position.x + vel_[0];
                target_pose_.pose.position.y = current_pose_.pose.position.y + vel_[1];
                target_pose_.pose.position.z = current_pose_.pose.position.z + vel_[2];

                target_pose_.header.stamp = ros::Time::now();
                local_pose_pub_.publish(target_pose_);
                // std::printf("[ DEBUG] Desired velocity: %.3f (m/s)\n", vel_desired_);
                // std::printf("[ DEBUG] Body velocity: %.3f (m/s)\n", avgBodyVelocity(current_vel_));
                std::printf("   Ascending - %.1f (m)/ %.1f\n", current_pose_.pose.position.z, setpoint.pose.position.z);
                return_reached = checkPosition(0.2, current_pose_, setpoint);
                if(return_reached)
                {
                    std::printf("\n[ INFO] Hover at setpoint [%.1f, %.1f, %.1f]\n", setpoint.pose.position.x, setpoint.pose.position.y, setpoint.pose.position.z);
                    // ros::param::get("hover_time", hover_time_);
                    hover_time_ = hover_;
                    hoverAt(hover_time_, setpoint, rate);
                }
                else
                {
                    ros::spinOnce();
                    rate.sleep();
                }
            }
        }
        else
        {
            ros::spinOnce();
            rate.sleep();
        }
    }
}

void landingAtFinal(geometry_msgs::PoseStamped setpoint, ros::Rate rate)
{
    bool land_reached = false;
    std::printf("[ INFO] Ready to Land\n");
    while(ros::ok() && !land_reached)
    {
        // ros::param::get("land_velocity", vel_desired_);
        vel_ = velLimit(land_vel_, current_pose_, targetTransfer(setpoint.pose.position.x, setpoint.pose.position.y, 0));

        target_pose_.pose.position.x = current_pose_.pose.position.x + vel_[0];
        target_pose_.pose.position.y = current_pose_.pose.position.y + vel_[1];
        target_pose_.pose.position.z = current_pose_.pose.position.z + vel_[2];

        target_pose_.header.stamp = ros::Time::now();
        local_pose_pub_.publish(target_pose_);
        // std::printf("[ DEBUG] Desired velocity: %.3f (m/s)\n", vel_desired_);
        // std::printf("[ DEBUG] Body velocity: %.3f (m/s)\n", avgBodyVelocity(current_vel_));
        std::printf("   Descending - %.1f (m)\n", current_pose_.pose.position.z);

        // ros::param::get("land_error", land_error_);
        land_reached = checkPosition(land_error_, current_pose_, targetTransfer(setpoint.pose.position.x, setpoint.pose.position.y, 0));

        if(current_state_.system_status == 3)
        {
            std::printf("\n[ INFO] Landing detected\n");
            flight_mode_.request.custom_mode = "AUTO.LAND";
            if(set_mode_client_.call(flight_mode_) && flight_mode_.response.mode_sent)
            {
                break;
            }
        }
        else if(land_reached)
        {
            flight_mode_.request.custom_mode = "AUTO.LAND";
            if(set_mode_client_.call(flight_mode_) && flight_mode_.response.mode_sent)
            {
                std::printf("\n[ INFO] LANDED\n");
            }
        }
        else
        {
            ros::spinOnce();
            rate.sleep();
        }
    }
}

void returnHome(geometry_msgs::PoseStamped home, ros::Rate rate)
{
    bool home_reached = false;
    while(ros::ok() && !home_reached)
    {
        vel_ = velLimit(vel_desired_, current_pose_, home);

        target_pose_.pose.position.x = current_pose_.pose.position.x + vel_[0];
        target_pose_.pose.position.y = current_pose_.pose.position.y + vel_[1];
        target_pose_.pose.position.z = current_pose_.pose.position.z + vel_[2];

        target_pose_.header.stamp = ros::Time::now();
        local_pose_pub_.publish(target_pose_);

        home_reached = checkPosition(0.1, current_pose_, home);
        if(home_reached)
        {
            hover_time_ = hover_;
            std::printf("[ INFO] Hovering at %.1f (m) in %.1f (s)\n", home_pose_.pose.position.z, hover_time_);
            
            hoverAt(hover_time_, home, rate);
            landingAtFinal(home, rate);
        }
        else
        {
            ros::spinOnce();
            rate.sleep();
        }
    }
}

double distanceMeasure(geometry_msgs::PoseStamped current, geometry_msgs::PoseStamped target)
{
    double xc = current.pose.position.x;
	double yc = current.pose.position.y;
	double zc = current.pose.position.z;
	double xt = target.pose.position.x;
	double yt = target.pose.position.y;
	double zt = target.pose.position.z;

	return sqrt((xt-xc)*(xt-xc) + (yt-yc)*(yt-yc) + (zt-zc)*(zt-zc));
}

double degreeOf(double rad)
{
    return (rad*180)/PI;
}

double radianOf(double deg)
{
    return (deg*PI)/180;
}

geometry_msgs::Point WGS84ToECEF(sensor_msgs::NavSatFix wgs84)
{
    geometry_msgs::Point ecef;
    double lambda = radianOf(wgs84.latitude);
    double phi = radianOf(wgs84.longitude);
    double s = sin(lambda);
    double N = a / sqrt(1 - e_sq * s * s);

    double sin_lambda = sin(lambda);
    double cos_lambda = cos(lambda);
    double cos_phi = cos(phi);
    double sin_phi = sin(phi);

    ecef.x = (wgs84.altitude + N) * cos_lambda * cos_phi;
    ecef.y = (wgs84.altitude + N) * cos_lambda * sin_phi;
    ecef.z = (wgs84.altitude + (1 - e_sq) * N) * sin_lambda;

    return ecef;
}

geographic_msgs::GeoPoint ECEFToWGS84(geometry_msgs::Point ecef)
{
    geographic_msgs::GeoPoint wgs84;
    double eps = e_sq / (1.0 - e_sq);
    double p = sqrt(ecef.x * ecef.x + ecef.y * ecef.y);
    double q = atan2((ecef.z * a), (p * b));
    double sin_q = sin(q);
    double cos_q = cos(q);
    double sin_q_3 = sin_q * sin_q * sin_q;
    double cos_q_3 = cos_q * cos_q * cos_q;
    double phi = atan2((ecef.z + eps * b * sin_q_3), (p - e_sq * a * cos_q_3));
    double lambda = atan2(ecef.y, ecef.x);
    double v = a / sqrt(1.0 - e_sq * sin(phi) * sin(phi));
    
    wgs84.altitude = (p / cos(phi)) - v;

    wgs84.latitude = degreeOf(phi);
    wgs84.longitude = degreeOf(lambda);

    return wgs84;
}

geometry_msgs::Point ECEFToENU(geometry_msgs::Point ecef, sensor_msgs::NavSatFix ref)
{
    geometry_msgs::Point enu;
    double lambda = radianOf(ref.latitude);
    double phi = radianOf(ref.longitude);
    double s = sin(lambda);
    double N = a / sqrt(1 - e_sq * s * s);

    double sin_lambda = sin(lambda);
    double cos_lambda = cos(lambda);
    double cos_phi = cos(phi);
    double sin_phi = sin(phi);

    double x0 = (ref.altitude + N) * cos_lambda * cos_phi;
    double y0 = (ref.altitude + N) * cos_lambda * sin_phi;
    double z0 = (ref.altitude + (1 - e_sq) * N) * sin_lambda;

    double xd, yd, zd;
    xd = ecef.x - x0;
    yd = ecef.y - y0;
    zd = ecef.z - z0;

    enu.x = -sin_phi * xd + cos_phi * yd;
    enu.y = -cos_phi * sin_lambda * xd - sin_lambda * sin_phi * yd + cos_lambda * zd;
    enu.z = cos_lambda * cos_phi * xd + cos_lambda * sin_phi * yd + sin_lambda * zd;

    return enu;
}

geometry_msgs::Point ENUToECEF(geometry_msgs::Point enu, sensor_msgs::NavSatFix ref)
{
    geometry_msgs::Point ecef;
    double lambda = radianOf(ref.latitude);
    double phi = radianOf(ref.longitude);
    double s = sin(lambda);
    double N = a / sqrt(1 - e_sq * s * s);

    double sin_lambda = sin(lambda);
    double cos_lambda = cos(lambda);
    double cos_phi = cos(phi);
    double sin_phi = sin(phi);

    double x0 = (ref.altitude + N) * cos_lambda * cos_phi;
    double y0 = (ref.altitude + N) * cos_lambda * sin_phi;
    double z0 = (ref.altitude + (1 - e_sq) * N) * sin_lambda;

    double xd = -sin_phi * enu.x - cos_phi * sin_lambda * enu.y + cos_lambda * cos_phi * enu.z;
    double yd = cos_phi * enu.x - sin_lambda * sin_phi * enu.y + cos_lambda * sin_phi * enu.z;
    double zd = cos_lambda * enu.y + sin_lambda * enu.z;

    ecef.x = xd + x0;
    ecef.y = yd + y0;
    ecef.z = zd + z0;

    return ecef;
}

geometry_msgs::Point WGS84ToENU(sensor_msgs::NavSatFix wgs84, sensor_msgs::NavSatFix ref)
{
    geometry_msgs::Point ecef = WGS84ToECEF(wgs84);
    geometry_msgs::Point enu = ECEFToENU(ecef, ref);
    return enu;
}

geographic_msgs::GeoPoint ENUToWGS84(geometry_msgs::Point enu, sensor_msgs::NavSatFix ref)
{
    geometry_msgs::Point ecef = ENUToECEF(enu, ref);
    geographic_msgs::GeoPoint wgs84 = ECEFToWGS84(ecef);
    return wgs84;
}