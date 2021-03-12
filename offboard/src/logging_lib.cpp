#include <offboard/offboard.h>
#include <offboard/logging.h>

void creates()
{ 
	std::fstream file; 
	file.open("position.csv", std::ios::out | std::ios::app); 
    
	file << "Point, x (m), y (m), z (m), x_convert (m), y_convert (m), z_convert (m), lat (degree), lon (degree), alt (m), gps_lat, gps_lon, gps_alt, rel alt (m) \n";
    file << ", , , , , , , , , , , , , \n";

    file.close(); 
} 

void updates(std::string name, double x_log, double y_log, double z_log, 
                   double x_convert, double y_convert, double z_convert,
                   double lat_log, double lon_log, double alt_log,
                   double lat_gps, double lon_gps, double alt_gps,
                   double rel_alt) 
{ 
	std::fstream file; 

    file.open("position.csv", std::ios::out | std::ios::app); 

    file << name << ", " << std::fixed << std::setprecision(8) << x_log << ", " 
						 << std::fixed << std::setprecision(8) << y_log << ", " 
						 << std::fixed << std::setprecision(8) << z_log << ", "
                         << std::fixed << std::setprecision(8) << x_convert << ", " 
						 << std::fixed << std::setprecision(8) << y_convert << ", " 
						 << std::fixed << std::setprecision(8) << z_convert << ", "
						 << std::fixed << std::setprecision(8) << lat_log << ", " 
						 << std::fixed << std::setprecision(8) << lon_log << ", " 
						 << std::fixed << std::setprecision(8) << alt_log << ", "
                         << std::fixed << std::setprecision(8) << lat_gps << ", "
                         << std::fixed << std::setprecision(8) << lon_gps << ", "
                         << std::fixed << std::setprecision(8) << alt_gps << ", "
                         << std::fixed << std::setprecision(8) << rel_alt << "\n";

	file.close(); 
} 

void updates_check(int i, double x_log, double y_log, double z_log, 
                   double x_convert, double y_convert, double z_convert,
                   double lat_log, double lon_log, double alt_log,
                   double lat_gps, double lon_gps, double alt_gps,
                   double rel_alt) 
{ 
	std::fstream file; 

    file.open("position.csv", std::ios::out | std::ios::app); 

    file << "checkpoint " << i << ", " << std::fixed << std::setprecision(8) << x_log << ", " 
									   << std::fixed << std::setprecision(8) << y_log << ", " 
									   << std::fixed << std::setprecision(8) << z_log << ", "
                                       << std::fixed << std::setprecision(8) << x_convert << ", " 
                              	       << std::fixed << std::setprecision(8) << y_convert << ", " 
						               << std::fixed << std::setprecision(8) << z_convert << ", "
         							   << std::fixed << std::setprecision(8) << lat_log << ", " 
									   << std::fixed << std::setprecision(8) << lon_log << ", " 
									   << std::fixed << std::setprecision(8) << alt_log << ", "
                                       << std::fixed << std::setprecision(8) << lat_gps << ", "
                                       << std::fixed << std::setprecision(8) << lon_gps << ", "
                                       << std::fixed << std::setprecision(8) << alt_gps << ", "
                                       << std::fixed << std::setprecision(8) << rel_alt << "\n";

	file.close(); 
} 

void updates_local(int num, double x, double y, double z) 
{ 
	std::fstream file; 

    file.open("position.csv", std::ios::out | std::ios::app); 

    file << "local target " << num << ", " << std::fixed << std::setprecision(8) << x << ", " 
										   << std::fixed << std::setprecision(8) << y << ", " 
										   << std::fixed << std::setprecision(8) << z << ", , , \n";
	file.close();
} 

void updates_global(int num, double lat, double lon, double alt) 
{ 
	std::fstream file; 

    file.open("position.csv", std::ios::out | std::ios::app); 

    file << "global target " << num <<  ", , , , , , , " << std::fixed << std::setprecision(8) << lat << ", " 
												         << std::fixed << std::setprecision(8) << lon << ", " 
												         << std::fixed << std::setprecision(8) << alt << "\n";

	file.close();
}

void creates_sensor()
{
	std::fstream file; 

	file.open("sensor.csv", std::ios::out | std::ios::app); 

	file << "Point, angular vel x (rad/s), angular vel y (rad/s), angular vel z (rad/s), " 
         << "linear acc x (m/s^2), linear acc y (m/s^2), linear acc z (m/s^2), "
         << "mag field x (T), mag field y (T), mag field z (T), static press (Pa), diff press (Pa) \n";
    file << ", , , , , , , , , , , \n";

    file.close(); 
}

void updates_sensor(std::string name, double av_x, double av_y, double av_z, 
                                      double la_x, double la_y, double la_z,
                                      double magx, double magy, double magz,
                                      double static_press, double diff_press)
{
    std::fstream file; 

	file.open("sensor.csv", std::ios::out | std::ios::app); 

    file << name << ", " << std::fixed << std::setprecision(8) << av_x << ", " 
						 << std::fixed << std::setprecision(8) << av_y << ", " 
						 << std::fixed << std::setprecision(8) << av_z << ", "
						 << std::fixed << std::setprecision(8) << la_x << ", " 
						 << std::fixed << std::setprecision(8) << la_y << ", " 
						 << std::fixed << std::setprecision(8) << la_z << ", "
                         << std::fixed << std::setprecision(8) << magx << ", "
                         << std::fixed << std::setprecision(8) << magy << ", "
                         << std::fixed << std::setprecision(8) << magz << ", "
                         << std::fixed << std::setprecision(8) << static_press << ", "
                         << std::fixed << std::setprecision(8) << diff_press << "\n";
	
    file.close(); 
}
void updates_check_ss(int i, double av_x, double av_y, double av_z, 
                             double la_x, double la_y, double la_z,
                             double magx, double magy, double magz,
                             double static_press, double diff_press)
{
    std::fstream file; 

	file.open("sensor.csv", std::ios::out | std::ios::app); 

    file << "checkpoint" << i << ", " 
                         << std::fixed << std::setprecision(8) << av_x << ", " 
						 << std::fixed << std::setprecision(8) << av_y << ", " 
						 << std::fixed << std::setprecision(8) << av_z << ", "
						 << std::fixed << std::setprecision(8) << la_x << ", " 
						 << std::fixed << std::setprecision(8) << la_y << ", " 
						 << std::fixed << std::setprecision(8) << la_z << ", "
                         << std::fixed << std::setprecision(8) << magx << ", "
                         << std::fixed << std::setprecision(8) << magy << ", "
                         << std::fixed << std::setprecision(8) << magz << ", "
                         << std::fixed << std::setprecision(8) << static_press << ", "
                         << std::fixed << std::setprecision(8) << diff_press << "\n";
	
    file.close(); 
}    