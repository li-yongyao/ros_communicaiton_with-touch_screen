bool imu_status
bool front_laser_status
bool rear_laser_status
bool sonar1_status
bool sonar2_status
bool sonar3_status
bool sonar4_status
bool sonar5_status
bool sonar6_status
bool sonar7_status
bool sonar8_status
bool camera_status
bool brush_status
int32 battery_voltage
float64 car_speed
int32 water_level

# front laser scan message 
# std_msgs/Header front_laser_header
# float32 front_laser_angle_min
# float32 front_laser_angle_max
# float32 front_laser_angle_increment
# float32 front_laser_range_min
# float32 front_laser_range_max
# float32[] front_laser_ranges
# float32[] front_laser_intensities
sensor_msgs/LaserScan front_laser_scan

# rear laser scan message
# std_msgs/Header rear_laser_header
# float32 rear_laser_angle_min
# float32 rear_laser_angle_max
# float32 rear_laser_angle_increment
# float32 rear_laser_range_min
# float32 rear_laser_range_max
# float32[] rear_laser_ranges
# float32[] rear_laser_intensities
sensor_msgs/LaserScan rear_laser_scan

#imu message
# std_msgs/Header imu_header
# geometry_msgs/Quaternion imu_orientation
# float64[9] imu_orientation_covariance
# geometry_msgs/Vector3 imu_angular_velocity
# float64[9] imu_angular_velocity_covariance
# geometry_msgs/Vector3 imu_linear_acceleration
# float64[9] imu_linear_acceleration_covariance
sensor_msgs/Imu imu_data

#sonar1 message
sensor_msgs/Range sonar1_data
sensor_msgs/Range sonar2_data
sensor_msgs/Range sonar3_data
sensor_msgs/Range sonar4_data
sensor_msgs/Range sonar5_data
sensor_msgs/Range sonar6_data
sensor_msgs/Range sonar7_data
sensor_msgs/Range sonar8_data



