# Vehicle Attitude Messages
float32 roll_rad               # body angle in NED frame (radians)
float32 pitch_rad              # body angle in NED frame (radians)
float32 yaw_rad                # body angle in NED frame -PI..+PI, (radians)

float32 rollspeed_rps   # Angular velocity about body north axis (x) in rad/s
float32 pitchspeed_rps  # Angular velocity about body east axis (y) in rad/s
float32 yawspeed_rps    # Angular velocity about body down axis (z) in rad/s
float32[4] q        # Quaternion (NED)

# Status
uint16 mode_flags   # Parameter to keep track of estimator mode as needed


# Barometric Altitude and Airspeed
float32 airspeed        # Airspeed, estimated
bool airspeed_valid     # False: Non-finite values or non-updating sensor

# Position
float64 lat_deg         # Latitude, (degrees)
float64 lon_deg         # Longitude, (degrees)
float32 alt_m         # Altitude AMSL, (meters)

float32 pnorth_m       # position in meters in NED frame
float32 peast_m       # 
float32 pdown_m       # 
float32[3] pned_std # Standard deviation of NED position estimate

# Position of reference point (local NED frame origin) in global (GPS / WGS84) frame
bool ned_global              # true if position (n, e, d) is valid and has valid global reference (ref_lat, ref_lon)
float64 ref_lat_deg             # Reference point latitude, (degrees)
float64 ref_lon_deg             # Reference point longitude, (degrees)
float32 ref_alt_deg             # Reference altitude AMSL,  (metres)

# Velocity
float32 vnorth_mps           # North velocity in NED earth-fixed frame, (metres/sec)
float32 veast_mps           # East velocity in NED earth-fixed frame, (metres/sec)
float32 vdown_mps           # Down velocity in NED earth-fixed frame, (metres/sec)
float32[3] vned_std # Standard-deviation of velocity estimate

# Wind
float32 windspeed_north_mps     # Wind component in north direction (meters/sec)
float32 windspeed_east_mps      # Wind component in east direction  (meters/sec)

# 25-element array for storing custom data (e.g. covariance matrix diagonal)
float32[25] data_array

# Sensor Errors
float32[3] gbias_rps # Gyro bias xyz (radians)
float32[3] abias_m_s2 # Accel bias xyz (m/s^2)

# "True" States (from EKF2)
bool umn_control     # True: UMN app output used to control vehicle.
float32[4] q_true    # Quaternion (NED)
