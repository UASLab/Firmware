# Time and Status
bool dead_reckoning     # True if this position is estimated through dead-reckoning

# Attitude
float32 roll_body               # body angle in NED frame
float32 pitch_body              # body angle in NED frame
float32 yaw_body                # body angle in NED frame

float32[4] q        # Quaternion (NED)

# Barometric Altitude and Airspeed
float32 baro_alt_meter          # barometer altitude measurement in m
float32 indicated_airspeed_m_s      # indicated airspeed in meters per second, -1 if unknown
float32 true_airspeed_m_s       # true filtered airspeed in meters per second, -1 if unknown

# Position
float64 lat         # Latitude, (degrees)
float64 lon         # Longitude, (degrees)
float32 alt         # Altitude AMSL, (meters)

float32 pos_n       # position in meters in NED frame
float32 pos_e       # 
float32 pos_d       # 

# Velocity
float32 vel_n           # North velocity in NED earth-fixed frame, (metres/sec)
float32 vel_e           # East velocity in NED earth-fixed frame, (metres/sec)
float32 vel_d           # Down velocity in NED earth-fixed frame, (metres/sec)

# Wind
float32 windspeed_north     # Wind component in north / X direction
float32 windspeed_east      # Wind component in east / Y direction