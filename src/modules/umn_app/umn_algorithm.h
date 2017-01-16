
#pragma once

// should be called every time new data is pushed into the filter
bool update(uint64_t time_usec, struct sensor_combined_s *sensorCombined_ptr, struct airspeed_s *airSpeed_ptr, struct vehicle_gps_position_s *gps_ptr, struct vehicle_land_detected_s *landDetected_ptr, struct umn_output_s *uout_ptr);