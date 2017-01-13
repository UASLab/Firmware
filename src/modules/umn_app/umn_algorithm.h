
#pragma once

// should be called every time new data is pushed into the filter
bool update(struct sensordata *sensorData_ptr, struct nav *navData_ptr, struct control *controlData_ptr);
