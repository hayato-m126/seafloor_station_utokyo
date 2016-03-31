# This express ALOC Sensor data
Header header
int8 status #-1:data set completed 0:wait 1:send 2:fail(NO_REPLY) 3:fail(communication error) 4:only position 5:data success 6:data empty
int16 hydrophone_level_threshold #[A/D scale]
time time_success_position #[ros time]
time time_success_data #[ros time]
time time_last_call #[ros time]
int8 address_me
int8 address_source_data
int8 address_target #not reliable, estimated by charp signal
int8 address_target_data #reliable, include in data
int8 receive_data_status #0:OK 1:CRCerr 2:NoData
int16 source_depth #[m]
int32[2] receive_level #[A/D scale]
float64 travel_time #[sec]
float64 sound_vertical_angle #[deg] up plus
float64 sound_horizontal_angle #[deg] me to source direction -180 to 180
float64 sound_vertical_angle_corrected #[deg]
float64 sound_horizontal_angle_corrected #[deg]
float64 slant_range #[m]
float64[3] hydrophone_time_difference #[micro sec] based on 1 matsuda check
int32[4] hydrophone_receive_level #[A/D scale]
float64 water_sound_speed #[m/s]
float64[2] target_position #[m]
float64 my_depth #[m]
int64[8] transmit_data
int64[8] receive_data
bool rotation_flag
float64 sound_horizontal_angle_source #[deg] source to me -180 to 180
