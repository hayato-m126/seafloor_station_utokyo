# This express Water Data
Header header
float64 speed #[m/s]
float64 temperature #[degrees Celsius]
float64 salinity #[PSU]
bool reliability #if speed calculated by temperature and salinity obtained by sensor, True, if uses initial value False
geometry_msgs/Point location #observe point in robot coordinate system
