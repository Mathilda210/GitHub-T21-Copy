# Message header metadata
std_msgs/Header header

# List of joint names
string[] name

# Joint positions (radians for rotary joints | meters for prismatic joints)
float64[] position

# Joint velocities (radians/sec for rotary joints | meters/sec for prismatic joints)
float64[] velocity

# Joint effort / torque (newton-meters for rotary joints | newtons for prismatic joints)
float64[] effort