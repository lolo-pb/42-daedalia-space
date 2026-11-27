#include "CubeSatFsw.h"

/*
Sensors — inputs to the controller:
Gyro :
    axis: [x, y, z]          // measurement direction
    rate                     // rotation rate along axis, rad/s

Magnetometer :
    axis: [x, y, z]          // measurement direction
    field                    // magnetic field along axis, tesla

CoarseSunSensor :
    axis: [x, y, z]          // pointing direction
    valid                    // whether the reading is valid
    illumination             // measured sunlight

FineSunSensor :
    mounting_orientation
    valid
    sun_angles: [x, y]       // angles to Sun, radians

StarTracker :
    mounting_orientation
    valid
    orientation: [x, y, z, w] // measured attitude quaternion

GPS :
    valid
    time: [week, seconds]
    position: [x, y, z]      // metres
    velocity: [x, y, z]      // metres/second
    longitude, latitude, altitude

Accelerometer :
    position: [x, y, z]      // mounting position
    axis: [x, y, z]          // measurement direction
    acceleration             // along axis, m/s²

The actual GPS structure provides position and velocity in more than one reference frame, plus geocentric and WGS-84 coordinates.

Magnetorquers — outputs from the controller:
Magnetorquer :
    axis: [x, y, z]          // configured magnetic axis
    max_dipole               // configured limit, A·m²
    commanded_dipole         // controller output, A·m²

We have five: +X, −X, +Y, −Y and +Z. A command can be positive or negative, reversing the coil’s magnetic dipole.

Other data that may be useful to the controller:

Spacecraft :
    time
    control_interval
    mass
    centre_of_mass: [x, y, z]
    inertia: 3×3 matrix

    position: [x, y, z]
    velocity: [x, y, z]
    orientation: quaternion
    angular_velocity: [x, y, z]
    magnetic_field: [x, y, z]
    sun_direction: [x, y, z]

    sensor_validity_flags
    control_mode
*/

void CubeSatFsw(struct SCType *S){
}
