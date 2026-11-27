#include "42.h"
#include "CubeSatFsw.h"
/*
    Notes:
    MTB is magnectic torque bar, aka magnetorquer
*/


/* Coil order: +X, -X, +Y, -Y, +Z. */
void CubeSatMixDipole(struct AcType *AC, const double requestedDipole[3]){
    double commands[5] = {  /// Note that were using half the command on every MTB so ye. TODO
         0.5 * requestedDipole[0],  // +X coil: half the X request
        -0.5 * requestedDipole[0],  // -X coil: the other half
         0.5 * requestedDipole[1],  // +Y coil: half the Y request
        -0.5 * requestedDipole[1],  // -Y coil: the other half
         requestedDipole[2]         // +Z coil: all of the Z request
    };

    for (long i = 0; i < 5; i++) {
        double limit = AC->MTB[i].Mmax;
        AC->MTB[i].Mcmd = Limit(commands[i], -limit, limit);
    }
}

void CubeSatFsw(struct SCType *S){
    struct AcType *AC = &S->AC;

    // TODO : Initial values are assumed
    static double estimatedAttitude[4] = {0.0, 0.0, 0.0, 1.0}; 
    double angularVelocity[3] = {0.0, 0.0, 0.0};
    double sunVector[3] = {0.0, 0.0, 0.0};
    int sunVectorValid = 0;

    /* Add the gyro readings to the acumulated estimaredAttitude */
    if (AC->Ngyro == 3) {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                angularVelocity[j] += AC->Gyro[i].Rate * AC->Gyro[i].Axis[j];
            }
        }
        double quaternionRate[4];
        QW2QDOT(estimatedAttitude, angularVelocity, quaternionRate);
        for (int i = 0; i < 4; i++) {
            estimatedAttitude[i] += quaternionRate[i] * AC->DT;
        }
        UNITQ(estimatedAttitude); // normalize it
    }

    /* This reconstruction assumes the 5 sensors               */
    /* TODO : i assume solar pannels count as this, check this */
    for (long i = 0; i < AC->Ncss; i++) {
        if (AC->CSS[i].Valid && AC->CSS[i].Scale > 0.0) {
            for (int j = 0; j < 3; j++) {
                sunVector[j] += AC->CSS[i].Illum / AC->CSS[i].Scale * AC->CSS[i].Axis[j];
            }
        }
    }
    if (MAGV(sunVector) > 0.0) { // |V| magnitude
        UNITV(sunVector); // normalize it
        sunVectorValid = 1;
    }

    /* Placeholders for the target and controller calculations. */
    double targetAttitude[4] = {0.0, 0.0, 0.0, 1.0};    
    double errorQuaternion[4] = {0.0, 0.0, 0.0, 1.0};
    
    double requestedTorque[3] = {0.0, 0.0, 0.0};
    double requestedDipole[3] = {0.0, 0.0, 0.0};

    CubeSatMixDipole(AC, requestedDipole);
    return;
}

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
