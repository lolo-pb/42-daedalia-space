#include "42.h"
#include "CubeSatFsw.h"
/*
    Notes:
    MTB is magnectic torque bar, aka magnetorquer
*/
/// PID ///
typedef struct {
    double proportional;
    double integral;
    double derivative;
    double minimum_integral ;
    double maximum_integral;
    double minimum_output ;
    double maximum_output;
    double accumulated_integral;
}PID;

double PidUpdate(PID *pid, double error, double measured_rate, double timestep) {
    pid->accumulated_integral = Limit(
        pid->accumulated_integral + error * timestep,
        pid->minimum_integral,
        pid->maximum_integral
    );

    return Limit(
        pid->proportional * error
            + pid->integral * pid->accumulated_integral
            - pid->derivative * measured_rate,
        pid->minimum_output,
        pid->maximum_output);
}
/// PID END ///


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
    static PID attitudePid[3] = { // 3 PIDS for each axis /// TODO : This needs tu tuning
        {.minimum_integral = -1.0, .maximum_integral = 1.0, .minimum_output = -1.0, .maximum_output = 1.0},
        {.minimum_integral = -1.0, .maximum_integral = 1.0, .minimum_output = -1.0, .maximum_output = 1.0},
        {.minimum_integral = -1.0, .maximum_integral = 1.0, .minimum_output = -1.0, .maximum_output = 1.0}
    };

    // TODO : Initial values are assumed
    static double estimatedAttitude[4] = {0.0, 0.0, 0.0, 1.0}; 
    double angularVelocity[3] = {0.0, 0.0, 0.0};
    double sunVector[3] = {0.0, 0.0, 0.0};
    int sunVectorValid = 0;
    double magneticField[3] = {0.0, 0.0, 0.0};

    /* Add the gyro readings to the acumulated estimaredAttitude */
    /// Gyro attitude estimate
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
    /// Sun vector calculation
    for (long i = 0; i < AC->Ncss; i++) {
        if (AC->CSS[i].Valid && AC->CSS[i].Scale > 0.0) {
            for (int j = 0; j < 3; j++) {
                sunVector[j] += AC->CSS[i].Illum / AC->CSS[i].Scale * AC->CSS[i].Axis[j];
            }
        }
    }
    if (MAGV(sunVector)) { // |V| magnitude
        UNITV(sunVector); // normalize it
        sunVectorValid = 1;
    }
    // TODO : we still dont use the sun vector to correct attitude estimate

    /* Three orthogonal magnetometer axes are assumed. */
    /// Calculate magnetic field from magnetometer
    if (AC->Nmag == 3) {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                magneticField[j] += AC->MAG[i].Field * AC->MAG[i].Axis[j];
            }
        }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////// This semaprates sensor readings wiht magnetorquer commands /////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////

    /* Placeholders for the target and controller calculations. */
    double targetAttitude[4] = {0.0, 0.0, 0.0, 1.0}; // TODO : this is a plceholder  
    double errorQuaternion[4];
    double requestedTorque[3];
    double requestedDipole[3] = {0.0, 0.0, 0.0};

    QxQT(estimatedAttitude, targetAttitude, errorQuaternion);
    RECTIFYQ(errorQuaternion);

    /// Update requested torque with PIDs
    for (int i = 0; i < 3; i++) 
        requestedTorque[i] = PidUpdate(&attitudePid[i], -errorQuaternion[i],angularVelocity[i], AC->DT);
        //                                             ^^^ this is negative bacuse of 42 convention - _-

    /* torque = dipole × magnetic field */
    /*  =>  (BxT)/(|B|^2)               */
    /// Target Magnetic dipole with taget torque and magnetic field 
    double fieldSquared = VoV(magneticField, magneticField);
    if (fieldSquared > 1.0e-18) {
        VxV(magneticField, requestedTorque, requestedDipole);
        for (int i = 0; i < 3; i++) requestedDipole[i] /= fieldSquared;
    }

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
