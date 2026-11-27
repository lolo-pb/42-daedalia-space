#ifndef CUBESAT_FSW_H
#define CUBESAT_FSW_H

struct SCType;
struct AcType;

void CubeSatFsw(struct SCType *S);
void CubeSatMixDipole(struct AcType *AC, const double requestedDipole[3]);

#endif
