/**
 * DRIFTPlex.h - A group of DRIFT motors controlling a single node
 * Created by Carson G. Ray
*/

#ifndef DRIFTPlex_h
#define DRIFTPlex_h

#include "Arduino.h"
#include "DRIFTMotor.h"
#include "ServoController.h"
#include <TrackRing.h>
#include <BusChain.h>
#include <math.h>
#include <ArduinoEigenDense.h>

using namespace Eigen;

class DRIFTPlex {
    private:
        //DRIFT motors
        DRIFTMotor* motors;
        //Home points
        Vector3f* homePoints;
        //Number of motors
        uint8_t numMotors;

        //3D position
        Vector3f position;
        //3D velocity
        Vector3f velocity;
        //Slant matrix
        Matrix3f slants;

        //Operating mode
        enum Mode {
          FORCE,
          POSITION,
        };

		//Default mode is force
        Mode mode = FORCE;
		//Target force vector
        Vector3f forceTarget;
		//Target POSITION
        Vector3f posLimit;
        //Collision flag
        bool collision = false;

        void setMode(Mode mode);
    public:
        void attach(DRIFTMotor* motors, Vector3f* homePoints, uint8_t numMotors);
        void localize();
        void setForceTarget();
        void setForceTarget(Vector3f force);
        void setPositionLimit(Vector3f target, bool collision);
        void updateController();
        Mode getMode();
        Vector3f getPosition();
        Vector3f getVelocity();
        Vector3f getPredictedPos();
        float getPredictedPos(uint8_t motor);
};

#endif