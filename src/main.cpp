// Assignment on Inverse Kinematics.
//
// Author: Ugo Pattacini - <ugo.pattacini@iit.it>

#include <cstdlib>
#include <string>
#include <cmath>

#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;


class Controller : public RFModule
{
    double link_length;

    BufferedPort<Vector> portMotors;
    BufferedPort<Vector> portEncoders;
    BufferedPort<Vector> portTarget;

    Vector encoders;
    Vector target;

public:
    bool configure(ResourceFinder &rf)override
    {
        link_length=rf.check("link-length",Value(60.0)).asFloat64();

        portMotors.open("/assignment_inverse-kinematics/motors:o");
        portEncoders.open("/assignment_inverse-kinematics/encoders:i");
        portTarget.open("/assignment_inverse-kinematics/target:i");

        // init the encoder values of the 3 joints
        encoders=zeros(3);

        // init the target
        target=zeros(3);

        return true;
    }

    bool close()override
    {
        portMotors.close();
        portEncoders.close();
        portTarget.close();

        return true;
    }

    double getPeriod()override
    {
        return 0.01;
    }

    bool updateModule()override
    {
        // update the target from the net
        if (Vector *tar=portTarget.read(false))
            target=*tar;

        // update the encoder readouts from the net
        if (Vector *enc=portEncoders.read(false))
            encoders=*enc;

        // retrieve the current target position
        Vector ee_d=target.subVector(0,1);

        // retrieve the current target orientation
        double phi_d=target[2];

        Vector &vel=portMotors.prepare();
        vel=zeros(3);

        // FILL IN THE CODE

        // deliver the computed velocities to the actuators
        portMotors.writeStrict();

        return true;
    }
};


int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError()<<"YARP doesn't seem to be available";
        return EXIT_FAILURE;
    }

    ResourceFinder rf;
    rf.configure(argc,argv);

    Controller controller;
    return controller.runModule(rf);
}
