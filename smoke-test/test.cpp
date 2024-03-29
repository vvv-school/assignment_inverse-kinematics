/*
 * Copyright (C) 2018 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Ugo Pattacini <ugo.pattacini@iit.it>
 * CopyPolicy: Released under the terms of the GNU GPL v3.0.
*/

#include <cstdlib>
#include <mutex>
#include <vector>
#include <cmath>
#include <limits>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

#include <robottestingframework/dll/Plugin.h>
#include <robottestingframework/TestAssert.h>

#include <yarp/robottestingframework/TestCase.h>
#include <yarp/os/Network.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/Property.h>
#include <yarp/os/Time.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>
#include <yarp/math/Rand.h>
#include <yarp/cv/Cv.h>

#include <iCub/ctrl/math.h>
#include <iCub/ctrl/pids.h>

using namespace std;
using namespace robottestingframework;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::cv;
using namespace iCub::ctrl;


/**********************************************************************/
cv::Point repoint(cv::Mat &imgMat, const Vector &p)
{
    return cv::Point((int)(imgMat.size().width/2.0+p[0]),
                     (int)(imgMat.size().height/2.0-p[1]));
}


/**********************************************************************/
void drawAxes(cv::Mat &imgMat, const Matrix &H,
              const cv::Scalar &axis_x=cv::Scalar(255,0,0),
              const cv::Scalar &axis_y=cv::Scalar(0,255,0))
{
    Vector x=20.0*H.getCol(0);
    Vector y=20.0*H.getCol(1);
    Vector p=H.getCol(2);
    cv::line(imgMat,repoint(imgMat,p),repoint(imgMat,p+x),axis_x);
    cv::line(imgMat,repoint(imgMat,p),repoint(imgMat,p+y),axis_y);
}


/**********************************************************************/
class Link
{
    double length;
    vector<Vector> points;

public:
    /******************************************************************/
    Link(const double length_) :
       length(length_), points(6,Vector(3,1.0))
    {
        double height=6;

        points[0][0]=0.0;
        points[0][1]=0.0;

        points[1][0]=0.0;
        points[1][1]=height/2.0;

        points[2][0]=length;
        points[2][1]=height/2.0;

        points[3][0]=length;
        points[3][1]=0.0;

        points[4][0]=length;
        points[4][1]=-height/2.0;

        points[5][0]=0.0;
        points[5][1]=-height/2.0;
    }

    /******************************************************************/
    Matrix draw(const Matrix &H, const double joint, cv::Mat &imgMat) const
    {
        double c=cos(joint);
        double s=sin(joint);

        Matrix H_=eye(3,3);
        H_(0,0)=c; H_(0,1)=-s;
        H_(1,0)=s; H_(1,1)=c;
        H_=H*H_;

        vector<cv::Point> pts;
        for (auto &point:points)
        {
            Vector p=H_*point;
            pts.push_back(repoint(imgMat,p));
        }
        vector<vector<cv::Point>> poly(1,pts);

        cv::fillPoly(imgMat,poly,cv::Scalar(96,176,224));
        cv::circle(imgMat,pts[0],4,cv::Scalar(0,255,0),cv::FILLED);
        cv::circle(imgMat,pts[3],4,cv::Scalar(0,255,0),cv::FILLED);

        Matrix T=eye(3,3); T(0,2)=length;
        return H_*T;
    }
};


/**********************************************************************/
class Robot
{
    vector<Link> links;
    Integrator *joints;

public:
    /******************************************************************/
    Robot(const int n, const double length, const double Ts)
    {
        for (int i=0; i<n; i++)
            links.push_back(Link(length));
        joints=new Integrator(Ts,Vector(n,0.0));
    }

    /******************************************************************/
    Matrix move(const Vector &velocity, ImageOf<PixelRgb> &img)
    {
        Matrix H=eye(3,3);
        joints->integrate(velocity);

        cv::Mat imgMat=toCvMat(img);
        for (int i=0; i<links.size(); i++)
            H=links[i].draw(H,joints->get()[i],imgMat);
        drawAxes(imgMat,H);

        return H;
    }

    /******************************************************************/
    Vector getJoints() const
    {
        return joints->get();
    }

    /******************************************************************/
    virtual ~Robot()
    {
        delete joints;
    }
};


/**********************************************************************/
class TestAssignmentInverseKinematics : public yarp::robottestingframework::TestCase
{
    Robot *robot;

    BufferedPort<ImageOf<PixelRgb>> portEnvironment;
    BufferedPort<Vector> portMotors;
    BufferedPort<Vector> portEncoders;
    BufferedPort<Vector> portTarget;

    Vector velocity;
    Vector target;
    Matrix Hee;

    double link_length;
    int env_edge;
    double period;

    mutex mtx;
    class Handler : public PeriodicThread
    {
        void run()override { tester->move(); }
    public:
        TestAssignmentInverseKinematics *tester;
        Handler() : PeriodicThread(0.0) { }
    } thread;

public:
    /******************************************************************/
    TestAssignmentInverseKinematics() : TestCase("TestAssignmentInverseKinematics")
    {
    }

    /******************************************************************/
    virtual ~TestAssignmentInverseKinematics()
    {
    }

    /******************************************************************/
    bool setup(yarp::os::Property &property)override
    {
        link_length=property.check("link-length",Value(60.0)).asFloat64();
        env_edge=property.check("environment-edge",Value(500)).asInt32();
        period=property.check("period",Value(0.01)).asFloat64();

        robot=new Robot(3,link_length,period);

        portEnvironment.open("/"+getName()+"/environment:o");
        portMotors.open("/"+getName()+"/motors:i");
        portEncoders.open("/"+getName()+"/encoders:o");
        portTarget.open("/"+getName()+"/target:o");

        if (!Network::connect(portEnvironment.getName(),"/environment"))
            ROBOTTESTINGFRAMEWORK_ASSERT_FAIL("Unable to connect to environment port");

        if (!Network::connect("/assignment_inverse-kinematics/motors:o",portMotors.getName()))
            ROBOTTESTINGFRAMEWORK_ASSERT_FAIL("Unable to connect to motors port");

        if (!Network::connect(portEncoders.getName(),"/assignment_inverse-kinematics/encoders:i"))
            ROBOTTESTINGFRAMEWORK_ASSERT_FAIL("Unable to connect to encoders port");

        if (!Network::connect(portTarget.getName(),"/assignment_inverse-kinematics/target:i"))
            ROBOTTESTINGFRAMEWORK_ASSERT_FAIL("Unable to connect to target port");

        velocity.resize(3,0.0);
        target.resize(3,0.0);
        Hee=eye(3,3);

        Rand::init();

        thread.tester=this;
        thread.setPeriod(period);
        thread.start();

        return true;
    }

    /******************************************************************/
    void tearDown()override
    {
        thread.stop();
        delete robot;

        portEnvironment.close();
        portMotors.close();
        portEncoders.close();
        portTarget.close();
    }

    /******************************************************************/
    void move()
    {
        lock_guard<mutex> lg(mtx);
        if (Vector *vel=portMotors.read(false))
            if (vel->length()==velocity.length())
                velocity=*vel;

        ImageOf<PixelRgb> &env=portEnvironment.prepare();
        env.resize(env_edge,env_edge); env.zero();

        cv::Mat imgMat=toCvMat(env);
        cv::circle(imgMat,repoint(imgMat,target),5,cv::Scalar(0,0,255),cv::FILLED);

        Vector rot(4,0.0);
        rot[2]=1.0; rot[3]=target[2];
        Matrix H=axis2dcm(rot).submatrix(0,2,0,2);
        H(0,2)=target[0]; H(1,2)=target[1]; H(2,2)=1.0;
        drawAxes(imgMat,H,cv::Scalar(0,0,255));

        Hee=robot->move(velocity,env);

        portEncoders.prepare()=robot->getJoints();
        portEncoders.writeStrict();
        portEnvironment.writeStrict();
    }

    /******************************************************************/
    void applyTarget(const double radius, const double alpha,
                     const double phi)
    {
        lock_guard<mutex> lg(mtx);
        double r=Rand::scalar(radius-10.0,radius+10.0);
        double a=CTRL_DEG2RAD*Rand::scalar(alpha-10.0,alpha+10.0);

        target[0]=r*cos(a);
        target[1]=r*sin(a);
        target[2]=CTRL_DEG2RAD*Rand::scalar(phi-5.0,phi+5.0);

        portTarget.prepare()=target;
        portTarget.writeStrict();
    }

    /******************************************************************/
    void assign_points(const string &test, const double min_dist,
                       const double min_phi, const unsigned int p1,
                       const unsigned int p2, unsigned int &score)
    {
        lock_guard<mutex> lg(mtx);
        if (norm(velocity)<1.0*CTRL_DEG2RAD)
        {
            if (norm(Hee.getCol(2).subVector(0,1)-target.subVector(0,1))<min_dist)
            {
                score+=p1;
                ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("Position reached! Gained %d point(s)",p1));

                Vector j=robot->getJoints();
                double phi=j[0]+j[1]+j[2];
                if (fabs(target[2]-phi)<min_phi*CTRL_DEG2RAD)
                {
                    score+=p2;
                    ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("%s Successful! Gained %d point(s)",
                                                     test.c_str(),p2));
                    return;
                }
            }
        }

        ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("%s Failed!",test.c_str()));
    }

    /******************************************************************/
    void run()override
    {
        ROBOTTESTINGFRAMEWORK_TEST_REPORT("Testing controller's performance");
        unsigned int score=0;

        applyTarget(100.0,45.0,0.0);
        ROBOTTESTINGFRAMEWORK_TEST_REPORT("Waiting 20 seconds...");
        Time::delay(20.0);
        assign_points("Got reachable goal?",1.0,1.0,1,2,score);

        applyTarget(100.0,180+45.0,180.0);
        ROBOTTESTINGFRAMEWORK_TEST_REPORT("Waiting 20 seconds...");
        Time::delay(20.0);
        assign_points("Got reachable goal?",1.0,1.0,1,2,score);

        applyTarget(150.0,180+45.0,0.0);
        ROBOTTESTINGFRAMEWORK_TEST_REPORT("Waiting 20 seconds...");
        Time::delay(20.0);
        assign_points("Got reachable position?",20.0,
                      std::numeric_limits<double>::infinity(),1,3,score);

        applyTarget(220.0,90.0,0.0);
        ROBOTTESTINGFRAMEWORK_TEST_REPORT("Waiting 20 seconds...");
        Time::delay(20.0);
        assign_points("Best effort done?",1.3*(220.0-3.0*link_length),
                      std::numeric_limits<double>::infinity(),1,3,score);

        ROBOTTESTINGFRAMEWORK_TEST_CHECK(score>0,Asserter::format("Total score = %d",score));
    }
};

ROBOTTESTINGFRAMEWORK_PREPARE_PLUGIN(TestAssignmentInverseKinematics)
