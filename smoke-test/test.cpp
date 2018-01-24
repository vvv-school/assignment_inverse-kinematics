/*
 * Copyright (C) 2018 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Ugo Pattacini <ugo.pattacini@iit.it>
 * CopyPolicy: Released under the terms of the GNU GPL v3.0.
*/

#include <cstdlib>
#include <vector>
#include <cmath>
#include <limits>
#include <string>

#include <opencv2/opencv.hpp>

#include <rtf/dll/Plugin.h>
#include <rtf/TestAssert.h>

#include <yarp/rtf/TestCase.h>
#include <yarp/os/Network.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Property.h>
#include <yarp/os/Time.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Mutex.h>
#include <yarp/os/LockGuard.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>
#include <yarp/math/Rand.h>

#include <iCub/ctrl/math.h>
#include <iCub/ctrl/pids.h>

using namespace std;
using namespace RTF;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;


/**********************************************************************/
cv::Point repoint(const ImageOf<PixelRgb> &img, const Vector &p)
{
    return cv::Point((int)(img.width()/2.0+p[0]),
                     (int)(img.height()/2.0-p[1]));
}


/**********************************************************************/
void drawAxes(const ImageOf<PixelRgb> &img, const Matrix &H)
{
    Vector x=20.0*H.getCol(0);
    Vector y=20.0*H.getCol(1);
    Vector p=H.getCol(2);
    cv::Mat imgMat=cv::cvarrToMat(img.getIplImage());
    cv::line(imgMat,repoint(img,p),repoint(img,p+x),cv::Scalar(255,0,0));
    cv::line(imgMat,repoint(img,p),repoint(img,p+y),cv::Scalar(0,255,0));
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
    Matrix draw(const Matrix &H, const double joint, ImageOf<PixelRgb> &img) const
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
            pts.push_back(repoint(img,p));
        }
        vector<vector<cv::Point>> poly(1,pts);

        cv::Mat imgMat=cv::cvarrToMat(img.getIplImage());
        cv::fillPoly(imgMat,poly,cv::Scalar(96,176,224));
        cv::circle(imgMat,pts[0],4,cv::Scalar(0,255,0),CV_FILLED);
        cv::circle(imgMat,pts[3],4,cv::Scalar(0,255,0),CV_FILLED);

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
        for (int i=0; i<links.size(); i++)
            H=links[i].draw(H,joints->get()[i],img);
        drawAxes(img,H);
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
class TestAssignmentInverseKinematics : public yarp::rtf::TestCase
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

    Mutex mutex;
    class Handler : public RateThread
    {
        void run()override { tester->move(); }
    public:
        TestAssignmentInverseKinematics *tester;
        Handler() : RateThread(0) { }
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
        link_length=property.check("link-length",Value(60.0)).asDouble();
        env_edge=property.check("environment-edge",Value(500)).asInt();
        period=property.check("period",Value(0.01)).asDouble();

        robot=new Robot(3,link_length,period);

        portEnvironment.open("/"+getName()+"/environment:o");
        portMotors.open("/"+getName()+"/motors:i");
        portEncoders.open("/"+getName()+"/encoders:o");
        portTarget.open("/"+getName()+"/target:o");

        RTF_ASSERT_ERROR_IF_FALSE(Network::connect(portEnvironment.getName(),
                                                   "/environment"),
                                  "Unable to connect to environment port");

        RTF_ASSERT_ERROR_IF_FALSE(Network::connect("/assignment_inverse-kinematics/motors:o",
                                                   portMotors.getName()),
                                  "Unable to connect to motors port");

        RTF_ASSERT_ERROR_IF_FALSE(Network::connect(portEncoders.getName(),
                                                   "/assignment_inverse-kinematics/encoders:i"),
                                  "Unable to connect to encoders port");

        RTF_ASSERT_ERROR_IF_FALSE(Network::connect(portTarget.getName(),
                                                   "/assignment_inverse-kinematics/target:i"),
                                  "Unable to connect to target port");

        velocity.resize(3,0.0);
        target.resize(3,0.0);
        Hee=eye(3,3);

        Rand::init();

        thread.tester=this;
        thread.setRate((int)(1000.0*period));
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
        LockGuard lg(mutex);
        if (Vector *vel=portMotors.read(false))
            if (vel->length()==velocity.length())
                velocity=*vel;

        ImageOf<PixelRgb> &env=portEnvironment.prepare();
        env.resize(env_edge,env_edge); env.zero();

        cv::Mat imgMat=cv::cvarrToMat(env.getIplImage());
        cv::circle(imgMat,repoint(env,target),5,cv::Scalar(0,0,255),CV_FILLED);

        Vector rot(4,0.0);
        rot[2]=1.0; rot[3]=target[2];
        Matrix H=axis2dcm(rot).submatrix(0,2,0,2);
        H(0,2)=target[0]; H(1,2)=target[1]; H(2,2)=1.0;
        drawAxes(env,H);

        Hee=robot->move(velocity,env);

        portEncoders.prepare()=robot->getJoints();
        portEncoders.writeStrict();
        portEnvironment.writeStrict();
    }

    /******************************************************************/
    void applyTarget(const double radius, const double alpha,
                     const double phi)
    {
        LockGuard lg(mutex);
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
                       const double min_phi, const unsigned int points,
                       unsigned int &score)
    {
        LockGuard lg(mutex);
        if (norm(velocity)<1.0*CTRL_DEG2RAD)
        {
            if (norm(Hee.getCol(2).subVector(0,1)-target.subVector(0,1))<min_dist)
            {
                Vector j=robot->getJoints();
                double phi=j[0]+j[1]+j[2];
                if (fabs(target[2]-phi)<min_phi*CTRL_DEG2RAD)
                {
                    score+=points;
                    RTF_TEST_REPORT(Asserter::format("%s Successful! Gained %d point(s)",
                                                     test.c_str(),points));
                    return;
                }
            }
        }

        RTF_TEST_REPORT(Asserter::format("%s Failed!",test.c_str()));
    }

    /******************************************************************/
    void run()override
    {
        RTF_TEST_REPORT("Testing controller's performance");
        unsigned int score=0;

        applyTarget(100.0,45.0,0.0);
        Time::delay(20.0);
        assign_points("Got reachable goal?",1.0,1.0,1,score);

        applyTarget(100.0,180+45.0,180.0);
        Time::delay(20.0);
        assign_points("Got reachable goal?",1.0,1.0,1,score);

        applyTarget(150.0,180+45.0,0.0);
        Time::delay(20.0);
        assign_points("Got reachable position?",10.0,
                      std::numeric_limits<double>::infinity(),2,score);

        applyTarget(220.0,90.0,0.0);
        Time::delay(20.0);
        assign_points("Best effort done?",1.1*(220.0-3.0*link_length),
                      std::numeric_limits<double>::infinity(),2,score);

        RTF_TEST_CHECK(score>0,Asserter::format("Total score = %d",score));
    }
};

PREPARE_PLUGIN(TestAssignmentInverseKinematics)
