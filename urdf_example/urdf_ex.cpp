//
// Created by dongju on 23. 9. 4.
//

#include <iostream>
#include "raisim/RaisimServer.hpp"
#include <camel-tools/trajectory.hpp>


double PDcontrol(double desiredPosition, double desiredVelocity, double position, double velocity)
{
    double KP = 100;
    double Kd = 5.0 ;

    double torque;
    double postionerror;
    double velocityerror;

    torque = KP*postionerror+Kd* velocityerror;

    return torque;

}

int main()
{
//    World set
    std::string urdfPath = "\\home\\dongju\\raisimLib\\camel_bipedal\\dongju_urdf\\serial_4bar\\urdf\\urdf_file.urdf";
    std::string name = "4barlinkage";
    raisim::World world;
    world.setTimeStep(0.001);
    auto ground = world.addGround(0, "gnd");
    auto pendulum = world.addArticulatedSystem(urdfPath);
    std::cout << "DOF : " << pendulum->getGeneralizedCoordinateDim() << std::endl;
    //

    Eigen::VectorXd initialJointPosition(pendulum->getGeneralizedCoordinateDim());
    std::cout<<1;
    Eigen::VectorXd initialJointVelocity(pendulum->getGeneralizedVelocityDim());
    initialJointPosition.setZero();
    initialJointVelocity.setZero();

    initialJointPosition << 1.0,1.0;
    pendulum->setGeneralizedCoordinate(initialJointPosition);
    pendulum->setGeneralizedVelocity(initialJointVelocity);

    //open Raisim server

    raisim::RaisimServer server(&world);
    server.launchServer();
    server.focusOn(pendulum);
    sleep(2);

    // control
    double position = pendulum->getGeneralizedCoordinate()[0];
    double velocity = pendulum->getGeneralizedVelocity()[0];
    double desiredPosition = 0;
    double desiredVelocity = 0;
    Eigen::VectorXd torque = Eigen::VectorXd(2);
    torque.setZero();

    CubicTrajectoryGenerator trajectoryGenerator;
    trajectoryGenerator.updateTrajectory(position, 3.141592, 0, 4);

    int iteration = 0;
    double localTime = 0;

    while (localTime <= 50)
    {
        iteration++;
        localTime = iteration * world.getTimeStep();
        position = pendulum->getGeneralizedCoordinate()[0];
        velocity = pendulum->getGeneralizedVelocity()[0];

        desiredPosition = trajectoryGenerator.getPositionTrajectory(localTime);
        desiredVelocity = trajectoryGenerator.getVelocityTrajectory(localTime);

        torque[1] = PDcontrol(desiredPosition, desiredVelocity, position, velocity);
        pendulum->setGeneralizedForce(torque);
        world.integrate();
        usleep(1000);

    }

    server.killServer();





}
