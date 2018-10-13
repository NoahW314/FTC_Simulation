package org.firstinspires.ftc.teamcode.teamcalamari.Simulation.HardwareSim.DriveSim.OldDrives;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.HardwareSim.MotorsSim.MotorSim;

/**
 * Created by DesignDrawing on 11/11/2017.
 */

public class OmniWheelDriveAtAngleAutoSim {
    //the array storing the motors
    public MotorSim[] wheelMotors;
    //powers of the motors
    public double[] wheelPowers = {0, 0, 0, 0};
    //the angle of the front of the robot relative to the blue wheel
    private int headAngle;
    //whether or not the robot is turning
    public boolean turning;

    //the constructor
    public OmniWheelDriveAtAngleAutoSim(MotorSim[] wheels, int headOffset){
        wheelMotors = wheels;
        headAngle = headOffset;
    }
    //drives the robot at a set angle
    public void driveAtAngle(double driveAngle, double speed) {
        //convert to radians
        double angleR = driveAngle*Math.PI/180;
        //extract the x and y components from the vector
        double x = Math.cos(angleR);
        double y = Math.sin(angleR);

        //convert to radians
        double headAngleR = headAngle*Math.PI/180;
        //rotates the x and y components to account for the front being offset
        double px = x*Math.cos(headAngleR)-y*Math.sin(headAngleR);
        double py = y*Math.cos(headAngleR)+x*Math.sin(headAngleR);

        //scale the components by the speed
        px*=speed;
        py*=speed;

        //store the powers of the wheels in an array so we can loop through them
        double[] f = {-px, py, px, -py};

        //add the calculated powers to the current wheel powers
        for(int i = 0; i < 4; i++) {
            wheelPowers[i]+=f[i];
        }
    }
    /*drives the robot towards a point.
    The robot will continue to try to reach this point until you stop calling this method.*/
    public void driveTo (double speed, double heading, VectorF currentPosition, VectorF finalPosition) {
        //extract and store the x and y components of the positions.
        //this step is totally pointless.
        double cx = currentPosition.get(0);
        double cy = currentPosition.get(1);
        double fx = finalPosition.get(0);
        double fy = finalPosition.get(1);

        //calculate the angle the robot needs to drive at reach the target position
        double driveAngle = Math.atan2(fy-cy, fx-cx);
        this.driveAtAngle(driveAngle*180/Math.PI-heading, speed);
    }
    //turn the robot
    public void turn(double speed){
        for(int i = 0; i < 4; i++) {
            //set the power
            wheelPowers[i]+=speed;
        }
        //we are turning
        turning = true;
    }
    /*turn the robot to a set angle.
    The robot will continue to try to reach this angle until you stop calling the method*/
    public void angleTurn (double currentAngle, double finalAngle, double turnSpeed, double angleError) {
        if(round(finalAngle, angleError) > round(currentAngle, angleError)) {
            turn(turnSpeed);
        }
        else if(round(finalAngle, angleError) < round(currentAngle, angleError)){
            turn(-turnSpeed);
        }
        else{
            turning = false;
        }
    }
    //set the calculated power to the wheel motors
    public void drive (double speed) {
        //scale the motor powers down so that the absolute values don't exceed 1

        //determine the maximum wheel power
        double maxi = Math.abs(wheelPowers[0]);
        for(int i = 1; i < 4; i++){
            if(Math.abs(wheelPowers[i]) > maxi){
                maxi = Math.abs(wheelPowers[i]);
            }
        }

        /*divide all the motor powers by the maximum power to preserve
        the ratios between the wheels while keep the powers under 1*/
        if(maxi != 0){
            for(int i = 0; i < 4; i++){
                wheelPowers[i]*=(speed/maxi);
            }
        }

        //set the wheel motor powers
        //reset the wheelPowers array for next time
        for(int i = 0; i < 4; i++) {
            wheelMotors[i].setPower(wheelPowers[i]);
            wheelMotors[i].run();
            wheelPowers[i] = 0;
        }
    }
    //rounds n to the nearest multiple of b
    double round(double n, double b){
        if(b != 0) {
            n = Math.round(n / b) * b;
        }
        return n;
    }
}
