package org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.OpModeTests.Rover;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.teamcalamari.Angle;
import org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.AutonomousNavigationSim.AllDirectionsNavigationSim;
import org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.AutonomousNavigationSim.AutonomousNavigationSim.RobotStates;
import org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.HardwareSim.RobotsSim.RoverRobotSim;
import org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.OpModeSim.LinearOpModeSim;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Crater Comp", group="Auto Comp")
public class AutoCompCrater extends LinearOpModeSim {

    private RoverRobotSim robot = new RoverRobotSim(RoverRobotSim.RoverCompPrograms.CRATER);

    private double speed = 0.5;
    private double driveSpeed = 0.8;

    private double stillTurnSpeed = 0.5;
    private double stillTurnTurnSpeed = 0.5;

    private double moveTurnSpeed = 0.5;

    private int targetNum = 4;
    private AllDirectionsNavigationSim nav = new AllDirectionsNavigationSim(targetNum, speed, driveSpeed, stillTurnSpeed, stillTurnTurnSpeed, moveTurnSpeed, this);

    @Override
    public void runOpMode(){

        nav.targets.set(0, new VectorF(17, 0));
        nav.targets.set(1, new VectorF(17, -3));
        nav.targets.set(2, new VectorF(51, 0));
        nav.targets.set(3, new VectorF(39, 0));

        nav.stationaryTurns.set(1, Angle.add(Angle.negate(robot.phoneAngle), new Angle(15)));
        nav.stationaryTurns.set(2, Angle.negate(robot.armAngle));

        nav.actions.set(0, robot.lowerRobot);
        robot.nav = nav;
        nav.actions.set(1 , robot.detectSamples);
        //nav.actions.set(2, robot.extendArm);

        nav.finalCode = new Runnable(){
            @Override
            public void run(){
                robot.stop();
            }
        };

        robot.init(hardwareMap);

        telemetry.addData("Status", "Imu inited");
        telemetry.update();

        //navigation initialization
        Angle initialHeading = new Angle(robot.imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle, AngleUnit.DEGREES);
        nav.update(new VectorF(17, 0), initialHeading, true);
        nav.initialize(robot.drive);

        telemetry.addData("Waiting ", "For Start");
        telemetry.update();
        
        waitForStart();
        
        robot.imu.lastReturned = 185;

        while(opModeIsActive()){
        	if(nav.robotState == RobotStates.STATIONARY_TURN) {
        		robot.imu.turning = true;
        	}
        	else {
        		if(nav.robotState == RobotStates.DRIVING && nav.i == 2) {
        			robot.imu.lastReturned = 120;
        		}
        		robot.imu.turning = false;
        	}
        	
            Angle heading = new Angle(robot.imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle, AngleUnit.DEGREES);
            heading.subtract(new Angle(135));
            robot.drive.updateEncoderMotion(heading);
            nav.updateHeading(heading, true);
            Position pose = robot.drive.getEncoderMotion().position;
            nav.updatePosition(new VectorF((float)pose.x, (float)pose.y), false);
            nav.run();

            telemetry.addData("State: ", nav.robotState+" "+nav.i);
            telemetry.addData("Heading", Math.round(nav.getHeading().getDegree()));
            telemetry.addData("Gold Mineral", robot.goldLocationString);
            telemetry.addData("Gold Masses", robot.masses);
            //telemetry.addData("Distance to Ground", robot.groundRange.getData(DistanceUnit.INCH));
            telemetry.addData("Lift position", robot.lift.getCurrentPosition());
            telemetry.addData("Shoulder position", robot.arm.shoulderMotor.getCurrentPosition());
            telemetry.addData("Elbow position", robot.arm.elbowMotor.getCurrentPosition());
            telemetry.update();
        }
    	robot.stop();
    }
}