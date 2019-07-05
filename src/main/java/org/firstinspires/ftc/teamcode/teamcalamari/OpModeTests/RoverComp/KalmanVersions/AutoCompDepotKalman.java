package org.teamcalamari.OpModeTests.RoverComp.KalmanVersions;

import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.teamcalamari.Angle;
import org.teamcalamari.AutonomousNavigationSim.AllDirectionsNavigation;
import org.teamcalamari.FilterStuff.FiltersAccelIntegration.KalmanFilterDropSensor;
import org.teamcalamari.HardwareSim.RobotsSim.RoverRobot;
import org.teamcalamari.OpModeSim.LinearOpMode;
import org.teamcalamari.OpModeTests.CompareAuto.Kalman.KalmanFilter;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Depot Comp", group="Auto Comp")
public class AutoCompDepotKalman extends LinearOpMode {

    private RoverRobot robot = new RoverRobot(RoverRobot.RoverCompPrograms.DEPOT);

    private double speed = 0.5;
    private double driveSpeed = 0.8;

    private double stillTurnSpeed = 0.5;
    private double stillTurnTurnSpeed = 0.5;

    private double moveTurnSpeed = 0.5;

    private int targetNum = 7;
    private AllDirectionsNavigation nav = new AllDirectionsNavigation(targetNum, speed, driveSpeed, stillTurnSpeed, stillTurnTurnSpeed, moveTurnSpeed, this);

    private double maxX = 72;
    private double maxY = 72;
    private double ultraXToRobotCenter = 8;
    private double ultraYToRobotCenter = 8;
    
    private KalmanFilterDropSensor kalmanFilter = KalmanFilter.getFilter();
    private List<Acceleration> emptyAccelList = new ArrayList<Acceleration>();
    private List<Velocity> emptyVeloList = new ArrayList<Velocity>();
    
    @Override
    public void runOpMode(){

        nav.targets.set(0, new VectorF(17, -3));
        nav.targets.set(1, new VectorF(17, -6));
        nav.targets.set(2, new VectorF(20, -6));
        nav.targets.set(3, new VectorF(39, 0));
        nav.targets.set(4, new VectorF(39, 0));
        nav.targets.set(5, new VectorF(55, 0));
        nav.targets.set(6, new VectorF(76.5f, 0));

        nav.speeds.set(0, 0.25);
        nav.speeds.set(1, 0.25);

        nav.stationaryTurns.set(2, Angle.add(Angle.negate(robot.phoneAngle), new Angle(25)));
        nav.stationaryTurns.set(3, new Angle(0));

        nav.actions.set(0, robot.lowerRobot);
        robot.nav = nav;
        nav.actions.set(2, robot.detectSamples);
        nav.actions.set(6, robot.dropMarker);

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
        nav.update(nav.targets.get(0), initialHeading, true);
        nav.initialize(robot.drive);

        telemetry.addData("Waiting ", "For Start");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()){
        	if(nav.i <= nav.targets.size()-1 && nav.stationaryTurns.get(nav.i) != null) {
        		robot.imu.lastReturned = (int) nav.stationaryTurns.get(nav.i).getDegree()+215;
        	}
        	
            Angle heading = new Angle(robot.imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle, AngleUnit.DEGREES);
            heading.subtract(new Angle(215));
            robot.drive.updateEncoderMotion(heading);
            nav.updateHeading(heading, true);
            
            double ultraCorrX = correctData(robot.ultraX.getData(DistanceUnit.INCH));
            double ultraCorrY = correctData(robot.ultraY.getData(DistanceUnit.INCH));
            double ultraPoseX = maxX-ultraCorrX-ultraXToRobotCenter;
            double ultraPoseY = maxY-ultraCorrY-ultraYToRobotCenter;
            
            Position encPose = robot.drive.getEncoderMotion().position;
            Position absEncPose = new Position(encPose.unit, encPose.x+nav.targets.get(0).get(0), encPose.y+nav.targets.get(0).get(1),
            		encPose.z, encPose.acquisitionTime);
            
            robot.ultraX.distance = obscureData(-absEncPose.x+1-Math.random()/2+maxX-ultraXToRobotCenter);
            robot.ultraY.distance = obscureData(-absEncPose.y+1-Math.random()/2+maxY-ultraYToRobotCenter);
            Position ultraPose = new Position(DistanceUnit.INCH, ultraPoseX, ultraPoseY, 0, 0);
            
            ArrayList<Position> poses = new ArrayList<Position>(2);
            poses.add(absEncPose);
            poses.add(ultraPose);
            kalmanFilter.update(emptyAccelList, emptyVeloList, poses);
            Position pose = kalmanFilter.getPosition();
            
            nav.updatePosition(new VectorF((float)pose.x, (float)pose.y), true);
            nav.run();

            telemetry.addData("State: ", nav.robotState+" "+nav.i);
            telemetry.addData("Heading", Math.round(nav.getHeading().getDegree()));
            telemetry.addData("Position", nav.getPosition().get(0)+"  "+nav.getPosition().get(1));
            telemetry.addData("Gold Mineral", robot.goldLocationString);
            telemetry.addData("Gold Location", robot.goldLocation);
            telemetry.addData("Gold Masses", robot.masses);
            //telemetry.addData("Distance to Ground", robot.groundRange.getData(DistanceUnit.INCH));
            telemetry.addData("Targets", nav.targets.get(6));
            //telemetry.addData("Turns", nav.stationaryTurns.get(6));
            /*telemetry.addData("Lift position", robot.lift.getCurrentPosition());
            telemetry.addData("Shoulder position", robot.arm.shoulderMotor.getCurrentPosition());
            telemetry.addData("Elbow position", robot.arm.elbowMotor.getCurrentPosition());*/
            telemetry.update();
        }
        robot.stop();
    }
    
    private double correctData(double data){
        return 0.9828*Math.pow(data, 1.0247);
    }
    private double obscureData(double data) {
    	return data > 0 ? Math.pow(data/0.9828, 1/1.0247) : 0;
    }
}