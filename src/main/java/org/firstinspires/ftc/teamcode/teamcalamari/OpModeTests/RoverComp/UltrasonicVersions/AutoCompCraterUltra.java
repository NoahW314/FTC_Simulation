package org.teamcalamari.OpModeTests.RoverComp.UltrasonicVersions;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.teamcalamari.Angle;
import org.teamcalamari.AutonomousNavigationSim.AllDirectionsNavigation;
import org.teamcalamari.HardwareSim.RobotsSim.RoverRobot;
import org.teamcalamari.OpModeSim.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.teamcalamari.AutonomousNavigationSim.AutonomousNavigation.RobotStates;

@Autonomous(name="Crater Comp", group="Auto Comp")
public class AutoCompCraterUltra extends LinearOpMode {

    private RoverRobot robot = new RoverRobot(RoverRobot.RoverCompPrograms.CRATER);

    private double speed = 0.5;
    private double driveSpeed = 0.8;

    private double stillTurnSpeed = 0.5;
    private double stillTurnTurnSpeed = 0.5;

    private double moveTurnSpeed = 0.5;

    private int targetNum = 6;//7
    private AllDirectionsNavigation nav = new AllDirectionsNavigation(targetNum, speed, driveSpeed, stillTurnSpeed, stillTurnTurnSpeed, moveTurnSpeed, this);

    private double maxX = 72;
    private double maxY = 72;
    private double ultraXToRobotCenter = 8;
    private double ultraYToRobotCenter = 8;
    
    @Override
    public void runOpMode(){

        nav.targets.set(0, new VectorF(17, -3));
        nav.targets.set(1, new VectorF(17, -6));
        nav.targets.set(2, new VectorF(20, -6));
        nav.targets.set(3, new VectorF(39, 0));
        nav.targets.set(4, new VectorF(39, 0));
        nav.targets.set(5, new VectorF(55, 0));
        //nav.targets.set(6, new VectorF(35, 0));

        nav.speeds.set(0, 0.25);
        nav.speeds.set(1, 0.25);

        nav.stationaryTurns.set(2, Angle.add(Angle.negate(robot.phoneAngle), new Angle(25)));
        nav.stationaryTurns.set(3, new Angle(0));
        //nav.stationaryTurns.set(4, Angle.negate(robot.armAngle));

        nav.actions.set(0, robot.lowerRobot);
        robot.nav = nav;
        nav.actions.set(2, robot.detectSamples);
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
        nav.update(nav.targets.get(0), initialHeading, true);
        nav.initialize(robot.drive);

        telemetry.addData("Waiting ", "For Start");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()){
        	if(nav.i <= nav.targets.size()-1 && nav.stationaryTurns.get(nav.i) != null) {
        		robot.imu.lastReturned = (int) nav.stationaryTurns.get(nav.i).getDegree()+135;
        	}
        	
            Angle heading = new Angle(robot.imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle, AngleUnit.DEGREES);
            heading.subtract(new Angle(135));
            robot.drive.updateEncoderMotion(heading);
            nav.updateHeading(heading, true);
            
            Position encPose = robot.drive.getEncoderMotion().position;
            
            robot.ultraX.distance = obscureData(-encPose.x-17+1-Math.random()/2+maxX-ultraXToRobotCenter);
            robot.ultraY.distance = obscureData(-encPose.y+3+1-Math.random()/2+maxY-ultraYToRobotCenter);
            
            double ultraCorrX = correctData(robot.ultraX.getData(DistanceUnit.INCH));
        	double ultraCorrY = correctData(robot.ultraY.getData(DistanceUnit.INCH));
        	double ultraPoseX = maxX-ultraCorrX-ultraXToRobotCenter;
        	double ultraPoseY = maxY-ultraCorrY-ultraYToRobotCenter;
        	
        	/*double poseMag = Math.sqrt(Math.pow(ultraPoseX, 2)+Math.pow(ultraPoseY, 2));
        	Angle poseAngle = new Angle(Math.atan2(ultraPoseY, ultraPoseX), AngleUnit.RADIANS);
        	
        	Angle newPoseAngle = (poseAngle.subtract(new Angle(45)));
        	double corrX = poseMag*Math.cos(newPoseAngle.getRadian());
        	double corrY = poseMag*Math.sin(newPoseAngle.getRadian());*/
            
            nav.updatePosition(new VectorF((float)ultraPoseX, (float)ultraPoseY), true);
            nav.run();

            telemetry.addData("State: ", nav.robotState+" "+nav.i);
            telemetry.addData("Heading", Math.round(nav.getHeading().getDegree()));
            telemetry.addData("Position", nav.getPosition().get(0)+"  "+nav.getPosition().get(1));
            telemetry.addData("Gold Mineral", robot.goldLocationString);
            telemetry.addData("Gold Masses", robot.masses);
            if(nav.i <= nav.targets.size()-1) {
            	telemetry.addData("Target", nav.targets.get(nav.i));
            	Angle turnTarget = nav.stationaryTurns.get(nav.i);
            	telemetry.addData("Turn Target", turnTarget == null ? 0 : turnTarget);
            }
            //telemetry.addData("Distance to Ground", robot.groundRange.getData(DistanceUnit.INCH));
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