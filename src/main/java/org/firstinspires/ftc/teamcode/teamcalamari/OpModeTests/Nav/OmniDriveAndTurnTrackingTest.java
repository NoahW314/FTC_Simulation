package org.firstinspires.ftc.teamcode.teamcalamari.OpModeTests.Nav;

import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.teamcalamari.OpModeType;
import org.firstinspires.ftc.teamcode.teamcalamari.Navigation.Angle;
import org.firstinspires.ftc.teamcode.teamcalamari.Navigation.DistanceMeasure;
import org.firstinspires.ftc.teamcode.teamcalamari.Navigation.Position;
import org.firstinspires.ftc.teamcode.teamcalamari.Navigation.AutonomousNavigation.AllDirectionsRobotNavigation;
import org.firstinspires.ftc.teamcode.teamcalamari.Navigation.AutonomousNavigation.AutonomousRobotNavigation;
import org.firstinspires.ftc.teamcode.teamcalamari.Navigation.AutonomousNavigation.Point;
import org.firstinspires.ftc.teamcode.teamcalamari.Navigation.AutonomousNavigation.Transition;
import org.firstinspires.ftc.teamcode.teamcalamari.OpModeSim.LinearOpMode;
import org.firstinspires.ftc.teamcode.teamcalamari.RobotAction.RobotAction;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.DriveClasses.KiwiDrive;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.DriveClasses.OmniWheelDrive;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Auto Path Test")
public class OmniDriveAndTurnTrackingTest extends LinearOpMode {
    //the angle of the front of the robot relative to the blue wheel
    private Angle angle = new Angle(0, AngleUnit.DEGREES);
    //the speed of the robot
    private double speed = 0.25;
    private double driveSpeed = 0.5;

    private double stillTurnSpeed = 0.25;
    private double stillTurnTurnSpeed = 0.5;

    //the motors, servos, sensors, servo values, and drivetrain
    private BNO055IMU imu;

    private OmniWheelDrive drive;

    //navigation variables
    private AutonomousRobotNavigation nav;

    private RobotAction log = new RobotAction() {
        ElapsedTime time = new ElapsedTime();
        public boolean start() {
            time.reset();
            return true;
        }
        public boolean act() {
            if(time.seconds() > 3) {
                return true;
            }
            else {
                telemetry.addData("Drive Action", "Running");
            }
            return false;
        }
    };

    @Override
    public void runOpMode() throws InterruptedException {

        List<Point> points = new ArrayList<>();
        List<Transition> transitions = new ArrayList<>();
        Point.PointBuilder pointBuilder = new Point.PointBuilder(stillTurnSpeed, stillTurnTurnSpeed, false);
        Transition.TransitionBuilder transBuilder = new Transition.TransitionBuilder(speed, driveSpeed, false);

        points.add(0, pointBuilder
                .addTurn(new Angle(-45))
                .addAction(new RobotAction() {
                    @Override
                    public boolean act() {
                        System.out.println("Actioning 1");
                        return true;
                    }
                })
                .build(new Position(0, 0)));

        points.add(1, pointBuilder
                .addTurn(new Angle(125))
                .addAngleTillP(new Angle(25))
                .addAction(new RobotAction(){
                    public boolean act(){
                        System.out.println("Actioning 2");
                        return true;
                    }
                })
                .addActionFirst(true)
                .build(new Position(24, 0)));
        points.add(2, pointBuilder.build(new Position(24, 24)));
        points.add(3, pointBuilder.build(new Position(0, 0)));

        transitions.add(0, transBuilder.addAction(log).addDistTillP(6, DistanceUnit.INCH).addContinuous(false).build());
        transitions.add(1, transBuilder.addTurn(new Angle(0)).addTurnToRequired(false).addTurnSpeed(0.5).build());
        transitions.add(2, transBuilder.build());

        //IMU initialization
        BNO055IMU.Parameters parametersIMU = new BNO055IMU.Parameters();
        parametersIMU.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parametersIMU.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parametersIMU.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parametersIMU.loggingEnabled      = true;
        parametersIMU.loggingTag          = "IMU";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parametersIMU);

        //drivetrain initialization
        drive = new KiwiDrive(hardwareMap, angle, OpModeType.AUTO, "RedMotor", "GreenMotor", "BlueMotor");
        drive.updateEncoders();
        drive.inchesPerTick = Math.PI*4/1120;
        drive.turnRadius = new DistanceMeasure(8, DistanceUnit.INCH);
        drive.turnError = new Angle(2);
        drive.setDriveError(2, DistanceUnit.INCH);

        //navigation initialization
        nav = new AllDirectionsRobotNavigation(this, points, transitions, drive);
        nav.initialize(new Position(0, 0), new Angle(0, AngleUnit.DEGREES), true);

        waitForStart();

        Angle prevHeading = drive.getEncoderHeading().copy();
        while(opModeIsActive()) {
            drive.updateEncoderMotionTurn(prevHeading);
            Angle heading = drive.getEncoderHeading().copy(); 
            nav.updateHeading(heading, true);
            Position pose = drive.getEncoderPosition();
            nav.updatePosition(pose, true);
            nav.run();
            drive.updateEncoders();
            
            prevHeading = heading.copy();
            
            telemetry.addData("Heading", heading);
            telemetry.addData("Position", drive.getEncoderPosition());
        }
    }
}
