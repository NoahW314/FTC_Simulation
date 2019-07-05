package org.firstinspires.ftc.teamcode.teamcalamari.OpModeTests.Nav;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.teamcalamari.OpModeType;
import org.firstinspires.ftc.teamcode.teamcalamari.Navigation.Angle;
import org.firstinspires.ftc.teamcode.teamcalamari.Navigation.Position;
import org.firstinspires.ftc.teamcode.teamcalamari.Navigation.AutonomousNavigation.AllDirectionsNavigation;
import org.firstinspires.ftc.teamcode.teamcalamari.OpModeSim.LinearOpMode;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.DriveClasses.KiwiDrive;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.DriveClasses.OmniWheelDrive;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Omni Drive Turn Track Test")
public class OmniDriveAndTurnTrackingTest extends LinearOpMode {

    //the angle of the front of the robot relative to the blue wheel
    private Angle angle = new Angle(0, AngleUnit.DEGREES);
    //the speed of the robot
    private double speed = 1;
    private double driveSpeed = 0.5;

    private double stillTurnSpeed = 0.75;
    private double stillTurnTurnSpeed = 0.5;

    private double moveTurnSpeed = 0.5;

    //the motors, servos, sensors, servo values, and drivetrain
    private BNO055IMU imu;

    private OmniWheelDrive drive;

    //navigation variables
    private int targetNum = 4;
    private AllDirectionsNavigation nav = new AllDirectionsNavigation(targetNum, speed, driveSpeed, stillTurnSpeed, stillTurnTurnSpeed, moveTurnSpeed, this);

    @Override
    public void runOpMode() throws InterruptedException {
        nav.targets.set(0, new VectorF(0, 0));
        nav.targets.set(1, new VectorF(24, 0));
        nav.targets.set(2, new VectorF(24, 24));
        nav.targets.set(3, new VectorF(0, 0));

        nav.stationaryTurns.set(0, new Angle(-45.0, AngleUnit.DEGREES));
        nav.stationaryTurns.set(1, new Angle(125.0, AngleUnit.DEGREES));

        nav.actionFirst.set(1, true);

        nav.drivingTurns.set(1, new Angle(0.0, AngleUnit.DEGREES));

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
        drive.turnError = new Angle(2);
        drive.setDriveError(2, INCH);

        //navigation initialization
        nav.update(new VectorF(0, 0), new Angle(0, AngleUnit.DEGREES));
        nav.initialize(drive);

        waitForStart();

        Angle prevHeading = new Angle(imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);
        while(opModeIsActive()) {
            Angle heading = new Angle(imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle, AngleUnit.DEGREES);
            drive.updateEncoderMotion(prevHeading, heading);
            nav.updateHeading(heading);
            Position pose = drive.getEncoderMotion().getPosition();
            nav.updatePosition(new VectorF((float)pose.x, (float)pose.y));
            nav.run();
            drive.updateEncoders();
            prevHeading = heading.copy();
        }
    }

}
