package org.firstinspires.ftc.teamcode.teamcalamari.OpModeTests.CompareAuto.Kalman;

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
import org.teamcalamari.DataLogger;
import org.teamcalamari.RobotAction;
import org.teamcalamari.AutonomousNavigationSim.AllDirectionsNavigation;
import org.teamcalamari.FilterStuff.FiltersAccelIntegration.KalmanFilterDropSensor;
import org.teamcalamari.HardwareSim.DriveSim.KiwiDrive;
import org.teamcalamari.HardwareSim.MotorsSim.Motor;
import org.teamcalamari.HardwareSim.MotorsSim.WheelMotor;
import org.teamcalamari.HardwareSim.SensorsSim.BNO055IMU;
import org.teamcalamari.HardwareSim.SensorsSim.DistanceSensorTC;
import org.teamcalamari.OpModeSim.LinearOpMode;
import org.teamcalamari.OpModeSim.OpModeType;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class CompareAutoKalman3 extends LinearOpMode{

    private String sensorName = "Kalman";
    private final long testNum = System.currentTimeMillis();
    private final int stepNum = 3;
    private DataLogger logger = new DataLogger("Compare "+sensorName+" "+stepNum+" "+testNum);

    //the speed of the robot
    private double speed = 0.5;
    private double driveSpeed = 0.8;

    //no turning in these routines
    private double stillTurnSpeed = 0;
    private double stillTurnTurnSpeed = 0;
    private double moveTurnSpeed = 0;

    //the motors, servos, sensors, servo values, and drivetrain
    public WheelMotor RedMotor;
    public WheelMotor BlueMotor;
    public WheelMotor GreenMotor;
    public KiwiDrive drive;

    private BNO055IMU imu;

    private DistanceSensorTC ultraX;
    private DistanceSensorTC ultraY;

    private double maxX = 60;
    private double maxY = 36;
    private double ultraXToRobotCenter = 8;
    private double ultraYToRobotCenter = 8;

    private KalmanFilterDropSensor kalmanFilter = KalmanFilter.getFilter();
    private List<Acceleration> emptyAccelList = new ArrayList<Acceleration>();
    private List<Velocity> emptyVeloList = new ArrayList<Velocity>();

    //navigation variables
    private int targetNum = 4;
    private AllDirectionsNavigation nav = new AllDirectionsNavigation(targetNum, speed, driveSpeed, stillTurnSpeed, stillTurnTurnSpeed, moveTurnSpeed, this);

    private boolean waiting = false;
    private boolean firstTimeWait = true;
    RobotAction wait = new RobotAction() {
        ElapsedTime time = new ElapsedTime();
        public boolean start() {
            //for safety
            drive.speed = 0;
            drive.drive();

            firstTimeWait = true;

            time.reset();
            return true;
        }
        @Override
        public boolean act() {
            waiting = true;
            return time.seconds() > 15;
        }
    };

    @Override
    public void runOpMode() throws InterruptedException {
        nav.targets.set(0, new VectorF(0, 0));
        nav.targets.set(1, new VectorF(24, -24));
        nav.targets.set(2, new VectorF(24, 0));
        nav.targets.set(3, new VectorF(48, 0));

        nav.finalCode = new Runnable() {
            public void run() {
                logger.closeDataLogger();
            }
        };

        nav.actions.set(1, wait);
        nav.actions.set(2, wait);

        //get the motors
        RedMotor = new WheelMotor(hardwareMap, "RedMotor");
        BlueMotor = new WheelMotor(hardwareMap, "BlueMotor");
        GreenMotor = new WheelMotor(hardwareMap, "GreenMotor");

        //set the motors to brake. only required for the hub
        RedMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BlueMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        GreenMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //reset the encoders before running
        RedMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BlueMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        GreenMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set the motors to run at a set speed
        RedMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BlueMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        GreenMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //initialize the drivetrain
        drive = new KiwiDrive(new Motor[]{RedMotor, BlueMotor, GreenMotor}, new Angle(0), OpModeType.AUTO);
        drive.updateEncoders();
        drive.inchesPerTick = Math.PI*4/1120;
        drive.turnError = new Angle(2);
        drive.driveError = 1;

        //IMU initialization
        BNO055IMU.Parameters parametersIMU = new BNO055IMU.Parameters();
        parametersIMU.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parametersIMU.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parametersIMU.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parametersIMU.loggingEnabled      = true;
        parametersIMU.loggingTag          = "IMU";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parametersIMU);

        //ultrasonic sensor initialization
        ultraX = new DistanceSensorTC(hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "ultraX"));
        ultraY = new DistanceSensorTC(hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "ultraY"));

        //navigation initialization
        nav.update(new VectorF(0, 0), new Angle(0, AngleUnit.DEGREES));
        nav.initialize(drive);

        //data logger initialization
        logger.addField("Heading");
        logger.addField("Position X");
        logger.addField("Position Y");
        logger.addField("Ultra X");
        logger.addField("Ultra Y");
        logger.addField("Enc X");
        logger.addField("Enc Y");
        logger.addField("Wait");
        logger.newLine();

        telemetry.addData("TestNum ", testNum);
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {
            Angle heading = new Angle(imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle, AngleUnit.DEGREES);
            drive.updateEncoderMotion(heading);
            nav.updateHeading(heading);
            
            Position encPose = drive.getEncoderMotion().position;
            
            ultraX.distance = obscureData(-encPose.x+1-Math.random()/2+maxX-ultraXToRobotCenter);
            ultraY.distance = obscureData(encPose.y+1-Math.random()/2+maxY-ultraYToRobotCenter);


            double ultraCorrX = correctData(ultraX.getData(DistanceUnit.INCH));
            double ultraCorrY = correctData(ultraY.getData(DistanceUnit.INCH));
            double ultraPoseX = maxX-ultraCorrX-ultraXToRobotCenter;
            double ultraPoseY = -(maxY-ultraCorrY-ultraYToRobotCenter);

            Position ultraPose = new Position(DistanceUnit.INCH, ultraPoseX, ultraPoseY, 0, 0);
            
            ArrayList<Position> poses = new ArrayList<Position>(2);
            poses.add(encPose);
            poses.add(ultraPose);
            kalmanFilter.update(emptyAccelList, emptyVeloList, poses);
            Position pose = kalmanFilter.getPosition();

            nav.updatePosition(new VectorF((float)pose.x, (float)pose.y));
            waiting = false;
            nav.run();

            if(!waiting) {
                logger.addField(nav.getHeading().getDegree());
                logger.addField(nav.getPosition().get(0));
                logger.addField(nav.getPosition().get(1));
                logger.addField(ultraPoseX);
                logger.addField(ultraPoseY);
                logger.addField(encPose.x);
                logger.addField(encPose.y);
                logger.addField(false);
                logger.newLine();
            }
            else if(firstTimeWait){
                logger.addField(nav.getHeading().getDegree());
                logger.addField(nav.getPosition().get(0));
                logger.addField(nav.getPosition().get(1));
                logger.addField(ultraPoseX);
                logger.addField(ultraPoseY);
                logger.addField(encPose.x);
                logger.addField(encPose.y);
                logger.addField(true);
                logger.newLine();
                firstTimeWait = false;
            }
        }
    }
    
    private double correctData(double data){
        return 0.9828*Math.pow(data, 1.0247);
    }
    private double obscureData(double data) {
    	return Math.pow(data/0.9828, 1/1.0247);
    }
}
