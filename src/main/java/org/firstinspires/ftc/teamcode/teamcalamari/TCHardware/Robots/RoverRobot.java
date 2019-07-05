package org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Robots;

import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.teamcalamari.DataLogger;
import org.firstinspires.ftc.teamcode.teamcalamari.OpModeType;
import org.firstinspires.ftc.teamcode.teamcalamari.Navigation.Angle;
import org.firstinspires.ftc.teamcode.teamcalamari.Navigation.AutonomousNavigation.AutonomousNavigation;
import org.firstinspires.ftc.teamcode.teamcalamari.RobotAction.RobotAction;
import org.firstinspires.ftc.teamcode.teamcalamari.RobotAction.RobotActionLinear;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.HardwareMap;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.DriveClasses.Drive.turnDirection;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.DriveClasses.KiwiDrive;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Gamepad.Gamepad;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Gamepad.GamepadButton;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Gamepad.GamepadJoystick;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Gamepad.GamepadTrigger;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Motors.DoubleJointedArmSystem;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Motors.LiftMotor;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Motors.Motor;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Motors.SwifferMotor;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Motors.WheelMotor;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Sensors.BNO055IMU;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Sensors.DistanceSensorTC;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Servos.CRServo;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.annotations.MotorType;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RoverRobot extends BaseRobot{


    public WheelMotor RedMotor;
    public WheelMotor YellowMotor;
    public WheelMotor GreenMotor;
    public KiwiDrive drive;
    
    public SwifferMotor collector;
    public LiftMotor lift;
    public DoubleJointedArmSystem arm;
    
    public int[] autoMotorTicks = new int[]{0,0,0};

    public DistanceSensorTC groundRange;
    public double inchesToGround = 3;
    
    public DistanceSensorTC ultraX;
    public DistanceSensorTC ultraY;

    public BNO055IMU imu;

    public double speed = 0.75;
    
    public HardwareMap hwMap;

    /**heading that the robot started the program at relative to the zero defined by ResetStartingHeading*/
    public Angle startingHeading;
    /**current heading as reported by the imu, relative to the starting heading*/
    public Angle relativeHeading;
    /**current heading relative to the zero defined by ResetStartingHeading*/
    public Angle absoluteHeading;
    
    public Angle phoneAngle = new Angle(0);
    public Angle armAngle = new Angle(90);
    public Angle liftAngle = new Angle(180);
    
    public boolean stopped = false;
    
    //collector controls
    public GamepadButton collectorStartInButton = GamepadButton.A;
    public GamepadButton collectorStartOutButton = GamepadButton.X;
    public GamepadButton collectorOffButton = GamepadButton.Y;
    public GamepadButton collectorOneInButton = GamepadButton.DPAD_DOWN;
    public GamepadButton collectorTwoInButton = GamepadButton.DPAD_UP;
    public GamepadButton collectorOneOutButton = GamepadButton.DPAD_LEFT;
    public GamepadButton collectorTwoOutButton = GamepadButton.DPAD_RIGHT;
    public GamepadButton collectorInButton = GamepadButton.LEFT_BUMPER;
    public GamepadButton collectorOutButton = GamepadButton.RIGHT_BUMPER;
    public GamepadJoystick collectorInAndOut = GamepadJoystick.LEFT_STICK_Y;
    public GamepadTrigger collectorIn = GamepadTrigger.LEFT_TRIGGER;
    public GamepadTrigger collectorOut = GamepadTrigger.RIGHT_TRIGGER;

    private double collectorOneInTime = 1;
    private double collectorTwoInTime = 2;
    private double collectorOneOutTime = 1;
    private double collectorTwoOutTime = 2;

    //lift controls
    public GamepadButton liftExtendFullyButton = GamepadButton.NONE;
    public GamepadButton liftRetractFullyButton = GamepadButton.NONE;
    public GamepadButton liftExtendButton = GamepadButton.NONE;
    public GamepadButton liftRetractButton = GamepadButton.NONE;
    public GamepadJoystick liftJoystick = GamepadJoystick.NONE;
    public GamepadTrigger liftUp = GamepadTrigger.NONE;
    public GamepadTrigger liftDown = GamepadTrigger.NONE;

    private int retractTicks = 0;
    private int extendTicks = 1440;

    //drivetrain controls
    public GamepadButton switchDriverOriented = GamepadButton.A;
    public GamepadButton armAsFront = GamepadButton.LEFT_BUMPER;
    public GamepadButton liftAsFront = GamepadButton.RIGHT_BUMPER;
    public boolean prevDriverOrientedState = false;
    public boolean driverOriented = true;

    //arm controls
    public GamepadButton armCraterExtension = GamepadButton.NONE;
    public GamepadButton armLanderExtension = GamepadButton.NONE;
    public GamepadButton armFoldUp = GamepadButton.NONE;
    public GamepadButton armDrivePosition = GamepadButton.NONE;
    public GamepadJoystick armShoulderJoystickC = GamepadJoystick.NONE;
    public GamepadJoystick armElbowJoystickC = GamepadJoystick.NONE;
    public GamepadJoystick armOverrideShoulder = GamepadJoystick.NONE;
    public GamepadJoystick armOverrideElbow = GamepadJoystick.NONE;
    public GamepadButton armSetOverrideShoulder = GamepadButton.NONE;
    public GamepadButton armSetOverrideElbow = GamepadButton.NONE;
    public GamepadButton armSetOverride = GamepadButton.NONE;

    //of form [shoulder, elbow]
    private int[] craterExtensionTicks = new int[]{100, 0};
    private int[] landerExtensionTicks = new int[]{50, 25};
    private int[] foldUpTicks = new int[]{0, 0};
    private int[] drivePositionTicks = new int[]{25, 72};

    public double armShoulderPowerCorrection = 0.3;
    public double armElbowPowerCorrection = 0;

    //lower should be first if being used
    public int[] armShoulderCorrectionRange = new int[]{90, 120};
    public int[] armElbowCorrectionRange = new int[]{0, -1};
    

    public RoverRobot(Program program){
        super(program);
    }

    @Override
    public void init(HardwareMap hwMap){
        this.hwMap = hwMap;


        collector = new SwifferMotor("collector", hwMap, CRServo.class, DcMotorSimple.Direction.FORWARD);

        lift = new LiftMotor(hwMap, "lift", DcMotorSimple.Direction.FORWARD);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setLimits(retractTicks, extendTicks);
        lift.startingTicks = autoMotorTicks[0];

        arm = new DoubleJointedArmSystem("shoulder", "elbow", hwMap);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorConfigurationType motorType = new MotorConfigurationType();
        motorType.processAnnotation(RevRoboticsCoreHexMotor.class.getDeclaredAnnotation(MotorType.class));
        arm.shoulderMotor.setMotorType(motorType);
        arm.elbowMotor.setMotorType(motorType);
        arm.shoulderMotor.startingTicks = autoMotorTicks[1];
        arm.elbowMotor.startingTicks = autoMotorTicks[2];

        
        //groundRange = new DistanceSensorTC(hwMap.get(ModernRoboticsI2cRangeSensor.class, "ground"));
        ultraX = new DistanceSensorTC(hwMap.get(ModernRoboticsI2cRangeSensor.class, "ultraX"));
        ultraY = new DistanceSensorTC(hwMap.get(ModernRoboticsI2cRangeSensor.class, "ultraY"));

        //get the motors
        RedMotor = new WheelMotor(hwMap, "RedMotor");
        YellowMotor = new WheelMotor(hwMap, "YellowMotor");
        GreenMotor = new WheelMotor(hwMap, "GreenMotor");

        //set the motors to brake. only required for the hub
        RedMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        YellowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        GreenMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //reset the encoders before running
        RedMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        YellowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        GreenMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set the motors to run at a set speed
        RedMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        YellowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        GreenMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //initialize the drivetrain
        drive = new KiwiDrive(new Motor[]{RedMotor, GreenMotor, YellowMotor}, new Angle(0), program.getOpModeType());
        drive.updateEncoders();
        drive.inchesPerTick = Math.PI*4/1120;
        drive.turnError = new Angle(2);
        drive.driveError = 1;
        

        //imu initialization
        BNO055IMU.Parameters parametersIMU = new BNO055IMU.Parameters();
        parametersIMU.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parametersIMU.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parametersIMU.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parametersIMU.loggingEnabled      = true;
        parametersIMU.loggingTag          = "IMU";

        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parametersIMU);
        
        super.init(hwMap);
    }

    public boolean wheelMotorsEnabled() {
        return enabledDevices.get("RedMotor") && enabledDevices.get("GreenMotor") && enabledDevices.get("YellowMotor");
    }
    public boolean armEnabled() {
    	return enabledDevices.get("shoulder") && enabledDevices.get("elbow");
    }

    @Override
    public void update(Gamepad gamepad1, Gamepad gamepad2) {
    	
        //store the joystick values in variables for easier access
        double glx = gamepad1.left_stick_x;
        double gly = gamepad1.left_stick_y;

        //only drive the robot if all the wheel motors are enabled
        if(wheelMotorsEnabled()) {
        	
        	if(!armAsFront.isNone()) {
        		if(armAsFront.getValue(gamepad1)) {
        			drive.headAngle = Angle.add(armAngle, new Angle(0));
        		}
        	}
        	if(!liftAsFront.isNone()) {
        		if(liftAsFront.getValue(gamepad1)) {
        			drive.headAngle = Angle.add(liftAngle, new Angle(0));
        		}
        	}
        	
        	if(!switchDriverOriented.isNone()){
                if(switchDriverOriented.getValue(gamepad1) && !prevDriverOrientedState){
                    driverOriented = !driverOriented;
                }
                prevDriverOrientedState = switchDriverOriented.getValue(gamepad1);
            }
        	
        	Angle driveHeading;
            Angle joystickAngle = new Angle(Math.atan2(gly, glx), AngleUnit.RADIANS);

            if ((program.equals(RoverTestPrograms.DRIVE_ABS) || program.equals(RoverTestPrograms.DRIVE_ABS_CORRECTIVE)
                    || program.equals(RoverCompPrograms.TELE_OP)) && driverOriented) {
                //get the relative heading from the imu
                relativeHeading = new Angle(imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);
                //get the absolute heading by adding the starting heading and the relative heading
                absoluteHeading = Angle.add(startingHeading, relativeHeading);

                driveHeading = Angle.add(absoluteHeading, joystickAngle);
            }
            else {
                //a little trick to prevent possible side effects from directly setting the drive heading to the joystick angle
                driveHeading = Angle.add(joystickAngle, new Angle(0));
            }

            drive.driveSpeed = Math.sqrt(Math.pow(glx, 2) + Math.pow(gly, 2));
            drive.driveAtAngle(driveHeading);
            //if we are moving turn at half the speed
            if (glx == 0 && gly == 0) {
                drive.speed = Math.abs(gamepad1.right_trigger - gamepad1.left_trigger) * speed;
                drive.turnSpeed = gamepad1.right_trigger;
                drive.turn(turnDirection.CW);
                drive.turnSpeed = gamepad1.left_trigger;
                drive.turn(turnDirection.CCW);
                drive.drive();
            } else {
                drive.speed = Math.sqrt(Math.pow(glx, 2) + Math.pow(gly, 2)) * speed;
                drive.turnSpeed = gamepad1.right_trigger / 2;
                drive.turn(turnDirection.CW);
                drive.turnSpeed = gamepad1.left_trigger / 2;
                drive.turn(turnDirection.CCW);
                drive.drive();
            }
        }
        
        if(enabledDevices.get("collector")){
	        collector.setGamepad(gamepad2);
	
	        collector.turnOnIn(collectorStartInButton);
	        collector.turnOnOut(collectorStartOutButton);
	        collector.turnOff(collectorOffButton);
	        
	        if(collector.getCurrentPower() == 0) {
	        	collector.runIn(collectorInButton);
	        	collector.runOut(collectorOutButton);
	        }
	        
	        if(collector.getCurrentPower() == 0) {
		        collector.runInTime(collectorOneInButton, collectorOneInTime);
		        collector.runInTime(collectorTwoInButton, collectorTwoInTime);
		        collector.runOutTime(collectorOneOutButton, collectorOneOutTime);
		        collector.runOutTime(collectorTwoOutButton, collectorTwoOutTime);
	        }
	        
	        if(collector.getCurrentPower() == 0) {
	        	collector.runIn(collectorIn);
	        	collector.runOut(collectorOut);
	        }
	        
	        if(collector.getCurrentPower() == 0) {
	        	collector.run(collectorInAndOut);
	        }
	
	        
	        collector.run();
	    }
	    if(enabledDevices.get("lift")){
	        lift.setGamepad(gamepad2);
	
	        lift.retractFully(liftRetractFullyButton);
	        lift.extendFully(liftExtendFullyButton);
	        lift.manualOverrideRetract(liftRetractButton);
	        lift.manualOverrideExtend(liftExtendButton);
	        lift.manualOverrideRetract(liftDown);
	        lift.manualOverrideExtend(liftUp);
	        lift.manualOverride(liftJoystick);
	        
	        lift.run();
	    }
	    if(armEnabled()){
            arm.setGamepad(gamepad2);

            arm.setManualOverride(armSetOverride);
            arm.setManualOverrideElbow(armSetOverrideElbow);
            arm.setManualOverrideShoulder(armSetOverrideShoulder);
            arm.manualOverride(armOverrideShoulder, armOverrideElbow);
            

            if(arm.shoulderMotor.getCurrentPower() == 0) {
                arm.shoulderMotor.correctedSetPower(armShoulderJoystickC, armShoulderPowerCorrection, armShoulderCorrectionRange);
            }
            if(arm.elbowMotor.getCurrentPower() == 0) {
                arm.elbowMotor.correctedSetPower(armElbowJoystickC, armElbowPowerCorrection, armElbowCorrectionRange);
            }
            
            boolean manualControlShoulder = arm.shoulderMotor.getCurrentPower() != 0 || arm.manualOverrideShoulder;
            boolean manualControlElbow = arm.elbowMotor.getCurrentPower() != 0 || arm.manualOverrideElbow;
            
            arm.runToPosition(armCraterExtension, craterExtensionTicks, new double[]{0.5, 0.5}, 
            		manualControlShoulder, manualControlElbow);
            arm.runToPosition(armDrivePosition, drivePositionTicks, new double[]{0.5, 0.5}, 
            		manualControlShoulder, manualControlElbow);
            arm.runToPosition(armFoldUp, foldUpTicks, new double[]{0.5, 0.5}, new double[] {5, 0},
            		manualControlShoulder, manualControlElbow);
            arm.runToPosition(armLanderExtension, landerExtensionTicks, new double[]{0.5, 0.5}, 
            		manualControlShoulder, manualControlElbow);
            
            arm.run();
        }
    }
    
    public void turnOnFlash(){System.out.println("Flash!");}
    public void turnOffFlash() {System.out.println("Unflash!");}
    
    public AutonomousNavigation nav;
    public int goldLocation;
    public String goldLocationString = "";
    public ArrayList<Integer> masses = new ArrayList<Integer>();
    public RobotAction detectSamples = new RobotAction(){
        final int frameNum = 1;
        int frameCount = 0;
        //SampleResult[] results = new SampleResult[frameNum];
        ElapsedTime timer = new ElapsedTime();
        
        @Override
        public boolean start(){
            turnOnFlash();
            timer.reset();
            return true;
        }

        @Override
        public boolean act(){
        	if(timer.seconds() > 1) {
	            /*SampleResult currResult = FtcRobotControllerActivity.scanSamples();
	            results[frameCount] = currResult;*/
	            frameCount++;
	
	            if(frameCount >= frameNum){
	            	//SampleResult sampleOrientation = SampleResult.combineResults(results);
                    goldLocation = 2;//sampleOrientation.getSampleNum(SampleResult.SampleType.GOLD);
                    //masses.add(sampleOrientation.masses[0][1][0]);
                    //masses.add(sampleOrientation.masses[1][1][0]);
                    masses.add((int) Math.round(Math.random()*10));
                    masses.add((int) Math.round(Math.random()*100));

                    goldLocationString = goldLocationToString(goldLocation);
                    
                    nav.targets.set(4, new VectorF(39, goldLocationToTarget(goldLocation)));
                    nav.targets.set(5, new VectorF(55, goldLocationToTarget(goldLocation)));
                    if(nav.targets.size() == 7){
                        nav.targets.set(5, new VectorF(55, goldLocationToTarget(goldLocation)));
                        nav.targets.set(6, new VectorF(76.5f, goldLocationToTarget(goldLocation)/2));
                        nav.stationaryTurns.set(5, goldLocationToAngle(goldLocation));
                        if(goldLocation == 0){
                            nav.targets.set(6, new VectorF(80, 12));
                            nav.stationaryTurns.set(6, goldLocationToAngle(0));
                            nav.stationaryTurns.set(5, null);
                        }
                        if(goldLocation == 1){
                            nav.targets.set(6, new VectorF(85, 0));
                            nav.stationaryTurns.set(6, goldLocationToAngle(1));
                            nav.stationaryTurns.set(5, null);
                        }
                        if(goldLocation == 2){
                            nav.targets.set(6, new VectorF(85, -4));
                        }
                    }

	                turnOffFlash();
	                return true;
	            }
        	}
            return false;
        }

        private Angle goldLocationToAngle(int i){
            switch(i){
                case 0:
                    return new Angle(-135);
                case 1:
                    return new Angle(-90);
                case 2:
                    return new Angle(-45);
                default:
                    return new Angle(0);
            }
        }
        private String goldLocationToString(int i){
            switch(i){
                case 0:
                    return "LEFT";
                case 1:
                    return "MIDDLE";
                case 2:
                    return "RIGHT";
                default:
                    return "ERROR!";
            }
        }
        private float goldLocationToTarget(int i){
            return 18f*(1-i);
        }
    };
    
    //test of lowering the robot to the ground
    public RobotAction lowerRobotTest = new RobotAction(){

        ElapsedTime timer = new ElapsedTime();

        @Override
        public boolean start(){
            timer.reset();
            return true;
        }

        @Override
        public boolean act(){
            lift.retractFully(1);
            if(groundRange.getData(DistanceUnit.INCH) <= inchesToGround){
                lift.stop();
                return true;
            }
            lift.run();
            return false;
        }
    };
    //lowers the robot to the ground
    public RobotAction lowerRobot = new RobotAction(){
        @Override
        public boolean act(){
            return true;
        }
    };
    //drops the team marker in the depot
    public RobotAction dropMarker = new RobotAction(){
        @Override
        public boolean act(){
            return true;
        }
    };
    //extends the arm so that the collector is over the depot
    public RobotAction extendArm = new RobotAction(){
    	@Override
        public boolean start(){
            arm.runToPosition(craterExtensionTicks, new double[]{0.5, 0.5}, new double[]{0.5, 0}, true);
            return true;
        }

        @Override
        public boolean act(){
            arm.runToPosition(craterExtensionTicks, new double[]{0.5, 0.5}, new double[]{0.5, 0}, false);
            arm.run();
            if(arm.inPosition()) {
            	arm.stop();
            	return true;
            }
            return false;
        }
    };
    //collapse the arm so it doesn't get in the way of another robot trying to place their team marker
    public RobotAction collapseArm = new RobotAction(){

        @Override
        public boolean start(){
            arm.runToPosition(foldUpTicks, new double[]{0.5, 0.5}, new double[]{0, 0.5}, true);
            return true;
        }

        @Override
        public boolean act(){
            arm.runToPosition(foldUpTicks, new double[]{0.5, 0.5}, new double[]{0, 0.5}, false);
            if(arm.inPosition()) {
            	arm.stop();
            	return true;
            }
            return false;
        }
    };
    //places the team marker in the depot
    public RobotAction placeTeamMarker = new RobotActionLinear(extendArm, dropMarker, collapseArm);
    
    
    @Override
    public void stop(){
    	if(!stopped) {
	        drive.stop();
	        lift.stop();
	        collector.stop();
	        arm.stop();
	
	        
	        DataLogger loggerH = new DataLogger("Heading Comp");
	        Angle heading = new Angle(imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle, AngleUnit.DEGREES);
	
	        loggerH.addField(heading.getDegree());
	        loggerH.newLine();
	        loggerH.closeDataLogger();
	        
	        
	        DataLogger loggerM = new DataLogger("Encoder Comp");
	        
	        /*int liftTicks = lift.getCurrentPosition();
	        int shoulderTicks = arm.shoulderMotor.getCurrentPosition();
	        int elbowTicks = arm.elbowMotor.getCurrentPosition();*/
	        int liftTicks = (int) Math.round(Math.random()*1000);
	        int shoulderTicks = (int) Math.round(Math.random()*300);
	        int elbowTicks = (int) Math.round(-Math.random()*300);
	        System.out.println(liftTicks+" "+shoulderTicks+" "+elbowTicks);
	        loggerM.addField(liftTicks);
	        loggerM.newLine();
	
	        loggerM.addField(shoulderTicks);
	        loggerM.newLine();
	        
	        loggerM.addField(elbowTicks);
	        loggerM.newLine();
	
	        loggerM.closeDataLogger();
	        
	        stopped = true;
    	}
    }

    public static int[] processEncoderFile() throws IOException{
        int[] ticks = new int[3];
        FileReader fr = new FileReader("F:/DataLogger/Encoder Comp.txt");
        String str = "";
        int i;

        while((i=fr.read()) != -1) {
            str+=(char)i;
        }
        fr.close();
        String[] lines = str.split("\n");

        ticks[0] = Integer.parseInt(lines[0]);
        ticks[1] = Integer.parseInt(lines[1]);
        ticks[2] = Integer.parseInt(lines[2]);
        
        return ticks;
    }
    public enum RoverCompPrograms implements Program{
        TELE_OP{@Override public OpModeType getOpModeType() { return OpModeType.TELE; }},
        DEPOT{@Override public OpModeType getOpModeType(){return OpModeType.AUTO;}},
        CRATER{@Override public OpModeType getOpModeType(){return OpModeType.AUTO;}},
    }

    public enum RoverTestPrograms implements Program{
        DRIVE{@Override public OpModeType getOpModeType() { return OpModeType.TELE; }},
        DRIVE_ABS{@Override public OpModeType getOpModeType() { return OpModeType.TELE; }},
        DRIVE_ABS_CORRECTIVE{@Override public OpModeType getOpModeType(){ return OpModeType.TELE; }},
        LOWER{@Override public OpModeType getOpModeType(){ return OpModeType.TELE; }}
    }
}
