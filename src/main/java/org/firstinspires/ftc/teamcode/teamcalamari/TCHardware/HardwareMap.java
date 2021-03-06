package org.firstinspires.ftc.teamcode.teamcalamari.TCHardware;

import java.util.HashMap;
import java.util.Iterator;

import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Motors.Motor;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Motors.MotorSimple;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Sensors.BNO055IMU;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Sensors.BNO055TestMulti;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Servos.CRServo;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Servos.Servo;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.AccelerationSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogOutput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.CompassSensor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.IrSeekerSensor;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.LegacyModule;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.PWMOutput;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.TouchSensorMultiplexer;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class HardwareMap implements Iterable<String>{
	
  public DeviceMapping<DcMotorController>     dcMotorController     = new DeviceMapping<DcMotorController>(DcMotorController.class);
  public DeviceMapping<Motor>                 dcMotor               = new DeviceMapping<Motor>(Motor.class);

  public DeviceMapping<ServoController>       servoController       = new DeviceMapping<ServoController>(ServoController.class);
  public DeviceMapping<Servo>                 servo                 = new DeviceMapping<Servo>(Servo.class);
  public DeviceMapping<CRServo>               crservo               = new DeviceMapping<CRServo>(CRServo.class);

  public DeviceMapping<LegacyModule>          legacyModule          = new DeviceMapping<LegacyModule>(LegacyModule.class);
  public DeviceMapping<TouchSensorMultiplexer>touchSensorMultiplexer = new DeviceMapping<TouchSensorMultiplexer>(TouchSensorMultiplexer.class);

  public DeviceMapping<DeviceInterfaceModule> deviceInterfaceModule = new DeviceMapping<DeviceInterfaceModule>(DeviceInterfaceModule.class);
  public DeviceMapping<AnalogInput>           analogInput           = new DeviceMapping<AnalogInput>(AnalogInput.class);
  public DeviceMapping<DigitalChannel>        digitalChannel        = new DeviceMapping<DigitalChannel>(DigitalChannel.class);
  public DeviceMapping<OpticalDistanceSensor> opticalDistanceSensor = new DeviceMapping<OpticalDistanceSensor>(OpticalDistanceSensor.class);
  public DeviceMapping<TouchSensor>           touchSensor           = new DeviceMapping<TouchSensor>(TouchSensor.class);
  public DeviceMapping<PWMOutput>             pwmOutput             = new DeviceMapping<PWMOutput>(PWMOutput.class);
  public DeviceMapping<I2cDevice>             i2cDevice             = new DeviceMapping<I2cDevice>(I2cDevice.class);
  public DeviceMapping<I2cDeviceSynch>        i2cDeviceSynch        = new DeviceMapping<I2cDeviceSynch>(I2cDeviceSynch.class);
  public DeviceMapping<AnalogOutput>          analogOutput          = new DeviceMapping<AnalogOutput>(AnalogOutput.class);
  public DeviceMapping<ColorSensor>           colorSensor           = new DeviceMapping<ColorSensor>(ColorSensor.class);
  public DeviceMapping<LED>                   led                   = new DeviceMapping<LED>(LED.class);

  public DeviceMapping<AccelerationSensor>    accelerationSensor    = new DeviceMapping<AccelerationSensor>(AccelerationSensor.class);
  public DeviceMapping<CompassSensor>         compassSensor         = new DeviceMapping<CompassSensor>(CompassSensor.class);
  public DeviceMapping<GyroSensor>            gyroSensor            = new DeviceMapping<GyroSensor>(GyroSensor.class);
  public DeviceMapping<IrSeekerSensor>        irSeekerSensor        = new DeviceMapping<IrSeekerSensor>(IrSeekerSensor.class);
  public DeviceMapping<LightSensor>           lightSensor           = new DeviceMapping<LightSensor>(LightSensor.class);
  public DeviceMapping<UltrasonicSensor>      ultrasonicSensor      = new DeviceMapping<UltrasonicSensor>(UltrasonicSensor.class);
  public DeviceMapping<VoltageSensor>         voltageSensor         = new DeviceMapping<VoltageSensor>(VoltageSensor.class);

  public HashMap<HardwareDevice, String> deviceMap = new HashMap<HardwareDevice, String>();
  
  public HardwareMap() {}

  @SuppressWarnings("unchecked")
  public <T> T get(Class<? extends T> classOrInterface, String deviceName) {
	  //class/interface simulators will be added as needed

	  if(classOrInterface.equals(BNO055IMU.class) || classOrInterface.equals(com.qualcomm.hardware.bosch.BNO055IMU.class)) {
		  return (T) new BNO055IMU(); 
	  }
	  else if(classOrInterface.equals(BNO055TestMulti.class)){
	      return (T) new BNO055TestMulti();
      }
	  else if(classOrInterface.equals(MotorSimple.class)) {
		  MotorSimple motorSimple = new MotorSimple();
		  deviceMap.put(motorSimple, deviceName);
		  return (T) motorSimple;
	  }
	  else if(classOrInterface.equals(ModernRoboticsI2cRangeSensor.class)) {
		  //attempted to return a range sensor, but failed
		  //return (T) new ModernRoboticsI2cRangeSensor(new I2cDeviceSynchImpl(new I2cDeviceImpl(new I2cController(), 0), false));
		  return null;
	  }
	  else {
		  return null;
	  }
  }
  
  public<T extends HardwareDevice> void replace(String deviceName, T hardwareDevice, T originalDevice) {
	  deviceMap.remove(originalDevice);
	  deviceMap.put(hardwareDevice, deviceName);
  }

  public class DeviceMapping<DEVICE_TYPE extends HardwareDevice>{
  	private Class<DEVICE_TYPE> deviceTypeClass;

   	public DeviceMapping(Class<DEVICE_TYPE> deviceTypeClass) {
   		this.deviceTypeClass = deviceTypeClass;
   	}
   	@SuppressWarnings("unchecked")
	public DEVICE_TYPE get(String deviceName) {
   		if(deviceTypeClass.equals(Motor.class)) {
   			Motor motor = new Motor();
   			deviceMap.put(motor, deviceName);
   			return (DEVICE_TYPE) motor;
   		}
   		else if(deviceTypeClass.equals(Servo.class)) {
   			Servo servo = new Servo(null, 0);
   			deviceMap.put(servo, deviceName);
   			return (DEVICE_TYPE) servo;
   		}
   		else if(deviceTypeClass.equals(CRServo.class)) {
   			CRServo servo = new CRServo();
   			deviceMap.put(servo, deviceName);
   			return (DEVICE_TYPE) servo;
   		}
   		else {
   			return null;
   		}
   	}
   	
   	public void replace(String deviceName, DEVICE_TYPE hardwareDevice, DEVICE_TYPE originalDevice) {
   		deviceMap.remove(originalDevice);
   		deviceMap.put(hardwareDevice, deviceName);
   	}
  }

	@Override
	public Iterator<String> iterator() {
		return deviceMap.values().iterator();
	}
	
	
	public int size() {
		return deviceMap.size();
	}
}
