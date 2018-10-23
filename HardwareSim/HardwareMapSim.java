package org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.HardwareSim;

import java.util.HashMap;
import java.util.Iterator;

import org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.HardwareSim.MotorsSim.MotorSim;
import org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.HardwareSim.SensorsSim.BNO055IMUSim;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.AccelerationSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogOutput;
import com.qualcomm.robotcore.hardware.CRServo;
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

public class HardwareMapSim implements Iterable<String>{
	
  public DeviceMapping<DcMotorController>     dcMotorController     = new DeviceMapping<DcMotorController>(DcMotorController.class);
  public DeviceMapping<MotorSim>               dcMotor               = new DeviceMapping<MotorSim>(MotorSim.class);

  public DeviceMapping<ServoController>       servoController       = new DeviceMapping<ServoController>(ServoController.class);
  public DeviceMapping<ServoSim>                 servo                 = new DeviceMapping<ServoSim>(ServoSim.class);
  public DeviceMapping<CRServo>               crservo               = new DeviceMapping<CRServo>(CRServo.class);

  public DeviceMapping<LegacyModule>          legacyModule          = new DeviceMapping<LegacyModule>(LegacyModule.class);
  public DeviceMapping<TouchSensorMultiplexer> touchSensorMultiplexer = new DeviceMapping<TouchSensorMultiplexer>(TouchSensorMultiplexer.class);

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

  public HashMap<HardwareDeviceSim, String> deviceMap = new HashMap<HardwareDeviceSim, String>();
  
  public HardwareMapSim() {}

  @SuppressWarnings("unchecked")
  public <T> T get(Class<? extends T> classOrInterface, String deviceName) {
	  //class/interface simulators will be added as needed
	  //for now we have none
	  if(classOrInterface.equals(BNO055IMU.class) || classOrInterface.equals(BNO055IMUSim.class)) {
		  return (T) new BNO055IMUSim(); 
	  }
	  else {
		  return null;
	  }
  }

  public class DeviceMapping<DEVICE_TYPE extends HardwareDevice>{
  	private Class<DEVICE_TYPE> deviceTypeClass;

   	public DeviceMapping(Class<DEVICE_TYPE> deviceTypeClass) {
   		this.deviceTypeClass = deviceTypeClass;
   	}
   	@SuppressWarnings("unchecked")
	public DEVICE_TYPE get(String deviceName) {
   		if(deviceTypeClass.equals(MotorSim.class)) {
   			MotorSim motor = new MotorSim();
   			deviceMap.put(motor, deviceName);
   			return (DEVICE_TYPE) motor;
   		}
   		else if(deviceTypeClass.equals(ServoSim.class)) {
   			ServoSim servoSim = new ServoSim(null, 0);
   			deviceMap.put(servoSim, deviceName);
   			return (DEVICE_TYPE) servoSim;
   		}
   		else {
   			return null;
   		}
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
