package org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.OpModeTests;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.teamcalamari.Angle;
import org.firstinspires.ftc.teamcode.teamcalamari.AxesSet;
import org.firstinspires.ftc.teamcode.teamcalamari.Location;
import org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.LogSim;
import org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.OpModeSim.OpModeSim;

import com.qualcomm.robotcore.util.ElapsedTime;

public class LogTest extends OpModeSim {
	public LogSim log = new LogSim(this);
	
	private Angle angle = new Angle(45, AngleUnit.DEGREES, AxesSet.ACCELEROMETER_AXES);
	//location, location, location
	public Location location = new Location();
	public ElapsedTime runtime = new ElapsedTime();

	@Override
	public void init() {
		try {
			log.loggers.put("angle", "Angle");
			log.loggers.put("location", "Location");
			log.errorLoggers.put("runtime", "Time");
			log.findFields();
		}catch(RuntimeException e) {
			throw e;
		}
		
		try {
			angle.setRadian(3*Angle.PI/4);
			location.position.x = 5;
			location.position.y = 10;
			runtime.reset();
		} catch(RuntimeException e) {
			try {
				log.logError(e);
			} catch (Exception e1) {
				e1.printStackTrace();
			}
		}
		log.log();
	}
	
	@Override
	public void start() {
		runtime.reset();
	}

	@Override
	public void loop() {
		try {
			if(runtime.seconds() > 10) {
				throw new RuntimeException("Ending OpMode");
			}
			angle.setRadian(3*Angle.PI/4*Math.round(runtime.seconds()));
			location.position.x = 5*runtime.seconds();
			location.position.y = 10*runtime.seconds();
		} catch(RuntimeException e) {
			log.logError(e);
			this.requestOpModeStop();
			return;
		}
		log.log();
	}
	
	@Override
	public void stop() {
		System.out.println("Stopping");
		log.close();
	}

}
