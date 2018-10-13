package org.firstinspires.ftc.teamcode.teamcalamari.Simulation.OpModeSim;

import java.util.concurrent.TimeUnit;

import org.firstinspires.ftc.teamcode.teamcalamari.OpModeType;
import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.TelemetrySim;
import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.HardwareSim.GamepadSim;
import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.HardwareSim.HardwareMapSim;

public abstract class OpModeSim {
	
	public GamepadSim gamepad1 = new GamepadSim();
	public GamepadSim gamepad2 = new GamepadSim();
	
	  private boolean isStopRequested = false;

	  public TelemetrySim telemetry = new TelemetrySim(OpModeType.TELE);

	  /**
	   * Hardware Mappings
	   */
	  public HardwareMapSim hardwareMap = new HardwareMapSim();
	  
	  // internal time tracking
	  private long startTime = 0; // in nanoseconds

	  /**
	   * OpMode constructor
	   * <p>
	   * The op mode name should be unique. It will be the name displayed on the driver station. If
	   * multiple op modes have the same name, only one will be available.
	   */
	  public OpModeSim() {
	    startTime = System.nanoTime();
	  }

	  /**
	   * User defined init method
	   * <p>
	   * This method will be called once when the INIT button is pressed.
	   */
	  abstract public void init();

	  /**
	   * User defined init_loop method
	   * <p>
	   * This method will be called repeatedly when the INIT button is pressed.
	   * This method is optional. By default this method takes no action.
	   */
	  public void init_loop() {};

	  /**
	   * User defined start method.
	   * <p>
	   * This method will be called once when the PLAY button is first pressed.
	   * This method is optional. By default this method takes not action.
	   * Example usage: Starting another thread.
	   *
	   */
	  public void start() {
		  //tG.start();
	  };

	  /**
	   * User defined loop method
	   * <p>
	   * This method will be called repeatedly in a loop while this op mode is running
	   */
	  abstract public void loop();

	  /**
	   * User defined stop method
	   * <p>
	   * This method will be called when this op mode is first disabled
	   *
	   * The stop method is optional. By default this method takes no action.
	   */
	  public void stop() {};

	  /**
	   * Requests that this OpMode be shut down if it the currently active opMode, much as if the stop
	   * button had been pressed on the driver station; if this is not the currently active OpMode,
	   * then this function has no effect. Note as part of this processing, the OpMode's {@link #stop()}
	   * method will be called, as that is part of the usual shutdown logic. Note that {@link #requestOpModeStop()}
	   * may be called from <em>any</em> thread.
	   *
	   * @see #stop()
	   */
	  public final void requestOpModeStop() {
		  if(!isStopRequested) {
			  System.out.println("Stop OpMode");
			  stop();
		  	  isStopRequested = true;
		  }
	  }
	  public final boolean isOpModeStopRequested() {
		  return isStopRequested;
	  }

	  /**
	   * Get the number of seconds this op mode has been running
	   * <p>
	   * This method has sub millisecond accuracy.
	   * @return number of seconds this op mode has been running
	   */
	  public double getRuntime() {
	    final double NANOSECONDS_PER_SECOND = TimeUnit.SECONDS.toNanos(1);
	    return (System.nanoTime() - startTime) / NANOSECONDS_PER_SECOND;
	  }

	  /**
	   * Reset the start time to zero.
	   */
	  public void resetStartTime() {
	    startTime = System.nanoTime();
	  }
}
