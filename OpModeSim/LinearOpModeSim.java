package org.firstinspires.ftc.teamcode.teamcalamari.Simulation.OpModeSim;

import java.util.Scanner;
import java.util.concurrent.CancellationException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.TimeUnit;

import org.firstinspires.ftc.teamcode.teamcalamari.OpModeType;

import com.qualcomm.robotcore.util.ThreadPool;

public abstract class LinearOpModeSim extends OpModeSim {

	  //------------------------------------------------------------------------------------------------
	  // State
	  //------------------------------------------------------------------------------------------------

	  private LinearOpModeHelper helper          = null;
	  private ExecutorService    executorService = null;
	  private volatile boolean   isStarted       = false;
	  private volatile boolean   stopRequested   = false;
	  private Scanner sc = null;
	  private Thread t = new Thread(new Timer30Sec());


	  //------------------------------------------------------------------------------------------------
	  // Construction
	  //------------------------------------------------------------------------------------------------

	  public LinearOpModeSim() {
	  }

	  //------------------------------------------------------------------------------------------------
	  // Operations
	  //------------------------------------------------------------------------------------------------

	  /**
	   * Override this method and place your code here.
	   * <p>
	   * Please do not swallow the InterruptedException, as it is used in cases
	   * where the op mode needs to be terminated early.
	   * @throws InterruptedException
	   */
	  abstract public void runOpMode() throws InterruptedException;

	  public void setScanner(Scanner s) {
		  sc = s;
	  }
	  
	  /**
	   * Pauses the Linear Op Mode until enter is pressed
	   */
	  public synchronized void waitForStart() {
		telemetry.opModeType = OpModeType.AUTO;
		telemetry.setHardwareMap(hardwareMap);
		System.out.println("Press enter to start OpMode");
		sc.nextLine();
		sc.close();
		start();
	  }

	  /**
	   * Puts the current thread to sleep for a bit as it has nothing better to do. This allows other
	   * threads in the system to run.
	   *
	   * <p>One can use this method when you have nothing better to do in your code as you await state
	   * managed by other threads to change. Calling idle() is entirely optional: it just helps make
	   * the system a little more responsive and a little more efficient.</p>
	   *
	   * <p>{@link #idle()} is conceptually related to waitOneFullHardwareCycle(), but makes no
	   * guarantees as to completing any particular number of hardware cycles, if any.</p>
	   *
	   * @see #opModeIsActive()
	   * @see #waitOneFullHardwareCycle()
	   */
	  public final void idle() {
	    // Otherwise, yield back our thread scheduling quantum and give other threads at
	    // our priority level a chance to run
	    Thread.yield();
	    }

	  /**
	   * Sleeps for the given amount of milliseconds, or until the thread is interrupted. This is
	   * simple shorthand for the operating-system-provided {@link Thread#sleep(long) sleep()} method.
	   *
	   * @param milliseconds amount of time to sleep, in milliseconds
	   * @see Thread#sleep(long)
	   */
	  public final void sleep(long milliseconds) {
	    try {
	      Thread.sleep(milliseconds);
	    } catch (InterruptedException e) {
	      Thread.currentThread().interrupt();
	    }
	  }

	  /**
	   * Answer as to whether this opMode is active and the robot should continue onwards. If the
	   * opMode is not active, the OpMode should terminate at its earliest convenience.
	   *
	   * <p>Note that internally this method calls {@link #sleep()} and should only be used in the op mode that is 
	   * being tested.  Internal methods should use {@link #opModeIsActiveInternal()}</p>
	   *
	   * @return whether the OpMode is currently active. If this returns false, you should
	   *         break out of the loop in your {@link #runOpMode()} method and return to its caller.
	   * @see #runOpMode()
	   * @see #isStarted()
	   * @see #isStopRequested()
	   */
	  public final boolean opModeIsActive() {
		telemetry.update();
	    boolean isActive = !this.isStopRequested() && this.isStarted();
	    if (isActive) {
	      sleep(100);
	      telemetry.moveHardware(hardwareMap.deviceMap.keySet());
	      if(telemetry.logHardware) {
	    	  telemetry.logHardware(hardwareMap.deviceMap.entrySet());
	      }
	    }
	    return isActive;
	  }
	  
	  public final boolean opModeIsActiveInternal() {
		  return !this.isStopRequested() && this.isStarted();
	  }

	  /**
	   * Has the opMode been started?
	   *
	   * @return whether this opMode has been started or not
	   * @see #opModeIsActive()
	   * @see #isStopRequested()
	   */
	  public final boolean isStarted() {
	    return this.isStarted || Thread.currentThread().isInterrupted();
	    }

	  /**
	   * Has the the stopping of the opMode been requested?
	   *
	   * @return whether stopping opMode has been requested or not
	   * @see #opModeIsActive()
	   * @see #isStarted()
	   */
	  public final boolean isStopRequested() {
	    return this.stopRequested || Thread.currentThread().isInterrupted();
	    }

	  /**
	   * From the non-linear OpMode; do not override
	   */
	  @Override
	  final public void init() {
	    this.executorService = ThreadPool.newSingleThreadExecutor("LinearOpMode");
	    this.helper          = new LinearOpModeHelper();
	    this.isStarted       = false;
	    this.stopRequested   = false;

	    this.executorService.execute(helper);
	  }

	  /**
	   * From the non-linear OpMode; do not override
	   */
	  @Override
	  final public void init_loop() {
	    handleLoop();
	  }

	  
	  private class Timer30Sec implements Runnable{
		  @Override
		  public void run() {
			  try {
				  Thread.sleep(30000L);
			  }
			  catch(InterruptedException e){
				  if(!stopRequested) {e.printStackTrace();}
				  else {System.out.println("User stopped OpMode");}
			  }
			  if(!stopRequested) {
				  System.out.println("OpMode timed out");
				  LinearOpModeSim.this.requestOpModeStop();
			  }
		  }
	  }
	  /**
	   * From the non-linear OpMode; do not override
	   */
	  @Override
	  final public void start() {
	    stopRequested = false;
	    isStarted = true;
	    synchronized (this) {
	      this.notifyAll();
	    }
	    t.start();
	  }

	  /**
	   * From the non-linear OpMode; do not override
	   */
	  @Override
	  final public void loop() {
	    handleLoop();
	  }

	  /**
	   * From the non-linear OpMode; do not override
	   */
	  @Override
	  final public void stop() {

	    // make isStopRequested() return true (and opModeIsActive() return false)
	    stopRequested = true;

	    t.interrupt();
	    
	    if (executorService != null) {  // paranoia
	    
	      // interrupt the linear opMode and shutdown it's service thread
	      executorService.shutdownNow();

	      /** Wait, forever, for the OpMode to stop. If this takes too long, then
	       * {@link OpModeManagerImpl#callActiveOpModeStop()} will catch that and take action */
	      try {
	        String serviceName = "user linear op mode";
	        ThreadPool.awaitTermination(executorService, 100, TimeUnit.DAYS, serviceName);
	      } catch (InterruptedException e) {
	        Thread.currentThread().interrupt();
	      }
	    }
	  }

	  protected void handleLoop() {
	    // if there is a runtime exception in user code; throw it so the normal error
	    // reporting process can handle it
	    if (helper.hasRuntimeException()) {
	      throw helper.getRuntimeException();
	    }

	    synchronized (this) {
	      this.notifyAll();
	    }
	  }

	  protected class LinearOpModeHelper implements Runnable {

	    protected RuntimeException exception  = null;
	    protected boolean          isShutdown = false;

	    public LinearOpModeHelper() {
	    }

	    @Override
	    public void run() {
	      ThreadPool.logThreadLifeCycle("LinearOpMode main", new Runnable() { @Override public void run() {
	        exception = null;
	        isShutdown = false;

	        try {
	          LinearOpModeSim.this.runOpMode();
	          requestOpModeStop();
	        } catch (InterruptedException ie){
	        } catch (CancellationException ie){
	        } catch (RuntimeException e) {
	          exception = e;
	        } finally {
	          // Do the necessary bookkeeping
	          isShutdown = true;
	        }
	      }});
	    }

	    public boolean hasRuntimeException() {
	      return (exception != null);
	    }

	    public RuntimeException getRuntimeException() {
	      return exception;
	    }
	    
	    public boolean isShutdown() {
	      return isShutdown;
	    }
	  }
}
