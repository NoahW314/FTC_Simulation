package org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.HardwareSim.SensorsSim;

import java.util.concurrent.CancellationException;
import java.util.concurrent.TimeUnit;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.FileLoggingAccelerationIntegratorHeadingSim;
import org.firstinspires.ftc.teamcode.teamcalamari.Tests.AccelTestsHeading.FileLoggingAccelerationIntegratorHeading;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ThreadPool;

public class BNO055TestMultiSim extends BNO055IMUSim {
	
	AccelerationIntegrator[] accelerationAlgorithms; 

    public void startAccelerationIntegration(Position initalPosition, Velocity initialVelocity, int... msPollInterval)
    // Start integrating acceleration to determine position and velocity by polling for acceleration every while
    {
    	accelerationAlgorithms = new AccelerationIntegrator[msPollInterval.length];
        synchronized (this.startStopLock)
        {
            // Stop doing this if we're already in flight
            this.stopAccelerationIntegration();

            // Set the current position and velocity
            this.accelerationAlgorithm.initialize(this.parameters, initalPosition, initialVelocity);
            
            for(int i = 0; i < msPollInterval.length; i++) {
            	accelerationAlgorithms[i] = ((FileLoggingAccelerationIntegratorHeadingSim)this.accelerationAlgorithm).clone(i);
            }

            // Make a new thread on which to do the integration
            /*this.accelerationMananger = ThreadPool.newSingleThreadExecutor("imu acceleration");

            // Start the whole schebang a rockin...
            this.accelerationMananger.execute(new AccelerationManagerMultiLog(msPollInterval));*/
            new Thread(new AccelerationManagerMultiLog(msPollInterval)).start();
        }
    }
    @Override
    public void stopAccelerationIntegration() // needs a different lock than 'synchronized(this)'
    {
        synchronized (this.startStopLock)
        {
            // Stop the integration thread
            if (this.accelerationMananger != null)
            {
            	for(int i = 0; i < accelerationAlgorithms.length; i++) {
            		((FileLoggingAccelerationIntegratorHeading)(this.accelerationAlgorithms[i])).logger.closeDataLogger();
            	}
                this.accelerationMananger.shutdownNow();
                ThreadPool.awaitTerminationOrExitApplication(this.accelerationMananger, 10, TimeUnit.SECONDS, "IMU acceleration", "unresponsive user acceleration code");
                this.accelerationMananger = null;
            }
        }
    }

    class AccelerationManagerMultiLog implements Runnable{
        protected final int[] msPollIntervals;
        protected final static long nsPerMs = ElapsedTime.MILLIS_IN_NANO;

        AccelerationManagerMultiLog(int... msPollIntervals)
        {
            this.msPollIntervals = msPollIntervals;
        }

        @Override public void run()
        {
            for(int i = 0; i < msPollIntervals.length; i++){
                new Thread(new AccelerationAlgorithmUpdatorLog(msPollIntervals[i], i)).start();
            }
        }

        class AccelerationAlgorithmUpdatorLog implements Runnable{
            public final int msPollInterval;
            public final int i;

            AccelerationAlgorithmUpdatorLog(int msPollInterval, int i){
            	this.msPollInterval = msPollInterval;
            	this.i = i;
            }

            @Override
            public void run() {
                // Don't let inappropriate exceptions sneak out
                try
                {
                    // Loop until we're asked to stop
                    while (!isStopRequested())
                    {
                        // Read the latest available acceleration
                        final Acceleration linearAcceleration = BNO055TestMultiSim.this.getLinearAcceleration();
                        final double heading = BNO055TestMultiSim.this.getAngularOrientation(
                                AxesReference.EXTRINSIC,
                                AxesOrder.XYZ,
                                org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES)
                                .thirdAngle;

                        // Have the algorithm do its thing
                        synchronized (dataLock)
                        {
                            ((FileLoggingAccelerationIntegratorHeadingSim)accelerationAlgorithms[i]).update(linearAcceleration, heading);
                        }

                        // Wait an appropriate interval before beginning again
                        if (msPollInterval > 0)
                        {
                            long msSoFar = (System.nanoTime() - linearAcceleration.acquisitionTime) / nsPerMs;
                            long msReadFudge = 5;   // very roughly accounts for delta from read request to acquisitionTime setting
                            Thread.sleep(Math.max(0,msPollInterval - msSoFar - msReadFudge));
                        }
                        else
                            Thread.yield(); // never do a hard spin
                    }
                }
                catch (InterruptedException|CancellationException e)
                {
                    return;
                }
            }
        }
    }

    protected boolean isStopRequested()
    {
        return Thread.currentThread().isInterrupted();
    }

}
