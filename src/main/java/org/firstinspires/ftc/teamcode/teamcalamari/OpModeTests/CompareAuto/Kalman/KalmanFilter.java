package org.teamcalamari.OpModeTests.CompareAuto.Kalman;

import org.teamcalamari.FilterStuff.FiltersAccelIntegration.KalmanFilterDropSensor;
import org.teamcalamari.FilterStuff.FiltersAccelIntegration.FakeFilters.KalmanFilterDropSensorFake;

public class KalmanFilter {
    public static KalmanFilterDropSensor getFilter(){
    	//space holder for time  (after all time and space are related so why not use one for the other ;-)
    	double t = 0;
    	KalmanFilterDropSensor filter = new KalmanFilterDropSensorFake(
    			new double[][] {{1, 0, 0}, {t, 1, 0}, {0.5*Math.pow(t, 2), t, 1}}, 
    			new double[][] {{0, 0, 1}, {0, 0, 1}}, new double[][] {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, 
    			new double[] {0, 0}, new double[][] {{0}, {0}, {0}}, new String[] {"Encoder", "Ultrasonic"},
    			new double[][] {{100, 100}, {100, 100}}, new double[][] {{100, 100}, {100, 100}});
    	return filter;
    }
}
