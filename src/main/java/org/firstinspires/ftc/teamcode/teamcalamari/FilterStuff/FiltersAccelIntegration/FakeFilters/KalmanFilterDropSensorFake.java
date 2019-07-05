package org.teamcalamari.FilterStuff.FiltersAccelIntegration.FakeFilters;

import org.ejml.simple.SimpleMatrix;
import org.teamcalamari.FilterStuff.FiltersAccelIntegration.KalmanFilterDropSensor;

public class KalmanFilterDropSensorFake extends KalmanFilterDropSensor {

	public KalmanFilterDropSensorFake(double[][] a, double[][] c, double[][] q, double[] r, double[][] b,
			String[] sensorNames, double[][] maxJumpValues, double[][] maxDiffPoseValues) {
		super(a, c, q, r, b, sensorNames, maxJumpValues, maxDiffPoseValues);
	}

	@Override
	public SimpleMatrix[] run() {
		
		for(int i = 0; i < 2; i++) {
			X[i].set(2, 0, currSensorValues.get(0).get(i));
		}
		
		return X;
	}
	
}
