package org.firstinspires.ftc.teamcode.teamcalamari.FilterStuff.FiltersAccelIntegration;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;

public class ExponentialMovingAverage extends AccelFilter {

	double[] smoothingFactor;
	double[] pFiltValues;
	double[] filtValues;
	
	public ExponentialMovingAverage(double[] smoothing) {
		smoothingFactor = new double[3];
		pFiltValues = new double[3];
		filtValues = new double[3];
		for(int i = 0; i < 3; i++) {
			smoothingFactor[i] = smoothing[i];
		}
	}

	@Override
	public Acceleration run() {
		int measNum = values.size()-1;
		for(int i = 0; i < 3; i++) {
			double currValue = AccelAxisFromInt(values.get(measNum), i);
			if(measNum == 0) {
				pFiltValues[i] = currValue;
			}
			
			filtValues[i] = smoothingFactor[i]*currValue+(1-smoothingFactor[i])*pFiltValues[i];
			
			pFiltValues[i] = filtValues[i];
		}
		return DoubleArrayToAccel(filtValues, values.get(measNum).acquisitionTime);
	}

}
