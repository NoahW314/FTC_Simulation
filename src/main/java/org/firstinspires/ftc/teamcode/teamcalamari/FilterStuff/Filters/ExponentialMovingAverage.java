package org.firstinspires.ftc.teamcode.teamcalamari.FilterStuff.Filters;

import java.util.ArrayList;

public class ExponentialMovingAverage implements Filter<Double[]> {

	double[] smoothingFactor;
	double[] pFiltValues;
	Double[] filtValues;
	
	public ExponentialMovingAverage(double[] smoothing) {
		smoothingFactor = new double[smoothing.length];
		pFiltValues = new double[smoothing.length];
		filtValues = new Double[smoothing.length];
		for(int i = 0; i < smoothing.length; i++) {
			smoothingFactor[i] = smoothing[i];
		}
	}

	@Override
	public Double[] run(ArrayList<Double[]> values, int measNum) {
		for(int i = 0; i < smoothingFactor.length; i++) {
			double currValue = values.get(measNum)[i+1];
			if(measNum == 0) {
				pFiltValues[i] = currValue;
			}
			
			filtValues[i] = smoothingFactor[i]*currValue+(1-smoothingFactor[i])*pFiltValues[i];
			
			pFiltValues[i] = filtValues[i];
		}
		return filtValues;
	}

}
