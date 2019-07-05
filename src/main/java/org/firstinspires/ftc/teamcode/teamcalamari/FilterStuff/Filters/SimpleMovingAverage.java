package org.firstinspires.ftc.teamcode.teamcalamari.FilterStuff.Filters;

import java.util.ArrayList;

public class SimpleMovingAverage implements Filter<Double[]> {
	
	int[] sampleSize;
	Double[] filtValues;
	double[] pFiltValues;
	
	public SimpleMovingAverage(int[] ss){
		sampleSize = new int[ss.length];
		filtValues = new Double[ss.length];
		pFiltValues = new double[ss.length];
		for(int i = 0; i < ss.length; i++) {
			sampleSize[i] = ss[i];
		}
	}

	@Override
	public Double[] run(ArrayList<Double[]> values, int measNum) {
		for(int i = 0; i < sampleSize.length; i++) {
			double currValue = values.get(measNum)[i+1];
			
			if(measNum == 0) {
				pFiltValues[i] = currValue;
			}
			
			if(measNum < sampleSize[i]) {
				filtValues[i] = (currValue+(measNum)*pFiltValues[i])/(measNum+1);
			}
			else {
				filtValues[i] = pFiltValues[i]+(currValue-values.get(measNum-sampleSize[i])[i+1])/sampleSize[i];
			}
			pFiltValues[i] = filtValues[i];
		}
		return filtValues;
	}
	
}
