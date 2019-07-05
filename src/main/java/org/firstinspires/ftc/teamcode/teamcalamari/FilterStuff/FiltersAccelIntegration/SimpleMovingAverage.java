package org.firstinspires.ftc.teamcode.teamcalamari.FilterStuff.FiltersAccelIntegration;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;

public class SimpleMovingAverage extends AccelFilter{
	
	int[] sampleSize;
	double[] filtValues;
	double[] pFiltValues;
	
	public SimpleMovingAverage(int[] ss){
		sampleSize = new int[3];
		filtValues = new double[3];
		pFiltValues = new double[3];
		for(int i = 0; i < 3; i++) {
			sampleSize[i] = ss[i];
		}
	}

	@Override
	public Acceleration run() {
		int measNum = values.size()-1;
		for(int i = 0; i < 3; i++) {
			double currentValue = AccelAxisFromInt(currValue, i);
			
			if(measNum == 0) {
				pFiltValues[i] = currentValue;
			}
			
			if(measNum < sampleSize[i]) {
				filtValues[i] = (currentValue+(measNum)*pFiltValues[i])/(measNum+1);
			}
			else {
				filtValues[i] = pFiltValues[i]+(currentValue-AccelAxisFromInt(values.get(measNum-sampleSize[i]), i))/sampleSize[i];
			}
			pFiltValues[i] = filtValues[i];
		}
		return DoubleArrayToAccel(filtValues, currValue.acquisitionTime);
	}
	
}
