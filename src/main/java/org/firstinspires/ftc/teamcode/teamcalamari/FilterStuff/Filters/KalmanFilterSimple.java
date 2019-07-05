package org.firstinspires.ftc.teamcode.teamcalamari.FilterStuff.Filters;

import java.util.ArrayList;

public class KalmanFilterSimple implements Filter<Double[]> {
	
	
	private Double[] X = new Double[3];
	private double[] pX = new double[3];
	private double[] P = new double[3];
	private double G, Z;
	private double A, C, Q, R;
	public KalmanFilterSimple(double a, double c, double q, double r) {
		A = a;
		C = c;
		Q = q;
		R = r;
	}
	public double[] pP = {1, 1, 1};
	private void predict(int i) {
		X[i] = A*pX[i];
	    P[i] = A*pP[i]*A+Q;
	}
	private void update(int i) {
		G = P[i]*C/(C*P[i]*C+R);
	    X[i] = X[i]+G*(Z-C*X[i]);
	    P[i] = (1-G*C)*P[i];
	    
	    pP[i] = P[i];
	    pX[i] = X[i];
	}

	@Override
	public Double[] run(ArrayList<Double[]> values, int measNum) {
		for(int i = 0; i < 3; i++) {
			Z = values.get(measNum)[i+1];
			if(measNum == 0) {
				pX[i] = Z;
			}
			predict(i);
			update(i);
		}
		return X;
	}

}
