package org.firstinspires.ftc.teamcode.teamcalamari.FilterStuff.Filters;

import java.util.ArrayList;

import org.ejml.simple.SimpleMatrix;

public class KalmanFilterAVP implements Filter<Double[]> {
	
	public int sensorNum = 1;
	public int stateNum = 3;
	
	private SimpleMatrix[] X = new SimpleMatrix[3];
	private SimpleMatrix[] pX = new SimpleMatrix[3];
	private SimpleMatrix[] pP = new SimpleMatrix[3];
	private SimpleMatrix[] P = new SimpleMatrix[3];
	private SimpleMatrix[] Z = new SimpleMatrix[3];
	private SimpleMatrix[] G = new SimpleMatrix[3];
	private SimpleMatrix A = new SimpleMatrix(stateNum, stateNum);
	private SimpleMatrix B = new SimpleMatrix(stateNum, 1);
	private SimpleMatrix C = new SimpleMatrix(sensorNum, stateNum);
	private SimpleMatrix U = new SimpleMatrix(1, 1);
	private SimpleMatrix Q = new SimpleMatrix(stateNum, stateNum);
	private SimpleMatrix[] R = new SimpleMatrix[3];
	
	public KalmanFilterAVP(double[][] a, double[][] c, double[][] q, double[] r, double[][] b) {
		for(int i = 0; i < 3; i++) {
			X[i] = new SimpleMatrix(stateNum, 1);
			pX[i] = new SimpleMatrix(stateNum, 1);
			pP[i] = new SimpleMatrix(fillDoubleArray(1, stateNum, stateNum));
			P[i] = new SimpleMatrix(stateNum, stateNum);
			Z[i] = new SimpleMatrix(sensorNum, 1);
			G[i] = new SimpleMatrix(stateNum, sensorNum);
			R[i] = SimpleMatrix.diag(r);
		}
		A = new SimpleMatrix(a);
		B = new SimpleMatrix(b);
		C = new SimpleMatrix(c);
		Q = new SimpleMatrix(q);
	}
	
	private double[][] fillDoubleArray(int value, int rows, int columns) {
		double[][] a = new double[rows][columns];
		for(int i = 0; i < rows; i++) {
			for(int j = 0; j < columns; j++) {
				a[i][j] = value;
			}
		}
		return a;
	}

	private void predict(int i) {
		X[i] = (A.mult(pX[i])).plus(B.mult(U));
		P[i] = A.mult(pP[i]).mult(A.transpose()).plus(Q);
	}
	private void update(int i) {
		G[i] = P[i].mult(C.transpose()).mult((C.mult(P[i]).mult(C.transpose()).plus(R[i]).invert()));
		X[i] = X[i].plus(G[i].mult(Z[i].minus(C.mult(X[i]))));
		P[i] = (SimpleMatrix.identity(3).minus(G[i].mult(C))).mult(P[i]);
		
		pP[i] = P[i];
	    pX[i] = X[i];
	}

	public Double[] run(ArrayList<Double[]> values, int measNum) {
		Double[] currValue = values.get(measNum);
		for(int i = 0; i < 3; i++) {
			Z[i].set(0, 0, currValue[i+1]);
			
			double time = currValue[0];
			A.set(1, 0, time);
			A.set(2, 1, time);
			A.set(2, 0, 0.5*Math.pow(time, 2));
			
			U.set(0, 0, 0);
			
			if(measNum == 0) {
				pX[i] = new SimpleMatrix(new double[][] {{Z[i].get(0, 0)}, {0}, {0}});
			}
			
			predict(i);
			update(i);
		}
		//only return the accelerations.
		//in an application of this filter you will likely want to return the entire state variable X.
		return new Double[] {X[0].get(0,0), X[1].get(0,0), X[2].get(0,0)};
	}

}
