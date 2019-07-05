package org.firstinspires.ftc.teamcode.teamcalamari.FilterStuff.Filters;

import java.util.ArrayList;

import org.ejml.simple.SimpleMatrix;

public class KalmanFilterDropSensor implements Filter<Double[]> {
	
	private int sensorNum = 5;
	private final int maxSensorNum = sensorNum;
	private int stateNum = 3;
	
	private boolean Vuforia = true;
	private boolean ultrasonic = true;
	private boolean[] isSensorActive = new boolean[sensorNum];
	private int[] sensorIndex = new int[sensorNum];
	private String[] sensorNames = new String[sensorNum];
	private double[] lastValues = new double[sensorNum];
	private double[] prevLastValues = new double[sensorNum];
	private double[] maxJumpValues = new double[sensorNum];
	private double[] maxPoseDiffValues = new double[sensorNum];
	
	/**Current State Matrix*/
	private SimpleMatrix[] X = new SimpleMatrix[3];
	/**Previous State Matrix*/
	private SimpleMatrix[] pX = new SimpleMatrix[3];
	/**Previous Prediction Error*/
	private SimpleMatrix[] pP = new SimpleMatrix[3];
	/**Current Prediction Error*/
	private SimpleMatrix[] P = new SimpleMatrix[3];
	/**Noisy Measurements*/
	private SimpleMatrix[] Z = new SimpleMatrix[3];
	/**Kalman Gain (variable to store value for use in two places)*/
	private SimpleMatrix[] G = new SimpleMatrix[3];
	/**State Transition Model*/
	private SimpleMatrix A = new SimpleMatrix(stateNum, stateNum);
	/**How much each control input contributes to each state*/
	private SimpleMatrix B = new SimpleMatrix(stateNum, 1);
	/**How much each sensor contributes to each state*/
	private SimpleMatrix C = new SimpleMatrix(sensorNum, stateNum);
	/**Control input*/
	private SimpleMatrix U = new SimpleMatrix(1, 1);
	/**Covariance of Process Noise*/
	private SimpleMatrix Q = new SimpleMatrix(stateNum, stateNum);
	/**Covariance of Sensor Noise (Diagonal contains sensor variance)*/
	private SimpleMatrix[] R = new SimpleMatrix[3];
	
	private SimpleMatrix fullC = new SimpleMatrix(sensorNum, stateNum);
	private SimpleMatrix[] fullR = new SimpleMatrix[3];
	
	public KalmanFilterDropSensor(double[][] a, double[][] c, double[][] q, double[] r, double[][] b, String[] sensorNames) {
		//initialize and set all matrixes not yet initialized
		for(int i = 0; i < 3; i++) {
			X[i] = new SimpleMatrix(stateNum, 1);
			pX[i] = new SimpleMatrix(stateNum, 1);
			pP[i] = new SimpleMatrix(fillDoubleArray(1, stateNum, stateNum));
			P[i] = new SimpleMatrix(stateNum, stateNum);
			Z[i] = new SimpleMatrix(sensorNum, 1);
			G[i] = new SimpleMatrix(stateNum, sensorNum);
			R[i] = SimpleMatrix.diag(r);
			fullR[i] = SimpleMatrix.diag(r);
		}
		A = new SimpleMatrix(a);
		B = new SimpleMatrix(b);
		C = new SimpleMatrix(c);
		fullC = new SimpleMatrix(c);
		Q = new SimpleMatrix(q);
		
		//assume all sensors start out activated
		for(int i = 0; i < sensorNum; i++) {
			isSensorActive[i] = true;
			sensorIndex[i] = i;
			this.sensorNames[i] = sensorNames[i];
		}
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

	private void removeRowColumn(SimpleMatrix mat, int index) {
		removeRow(mat, index);
		removeColumn(mat, index);
	}
	private void removeRow(SimpleMatrix mat, int rowNum) {
		SimpleMatrix lowerPart = mat.extractMatrix(rowNum+1, SimpleMatrix.END, 0, SimpleMatrix.END);
		mat.reshape(rowNum, mat.numCols());
		mat.set(mat.concatRows(lowerPart));
	}
	private void removeColumn(SimpleMatrix mat, int colNum) {
		SimpleMatrix rightPart = mat.extractMatrix(0, SimpleMatrix.END, colNum+1, SimpleMatrix.END);
		for(int i = mat.numCols()-1; i > colNum-1; i--) {
			removeEndColumn(mat);
		}
		mat.set(mat.concatColumns(rightPart));
	}
	private void removeEndColumn(SimpleMatrix mat) {
		mat.set(mat.extractMatrix(0, SimpleMatrix.END, 0, mat.numCols()-1));
	}
	private void addRow(SimpleMatrix mat, SimpleMatrix fullMat, int sensorID, int[] sensorIndices) {
		SimpleMatrix row = new SimpleMatrix(1, mat.numCols());
		
		int j;
		int lastBeforeSensorID = -1;
		for(int i = 0; i < fullMat.numRows(); i++) {
			if((j = sensorIndices[i]) != -1) {
				row.set(0, j, fullMat.get(sensorID, i));
				
				if(lastBeforeSensorID == -1 && i > sensorID) {
					lastBeforeSensorID = j;
				}
			}
		}
		
		SimpleMatrix upper = mat.extractMatrix(0, lastBeforeSensorID, 0, SimpleMatrix.END);
		SimpleMatrix lower = mat.extractMatrix(lastBeforeSensorID, SimpleMatrix.END, 0, SimpleMatrix.END);
		
		mat.set(upper.concatRows(row, lower));
	}
	private void addColumn(SimpleMatrix mat, SimpleMatrix fullMat, int sensorID, int[] sensorIndices) {
		SimpleMatrix column = new SimpleMatrix(mat.numRows(), 1);
		
		for(int i = 0; i < fullMat.numCols(); i++) {
			int j;
			if((j = sensorIndices[i]) != -1) {
				column.set(j, 0, fullMat.get(i, sensorID));
			}
		}
		
		SimpleMatrix left = mat.extractMatrix(0, SimpleMatrix.END, 0, sensorIndices[sensorID]);
		SimpleMatrix right = mat.extractMatrix(0, SimpleMatrix.END, sensorIndices[sensorID], SimpleMatrix.END);
		
		mat.set(left.concatColumns(column, right));
	}
	private void setSensorIndex(int sensorID) {
		boolean set = false;
		int j = 0;
		for(int i = 0; i < sensorIndex.length; i++) {
			if((j = sensorIndex[i]) != -1) {
				if(i > sensorID) {
					sensorIndex[sensorID] = j;
					set = true;
				}
			}
		}
		if(!set) {
			sensorIndex[sensorID] = j+1;
		}
		
		//increase the index in R of all the sensors above this one
		for(int k = 0; k < sensorNum; k++) {
			if(sensorIndex[k] > sensorIndex[sensorID]) {
				sensorIndex[k]+=1;
			}
		}
	}
	
	public void activateSensor(int sensorID) {
		//don't active an already active sensor
		if(isSensorActive[sensorID]) return;
		
		sensorNum++;
		isSensorActive[sensorID] = true;
		
		for(int i = 0; i < 3; i++) {
			//add the row and column corresponding to this sensor in R
			addRow(R[i], fullR[i], sensorID, sensorIndex);
			
			setSensorIndex(sensorID);
			
			//the column's active sensor number is one higher since the row has already been added in
			addColumn(R[i], fullR[i], sensorID, sensorIndex);
			//probably unnecessary
			Z[i] = new SimpleMatrix(sensorNum, 1);
			
			sensorIndex[sensorID] = -1;
			//decrease the index in R of all the sensors above this one
			for(int j = 0; j < sensorNum; j++) {
				if(sensorIndex[j] > sensorIndex[sensorID]) {
					sensorIndex[j]-=1;
				}
			}
		}
		
		//add the row corresponding to this sensor in C
		addRow(C, fullC, sensorID, sensorIndex);
		
		setSensorIndex(sensorID);
	}
	/**@param sensorID the place of the sensor in the array isSensorActive 
	 * and the index of the sensor name in sensorNames*/
	public void deactivateSensor(int sensorID) {
		//don't deactivate an already deactivated sensor
		if(!isSensorActive[sensorID]) return;
		
		sensorNum--;
		isSensorActive[sensorID] = false;
		
		int index = sensorIndex[sensorID];
		for(int i = 0; i < 3; i++) {
			//remove the row and column corresponding to this sensor in R
			removeRowColumn(R[i], index);
			//probably serves no actual purpose... probably.
			Z[i] = new SimpleMatrix(sensorNum, 1);
		}
		//remove the row corresponding to this sensor in C
		removeRow(C, index);
		
		//this sensor no longer has a place in any matrix
		sensorIndex[sensorID] = -1;
		//decrease the index in R of all the sensors above this one
		for(int i = 0; i < sensorNum; i++) {
			if(sensorIndex[i] > index) {
				sensorIndex[i]-=1;
			}
		}
	}
	
	private void predict(int i) {
		X[i] = (A.mult(pX[i])).plus(B.mult(U));
		P[i] = A.mult(pP[i]).mult(A.transpose()).plus(Q);
	}
	private void update(int i) {
		G[i] = P[i].mult(C.transpose()).mult((C.mult(P[i]).mult(C.transpose()).plus(R[i])).invert());
		X[i] = X[i].plus(G[i].mult(Z[i].minus(C.mult(X[i]))));
		P[i] = (SimpleMatrix.identity(3).minus(G[i].mult(C))).mult(P[i]);
		
		pP[i] = P[i];
	    pX[i] = X[i];
	}

	@Override
	public Double[] run(ArrayList<Double[]> values, int measNum) {
		Double[] currValue = values.get(measNum);
		
		for(int i = 0; i < 3; i++) {
			for(int j = 0; j < isSensorActive.length; j++) {
				if(isSensorActive[j]) {
					double lastJump = lastValues[i+j*3]-prevLastValues[i+j*3];
					double currentJump = currValue[i+j*3+1]-lastValues[i+j*3];
					if(Math.abs(currentJump-lastJump) >= maxJumpValues[i+j*3]) {
						deactivateSensor(j);
					}
				}
				if(isSensorActive[j]) {
					if(Math.abs(currValue[i+j*3+1]-X[i].get(2, 0)) >= maxPoseDiffValues[i+j*3]) {
						deactivateSensor(j);
					}
				}
			}
			
			
			for(int j = 0; j < maxSensorNum; j++) {
				int k;
				//only set the measurements of active sensors
				if((k = sensorIndex[i]) != -1) {
					Z[i].set(k, 0, currValue[i+j*3+1]);
				}
			}
			
			
			double time = currValue[0];
			A.set(1, 0, time);
			A.set(2, 1, time);
			A.set(2, 0, 0.5*Math.pow(time, 2));
			
			U.set(0, 0, 0);
			
			if(measNum == 0) {
				for(int j = 0; j < lastValues.length; j++) {
					lastValues[i+j*3] = currValue[i+j*3+1];
				}
				pX[i] = new SimpleMatrix(new double[][] {{(Z[i].get(0, 0)+Z[i].get(1, 0))/2}, {0}, {0}});
			}
			
			predict(i);
			update(i);
			
			for(int j = 0; j < lastValues.length; j++) {
				prevLastValues[i+j*3] = lastValues[i+j*3];
				lastValues[i+j*3] = currValue[i+j*3+1];
			}
		}
		
		return new Double[] {X[0].get(0,0), X[1].get(0,0), X[2].get(0,0),
							X[0].get(1,0), X[1].get(1,0), X[2].get(1,0),
							X[0].get(2,0), X[1].get(2,0), X[2].get(2,0)};
	}

}
