package org.teamcalamari.FilterStuff.FiltersAccelIntegration;

import java.util.ArrayList;
import java.util.List;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public class KalmanFilterDropSensor extends BasicPoseKalmanFilter {
	private final int maxSensorNum;
	
	private boolean[] isSensorActive = new boolean[sensorNum];
	private int[] sensorIndex = new int[sensorNum];
	private String[] sensorNames = new String[sensorNum];
	private ArrayList<ArrayList<Double>> lastValues = new ArrayList<>();
	private ArrayList<ArrayList<Double>> prevLastValues = new ArrayList<>();
	private ArrayList<Double[]> maxJumpValues = new ArrayList<>();
	private ArrayList<Double[]> maxPoseDiffValues = new ArrayList<>();
	
	private SimpleMatrix fullC = new SimpleMatrix(sensorNum, stateNum);
	private SimpleMatrix[] fullR = new SimpleMatrix[3];
	
	public KalmanFilterDropSensor(double[][] a, double[][] c, double[][] q, double[] r, double[][] b, String[] sensorNames, 
							double[][] maxJumpValues, double[][] maxDiffPoseValues) {
		super(a, c, q, r, b, 2);
		maxSensorNum = sensorNum;
		
		for(int i = 0; i < R.length; i++) {
			fullR[i] = R[i].copy();
		}
		fullC = C.copy();
		
		//assume all sensors start out activated
		for(int i = 0; i < sensorNum; i++) {
			isSensorActive[i] = true;
			sensorIndex[i] = i;
			this.sensorNames[i] = sensorNames[i];
			
			this.maxJumpValues.add(doubleAToDoubleA(maxJumpValues[i]));
			this.maxPoseDiffValues.add(doubleAToDoubleA(maxDiffPoseValues[i]));
		}
	}
	
	private Double[] doubleAToDoubleA(double[] dArr) {
		Double[] DArr = new Double[dArr.length];
		for(int i = 0; i < dArr.length; i++) {
			DArr[i] = dArr[i];
		}
		return DArr;
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
	
	/**@param sensorID the place of the sensor in the array isSensorActive 
	and the index of the sensor name in sensorNames*/
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
	and the index of the sensor name in sensorNames*/
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

	public SimpleMatrix[] run() {
		int measNum = length;

		for(int i = 0; i < dims; i++) {
			for(int j = 0; j < isSensorActive.length; j++) {
				if(isSensorActive[j]) {
					double lastJump = lastValues.get(j).get(i)-prevLastValues.get(j).get(i);
					double currentJump = currSensorValues.get(j).get(i)-lastValues.get(j).get(i);
					if(Math.abs(currentJump-lastJump) >= maxJumpValues.get(j)[i]) {
						deactivateSensor(j);
					}
				}
				if(isSensorActive[j]) {
					if(Math.abs(currSensorValues.get(j).get(i)-X[i].get(2, 0)) >= maxPoseDiffValues.get(j)[i]) {
						deactivateSensor(j);
					}
				}
			}
			
			
			for(int j = 0; j < maxSensorNum; j++) {
				int k;
				//only set the measurements of active sensors
				if((k = sensorIndex[i]) != -1) {
					Z[i].set(k, 0, currSensorValues.get(j).get(i));
				}
			}
			
			
			double time = currTimestamp-prevTimestamp;
			A.set(1, 0, time);
			A.set(2, 1, time);
			A.set(2, 0, 0.5*Math.pow(time, 2));
			
			U.set(0, 0, 0);
			
			if(measNum == 0) {
				for(int j = 0; j < lastValues.size(); j++) {
					lastValues.get(j).set(i, currSensorValues.get(j).get(i));
				}
				pX[i] = new SimpleMatrix(new double[][] {{(Z[i].get(0, 0)+Z[i].get(1, 0))/2}, {0}, {0}});
			}
			
			predict(i);
			update(i);
			
			for(int j = 0; j < lastValues.size()-1; j++) {
				prevLastValues.get(j).set(i, lastValues.get(j).get(i));
				lastValues.get(j).set(i, currSensorValues.get(j).get(i));
			}
		}
		
		return X;
	}

	/*currValue = linearAcceleration;
	values.add(linearAcceleration);
	SimpleMatrix[] states = run();
	long time = currValue.acquisitionTime;
	Acceleration acceleration = DoubleArrayToAccel(new double[] {states[0].get(0,0), states[1].get(0,0), states[2].get(0,0)}, time);
	velocity = DoubleArrayToVelocity(new double[] {states[0].get(1,0), states[1].get(1,0), states[2].get(1,0)}, time);
	position = DoubleArrayToPosition(new double[] {states[0].get(2,0), states[1].get(2,0), states[2].get(2,0)}, time);
	filtValues.add(acceleration);
		
	prevAccel = acceleration;
	prevVelocity = velocity;*/
	
	public void update(List<Acceleration> accelerations, List<Velocity> velocities, List<Position> poses) {
		length++;
		if(accelerations.size() > 0 ) {
			currTimestamp = accelerations.get(0).acquisitionTime;
		}
		else if(velocities.size() > 0) {
			currTimestamp = velocities.get(0).acquisitionTime;
		}
		else if(poses.size() > 0) {
			currTimestamp = poses.get(0).acquisitionTime;
		}
		
		int n = 0;
		for(int i = 0; i < accelerations.size()-1; i++, n++) {
			currSensorValues.get(n).set(0, accelerations.get(i).xAccel);
			currSensorValues.get(n).set(1, accelerations.get(i).yAccel);
		}
		for(int i = 0; i < velocities.size()-1; i++, n++) {
			currSensorValues.get(n).set(0, velocities.get(i).xVeloc);
			currSensorValues.get(n).set(1, velocities.get(i).yVeloc);
		}
		for(int i = 0; i < poses.size()-1; i++, n++) {
			currSensorValues.get(n).set(0, poses.get(i).x);
			currSensorValues.get(n).set(1, poses.get(i).y);
		}
		
		
		SimpleMatrix[] states = run();
		
		acceleration = new Acceleration(unit, states[0].get(0,0), states[1].get(0,0), 0, currTimestamp);
		velocity = new Velocity(unit, states[0].get(1,0), states[1].get(1,0), 0, currTimestamp);
		position = new Position(unit, states[0].get(2,0), states[1].get(2,0), 0, currTimestamp);
		
		prevTimestamp = currTimestamp;
	}
	
	public void update(Acceleration linearAcceleration, Position encPose, Position ultraPose, Position vufPose){
		length++;		
		currTimestamp = linearAcceleration.acquisitionTime;
		
		
		currSensorValues.get(0).set(0, linearAcceleration.xAccel);
		currSensorValues.get(0).set(1, linearAcceleration.yAccel);
		
		currSensorValues.get(1).set(0, encPose.x);
		currSensorValues.get(1).set(1, encPose.y);
		
		currSensorValues.get(2).set(0, ultraPose.x);
		currSensorValues.get(2).set(1, ultraPose.y);
		
		currSensorValues.get(3).set(0, vufPose.x);
		currSensorValues.get(3).set(1, vufPose.y);
		
		
		SimpleMatrix[] states = run();
		
		acceleration = new Acceleration(unit, states[0].get(0,0), states[1].get(0,0), 0, currTimestamp);
		velocity = new Velocity(unit, states[0].get(1,0), states[1].get(1,0), 0, currTimestamp);
		position = new Position(unit, states[0].get(2,0), states[1].get(2,0), 0, currTimestamp);
		
		prevTimestamp = currTimestamp;
	}
}
