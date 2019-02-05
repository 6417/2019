/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ch.fridolinsrobotik.drivesystems.swerve;

import ch.fridolinsrobotik.utilities.Algorithms;

/**
 * Add your docs here.
 */
public class SwerveCalculation {

	private double[] rotationAngle = new double[4];
	
	private double[] a1 = new double[4];
	private double[] a2 = new double[4];
	private double[] a = new double[4];
	private double[] b1 = new double[4];
	private double[] b2 = new double[4];
	private double[] b = new double[4];
	private double[] drivePowerRaw = new double[4];
	private double drivePowerRawMax = 0;
	private double drivePowerRawMin = 0;
	
	private double[] drivePower = new double[4];
	private double[] steerAngle = new double[4];
	
	
	public SwerveCalculation(double wheelDistanceLength, double wheelDistanceWidth) {
		
		//Calculates the angle for the diagonal wheels
		rotationAngle[0] = -1 * (90 + Math.toDegrees(Math.atan(wheelDistanceWidth / wheelDistanceLength)));
		rotationAngle[1] = -1 * (90 - Math.toDegrees(Math.atan(wheelDistanceWidth / wheelDistanceLength)));
		rotationAngle[2] = 90 + Math.toDegrees(Math.atan(wheelDistanceWidth / wheelDistanceLength));
		rotationAngle[3] = 90 - Math.toDegrees(Math.atan(wheelDistanceWidth / wheelDistanceLength));
	}
	
	
	public void calculateValues(double robotAngle, double robotPower, double turnPower) {
		
		for(int i = 0; i < 4; i++) {
			
			a1[i] = turnPower * Math.sin(Math.toRadians(rotationAngle[i]));
			a2[i] = robotPower * Math.sin(Math.toRadians(robotAngle));
			a[i] = a1[i] + a2[i];
			b1[i] = turnPower * Math.cos(Math.toRadians(rotationAngle[i]));
			b2[i] = robotPower * Math.cos(Math.toRadians(robotAngle));
			b[i] = b1[i] + b2[i];
			
			if(a[i] == 0 && b[i] == 0) {
				steerAngle[i] = 0;
			}else {
				steerAngle[i] = Math.toDegrees(Math.atan(a[i] / b[i]));
			}
			
			if(steerAngle[i] == 0) {
				drivePowerRaw[i] = b[i] / Math.cos(Math.toRadians(steerAngle[i]));
			}else {
				drivePowerRaw[i] = a[i] / Math.sin(Math.toRadians(steerAngle[i]));
			}
			
			if(drivePowerRaw[i] > drivePowerRawMax) {
				drivePowerRawMax = drivePowerRaw[i];
			}
			
			if(drivePowerRaw[i] < drivePowerRawMin) {
				drivePowerRawMin = drivePowerRaw[i];
			}
		}
		
		if(drivePowerRawMax < drivePowerRawMin * -1) {
			drivePowerRawMax = drivePowerRawMin;
		}
		
		for(int i = 0; i < 4; i++) {
			
			if(drivePowerRawMax > 1) {
				drivePower[i] = Algorithms.scale(drivePowerRaw[i], 0, drivePowerRawMax, 0, 1);
			}else if(drivePowerRawMax < -1) {
				drivePower[i] = Algorithms.scale(drivePowerRaw[i], 0, drivePowerRawMax, 0, -1);
			}else {
				drivePower[i] = drivePowerRaw[i];
			}
		}
		
	}


	public double[] getDrivePower() {
		return drivePower;
	}

	public double[] getSteerAngle() {
		return steerAngle;
    }


    private double wheelDirection = 0;
	private boolean rotationDirection = false;
	private double rotationDistance = 0;
    
    public void calculateFastestWayToAngle(double currentAngle, double desiredAngle) {
		
		double wheelAngleForward = 0;
		double wheelAngleBackward = 0;
		double desiredAngleForward = 0;
		double desiredAngleBackward = 0;
		double distanceNoReverseForward = 0;
		double distanceNoReverseBackward = 0;
		double distanceReverseForward = 0;
		double distanceReverseBackward = 0;
		boolean distanceDirection = false;
		double distanceClockwiseForward = 0;
		double distanceClockwiseBackward = 0;
		double distanceCounterClockwiseForward = 0;
		double distanceCounterClockwiseBackward = 0;

		wheelAngleForward = currentAngle;
		desiredAngleForward = desiredAngle;
		
		if(wheelAngleForward + 180 > 180) {
			wheelAngleBackward = wheelAngleForward - 180;
		}else {
			wheelAngleBackward = wheelAngleForward + 180;
		}
		
		if(desiredAngleForward + 180 > 180) {
			desiredAngleBackward = desiredAngleForward - 180;
		}else {
			desiredAngleBackward = desiredAngleForward + 180;
		}
		
		if(wheelAngleForward - desiredAngleForward < 0) {
			distanceNoReverseForward = desiredAngleForward - wheelAngleForward;
		}else {
			distanceNoReverseForward = wheelAngleForward - desiredAngleForward;
		}
		
		if(wheelAngleBackward - desiredAngleBackward < 0) {
			distanceNoReverseBackward = desiredAngleBackward - wheelAngleBackward;
		}else {
			distanceNoReverseBackward = wheelAngleBackward - desiredAngleBackward;
		}
		
		if(wheelAngleBackward - desiredAngleForward < 0) {
			distanceReverseForward = desiredAngleForward - wheelAngleBackward;
		}else {
			distanceReverseForward = wheelAngleBackward - desiredAngleForward;
		}
		
		if(wheelAngleForward - desiredAngleBackward < 0) {
			distanceReverseBackward = desiredAngleBackward - wheelAngleForward;
		}else {
			distanceReverseBackward = wheelAngleForward - desiredAngleBackward;
		}
		
		if(distanceNoReverseForward <= distanceNoReverseBackward && distanceReverseForward <= distanceReverseBackward) {
			distanceDirection = true;
		}else {
			distanceDirection = false;
		}
		
		if(distanceDirection == true) {
			if(distanceNoReverseForward < distanceReverseForward) {
				wheelDirection = 1;
			}else {
				wheelDirection = -1;
			}
		}else {
			if(distanceNoReverseBackward <= distanceReverseBackward) {
				wheelDirection = 1;
			}else {
				wheelDirection = -1;
			}
		}
		
		if(desiredAngleForward < wheelAngleForward) {
			distanceClockwiseForward = desiredAngleForward + 360 - wheelAngleForward;
		}else {
			distanceClockwiseForward = desiredAngleForward - wheelAngleForward;
		}
		
		if(desiredAngleBackward < wheelAngleForward) {
			distanceClockwiseBackward = desiredAngleBackward + 360 - wheelAngleForward;
		}else {
			distanceClockwiseBackward = desiredAngleBackward - wheelAngleForward;
		}
		
		if(wheelAngleForward < desiredAngleForward) {
			distanceCounterClockwiseForward = wheelAngleForward + 360 - desiredAngleForward;
		}else {
			distanceCounterClockwiseForward = wheelAngleForward - desiredAngleForward;
		}
		
		if(wheelAngleForward < desiredAngleBackward) {
			distanceCounterClockwiseBackward = wheelAngleForward + 360 - desiredAngleBackward;
		}else {
			distanceCounterClockwiseBackward = wheelAngleForward - desiredAngleBackward;
		}
		
		if(distanceClockwiseForward < distanceCounterClockwiseForward && distanceClockwiseForward < distanceCounterClockwiseBackward) {
				rotationDirection = true;		
		}else {
			if(distanceClockwiseBackward < distanceCounterClockwiseForward && distanceClockwiseBackward < distanceCounterClockwiseBackward) {
				rotationDirection = true;
			}else {
				rotationDirection = false;
			}
		}
		
		if(wheelDirection == 1) {
			if(distanceDirection == true) {
				rotationDistance = distanceNoReverseForward;
			}else {
				rotationDistance = distanceNoReverseBackward;
			}
		}else {
			if(distanceDirection == true) {
				rotationDistance = distanceReverseForward;
			}else {
				rotationDistance = distanceReverseBackward;
			}
		}
	}
	
	public double getWheelDirection() {
		return wheelDirection;
	}

	public boolean getRotationDirection() {
		return rotationDirection;
	}

	public double getRotationDistance() {
		return rotationDistance;
    }
    

    public double getWheelAngleDegrees(int encoderTicks, int encoderTicksPerRotation) {
		
		double wheelRotationsDecimal = 0;
		int wheelRotations = 0;
		boolean wheelAngleOperationFlip = false;
		double wheelAngle = 0;

		wheelRotationsDecimal = (double) encoderTicks / (encoderTicksPerRotation / 2);
		
		wheelRotations = (int)Math.floor(wheelRotationsDecimal);
		
		if (wheelRotations%2 == 0) {
			
			wheelAngleOperationFlip = false;
		}else {
			
			wheelAngleOperationFlip = true;
		}
		
		if(wheelAngleOperationFlip == false) {
			
			wheelAngle = (wheelRotationsDecimal - wheelRotations) * 180;
		}else {
			
			wheelAngle = -180 + (wheelRotationsDecimal - wheelRotations) * 180;
		}
		
		return wheelAngle;
    }
    

    public double getTalonSteerOutput(int encoderTicks, int encoderTicksPerRotation, double rotationDistance, boolean rotationDirection) {
        
        double rampValue = 0;
	    int desiredEncoderTicks = 0;
	    int encoderTicksDifference = 0;
	    double talonOutput = 0;

		rampValue = encoderTicksPerRotation / 360 * 20;
		
		if(rotationDistance != 0) {
			if(rotationDirection == true) {
				
				desiredEncoderTicks = (int) (encoderTicks + Math.round(encoderTicksPerRotation / 360 * rotationDistance));
				encoderTicksDifference = desiredEncoderTicks - encoderTicks;
				
				if(encoderTicksDifference < rampValue) {
					
					talonOutput = Algorithms.scale(encoderTicksDifference, 0, rampValue, 0, 1);
				}else {
					
					talonOutput = 1;
				}
			}else {
				
				desiredEncoderTicks = (int) (encoderTicks - Math.round(encoderTicksPerRotation / 360 * rotationDistance));
				encoderTicksDifference = desiredEncoderTicks - encoderTicks;

				if(encoderTicksDifference > -rampValue) {
					
					talonOutput = Algorithms.scale(encoderTicksDifference, 0, -rampValue, 0, -1);
				}else {
					
					talonOutput = -1;
				}
			}						
		}else {
			talonOutput = 0;
		}
		
		return talonOutput;
	}

}               