/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ch.fridolinsrobotik.sensors.utils;

/**
 * Add your docs here.
 */
public class EncoderConverter {

    private double distancePerPulse = 1;

    public EncoderConverter(double ditsancePerPulse) {
        super();
        setDistancePerPulse(distancePerPulse);
    }

    public void setDistancePerPulse(double distance) {
        this.distancePerPulse = distance;
    }

    public double getDistancePerPulse() {
        return this.distancePerPulse;
    }

    public double getDistance(double encoderPulses) {
        return encoderPulses * this.distancePerPulse;
    }
    
}
