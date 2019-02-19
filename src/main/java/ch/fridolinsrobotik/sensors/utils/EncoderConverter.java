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

    private double m_distancePerPulse = 1;

    public EncoderConverter(double distancePerPulse) {
        super();
        setDistancePerPulse(distancePerPulse);
    }

    public void setDistancePerPulse(double distancePerPulse) {
        this.m_distancePerPulse = distancePerPulse;
    }

    public double getDistancePerPulse() {
        return this.m_distancePerPulse;
    }

    public double getDistance(double encoderPulses) {
        return encoderPulses * this.m_distancePerPulse;
    }

    public double getPulses(double distance) {
        return distance / this.m_distancePerPulse;
    }
    
}
