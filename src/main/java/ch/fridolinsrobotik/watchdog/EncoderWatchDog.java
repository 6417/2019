/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ch.fridolinsrobotik.watchdog;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class EncoderWatchDog {

    private double runTime, timeStarted, encoderTicksStarted, timeOut, minimumEncoderTicks;
    private boolean isActive;
    private String encoderName;
    
    /**
     * A watchdog to detect malicious Enoder
     * @param timeOut timeout in seconds within the encoder ticks must ...
     * @param minimumEncoderTicks encoderticks window which the encoder must leave to be healthy
     */
    public EncoderWatchDog(String encoderName, double timeOut, double minimumEncoderTicks) {
      this.isActive = false;
      this.encoderName = encoderName;
      this.timeStarted = 0;
      this.timeOut = timeOut;
      this.minimumEncoderTicks = minimumEncoderTicks;
    }

    //Method withelse a Encoder position to go to, used for Grippers
    public boolean healthy(double encoderTicks) {
      if(!this.isActive) {
        this.isActive = true;
        timeStarted = Timer.getFPGATimestamp();
        this.encoderTicksStarted = encoderTicks;
        return true;
      }

      runTime = Timer.getFPGATimestamp() - timeStarted;

      //Give the Data to the SmartDashboard for checking the values
      SmartDashboard.putNumber("TimeStarted", timeStarted);
      SmartDashboard.putNumber("RunTime", runTime);
      SmartDashboard.putNumber("EncoderTicksStarted", encoderTicksStarted);
      SmartDashboard.putNumber("EncoderTicks", encoderTicks);
  
      //Encoder does not count any ticks
      if(runTime >= timeOut && encoderTicks >= encoderTicksStarted - minimumEncoderTicks && encoderTicks <= encoderTicksStarted + minimumEncoderTicks) {
        /* report error */
        DriverStation.reportError("Encoder " + encoderName + " faulty!", false);
        return false;
      }
  
      //Encoder is working
      return true;
    }

    public void deactivate() {
      this.isActive = false;
    }

}
