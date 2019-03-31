/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.kauailabs.navx.frc.AHRS;

import ch.fridolinsrobotik.drivesystems.swerve.SwerveDrive;
import ch.fridolinsrobotik.utilities.Algorithms;
import ch.fridolinsrobotik.utilities.Deadzone;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SCargoGripper;
import frc.robot.subsystems.SCart;
import frc.robot.subsystems.SHatchGripper;
import frc.robot.subsystems.SLiftingUnit;
import frc.robot.subsystems.SRemoteControl;
import frc.robot.subsystems.SSwerve;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static OI oi;
  public static AHRS ahrs;
  public static PowerDistributionPanel PDP;

  //Variables
  private int lastPos;
  private double liftingBreak = 0;

  private boolean cameraDrive = false;

  // Create Subsystems
  public static SCargoGripper cargoGripper;
  public static SHatchGripper hatchGripper;
  public static SCart cart;
  public static SLiftingUnit liftingUnit;
  public static SRemoteControl remoteControl;
  public static SSwerve swerveDrive;
  public static SwerveDrive swerve = new SwerveDrive();
  
  // Shuffleboard
  public static ShuffleboardTab shuffleSettings = Shuffleboard.getTab("Settings");
  public static ShuffleboardTab shuffleSubsystems = Shuffleboard.getTab("Subsystems");
  public static NetworkTable raspberry = NetworkTableInstance.getDefault().getTable("RaspberryPIControlSystem");
  public static NetworkTable vision = NetworkTableInstance.getDefault().getTable("vision");


  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {

    Motors.initialize();

    if (RobotMap.CARGO_GRIPPER_SUBSYSTEM_IS_IN_USE) {
      cargoGripper = new SCargoGripper();
    }
    if (RobotMap.HATCH_GRIPPER_SUBSYSTEM_IS_IN_USE) {
      hatchGripper = new SHatchGripper();
    }
    if (RobotMap.SWERVE_DRIVE_SUBSYSTEM_IS_IN_USE) {
      swerve.setSpeedFactor(RobotMap.DRIVE_SPEED_MULITPLIER);
      swerveDrive = new SSwerve();
    }
    if (RobotMap.CART_SUBSYSTEM_IS_IN_USE) {
      cart = new SCart();
    }
    if (RobotMap.LIFTING_UNIT_SUBSYSTEM_IS_IN_USE) {
      liftingUnit = new SLiftingUnit();
    }
    if(RobotMap.LIFTING_UNIT_SUBSYSTEM_IS_IN_USE && RobotMap.CART_SUBSYSTEM_IS_IN_USE) {
      remoteControl = new SRemoteControl();
    }

    oi = OI.getInstance();

    PDP = new PowerDistributionPanel(62);

    try {
      ahrs = new AHRS(SPI.Port.kMXP);
    } catch (RuntimeException ex) {
      System.out.println("Error instantiating navX-MXP:  " + ex.getMessage());
    }

    CameraServer.getInstance().addAxisCamera("Hatch", "10.64.17.6");
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode. You
   * can use it to reset any subsystem information you want to clear when the
   * robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    // Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString code to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons to
   * the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    raspberry.getEntry("timeStart").setBoolean(true);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    double joystickX = OI.JoystickMainDriver.getX();
    double joystickY = -OI.JoystickMainDriver.getY();
    double joystickZ = OI.JoystickMainDriver.getZ();
    
    swerveDrive.manualDrive(joystickX, joystickY, -joystickZ, ahrs.getYaw());
  }

  @Override
  public void teleopInit() {
    if(RobotMap.LIFTING_UNIT_SUBSYSTEM_IS_IN_USE) {
      liftingUnit.enableAutonomous(true);
      cart.enableAutonomous(true);
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    // Scheduler.getInstance().run();
    // double t1 = Timer.getFPGATimestamp();
    // DriverStation.reportWarning(Double.valueOf(Timer.getFPGATimestamp() - t1).toString(), false);
    double joystickX = OI.JoystickMainDriver.getX();
    double joystickY = -OI.JoystickMainDriver.getY();
    double joystickZ = OI.JoystickMainDriver.getZ();

    double joystickYsupport = Deadzone.getAxis(-OI.JoystickSupportDriver.getY(Hand.kLeft), RobotMap.DEADZONE_RANGE);
    double joystickXsupport = Deadzone.getAxis(OI.JoystickSupportDriver.getX(Hand.kLeft), RobotMap.DEADZONE_RANGE);
    double joystickZrotateSupport = Deadzone.getAxis(-OI.JoystickSupportDriver.getRawAxis(3), RobotMap.DEADZONE_RANGE);

    if (RobotMap.SWERVE_DRIVE_SUBSYSTEM_IS_IN_USE) {
      // if (OI.JoystickMainDriver.getRawButton(1)) {
      //   if (vision.getEntry("targetDetected").getBoolean(false)) {
      //     double visionInput = vision.getEntry("targetDistance").getDouble(0);
      //     visionInput = Math.tanh(visionInput / 320 * 1.5) / 2;
      //     System.out.println(visionInput);
      //     swerveDrive.manualDrive(joystickX, joystickY, visionInput, 0);
      //   } else {
      //     swerveDrive.manualDrive(joystickX, joystickY, 0, 0);
      //   }
      // }
      swerveDrive.manualDrive(joystickX, joystickY, -joystickZ, ahrs.getYaw());
    }

    if (RobotMap.ROBOT_ELEVATOR_SUBSYSTEM_IN_USE && RobotMap.LIFTING_UNIT_SUBSYSTEM_IS_IN_USE) {

      double liftMasterOutput = 0;
      double elevatorRightOutput = 0;
      double elevatorLeftOutput = 0;

      if(!Motors.robotElevatorLeft.getSensorCollection().isRevLimitSwitchClosed()) {
        Motors.robotElevatorLeft.getSensorCollection().setQuadraturePosition(0,0);
      }
      if(!Motors.robotElevatorRight.getSensorCollection().isRevLimitSwitchClosed()) {
        Motors.robotElevatorRight.getSensorCollection().setQuadraturePosition(0,0);
      }
      // double tanh = Math.tanh(diffrenceLiftingElevator / 2000);
      final double liftingUnitElevatorRatio = 100.0;
      double dLM = Math.tanh(
          (
              (Motors.robotElevatorRight.getSelectedSensorPosition(0) / liftingUnitElevatorRatio)
              - (8300 - liftingUnit.getPosition())
          ) / 500.0) * 0.5;
      double levelingElevator = Math.tanh(
        (Motors.robotElevatorRight.getSelectedSensorPosition(0) - Motors.robotElevatorLeft.getSelectedSensorPosition(0))
         / liftingUnitElevatorRatio / 500.0) * 0.5;
      SmartDashboard.putNumber("levelingElevator ", levelingElevator);
      SmartDashboard.putNumber(" dLM ", dLM);
      SmartDashboard.putNumber(" Elevator dif: ", (Motors.robotElevatorRight.getSelectedSensorPosition(0) - Motors.robotElevatorLeft.getSelectedSensorPosition(0)) / liftingUnitElevatorRatio / 2000.0);
      // Motors.liftMaster.set(ControlMode.PercentOutput, joystickZrotateSupport - tanh + tanh2);
      if(OI.JoystickSupportDriver.getPOV(RobotMap.SUPPORT_POV_CHANNEL_ID) == 270) {
        Motors.liftMaster.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.Disabled);
        if(liftingUnit.getPosition() <= 1000) {
          // liftMasterOutput = - joystickZrotateSupport + liftingBreak;
          // elevatorRightOutput =  joystickZrotateSupport - liftingBreak;
          // elevatorLeftOutput = joystickZrotateSupport - liftingBreak;
          // double normalizeFactor = 1;
          // Double[] outputs = {liftMasterOutput, elevatorRightOutput, elevatorLeftOutput};
          // for(Double output : outputs) {
          //   normalizeFactor = Math.max(Math.abs(output), normalizeFactor);
          // }
          Motors.liftMaster.set(ControlMode.Position, 0, DemandType.ArbitraryFeedForward, -0.6);
          Motors.robotElevatorRight.set(ControlMode.PercentOutput, joystickZrotateSupport);
          Motors.robotElevatorLeft.set(ControlMode.PercentOutput, joystickZrotateSupport);
        } else {
          liftMasterOutput = - joystickZrotateSupport - dLM + levelingElevator;
          elevatorRightOutput =  joystickZrotateSupport - dLM - levelingElevator;
          elevatorLeftOutput = joystickZrotateSupport - dLM + levelingElevator;

          double normalizeFactor = 1;
          Double[] outputs = {liftMasterOutput, elevatorRightOutput, elevatorLeftOutput};
          for(Double output : outputs) {
            normalizeFactor = Math.max(Math.abs(output), normalizeFactor);
          }
          SmartDashboard.putNumber("Normalize Factor", normalizeFactor);
          SmartDashboard.putNumber("LU Output", liftMasterOutput / normalizeFactor);
          SmartDashboard.putNumber("ELR Output", elevatorRightOutput / normalizeFactor);
          SmartDashboard.putNumber("ELL Output", elevatorLeftOutput /normalizeFactor);
          SmartDashboard.putNumber("Break", liftingBreak);
          Motors.liftMaster.set(ControlMode.PercentOutput, liftMasterOutput / normalizeFactor);
          Motors.robotElevatorRight.set(ControlMode.PercentOutput, elevatorRightOutput / normalizeFactor);
          Motors.robotElevatorLeft.set(ControlMode.PercentOutput, elevatorLeftOutput / normalizeFactor);
        }
       
      } else if(OI.JoystickSupportDriver.getPOV(RobotMap.SUPPORT_POV_CHANNEL_ID) == 180) {
        Motors.robotElevatorRight.set(ControlMode.PercentOutput, joystickZrotateSupport);
        Motors.robotElevatorLeft.set(ControlMode.PercentOutput, joystickYsupport);
      } else {

      }

      // Motors.robotElevatorLeft.set(ControlMode.PercentOutput, joystickYsupport);
      
      // Motors.robotElevatorRight.set(ControlMode.Position, Algorithms.limit(joystickZrotateSupport * 450000, 0, 450000), DemandType.AuxPID, 0);
      // Motors.robotElevatorLeft.follow(Motors.robotElevatorRight, FollowerType.AuxOutput1);

      if(liftingUnit.getPosition() <= 1000) {
        if(liftingUnit.getPosition() >= 500) {
          if(liftMasterOutput > 0) {
            liftingBreak = liftingBreak - (Math.abs(lastPos) - Math.abs(liftingUnit.getPosition())) * 0.2;  
          } else {
          liftingBreak = liftingBreak + (Math.abs(lastPos) - Math.abs(liftingUnit.getPosition())) * 0.2;
          }
          liftingBreak = Algorithms.limit(liftingBreak, -1, 1);
        } else {
          liftingBreak = 1;
        }
      } else {
        liftingBreak = 0;
      }
      System.out.print(" Lifting Break: " + liftingBreak);
      System.out.println(" Lifting Unit Position: " + liftingUnit.getPosition());
      lastPos = liftingUnit.getPosition();

    }

    if (RobotMap.LIFTING_UNIT_SUBSYSTEM_IS_IN_USE) {

      if(OI.JoystickSupportDriver.getPOV(RobotMap.SUPPORT_POV_CHANNEL_ID) == 0) {
        if(liftingUnit.drive_autonomous == true) {
          liftingUnit.drive_autonomous = false;
          liftingUnit.stopMotor();
        }
        liftingUnit.enableAutonomous(false);
        liftingUnit.setMaximumLoweringSpeed(-0.3);
        liftingUnit.setMaximumRaiseSpeed(0.4);
        liftingUnit.drive(joystickZrotateSupport);
      } else {
        liftingUnit.enableAutonomous(true);
        if(liftingUnit.drive_manual == true) {
          liftingUnit.drive_manual = false;
          liftingUnit.setTargetPosition(liftingUnit.getPosition());
          liftingUnit.drive();
        }
        // // liftingUnit.setTargetPosition(Algorithms.limit(joystickZrotateSupport, 0, 1) * 5000);
        // liftingUnit.setTargetPosition(7500);
        // liftingUnit.drive();
      }
    }

    if (RobotMap.CART_SUBSYSTEM_IS_IN_USE) {
      if (OI.JoystickSupportDriver.getPOV(RobotMap.SUPPORT_POV_CHANNEL_ID) == 90) {
        if(cart.drive_autonomous == true) {
          cart.drive_autonomous = false;
          cart.stop();
        }
        cart.enableAutonomous(false);
        cart.drive(joystickZrotateSupport / 4);
      } else {
        cart.enableAutonomous(true);
        if(cart.drive_manual == true) {
          cart.drive_manual = false;
          cart.setPosition((int)cart.getPosition());
          cart.drive();
        }
      }
    }

  }

}
