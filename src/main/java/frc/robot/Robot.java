/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import ch.fridolinsrobotik.drivesystems.swerve.SwerveDrive;
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
import frc.robot.subsystems.SRobotElevator;
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
  public static SRobotElevator elevator;
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
    if(RobotMap.ROBOT_ELEVATOR_SUBSYSTEM_IN_USE && RobotMap.LIFTING_UNIT_SUBSYSTEM_IS_IN_USE) {
      elevator = new SRobotElevator();
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

      SmartDashboard.putNumber("levelingElevator ", elevator.calculateLevelingElevator());
      SmartDashboard.putNumber(" dLM ", elevator.calculateDLM());
      SmartDashboard.putNumber(" Elevator dif: ", (Motors.robotElevatorRight.getSelectedSensorPosition(0) - Motors.robotElevatorLeft.getSelectedSensorPosition(0)) / elevator.getLiftingUnitElevatorRatio() / 2000.0);

      if(OI.JoystickSupportDriver.getPOV(RobotMap.SUPPORT_POV_CHANNEL_ID) == 270) {

        elevator.elevate(joystickZrotateSupport);

          SmartDashboard.putNumber("Normalize Factor", elevator.getNormalizeFactor());
          SmartDashboard.putNumber("LU Output", elevator.getLiftMasterOutput() / elevator.getNormalizeFactor());
          SmartDashboard.putNumber("ELR Output", elevator.getElevatorRightOutput() / elevator.getNormalizeFactor());
          SmartDashboard.putNumber("ELL Output", elevator.getElevatorLeftOutput() /elevator.getNormalizeFactor());
          SmartDashboard.putNumber("Break", elevator.getLiftingBreak());
       
      } else if(OI.JoystickSupportDriver.getPOV(RobotMap.SUPPORT_POV_CHANNEL_ID) == 180) {
        elevator.moveElevators(joystickZrotateSupport, joystickYsupport);
      } else {

      }

      System.out.print(" Lifting Break: " + elevator.calculateBreak());
      System.out.println(" Lifting Unit Position: " + liftingUnit.getPosition());
    
    }

    if (RobotMap.LIFTING_UNIT_SUBSYSTEM_IS_IN_USE) {

      if(OI.JoystickSupportDriver.getPOV(RobotMap.SUPPORT_POV_CHANNEL_ID) == 0) {
        if(liftingUnit.drive_autonomous == true) {
          liftingUnit.drive_autonomous = false;
          liftingUnit.stopMotor();
        }
        liftingUnit.enableAutonomous(false);
        liftingUnit.setMaximumLoweringSpeed(-0.5);
        liftingUnit.setMaximumRaiseSpeed(0.7);
        liftingUnit.drive(joystickZrotateSupport);
      } else {
        // if(liftingUnit.isZeroed()) {
          liftingUnit.enableAutonomous(true);
          if(liftingUnit.drive_manual == true) {
            liftingUnit.drive_manual = false;
            liftingUnit.setTargetPosition(liftingUnit.getPosition());
            liftingUnit.drive();
          }
        // }
      }
    }

    if (RobotMap.CART_SUBSYSTEM_IS_IN_USE) {
      cart.drive(joystickZrotateSupport / 4.0);
    }

  }

}
