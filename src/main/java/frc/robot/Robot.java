/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import ch.fridolinsrobotik.utilities.Algorithms;
import ch.fridolinsrobotik.utilities.Deadzone;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Joystick.ButtonType;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.CLiftingUnitSetHeight;
import frc.robot.subsystems.SCargoGripper;
import frc.robot.subsystems.SCart;
import frc.robot.subsystems.SHatchGripper;
import frc.robot.subsystems.SLiftingUnit;
import frc.robot.subsystems.SSwerve;
import frc.robot.subsystems.test.TestSCart;
import frc.robot.subsystems.test.TestSubsystem;

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

  // Create Subsystems
  public static SCargoGripper cargoGripper;
  public static SHatchGripper hatchGripper;
  public static SCart cart;
  public static TestSCart testCart;
  public static SLiftingUnit liftingUnit;
  public static SSwerve swerveDrive;
  public static CLiftingUnitSetHeight liftingUnitSetHeight;

  // Shuffleboard
  public static ShuffleboardTab shuffleSettings = Shuffleboard.getTab("Settings");
  public static ShuffleboardTab shuffleSubsystems = Shuffleboard.getTab("Subsystems");
  public static NetworkTable raspberry = NetworkTableInstance.getDefault().getTable("RaspberryPIControlSystem");

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
      swerveDrive = new SSwerve();
    }
    if (RobotMap.LIFTING_UNIT_SUBSYSTEM_IS_IN_USE) {
      liftingUnit = new SLiftingUnit();
      liftingUnitSetHeight = new CLiftingUnitSetHeight();
    }
    if (RobotMap.CART_SUBYSTEM_IS_IN_USE) {
      cart = new SCart();
    }

    oi = OI.getInstance();

    PDP = new PowerDistributionPanel(62);

    try {
      ahrs = new AHRS(SPI.Port.kMXP);
    } catch (RuntimeException ex) {
      System.out.println("Error instantiating navX-MXP:  " + ex.getMessage());
    }
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
    Scheduler.getInstance().run();
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
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    if(RobotMap.LIFTING_UNIT_SUBSYSTEM_IS_IN_USE) {
      liftingUnitSetHeight.start();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();

    System.out.println(OI.JoystickSupportDriver.getPOV());

    double joystickX = OI.JoystickMainDriver.getX();
    double joystickY = -OI.JoystickMainDriver.getY();
    double joystickZ = OI.JoystickMainDriver.getZ();

    double joystickYsupport = Deadzone.getAxis(-OI.JoystickSupportDriver.getY(Hand.kLeft), RobotMap.DEADZONE_RANGE);
    double joystickXsupport = Deadzone.getAxis(OI.JoystickSupportDriver.getX(Hand.kLeft), RobotMap.DEADZONE_RANGE);
    double joystickZrotateSupport = Deadzone.getAxis(-OI.JoystickSupportDriver.getRawAxis(3), RobotMap.DEADZONE_RANGE);

    if (RobotMap.SWERVE_DRIVE_SUBSYSTEM_IS_IN_USE) {
      swerveDrive.manualDrive(joystickX, joystickY, joystickZ, ahrs.getYaw());
    }

    if (RobotMap.LIFTING_UNIT_SUBSYSTEM_IS_IN_USE) {
      if(OI.JoystickSupportDriver.getPOV(RobotMap.SUPPORT_POV_CHANNEL_ID) == 0) {
        liftingUnit.setMotionMagicEnabled(true);
        // liftingUnit.setTargetPosition(Algorithms.limit(joystickZrotateSupport, 0, 1) * 5000);
        liftingUnit.setTargetPosition(7500);
        liftingUnit.drive();
      } else {
        liftingUnit.setMotionMagicEnabled(false);
        liftingUnit.drive(joystickZrotateSupport);
      }
      // System.out.println(joystickZrotateSupport);
    }
    if (RobotMap.CART_SUBSYSTEM_IS_IN_USE) {
      if (OI.JoystickSupportDriver.getRawButton(1)) {
        cart.setPosition(
            Algorithms.limit(joystickYsupport * RobotMap.CART_DRIVE_LENGTH_MM, 0, RobotMap.CART_DRIVE_LENGTH_MM));
            cart.drive();
      } else {
        cart.drive(joystickYsupport);
      }
    }
  }

}
