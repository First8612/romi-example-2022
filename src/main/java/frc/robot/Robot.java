// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  private final int axis_sideToSide = 0;
  private final int axis_forwardBack = 1;
  private final int axis_rotate = 2;
  private final int axis_throttle = 3;
  private final PWMSparkMax m_leftDrive = new PWMSparkMax(0);
  private final PWMSparkMax m_rightDrive = new PWMSparkMax(1);
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftDrive, m_rightDrive);
  private final GenericHID m_controller = new Joystick(0);
  private final Timer m_timer = new Timer();
  private final JoystickButton m_boostButton = new JoystickButton(m_controller, 1);
  private final JoystickButton m_spinLeftButton = new JoystickButton(m_controller, 3);
  private final JoystickButton m_spinRightButton = new JoystickButton(m_controller, 4);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightDrive.setInverted(true);
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    m_timer.reset();
    m_timer.start();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // Drive for 2 seconds
    if (m_timer.get() < 2.0) {
      m_robotDrive.arcadeDrive(0.5, 0.0); // drive forwards half speed
    } else {
      m_robotDrive.stopMotor(); // stop robot
    }
  }

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {
  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    double driveFactor = 
      (m_controller.getRawAxis(axis_throttle) // will be -1 to 1
      * -1 // reverse (forward is faster)
      + 1) // 0 to 2
      / 2; // 0 to 1

    if (m_spinRightButton.get()) {
      m_robotDrive.arcadeDrive(0, 1);
    } else if (m_spinLeftButton.get()) {
      m_robotDrive.arcadeDrive(0, -1);
    } else {

      if (m_boostButton.get()) {
        driveFactor = 1;
      }

      m_robotDrive.arcadeDrive(m_controller.getRawAxis(axis_forwardBack) * driveFactor, m_controller.getRawAxis(axis_rotate) * 0.5);
    }

  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
