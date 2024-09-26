// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import com.pathplanner.lib.commands.PathPlannerAuto;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  // The driver's controller
  public XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  public void setSwerveAngleVelocity(double velocity, double angle) {
    SwerveModuleState desiredState = new SwerveModuleState(velocity, Rotation2d.fromDegrees(angle));
    m_robotDrive.setSwerveDesiredState(desiredState);
  }  
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> {
                double ROBOT_ROTATION = 0.0 - 90; // REPLACE WITH ACTUAL ROTATION

                // Get joystick inputs
                double forward = m_driverController.getLeftY(); // Forward/backward movement
                double sideways = m_driverController.getLeftX(); // Left/right movement
                double rotation = m_driverController.getRightX(); // Rotation

                if(!isOutsideDeadzone(rotation,ROTATION_DEADZONE)) rotation = 0;
                if(!isOutsideDeadzone(sideways,POSITION_DEADZONE) && !isOutsideDeadzone(forward,POSITION_DEADZONE)) {
                    sideways = 0;
                    forward = 0;
                }


                // Calculate desired speeds
                double speedX = -sideways * DriveConstants.kMaxSpeedMetersPerSecond; // Move west
                double speedY = forward * DriveConstants.kMaxSpeedMetersPerSecond; // Use as needed (could be 0 for direct west)
                double rotationRate = -rotation * ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond; // Rotation speed

                // Create ChassisSpeeds object for field-oriented control
                ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speedX, speedY, rotationRate, Rotation2d.fromDegrees(ROBOT_ROTATION));
                
                // Convert to module states
                // Set module states (implement as needed)
                // m_robotDrive.setModuleStates(moduleStates);
                m_robotDrive.driveRelative(speeds); // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            },
            m_robotDrive));
  }
  double POSITION_DEADZONE = 0.2; // Adjust position deadzone as needed
  double ROTATION_DEADZONE = 0.1; // Adjust rotation deadzone as needed

  public boolean isOutsideDeadzone(double value, double deadzone) {
    return Math.abs(value) >= deadzone;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * it is highly recommended to create all of your autos when code starts, instead of creating them when you want to run them. 
   * Large delays can happen when loading complex autos/paths, so it is best to load them before they are needed.
   * In the interest of simplicity, this example will show an auto being loaded in the getAutonomousCommand function,
   * which is called when auto is enabled. 
   * This is not the recommended way to load your autos.
   */
  public Command getAutonomousCommand() {
    return new PathPlannerAuto("Pick Up Rings");
  }
}
