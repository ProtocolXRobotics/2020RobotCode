/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.commands.TankDrive;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain drive = new Drivetrain();

  private final TankDrive m_autoCommand = new TankDrive(drive);



  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(Constants.ksVolts,
                                       Constants.kvVoltSecondsPerMeter,
                                       Constants.kaVoltSecondsSquaredPerMeter),
            Constants.kDriveKinematics,
            10);

      TrajectoryConfig config =
        new TrajectoryConfig(Constants.maxSpeed,
                             Constants.maxAcceleration)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.kDriveKinematics)
                // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);
            //Trajectory exampleTrajectory = TrajectoryUtil.fromPathweaverJson(Paths.get("/home/lvuser/deploy/output/example.wpilib.json"));

            RamseteCommand ramsyeet = new RamseteCommand(
              exampleTrajectory,
              drive::getPose,
              new RamseteController(Constants.ramseteB, Constants.ramseteZeta),
              new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter,
              Constants.kaVoltSecondsSquaredPerMeter),
              Constants.kDriveKinematics, 
              drive::getWheelSpeeds,
              new PIDController(Constants.kPDriveVel, 0, 0), //Check which pid controller to import
              new PIDController(Constants.kPDriveVel, 0, 0),
              // RamseteCommand passes volts to the callback
              drive::tankDriveVolts, 
              drive
              );     

    return ramsyeet.andThen(() -> drive.tankDriveVolts(0, 0));
  }
}
