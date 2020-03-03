/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.CurvatureDrive;
import frc.robot.commands.Hopper;
import frc.robot.commands.SetIntakeSpeed;
import frc.robot.commands.SetShooterSpeed;
import frc.robot.commands.ToggleLight;
import frc.robot.commands.Unjam;
import frc.robot.subsystems.Beltevator;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain drivetrain = new Drivetrain();
  private final Intake intake = new Intake();
  private final Shooter shooter = new Shooter();
  private final Beltevator beltevator = new Beltevator();
  //private final Climber climber = new Climber();
  private final Indexer indexer = new Indexer();
  private final Limelight limelight = new Limelight();

  private final XboxController driver = new XboxController(0);
  private final XboxController operator = new XboxController(1);

  

 
  private ArcadeDrive arcadeDrive = new ArcadeDrive(driver, drivetrain);

                                
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    drivetrain.setDefaultCommand(arcadeDrive);
    configureButtonBindings();
    
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(operator, Button.kBumperLeft.value)
        .whenPressed(new InstantCommand(intake::actuateIntake, intake));

    new JoystickButton(operator, Button.kBumperRight.value)
        .toggleWhenPressed(new SetIntakeSpeed(intake, 0.6));

    new JoystickButton(operator, Button.kY.value)
        .whileHeld(new Hopper(beltevator,indexer));
    
    new JoystickButton(operator, Button.kX.value)
        .whileHeld(new SetShooterSpeed(shooter, 2500));

    new JoystickButton(operator, Button.kB.value)
        .whileHeld(new SetShooterSpeed(shooter, 4000)); 
        
        
    new JoystickButton(driver, Button.kBumperLeft.value)
        .whileHeld(new ToggleLight(limelight));  

    new JoystickButton(operator, Button.kA.value)
        .whileHeld(new Unjam(beltevator, indexer));  
    
    
    
  }



  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    /*
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(Constants.ksVolts,
                                       Constants.kvVoltSecondsPerMeter,
                                       Constants.kaVoltSecondsSquaredPerMeter),
            Constants.kDriveKinematics,
            10);
?
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
*/  Command baseline = new CurvatureDrive(-0.5, 0, drivetrain).withTimeout(2);

    return baseline;
  }
}
