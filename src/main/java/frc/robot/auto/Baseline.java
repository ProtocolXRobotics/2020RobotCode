package frc.robot.auto;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.ActuateIntake;
import frc.robot.commands.Hopper;
import frc.robot.commands.LimelightAlign;
import frc.robot.commands.SetIntakeSpeed;
import frc.robot.subsystems.Beltevator;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;


public class Baseline extends SequentialCommandGroup {
    /**
     * Add your docs here.
     */
    public Baseline(Drivetrain drivetrain, Intake intake, Shooter shooter, Indexer indexer, Beltevator beltevator, Limelight limelight, Trajectories trajectories) {
        addCommands(
                new AutoShoot(shooter, 1500), //Start spinning shooter
                new ParallelDeadlineGroup(
                new WaitCommand(2), 
                new LimelightAlign(limelight, drivetrain)), // Wait for shooter to spin up and align with limelight
                new Hopper(beltevator, indexer).withTimeout(3), //Unload balls
                new AutoShoot(shooter, 0), // Set shooter back to 0
                new ActuateIntake(intake), // Drop intake out
                //Drive thru trench and intake in Parallel
                new ParallelDeadlineGroup(
                        new RamseteCommand(
                        trajectories.getDriveStraight(),
                        drivetrain::getPose,
                        new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
                        new SimpleMotorFeedforward(Constants.ksVolts,
                                Constants.kvVoltSecondsPerMeter,
                                Constants.kaVoltSecondsSquaredPerMeter),
                        Constants.kDriveKinematics,
                        drivetrain::getWheelSpeeds,
                        new PIDController(Constants.kPDriveVel, 0, Constants.kDDriveVel),
                        new PIDController(Constants.kPDriveVel, 0, Constants.kDDriveVel),
                        // RamseteCommand passes volts to the callback
                        drivetrain::tankDriveVolts,
                        drivetrain), 
                        new SetIntakeSpeed(intake, 0.8)),
                new AutoLimelightShoot(drivetrain, limelight, shooter), //Turn to target and spin up to RPM based off formula
                new WaitCommand(1),
                new Hopper(beltevator, indexer).withTimeout(3), //Unload balls
                new AutoShoot(shooter, 0)
                
        );
    }
}