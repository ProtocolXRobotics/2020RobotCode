package frc.robot.auto;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.SetIntakeSpeed;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;


public class Trench8Ball extends SequentialCommandGroup {
    /**
     * Add your docs here.
     */
    public Trench8Ball(Drivetrain drivetrian, Intake intake, Shooter shooter, Trajectories trajectories) {
        addCommands(
                new ParallelDeadlineGroup(
                new WaitCommand(3.5), 
                new AutoShoot(shooter, 3000)),
                //Intake in Parallel
                new ParallelDeadlineGroup(
                        new RamseteCommand(
                        trajectories.getCenterStartToEndOfTrench(),
                        drivetrian::getPose,
                        new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
                        new SimpleMotorFeedforward(Constants.ksVolts,
                                Constants.kvVoltSecondsPerMeter,
                                Constants.kaVoltSecondsSquaredPerMeter),
                        Constants.kDriveKinematics,
                        drivetrian::getWheelSpeeds,
                        new PIDController(Constants.kPDriveVel, 0, Constants.kDDriveVel),
                        new PIDController(Constants.kPDriveVel, 0, Constants.kDDriveVel),
                        // RamseteCommand passes volts to the callback
                        drivetrian::tankDriveVolts,
                        drivetrian), 
                        new SetIntakeSpeed(intake, 0.8)),
                new RamseteCommand(
                        trajectories.getEndOfTrenchToStartOfTrench(),
                        drivetrian::getPose,
                        new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
                        new SimpleMotorFeedforward(Constants.ksVolts,
                                Constants.kvVoltSecondsPerMeter,
                                Constants.kaVoltSecondsSquaredPerMeter),
                        Constants.kDriveKinematics,
                        drivetrian::getWheelSpeeds,
                        new PIDController(Constants.kPDriveVel, 0, Constants.kDDriveVel),
                        new PIDController(Constants.kPDriveVel, 0, Constants.kDDriveVel),
                        // RamseteCommand passes volts to the callback
                        drivetrian::tankDriveVolts,
                        drivetrian)
        );
    }
}