package frc.robot.auto;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class DriveTrajectory extends CommandBase {

   
    private final Drivetrain drivetrain;
    Trajectory trajectory;
    RamseteCommand command;
    boolean isFinished;

    public DriveTrajectory(Drivetrain drivetrain, Trajectory trajectory){
        this.trajectory = trajectory;
        this.drivetrain = drivetrain;
    }

    @Override
    public void initialize() {
        System.out.println("Trajectory Started");
        isFinished = false;
        new RamseteCommand(
                trajectory,
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
                drivetrain
        );
    }


    @Override
    public void execute() {
        if(command.isFinished()){
            isFinished = true;
        }
        System.out.println("Running Path");
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.tankDriveVolts(0,0);
        System.out.println("Trajectory Finished");
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}