/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;


/**
 * An example command that uses an example subsystem.
 */

public class LimelightShoot extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Limelight limelight;
  private final Shooter shooter;
  public LimelightShoot(Limelight limelight, Shooter shooter) {
    this.limelight = limelight;
    this.shooter = shooter;
  
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(limelight, shooter);
  }

  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double RPM = limelight.formulaRpm();
    shooter.setVelocity(RPM);
    
    

  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setPower(0.0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}