/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.Beltevator;
import frc.robot.subsystems.Indexer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class Hopper extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Beltevator beltevator;
  private final Indexer indexer;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Hopper(Beltevator beltevator, Indexer indexer) {
    this.beltevator = beltevator;
    this.indexer = indexer;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(indexer, beltevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    beltevator.spinAccelWheel(0.9);
    indexer.setPower(0.5, 0.5);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    beltevator.spinAccelWheel(0);
    indexer.setPower(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
