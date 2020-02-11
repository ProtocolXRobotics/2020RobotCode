/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */

  VictorSPX leftIndexer = new VictorSPX(Constants.leftIndexer);
  VictorSPX rightIndexer = new VictorSPX(Constants.rightIndexer);

  public Indexer() {
   
  }

  public void setPower(double leftPow, double rightPow) {
    leftIndexer.set(ControlMode.PercentOutput, leftPow);
    rightIndexer.set(ControlMode.PercentOutput, rightPow);
    
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}
