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

public class Climber extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */

  VictorSPX winch = new VictorSPX(Constants.winch);
  
  
  public Climber() {
   
  }

  public void climb(double power) {
    winch.set(ControlMode.PercentOutput, power);
    
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}
