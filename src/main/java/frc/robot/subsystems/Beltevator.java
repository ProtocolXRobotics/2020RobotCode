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

public class Beltevator extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */

  VictorSPX accelWheel = new VictorSPX(Constants.accelWheel);

  public Beltevator() {
   
  }

  public void spinAccelWheel(double power) {
    accelWheel.set(ControlMode.PercentOutput, power);
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}
