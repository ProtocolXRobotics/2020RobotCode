/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  public Drivetrain() {
    CANSparkMax leftMotor1 = new CANSparkMax(Constants.masterLeftDrive, MotorType.kBrushless);
    CANSparkMax leftMotor2 = new CANSparkMax(Constants.slaveLeftDrive, MotorType.kBrushless);
    CANSparkMax rightMotor1 = new CANSparkMax(Constants.masterRightDrive, MotorType.kBrushless);
    CANSparkMax rightMotor2 = new CANSparkMax(Constants.slaveRightDrive, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
