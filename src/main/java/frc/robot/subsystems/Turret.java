/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Turret extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  CANSparkMax turret = new CANSparkMax(Constants.turret, MotorType.kBrushed);
  CANPIDController turretPID = new CANPIDController(turret);
  CANEncoder turretEncoder = turret.getAlternateEncoder();

  public Turret() {
    double kP = 0;
    double kI = 0;
    double kD = 0;
    turretPID.setP(kP);
    turretPID.setI(kI);
    turretPID.setD(kD);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}
