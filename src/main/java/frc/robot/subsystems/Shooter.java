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

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  CANSparkMax masterShooter = new CANSparkMax(Constants.masterShooter, MotorType.kBrushless);
  CANSparkMax slaveShooter = new CANSparkMax(Constants.slaveShooter, MotorType.kBrushless);
  CANPIDController shooterPID = masterShooter.getPIDController();
  CANEncoder shooterEnc = masterShooter.getEncoder();
  double kS = 0.132;
  double kV = 0.7;
  double kA = 0.123;

  private final SimpleMotorFeedforward motorFeedForward = 
      new SimpleMotorFeedforward(kS, kV, kA);

  public Shooter() {
    double kP, kI, kD, kF;
    
    kP = 0.00175; // p = .0005 (with target at x3/2)
    kI = 2.2e-7;
    kD = 1e-11;
    kF = 0;
    slaveShooter.follow(masterShooter, true);
    masterShooter.setInverted(true);
  
    shooterPID.setP(kP);
    shooterPID.setI(kI);
    shooterPID.setD(kD);
    shooterPID.setFF(kF);

    shooterPID.setOutputRange(0, 1);
    masterShooter.setSmartCurrentLimit(40);
    masterShooter.setClosedLoopRampRate(0.2);
    
    
  }

  public void setVelocity(double RPM) {
    shooterPID.setReference(RPM, ControlType.kVelocity);
    SmartDashboard.putNumber("Error", RPM - shooterEnc.getVelocity());
}

public void setVelocityFeedforward(double RPM) {
  shooterPID.setReference(RPM, ControlType.kVelocity, 0, motorFeedForward.calculate(RPM/60, (RPM-shooterEnc.getVelocity())/60));
}


public void setPower(double power) {
  masterShooter.set(power);
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Velocity", shooterEnc.getVelocity());
   
  }
}
