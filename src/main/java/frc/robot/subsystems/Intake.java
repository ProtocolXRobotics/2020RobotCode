/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */

  Solenoid retractor1 = new Solenoid(0);
  //DoubleSolenoid retractor2 = new DoubleSolenoid(1, 3);
  VictorSPX intakeMotor = new VictorSPX(Constants.intakeMotor);

  public Intake() {
   
  }

  public void setPower(double power) {
    intakeMotor.set(ControlMode.PercentOutput, power);
  }
  public void actuateIntake() {
    if(retractor1.get()) {
      retractor1.set(false);
    }
    else {
      retractor1.set(true);

    }
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}

  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
