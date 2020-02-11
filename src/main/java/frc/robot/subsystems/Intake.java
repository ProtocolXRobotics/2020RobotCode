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
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */

  DoubleSolenoid retractor1 = new DoubleSolenoid(Constants.retractorforward, Constants.retractorreverse);
  DoubleSolenoid retractor2 = new DoubleSolenoid(Constants.retractorforward, Constants.retractorreverse);
  VictorSPX intakeMotor = new VictorSPX(Constants.intakeMotor);

  public Intake() {
   
  }

  public void setPower(double power) {
    intakeMotor.set(ControlMode.PercentOutput, power);
  }
  public void actuateIntake() {
    if(retractor1.get()==Value.kForward) {
      retractor1.set(Value.kReverse);
      retractor2.set(Value.kReverse);
    }
    else if(retractor1.get()==Value.kReverse) {
      retractor1.set(Value.kForward);
      retractor2.set(Value.kForward);
    }
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}
