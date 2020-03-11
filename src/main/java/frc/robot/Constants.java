/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static int masterLeftDrive = 5;
    public static int slaveLeftDrive = 1;

    public static int masterRightDrive = 2;
    public static int slaveRightDrive = 4;

    public static int masterShooter = 6;
    public static int slaveShooter = 3;

    public static int intakeMotor = 3;
    public static int leftIndexer = 2;
    public static int rightIndexer = 1;
    public static int accelWheel = 0;


    public static int climb = 11;
    public static int winch = 12;

    
    // Use Drive Characterization to find these constants
    public static final double ksVolts = 0.22;
    public static final double kvVoltSecondsPerMeter = 1.98;
    public static final double kaVoltSecondsSquaredPerMeter = 0.2;
    public static final double kPDriveVel = 8.5;
    public static final double kDDriveVel = 8.5;

    public static final double trackwidth = 2;

    public static final double kMaxSpeedMetersPerSecond = Units.feetToMeters(12.0);; // m/s
    public static final double kMaxAccelerationMetersPerSecondSquared = Units.feetToMeters(8); // m/s^2

    public static final double kMinSpeedMetersPerSecond = Units.feetToMeters(8.0); //Find good value
    public static final double kMinAccelerationMetersPerSecondSquared = Units.feetToMeters(4.0);
    
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(Constants.trackwidth);

    public static final int retractorforward = 0;
    public static final int retractorreverse = 1;



   


}
