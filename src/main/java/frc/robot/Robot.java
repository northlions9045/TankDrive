// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;


/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
 private WPI_VictorSPX leftMaster = new WPI_VictorSPX(3);
 private WPI_VictorSPX leftSlave = new WPI_VictorSPX(4);
 private WPI_VictorSPX rightMaster = new WPI_VictorSPX(1);
 private WPI_VictorSPX rightSlave = new WPI_VictorSPX(2);
 private XboxController joy1 = new XboxController(0);
 private DifferentialDrive drive = new DifferentialDrive(rightMaster, leftMaster);
 private Encoder lula = new Encoder(0, 1);
 private Encoder bolsonaro = new Encoder(2, 3);
 private final ShuffleboardTab encoderTab =
    Shuffleboard.getTab("Encoders");
 private final GenericEntry leftEncoderEntry =
    encoderTab.add("Left Encoder", 0).getEntry();

 private final GenericEntry rightEncoderEntry =
    encoderTab.add("Right Encoder", 0).getEntry();

  private final double wheelDiameter = 0.1524;
  private final double ticksPerRevolution = 8192;

  private final double metersPerTick =
    (Math.PI * wheelDiameter) / ticksPerRevolution;
    
  public Robot() {}

  @Override
  public void robotInit() {
    leftMaster.setInverted(false);
    rightMaster.setInverted(true);

    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);

    leftSlave.setInverted(InvertType.FollowMaster);
    rightSlave.setInverted(InvertType.FollowMaster);

    lula.setDistancePerPulse(metersPerTick);
    bolsonaro.setDistancePerPulse(metersPerTick);
    



  }
  
  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {

}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    double speed = -joy1.getLeftY() * 1;
    double turn = -joy1.getRightX() * 1;
    drive.arcadeDrive(speed, turn);

    
    leftEncoderEntry.setDouble(lula.get());
    rightEncoderEntry.setDouble(bolsonaro.get());
    
    leftEncoderEntry.setDouble(lula.getDistance());
    rightEncoderEntry.setDouble(bolsonaro.getDistance());
    
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
