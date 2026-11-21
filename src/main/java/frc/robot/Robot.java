// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


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
 private DifferentialDrive drive = new DifferentialDrive(leftMaster, rightMaster);
 private Encoder lula = new Encoder(0, 1, true, EncodingType.k4X); //encoder esquerdo (para alterar a orientação de leitura, basta mudar o boolean)
 private Encoder bolsonaro = new Encoder(2, 3, true, EncodingType.k4X); //encoder direito
 private final double kP = 0.5;  //proporcional(PID) LEMBRAR DE AJUSTAR!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 private double setpoint = 4; //LEMBRAR DE AJUSTAR!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 

  private final double wheelDiameter = 0.1524;
  private final double ticksPerRevolution = 8192; 

  private final double metersPerTick =
    (Math.PI * wheelDiameter) / ticksPerRevolution; //conversao de ticks pra metros
    
  public Robot() {}

  @Override
  public void robotInit() {
    leftMaster.setInverted(false); //orientação de inversão FÍSICA dos motores
    rightMaster.setInverted(true);

    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);

    leftSlave.setInverted(InvertType.FollowMaster);
    rightSlave.setInverted(InvertType.FollowMaster);

    lula.setDistancePerPulse(metersPerTick); //sempre que usar o metodo getDistance nos encoders ja vai ler em metros automaticamente
    bolsonaro.setDistancePerPulse(metersPerTick);

    drive.setDeadband(0.05);




  }
  
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("leftEncoder", lula.get());
    SmartDashboard.putNumber("rightEncoder", bolsonaro.get());
  }

  @Override
  public void autonomousInit() {
    //reseta os encoders
    lula.reset();
    bolsonaro.reset();

    setpoint = 5;
  
  }
   


  @Override
  public void autonomousPeriodic() {

    //lendo a posição do sensor 
      double leftDistance = lula.getDistance();
      double rightDistance = bolsonaro.getDistance();

      double sensorPosition = ((leftDistance + rightDistance) / 2);

      //calculando o erro(distancia que falta pra chegar ate o setpoint)
      double error = setpoint - sensorPosition;

      //calculando saida
      double outputSpeed = kP * error;

      //saida pros motores
      drive.arcadeDrive(outputSpeed, 0, false);

      

    SmartDashboard.putNumber("Left Distance (m)", leftDistance);
    SmartDashboard.putNumber("Right Distance (m)", rightDistance);
    SmartDashboard.putNumber("Average Distance (m)", sensorPosition);
    SmartDashboard.putNumber("Setpoint (m)", setpoint);
    SmartDashboard.putNumber("Error (m)", error);
    SmartDashboard.putNumber("PID Output", outputSpeed);
    
    
}


  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    double speed = -joy1.getLeftY() * 1;
    double turn = -joy1.getRightX() * 1;

     drive.arcadeDrive(speed, turn);
     

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
