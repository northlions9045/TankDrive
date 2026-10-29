package frc.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class Drivetrain extends SubsystemBase{
    private final WPI_VictorSPX m_leftMotor1 = new WPI_VictorSPX(0);
    private final WPI_VictorSPX m_leftMotor2 = new WPI_VictorSPX(1);
    private final WPI_VictorSPX m_rightMotor1 = new WPI_VictorSPX(14);
    private final WPI_VictorSPX m_rightMotor2 = new WPI_VictorSPX(15);
    private final DifferentialDrive m_drive = new DifferentialDrive(
        (velocidade) -> {m_leftMotor1.set(velocidade); m_leftMotor2.set(velocidade);},
        (velocidade) -> {m_rightMotor1.set(velocidade); m_rightMotor2.set(velocidade);}
    );
    public Drivetrain(){ //caso algum dos lados esteja invertido por conta da construção mecânica, coloca setInverted true nos motores: m_leftMotor1.setInverted(true); por exemplo
    
        
    }

   public void arcadeDrive(double velocidade, double giro) {
        m_drive.arcadeDrive(velocidade, giro);
    }


    
}
