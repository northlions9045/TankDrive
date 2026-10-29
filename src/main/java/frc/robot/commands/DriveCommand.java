package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain.Drivetrain;
import edu.wpi.first.wpilibj.XboxController;

public class DriveCommand extends Command {
private final Drivetrain m_drivetrain;
private final XboxController m_controller;

public DriveCommand(Drivetrain drivetrain, XboxController controller) {
    this.m_controller = controller;
    this.m_drivetrain = drivetrain;
    addRequirements(m_drivetrain); //não deixa que outro command tente acessar o drivetrain
}
    @Override // avisa que está substituindo esse método na classe mãe(Command)
    public void execute() {// fica rodando a cada 20ms
    double velocidade = m_controller.getLeftY(); //analogico esquerdo controla a velocidade pra frente e pra tras
    double giro = m_controller.getRightX(); // analogico direito controla o giro
    double multiplicador; 
    if (m_controller.getRightTriggerAxis() > 0.7) { //ao apertar RT o robo atinge a velocidade maxima, caso contrario so vai rodar em 70% da velocidade
      multiplicador = 1;
    }
    else {
      multiplicador = 0.7;
    }
    if (Math.abs(velocidade) < 0.05) { // caso o analogico mandar um sinal menor que 0,5% da velocidade maxima ele vai retornar 0
      velocidade = 0;
    }
    if (Math.abs(giro) < 0.05) {
      giro = 0;
    }
    velocidade = velocidade * multiplicador;
    giro = giro * multiplicador;
    m_drivetrain.arcadeDrive(velocidade, giro);


    }
}


