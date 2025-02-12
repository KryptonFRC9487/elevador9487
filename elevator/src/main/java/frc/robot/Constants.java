package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class Constants {

      /**
   * Configurações do elevador.
   */
  public static final class ElevatorConstants {

    public static enum ElevatorPose {
      INITAL(2),
      L1(2),
      L2(10),
      L3(2),
      L4(54);

      public final double value;

      private ElevatorPose(double value) {
        this.value = value;
      }
    }

    // IDs dos motores
    public static final int LEFT_MOTOR_ID = 15;
    public static final int RIGHT_MOTOR_ID = 16;

    // Constantes do elevador
    public static final double GEARING = 5.0;
    public static final double MASS_KG = 5.0;
    public static final double DRUM_CIRCUMFERENCE = 2.0 * Math.PI * Units.inchesToMeters(2.0);
    public static final double ENCODER_ROTATIONS_TO_METERS = DRUM_CIRCUMFERENCE / GEARING;

    public static final int CURRENT_LIMIT = 60;

    // PID Gains Elevator
    public static final double kP = 0.3; // Proporcional
    public static final double kI = 0.0; // Integral
    public static final double kD = 0.0; // Derivativo
    public static final double kFF = 0.05; // Feedforward

    public static final double kVelocityFF = 0.001; // Feedforward

    // Feedforward Gains
    public static final double kS = 0.095388; // Tensão estática TODO
    public static final double kG = 0.54402; // Gravidade TODO
    public static final double kV = 7.43; // Velocidade TODO
    public static final double kA = 1.0; // Aceleração TODO

    // Restrições de movimento
    public static final double MIN_HEIGHT_METERS = 0.005; // TODO
    public static final double MAX_HEIGHT_METERS = 1.57; // TODO
    public static final double MAX_VELOCITY = 3.67; // Velocidade máxima (m/s) TODO
    public static final double MAX_ACCELERATION = 3.0; // Aceleração máxima (m/s^2) TODO
    public static final TrapezoidProfile.Constraints MOVEMENT_CONSTRAINTS = new TrapezoidProfile.Constraints(
        MAX_VELOCITY, MAX_ACCELERATION);
  }
    
}
