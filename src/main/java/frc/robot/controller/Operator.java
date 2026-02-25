package frc.robot.controller;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import frc.robot.constants.ControllerConstants;
import frc.robot.subsystems.Turret;
import lombok.Getter;
import lombok.Setter;

public class Operator extends CustomXboxController implements Controller {
  // Singleton instance
  private static Operator INSTANCE;

  @Getter @Setter private Turret turret = new Turret();

  // Private constructor to prevent instantiation from outside
  private Operator() {
    super(ControllerConstants.OPERATOR_CONTROLLER_PORT);
  }

  // Public method to access the single instance
  public static Operator getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new Operator();
    }
    return INSTANCE;
  }

  public static Operator init(Turret turret) {
    Operator operator = getInstance();
    operator.setTurret(turret);
    return getInstance();
  }

  @Override
  public Operator configureBindings() {
    // TODO: add methods to bind controller
    this.a().onTrue(turret.setZero());
    this.b().onTrue(turret.moveToZero());
    this.povRight().whileTrue(turret.setSpeed(RotationsPerSecond.of(-0.1)));
    this.povLeft().whileTrue(turret.setSpeed(RotationsPerSecond.of(0.1)));
    this.povRight().onFalse(turret.stop());
    this.povLeft().onFalse(turret.stop());
    return this;
  }
}
