package frc.robot.controller;

import frc.robot.constants.ControllerConstants;

public class Operator extends CustomXboxController implements Controller{
    /**
     * Initializes the operator xbox controller.  All subsystems the controller will need to interact
     * with will need to be supplied to the constructor.
     */
    public Operator() {
        super(ControllerConstants.OPERATOR_CONTROLLER_PORT);
    }

    @Override
    public Operator configureBindings() {
        // TODO: add methods to bind controller
        return this;
    }
}
