/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.util.Utils;

// TODO make PID
public class Hood extends PIDSubsystem {

    private PWM servo;
    private Encoder encoder;

    // private static final double MIN_TO_MOVE = 0.09;

    private static final double TICKS_P_ENC_REV = 2048.0;
    private static final double DIST_P_PULSE = 360.0 * (36.0 /345.0) / TICKS_P_ENC_REV;

    private static final double MAX_GOAL = 30.0;
    private static final double MIN_GOAL = 0.0;
    private static final double RANGE = MAX_GOAL - MIN_GOAL;
    private static final double kP = 1.3 / RANGE;
    private static final double kI = 0.0 / RANGE;
    private static final double kD = 0.0 / RANGE;
    private static final double DEADBAND = 0.25;

    // private double setpoint = 0.0;

    public Hood() {
        super(new PIDController(kP, kI, kD));
        getController().setTolerance(DEADBAND);
        // setName("Hood");
        servo = new PWM(0);
        encoder = new Encoder(5,4);
        encoder.setDistancePerPulse(DIST_P_PULSE);
    }

    public boolean isReady(){
        return this.getController().atSetpoint();
    }

    // private double calculateOutput(){
    //     double output = 0.0;
    //     if (!Utils.withinThreshold(getMeasurement(), setpoint, 0.4)){
    //         output = (this.setpoint - this.getMeasurement()) / RANGE * kP;
    //         if (Math.abs(output) < MIN_TO_MOVE)
    //             output = Math.copySign(MIN_TO_MOVE, output);
    //     }
    //     return output;
    // }

    private void display(String name, double value) {
        SmartDashboard.putNumber("Shooter/Hood/" + name, value);
    }

    public void monitor(){
        PIDController controller = getController();
        display("Raw Angle", getMeasurement());
        display("Current Setpoint", controller.getSetpoint());
        display("Current Error", controller.getPositionError());
    }

    // public void periodic(){
    //     servo.setSpeed(calculateOutput());
    // }


    @Override
    protected void useOutput(double output, double setpoint) {
        display("Output", output);
        servo.setSpeed(output);
    }

    @Override
    protected double getMeasurement() {
        return encoder.getDistance();
    }

    @Override
    public void setSetpoint(double setpoint) {
        setpoint = Utils.limit(setpoint, MAX_GOAL, MIN_GOAL);
        super.setSetpoint(setpoint);
    }
}
