/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWM;
import frc.util.Utils;

public class Hood extends TerribleSubsystem {

    private PWM motor;
    private Encoder encode;

    private double minToMove = 0.05;

    private static final double TICKS_P_ENC_REV = 2048.0;
    private static final double DIST_P_PULSE = 1.0/(((345.0 / 36.0)*TICKS_P_ENC_REV) / 360.0);
    //1.0 / TICKS_P_ENC_REV * 36.0 / 40.5 * 45.0;

    // TODO these values are gonna change?? (see Vince)
    private static final double MAX_GOAL = 30.0;
    private static final double MIN_GOAL = 0.0;
    private static final double RANGE = MAX_GOAL - MIN_GOAL;
    private static final double kP = 0.5;

    private double setpoint = 0.0;

    public Hood() {
        setName("Hood");
        motor = new PWM(0);
        encode = new Encoder(5,4);
        encode.setDistancePerPulse(DIST_P_PULSE);
    }

    public double getMeasurement(){
        return encode.getDistance();
    }

    public double getSetpoint(){
        return setpoint;
    }

    public boolean isReady(){
        if (!Utils.withinThreshold(getMeasurement(), setpoint, 0.2)) return true;
        else return false;
    }

    public void setSetpoint(double value){
        setpoint = Utils.limit(value, MAX_GOAL, MIN_GOAL);
    }

    private double calculateOutput(){
        double output = 0.0;
        if (!Utils.withinThreshold(getMeasurement(), setpoint, 0.2)){
            output = (this.setpoint - this.getMeasurement()) / RANGE * kP;
            if (Math.abs(output) < minToMove)
                output = Math.copySign(minToMove, output);
        }
        return output;
    }

    public void monitor(){
        display("Real Angle", getMeasurement()+45);
        display("Raw Angle", getMeasurement());
        display("Current Setpoint", setpoint);
        display("Current Error", setpoint-getMeasurement());
        display("Calculated Output", calculateOutput());
    }

    public void periodic(){
        motor.setSpeed(calculateOutput());
    }

}
