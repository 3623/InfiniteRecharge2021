/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.util.Utils;

// TODO make this velocity closed loop
public class Hood extends SubsystemBase {

    private PWM motor;
    private Encoder encode;

    private double minToMove = 0.05;

    private static final double TICKS_PER_ENCODER_REV = 2048.0;
    private static final double DISTANCE_PER_PULSE = 1.0/(((345.0 / 36.0)*TICKS_PER_ENCODER_REV) / 360.0);
    //1.0 / TICKS_PER_ENCODER_REV * 36.0 / 40.5 * 45.0;

    private double MAX_GOAL = 30.0;
    private double MIN_GOAL = 0.0;

    private double setpoint = 0.0;

    public Hood() {
        motor = new PWM(0);
        encode = new Encoder(5,4);
        encode.setDistancePerPulse(DISTANCE_PER_PULSE);
    }

    public double getMeasurement(){
        return encode.getDistance();
    }

    public double getSetpoint(){
        return setpoint;
    }

    public boolean isReady(){
        if (!Utils.withinThreshold(this.getMeasurement(), this.setpoint, 0.2)) return true;
        else return false;
    }

    public void setSetpoint(double setpointIn){
        if (setpointIn <= MIN_GOAL) this.setpoint = MIN_GOAL;
        else if (setpointIn >= MAX_GOAL) this.setpoint = MAX_GOAL;
        else this.setpoint = setpointIn;
    }

    private double calculateOutput(){
        double output = 0.0;
        if (!Utils.withinThreshold(this.getMeasurement(), this.setpoint, 0.2)){
            if (this.getMeasurement() < this.setpoint){
                output = ((this.setpoint-this.getMeasurement())/17.5);
            }
            else if (this.getMeasurement() > this.setpoint){
                output = -((this.getMeasurement()-this.setpoint)/17.5);
            }
            if (Math.abs(output) < minToMove){
                if (output < 0) output -= minToMove;
                else output += minToMove; 
            }
        }
        return output;
    }
    
    public void monitor(){
        SmartDashboard.putNumber("Hood/Real Angle", this.getMeasurement()+45);
        SmartDashboard.putNumber("Hood/Raw Angle", this.getMeasurement());
        SmartDashboard.putNumber("Hood/Current Setpoint", this.setpoint);
        SmartDashboard.putNumber("Hood/Current Error", this.setpoint-this.getMeasurement());
        SmartDashboard.putNumber("Hood/Calculated Output", calculateOutput());
    }

    public void periodic(){
        motor.setSpeed(calculateOutput());
    }
    
}
