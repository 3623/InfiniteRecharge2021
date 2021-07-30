/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Feeder extends SubsystemBase {

    private WPI_VictorSPX feederSPX;
    private boolean running = false;
    private static final int TOTAL_REDUCTION = 14;

    public Feeder() {
        feederSPX = new WPI_VictorSPX(Constants.Shooter.FEEDER_MOTOR_SPX);
    }

    public void run(double feederSpeed) {
        feederSPX.set(ControlMode.PercentOutput, feederSpeed);
        running = true;
    }

    public void stop() {
        feederSPX.set(ControlMode.PercentOutput, 0.0);
        running = false;
    }

    public boolean getRunning(){
        return running;
    }

    public double getTheoreticalSpeed(){
        return feederSPX.get()*21020/TOTAL_REDUCTION;
    }

}
