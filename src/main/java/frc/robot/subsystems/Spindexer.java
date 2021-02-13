/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
* An example subsystem. You can replace me with your own Subsystem.
*/
public class Spindexer extends SubsystemBase {
    public static final double SHOOT_TIME = 1.5;
    public static final double INDEX_TIME = 5.0;
    private static final double INDEX_SPEED = 0.4;
    private static final double SHOOT_SPEED = 0.9;
    private boolean indexing = false;
    private boolean shooting = false;
    private WPI_VictorSPX spindexerSPX;

    // private DigitalInput ballSensor1, ballSensor2, ballSensor3, ballSensor4,
    // ballSensor5;

    public Spindexer() {
        spindexerSPX = new WPI_VictorSPX(Constants.Shooter.SPINDEXER_MOTOR_SPX);
    }

    /**
     * @param indexing True if trying to shoot
     */
    public void setIndexing(boolean indexing) {
        this.indexing = indexing;
        chooseOutput();
    }

    /**
     * @param shooting True if trying to shoot
     */
    public void setShooting(boolean shooting) {
        this.shooting = shooting;
        chooseOutput();
    }

    private void chooseOutput() {
        if (shooting) setSpinning(SHOOT_SPEED);
        else if (indexing) setSpinning(INDEX_SPEED);
        else setSpinning(0.0);
    }

    private void setSpinning(double speed) {
        spindexerSPX.set(ControlMode.PercentOutput, speed);
    }

    public boolean getShooting(){
        return shooting;
    }
}
