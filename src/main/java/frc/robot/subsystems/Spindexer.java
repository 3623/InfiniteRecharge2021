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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
* An example subsystem. You can replace me with your own Subsystem.
*/
public class Spindexer extends TerribleSubsystem {
    public static final double SHOOT_TIME = 3.5;
    public static final double INDEX_TIME = 5.0;
    private static final double READY_SPEED = 0.2;
    private static final double INDEX_SPEED = 0.4;
    private static final double SHOOT_SPEED = 0.9;
    private boolean indexing = false;
    private boolean shooting = false;
    private boolean readying = false;
    private WPI_VictorSPX spindexerSPX;

    private DigitalInput spindexerInPosition; //, ballSensor2, ballSensor3, ballSensor4,
    // ballSensor5;
    //private Encoder spinCoder;

    public Spindexer() {
        spindexerSPX = new WPI_VictorSPX(Constants.Shooter.SPINDEXER_MOTOR_SPX);
        spindexerSPX.setInverted(true);
        //spinCoder = new Encoder(0, 1);
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

    /**
     * @param readyUp True if trying to be in position to shoot
     */
    public void setReadyingUp(boolean readyUp) {
        this.readying = readyUp;
        chooseOutput();
    }

    public boolean isReady(){
        boolean readyState = true;
        if (/*spindexerInPosition.get() ==*/ true) readyState = true;
        return readyState;
    }

    private void chooseOutput() {
        if (shooting) setSpinning(SHOOT_SPEED);
        else if (indexing) setSpinning(INDEX_SPEED);
        else if (isReady() && readying) setSpinning(0.0);
        else if (!isReady() && readying) setSpinning(READY_SPEED);
        else setSpinning(0.0);
    }

    private void setSpinning(double speed) {
        spindexerSPX.set(ControlMode.PercentOutput, speed);
    }

    public boolean getShooting(){
        return shooting;
    }

    public void stopSpinning(){
        spindexerSPX.set(ControlMode.PercentOutput, 0.0);
        shooting = false;
        indexing = false;
        readying = false;
    }

    public void periodic(){
        display("Ready Position", isReady());
    }
}
