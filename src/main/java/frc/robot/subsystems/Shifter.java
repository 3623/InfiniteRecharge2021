/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shifter extends SubsystemBase {

    private Solenoid piston;

    public Shifter() {
        piston = new Solenoid(Constants.Shifter.SHIFTER_SOLENOID);
    }

    public void setGear(boolean high) {
        piston.set(high);
    }

    // TODO does this stay accuracte even after disables??
    public boolean getGear(){
        return piston.get();
    }
}
