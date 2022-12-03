package org.firstinspires.ftc.teamcode.whiteout.RobotLinearActuators;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class RobotLinearActuatorHW {
    public enum Side {
        LEFT, RIGHT
    }

    private Servo LAS = null; //Linear actuator

    private double curr_inclination = 0.9;
    private double correction = 0.0;

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, Side sd) {

        if (sd == Side.LEFT) {
            LAS = ahwMap.get(Servo.class, "SLL0");
            correction = 0.0;
        } else {
            //Right
            LAS = ahwMap.get(Servo.class, "SLL1");
            correction = 0.012;
        }
        LAS.setPosition(curr_inclination);

    }

    public void pullUp(double delta) {
        if (curr_inclination + delta > 0.9) {
            //Max inclination reached
            return;
        }
        if (curr_inclination + delta < 0.6) {
            //Min inclination reached
            return;
        }
        if (delta > 0) {
            curr_inclination = curr_inclination + delta + correction;
        } else {
            curr_inclination = curr_inclination + delta - correction;
        }
        LAS.setPosition(curr_inclination);
    }


    public double getPos(){
        return LAS.getPosition();
    }


}

