package org.firstinspires.ftc.teamcode.whiteout.RobotArm;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

public class RobotArmHW {
    public DcMotor S0 = null;

    public Servo S11; //base servo 1
    public Servo S12; //base servo 2
    public Servo S2; //link servo
    public Servo S3; //Gripper Servo

    //We have gobilda 5202 223 RPM
    //Encoder Resolution Formula = 26.9*28
    private static final double TICK_COUNTS_PER_DEGREE = (26.9*28)/(360);
    private static final double MOTOR_SPEED = 0.1;
    private double currOrientationInDegree = 0.0;
    private static double LINK_1_LEN_INCH = 7.5;
    private static double LINK_2_LEN_INCH = 8.5;

    private double last_X_pos = 0;
    private double last_Y_pos = 0;
    private double baseAbsPosInDegree = 0;
    private double linkAbsPosInDegree = 0;

    private double s1ServoCurrPos = 0.0;
    private double s2ServoCurrPos = 0.0;
    private double s3ServoCurrPos = 0.0;



    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        RobotLog.ii("L124", "Enter - init");

        S0  = ahwMap.get(DcMotor.class, "S0");

        S11 = ahwMap.get(Servo.class, "S11");
        S12 = ahwMap.get(Servo.class, "S12");
        S2  = ahwMap.get(Servo.class, "S2" );
        S3  = ahwMap.get(Servo.class, "S3" );

        S0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        S0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //No need to reverse direction for one base Servo
        //S11.setDirection(Servo.Direction.REVERSE);
        //S12.setDirection(Servo.Direction.REVERSE);
        initRobotArm();
        RobotLog.ii("L124", "Exit - init");
    }

    //Initializes the Robot Arm to given position
    public void initRobotArm() {
        //Servo has range of 300 degrees. We only want to operate in 180
        //1 degree = 1/300.
        // 0.5 position = 150 degree
        // left max = 150 -90 = 60
        // right max = 150 + 90 = 210
        // position left  range = 60*1/300 = 0.2
        // position right  range = 210*1/300 = 0.7
        S2.scaleRange(0.0,0.6);

        //ditto for base servos
        S11.scaleRange(0.0,0.8);
        S12.scaleRange(0.0,0.8);

        S3.scaleRange(0.0, 0.5); //half the range is fine

        last_X_pos = LINK_2_LEN_INCH;
        last_Y_pos = LINK_1_LEN_INCH;
        baseAbsPosInDegree = 90;
        linkAbsPosInDegree = 90;

        double pos = 1.0;

        s1ServoCurrPos = 0.1;
        s2ServoCurrPos = 0.9;
        s3ServoCurrPos = 0.0;

        //We dont set the init pos here
        //as the arm will go out of dimensions
        S11.setPosition(s1ServoCurrPos);
        S12.setPosition(s1ServoCurrPos);

        S2.setPosition(s2ServoCurrPos);
        S3.setPosition(s3ServoCurrPos);


    }


    public double getArmTipXPos() {
        return (last_X_pos);
    }

    public double getArmTipYPos() {
        return (last_Y_pos);
    }

    public double getBaseAbsPosInDegree() { return (baseAbsPosInDegree); }

    public double getLinkAbsPosInDegree() {return (linkAbsPosInDegree); }

    public double getMaxLen() {
        return (LINK_1_LEN_INCH + LINK_2_LEN_INCH);
    }

    public double getCurrOrientationInDegree() {
        return (currOrientationInDegree);
    }

    public int getMotorEncoderRawValue() {
        return (S0.getCurrentPosition());
    }

    public void servoSetPosRaw(double newPos, int servoNum) {
        if (servoNum == 1) {
            S11.setPosition(newPos);
            S12.setPosition(newPos);
            s1ServoCurrPos = newPos;
        } else if (servoNum == 2) {
            S2.setPosition(newPos);
            s2ServoCurrPos = newPos;
        }else if (servoNum == 3) {
            S3.setPosition(newPos);
            s3ServoCurrPos = newPos;
        }
    }
    public double getServoPosRaw(int servoNum) {
        double pos = 0.0;
        if (servoNum == 1) {
            pos = s1ServoCurrPos;
        } else if (servoNum == 2) {
            pos = s2ServoCurrPos;
        }else if (servoNum == 3) {
            pos = s3ServoCurrPos;
        }
        return pos;
    }

    public void servoSetPosRawRelative(double deltaPos, int servoNum) {
        if (servoNum == 1) {
            if ((s1ServoCurrPos + deltaPos < 0.0) || (s1ServoCurrPos + deltaPos > 1.0) ) {
                deltaPos = 0.0;
            }
            servoSetPosRaw(s1ServoCurrPos + deltaPos, 1);
        } else if (servoNum == 2) {
            if ((s2ServoCurrPos + deltaPos < 0.0) || (s2ServoCurrPos + deltaPos > 1.0) ) {
                deltaPos = 0.0;
            }
            servoSetPosRaw(s2ServoCurrPos + deltaPos, 2);
        }else if (servoNum == 3) {
            if ((s3ServoCurrPos + deltaPos < 0.0) || (s3ServoCurrPos + deltaPos > 1.0) ) {
                deltaPos = 0.0;
            }
            servoSetPosRaw(s3ServoCurrPos + deltaPos, 3);
        }
    }

    public double convertDegreeToAbsPos(double absDegree) {
        double perDeg = 1.0/180.0; //we have a 180 degree servo
        return (absDegree*perDeg);
    }

    public void rotateBaseServo(double absDegree) {
        double newPos = convertDegreeToAbsPos(absDegree);
        S11.setPosition(newPos);
        S12.setPosition(newPos);
        s1ServoCurrPos = newPos;
        RobotLog.ii("L124", "Base Servo new pos = %2f", newPos);
    }

    public void rotateLinkServo(double absDegree) {
        double newPos = convertDegreeToAbsPos(absDegree);
        S2.setPosition(newPos);
        s2ServoCurrPos = newPos;
        RobotLog.ii("L124", "Link Servo new pos = %2f", newPos);
        //sleep(100);
    }

    public double calculateAngleUsingLawOfCosines(double adjSide1, double adjSide2, double oppSide) {
        double T1 = (adjSide1*adjSide1) + (adjSide2*adjSide2) - (oppSide*oppSide);
        double T2 = 2*adjSide1*adjSide2;
        double angInDegree = Math.toDegrees(Math.acos(T1/T2));
        return angInDegree;
    }

    public void moveRobotArmTo(double newX, double newY) {
        //Calculate the value of hypo
        double hyp = Math.sqrt((newX*newX)+(newY*newY));

        //Check if it falls within the acceptable range of the robot
        if (hyp >= getMaxLen()) {
            //This cannot be achived. Dont do anything
            RobotLog.ii("L124", "Invalid hyp=%2f > max %2f", hyp, getMaxLen());
            return;
        }

        double thetaInDegree =  Math.toDegrees(Math.atan(newY/newX));

        //calculate phi and delta angles using law of cosines
        double phiInDegree   = calculateAngleUsingLawOfCosines(LINK_1_LEN_INCH, hyp, LINK_2_LEN_INCH);
        double deltaInDegree = calculateAngleUsingLawOfCosines(LINK_1_LEN_INCH, LINK_2_LEN_INCH, hyp);

        //Find absolute pos
        baseAbsPosInDegree = 180 - (thetaInDegree + phiInDegree);
        linkAbsPosInDegree = 180 - (deltaInDegree);

        //rotate respective servos to new pos
        rotateBaseServo(baseAbsPosInDegree);
        rotateLinkServo(linkAbsPosInDegree);

        //update tip pos
        last_X_pos = newX;
        last_Y_pos = newY;
        RobotLog.ii("L124", "X,Y,B,L %2f,%2f,%2f,%2f",last_X_pos, last_Y_pos, baseAbsPosInDegree, linkAbsPosInDegree);
    }

    public void rotateArmTo(double deltaDegree) {
        int newPos = 0;

        //Check of the motor is not already running
        if (S0.isBusy()) {
            RobotLog.ii("L124", "Motor is Busy");
            return;
        }

        newPos = S0.getCurrentPosition() + (int) (deltaDegree * TICK_COUNTS_PER_DEGREE);

        //Ensure newPos is always >= 0
        if (newPos < 0) {
            RobotLog.ii("L124", "Motor New Position negative. Setting to zero");
            newPos = 0;
        }

        S0.setTargetPosition(newPos);
        S0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        S0.setPower(Math.abs(MOTOR_SPEED));

        currOrientationInDegree = currOrientationInDegree + deltaDegree;
        if (currOrientationInDegree > 360) {
            currOrientationInDegree = currOrientationInDegree - 360;
        }
        RobotLog.ii("L124", "Motors run to positoin %2d", newPos);
    }



}

