package org.firstinspires.ftc.teamcode.whiteout.RobotSlide;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

public class RobotSlideHW {

    // motors that push the slide
    public DcMotorEx SL1 = null;
    public DcMotorEx SL2 = null;

    //Servos
    private Servo SLS0L = null; //Left servo to tilt the box
    private Servo SLS0R = null; //Right servo to tilt the box
    private Servo SLS1 = null; //Servo to rotate the arm
    private CRServo SLS1CR = null; //Servo to rotate the arm


    private Servo SLL0 = null; //Left Linear actuator
    private Servo SLL1 = null; //Right Linear actuator

    private Servo SLS2 = null; //Servo holding magnet

    //In order to find the number of ticks counted per inch by motor encoder:
    //1. Find the motor part. Say its gobilda yellow jacket 312 RPM motor
    //2. Look up the spec sheet
    //3. find "Encoder Resolution Formula" or ERF
    //4. 1 - rotation should count ERF worth of ticks
    //5. Say motor is connected to egear of Diameter D.
    //6. Divide ERF/(Pi*D) to get ticks per inch
    //private static final double TICK_COUNTS_PER_INCH = (19.2*28)/(2 * 3.1415);
    private static final double TICK_COUNTS_PER_INCH = (13.7 * 28) / (1 * 3.1415);
    private static final double MOTOR_SPEED = 0.5;

    private static final double MAX_SLIDE_POS_INCH = 100;
    private static final double MIN_SLIDE_POS_INCH = 0;

    private double curr_slide_arm_in_inch = 0.0;
    private double curr_inclination = 0.9;

    private double box_curr_pos_L = 0.5;
    private double box_curr_pos_R = 0.5;
    private double box_arm_curr_pos = 1.0;
    private double magnet_curr_pos   = 0.0;

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        RobotLog.ii("SL124", "Enter - init");

        SL1 = ahwMap.get(DcMotorEx.class, "SL1");
        SL2 = ahwMap.get(DcMotorEx.class, "SL2");

        //reverse direction for one of the slide drive motors?
        SL1.setDirection(DcMotorSimple.Direction.REVERSE);
        //SL2.setDirection(DcMotorSimple.Direction.REVERSE);

        //Uncomment only if supporting encoder mode -- START
        //SL1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //SL2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Reset the encoder only once. They same state
        //will used throughout
        //SL1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //SL2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Uncomment only if supporting encoder mode -- EDN

        SLS0L = ahwMap.get(Servo.class, "SLS0L");
        SLS0R = ahwMap.get(Servo.class, "SLS0R");
        SLS1 = ahwMap.get(Servo.class, "SLS1");
        SLS2 = ahwMap.get(Servo.class, "SLS2");

        //SLS1CR = ahwMap.get(CRServo.class, "SLS1");


        SLL0 = ahwMap.get(Servo.class, "SLL0");
        SLL1 = ahwMap.get(Servo.class, "SLL1");

        //need to use the correct initial position
        SLS0L.setPosition(box_curr_pos_L);
        SLS0R.setPosition(box_curr_pos_R);

        SLS1.setPosition(box_arm_curr_pos);
        SLS2.setPosition(magnet_curr_pos);


        SLL0.setPosition(curr_inclination);
        SLL1.setPosition(curr_inclination);

        curr_slide_arm_in_inch = 0.0;

        box_curr_pos_L = 0.5;
        box_curr_pos_R = 0.5;
        box_arm_curr_pos = 1.0;
        magnet_curr_pos = 0.0;

        RobotLog.ii("SL124", "Exit - init");
    }

    public void initSlideEncoders() {
        RobotLog.ii("SL124", "initSlideEncoders - enter");
        SL1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SL2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Reset the encoder only once. They same state
        //will used throughout
        SL1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SL2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RobotLog.ii("SL124", "initSlideEncoders - Exit");

    }

    public double getSlideServoCurrPos(int servoNum) {
        double currPos_T = 0;
        if (servoNum == 0) {
            currPos_T = SLS0L.getPosition();
        } else if (servoNum == 1) {
            currPos_T = SLS1.getPosition();
        } else if (servoNum == 2) {
            currPos_T = SLS2.getPosition();
        }
        return (currPos_T);
    }

    public void rotateBoxArm(double power) {
        SLS1CR.setPower(power);
    }

    public void setSlideServoCurrPos(int servoNum, double delta) {
        if (servoNum == 0) {
            box_curr_pos_L = box_curr_pos_L + delta;
            SLS0L.setPosition(box_curr_pos_L);

            box_curr_pos_R = box_curr_pos_R - delta;
            SLS0R.setPosition(box_curr_pos_R);
        } else if (servoNum == 1) {
            box_arm_curr_pos = box_arm_curr_pos + delta;
            SLS1.setPosition(box_arm_curr_pos);
        } else   if (servoNum == 2) {
            magnet_curr_pos = magnet_curr_pos + delta;
            SLS2.setPosition(magnet_curr_pos);
        }
        return;
    }

    public void setSlideServoCurrPosAbs(int servoNum, double abs) {
        if (servoNum == 1) {
            box_arm_curr_pos = abs;
            SLS1.setPosition(box_arm_curr_pos);
        } else if (servoNum == 2) {
            magnet_curr_pos = abs;
            SLS2.setPosition(magnet_curr_pos);
        }
        return;
    }


    public void pullSlideUp(double delta) {
        //double temp = curr_inclination;
        if (curr_inclination + delta > 0.9) {
            //Max inclination reached
            return;
        }
        if (curr_inclination + delta < 0.6) {
            //Min inclination reached
            return;
        }

        curr_inclination = curr_inclination + delta;
        SLL0.setPosition(curr_inclination);
        //compensate the box slope for the slide tilt
        //when slide tilt is 0.9, box is horizonal
        //when slide til is 0.8, box slope should be changed.
        setSlideServoCurrPos(0, delta/2);

        SLL0.setPosition(curr_inclination);
        SLL1.setPosition(curr_inclination);

/*
        for (int i = 0; i < 4; ++i) {
            temp = temp + delta/4;
            SLL1.setPosition(temp);
            SLL0.setPosition(temp);

        }

 */
        RobotLog.ii("SL124", "pull Slide Up Curr Inc %2f", curr_inclination);

    }

    public double getSlideArmInInch() {
        return (curr_slide_arm_in_inch);
    }

    public double getSlideCurrIncl() {
        return (curr_inclination);
    }

    public int getSlideEncoderValue(int slideNum) {
        int currPos = 0;
        if (slideNum == 1) {
            currPos = SL1.getCurrentPosition();
        } else if (slideNum == 2) {
            currPos = SL2.getCurrentPosition();
        }
        return currPos;
    }


    public double getMaxLen() {
        return (MAX_SLIDE_POS_INCH);
    }

    public void moveSlideTo(double deltaInch) {
        RobotLog.ii("SL124", "moveSlideTo - Enter %2f, %d", deltaInch, SL1.getCurrentPosition());
        int newPos = 0;

        SL1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SL2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Check of the motor is not already running
        if (SL1.isBusy() || SL2.isBusy()) {
            RobotLog.ii("SL124", "Motors Busy early exit");
            motorSetRawSpeed(0);
        }

        //check if we are withing bounds
        if (getSlideArmInInch() + deltaInch > MAX_SLIDE_POS_INCH) {
            deltaInch = MAX_SLIDE_POS_INCH - getSlideArmInInch();
            RobotLog.ii("SL124", "Max Position reached %2f", deltaInch);
        }

        if (getSlideArmInInch() + deltaInch < MIN_SLIDE_POS_INCH) {
            deltaInch =  getSlideArmInInch() - MIN_SLIDE_POS_INCH;
            RobotLog.ii("SL124", "Min Position reached %2f", deltaInch);
        }

        newPos = SL1.getCurrentPosition() + (int) (deltaInch * TICK_COUNTS_PER_INCH);
        //Ensure newPos is always >= 0
        //if (newPos < 0) {
        //    RobotLog.ii("SL124", "Motor New Position negative. Setting to zero");
        //    newPos = 0;
        //}
        RobotLog.ii("SL124", "new Position  %2d", newPos);

        SL1.setTargetPosition(newPos);
        SL2.setTargetPosition(newPos);

        SL1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SL2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (deltaInch > 0) {
            //SL1.setPower(Math.abs(1.0));
            //SL2.setPower(Math.abs(1.0));
            SL1.setVelocity(3000);
            SL2.setVelocity(3000);
        }else{
            //SL1.setPower(Math.abs(0.3));
            //SL2.setPower(Math.abs(0.3));
            SL1.setVelocity(1500);
            SL2.setVelocity(1500);
        }

        curr_slide_arm_in_inch = curr_slide_arm_in_inch + deltaInch;
        RobotLog.ii("SL124", "Motors run to positoin %2d", newPos);
        RobotLog.ii("SL124", "moveSlideTo - Exit");

    }

    public void motorSetRawSpeed(double speed) {
        SL1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SL2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SL1.setPower(speed);
        SL2.setPower(speed);
    }

}

