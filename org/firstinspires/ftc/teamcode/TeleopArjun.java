// Move forward/backward based on D-Pad(up/down) - Constant Speed

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@TeleOp(name = "TeleopArjun", group = "teamcode")

public class TeleopArjun extends LinearOpMode {

    DcMotor topLeft;
    DcMotor topRight;
    DcMotor botLeft;
    DcMotor botRight;
    Servo myServo;
    /*
    //BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    public double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }
    */
    public double globalAngle;
    public double correction;
    public double pow;
    public long sec;
    public static final double motorOutputCount = 753.8;
    public static final double wheelCircumference = 100*Math.PI;
    public double distance;
    public double rotationsNeeded;
    
    public void my_init() {
        // initialize motor variable
        // name we gave ti8pz77he motor in the configuration file on the Robot
        // Controller phone
        topLeft = hardwareMap.dcMotor.get("bottomRight");
        topRight = hardwareMap.dcMotor.get("topLeft");
        botLeft = hardwareMap.dcMotor.get("topRight");
        botRight = hardwareMap.dcMotor.get("bottomLeft");
        myServo = hardwareMap.servo.get("Servo");

        telemetry.addData("Status", "Initialized");
        
        telemetry.update();

        topLeft.setDirection(DcMotor.Direction.REVERSE);
        botLeft.setDirection(DcMotor.Direction.REVERSE);
        topRight.setDirection(DcMotor.Direction.FORWARD);
        botRight.setDirection(DcMotor.Direction.FORWARD);
        topLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        topRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        botLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        botRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        /*
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        //telemetry.addData("Mode", "calibrating...");
        //telemetry.update();
        
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }
        
        //telemetry.addData("Mode", "waiting for start");
        //telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        //telemetry.update();
        */
        //myServo.setDirection(Servo.Direction.FORWARD);
        //myServo.setPosition(0.0);
    }
    public void goForward(double pow) {
        topLeft.setPower(pow);
        botLeft.setPower(-pow);
        topRight.setPower(pow);
        botRight.setPower(-pow);
    }
    public void goForwardSecs(double pow, long secs) {
        topLeft.setPower(pow);
        botLeft.setPower(-pow);
        topRight.setPower(pow);
        botRight.setPower(-pow);
        
        sleep(secs);
    }
    public void goSideways(double pow) {
        topLeft.setPower(-pow);
        botLeft.setPower(-pow);
        topRight.setPower(pow);
        botRight.setPower(pow);
    }
    public void goSidewaysSecs(double pow, long sec) {
        topLeft.setPower(-pow);
        botLeft.setPower(-pow);
        topRight.setPower(pow);
        botRight.setPower(pow);
        sleep(sec);
    }
    public void rotateRobot(double pow) {
        topLeft.setPower(-pow);
        botLeft.setPower(pow);
        topRight.setPower(pow);
        botRight.setPower(-pow);
    }
    public void goDiagonal(double left, double right) {
        topLeft.setPower(right);
        botLeft.setPower(left);
        topRight.setPower(left);
        botRight.setPower(right);
    }
    public void goSidewaysDistance(double pow, double distance) {
        double rotationsNeeded = distance/wheelCircumference;
        double encoderTarget = (rotationsNeeded*motorOutputCount);
        int intEncoder = (int)Math.round(encoderTarget);
        //topLeft  = hardwareMap.dcMotor.get("topleft");
        //topRight  = hardwareMap.dcMotor.get("topright");
        //botLeft  = hardwareMap.dcMotor.get("botleft");
        //botRight  = hardwareMap.dcMotor.get("botright");

        //topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET);
        //topRight.setMode(DcMotor.RunMode.STOP_AND_RESET);
        //botLeft.setMode(DcMotor.RunMode.STOP_AND_RESET);
        //botRight.setMode(DcMotor.RunMode.STOP_AND_RESET);

        topLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        botLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        topLeft.setTargetPosition(intEncoder);
        topRight.setTargetPosition(intEncoder);
        botLeft.setTargetPosition(intEncoder);
        botRight.setTargetPosition(intEncoder);


        topLeft.setPower(-pow);
        topRight.setPower(-pow);
        botLeft.setPower(pow);
        botRight.setPower(pow);

        topLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        botLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        topRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        botRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        topLeft.setPower(0);
        topRight.setPower(0);
        botLeft.setPower(0);
        botRight.setPower(0);
    }
    /*
    public void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    
    // * Get current cumulative angle rotation from last reset.
    // * @return Angle in degrees. + = left, - = right.
     
    
    public void gyroRotate(int degrees, double power) {
        double  leftPower;
        double rightPower;

        
        resetAngle();

       

        if (degrees < 0)
        {  
            leftPower = power;
            rightPower = -power;
        }
        else if (degrees > 0)
        {  
            leftPower = -power;
            rightPower = power;
        }
        else return;

        
        topLeft.setPower(leftPower);
        botLeft.setPower(leftPower);
        topRight.setPower(rightPower);
        botRight.setPower(rightPower);

        // let this rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            //while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > degrees) {
                //topLeft.setPower(leftPower);
                //botLeft.setPower(leftPower);
                //topRight.setPower(rightPower);
                //botRight.setPower(rightPower);
            }
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {
                //topLeft.setPower(-leftPower);
                //botLeft.setPower(-leftPower);
                //topRight.setPower(-rightPower);
                //botRight.setPower(-rightPower);
            }

        // turn the motors off.
        topLeft.setPower(0);
        botLeft.setPower(0);
        topRight.setPower(0);
        botRight.setPower(0);

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }
    */
    @Override
    public void runOpMode() {
        my_init();
        waitForStart();
        
        while(opModeIsActive()) {
            double lefty = this.gamepad1.left_stick_y;
            double leftx = this.gamepad1.left_stick_x;
            double rightx = this.gamepad1.right_stick_x;
            
            if(lefty != 0) {
                goForward(lefty/2);
            } else if(leftx != 0) {
                goSideways(leftx/2);
            } else if(rightx != 0) {
                rotateRobot(rightx/2);
            }else {
                goForward(0);
            }
            /* 
            else if(lefty == 0) {
                //telemetry.addData("Angle: ", "%f", getAngle());
            }  else if(gamepad1.a) {
                //gyroRotate(90, 0.5);
            } else if(gamepad1.b) {
                //gyroRotate(-90, 0.5);
            } */
            if(gamepad1.x) {
                goForward(0);
                return;
            } 
            
        telemetry.addData("Motor Power", topLeft.getPower());
        telemetry.addData("Motor Power", topRight.getPower());
        telemetry.addData("Motor Power", botLeft.getPower());
        telemetry.addData("Motor Power", botRight.getPower());
        telemetry.addData("Status", "Running");
        telemetry.update();            
        }
        


    }
}
