package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Projects.HWMapBasic;

@TeleOp(name = "wrist")
public class wrist extends LinearOpMode {
    public HWMapBasic robot = new HWMapBasic();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        double speed = .8;
        int rightPosition = 0;
        int leftPosition = 0;
        int noU = -8000;
        int pos = 1;
        double rDiff = .5;
        double lDiff = .5;
        int[] positions;
        robot.fRightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.fLeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.bRightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.bLeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /// init motor pos and encoder



        waitForStart();
        boolean isSpinning = false;
        while (opModeIsActive()) {
            boolean aButtonHeld = false;
           if(gamepad1.a){
               robot.rDiff.setPosition(.8);
               robot.lDiff.setPosition(.2);
               pos = 3;
           }
           else if(gamepad1.b){
               robot.rDiff.setPosition(.2);
               robot.lDiff.setPosition(.8);
               pos = 1;
           }
           else if(gamepad1.x){
               robot.rDiff.setPosition(.5);
               robot.lDiff.setPosition(.5);
               pos = 2;
           }

           if(gamepad1.dpad_right){
               if(pos == 1){
                   robot.rDiff.setPosition(.4);
                   robot.lDiff.setPosition(1);
               }
               if(pos==2){
                   robot.rDiff.setPosition(.7);
                   robot.lDiff.setPosition(.7);
               }
              if(pos == 3){
                  robot.rDiff.setPosition(1);
                  robot.lDiff.setPosition(.4);
              }
           }



        }
    }


    int[] WaitTillTargetReached(int tolerance, boolean lock){
        int leftDifference = Math.abs(robot.leftLift.getTargetPosition() - robot.leftLift.getCurrentPosition());
        int rightDifference = Math.abs(robot.rightLift.getTargetPosition() - robot.rightLift.getCurrentPosition());
        int check=102930293;
        while(leftDifference > tolerance || rightDifference > tolerance)

        {

            leftDifference = Math.abs(robot.leftLift.getTargetPosition() - robot.leftLift.getCurrentPosition());
            rightDifference = Math.abs(robot.rightLift.getTargetPosition() - robot.rightLift.getCurrentPosition());

            robot.leftLift.setPower(0.8);
            robot.rightLift.setPower(0.8);
            if (check == robot.rightLift.getCurrentPosition() + robot.leftLift.getCurrentPosition()) {
                break;
            }
            else {
                check = robot.rightLift.getCurrentPosition() + robot.leftLift.getCurrentPosition();
            }
            sleep(1);
            int a = robot.rightLift.getCurrentPosition();
            int c = robot.leftLift.getCurrentPosition();
            telemetry.addLine("current position: " + a + "," + c);
            telemetry.addLine("target position: " + robot.leftLift.getTargetPosition());
            telemetry.update();

        }
        int a = robot.rightLift.getCurrentPosition();
        int c = robot.leftLift.getCurrentPosition();
        telemetry.addLine("current position: " + a + "," + c);
        telemetry.addLine("target position: " + robot.leftLift.getTargetPosition());
        telemetry.update();
        int[] positions = new int[] {a,c};


        if(!lock)
        {
            robot.leftLift.setPower(0);
            robot.rightLift.setPower(0);
        }
        return(positions);

    }
    private void cycle() {

    }
}
