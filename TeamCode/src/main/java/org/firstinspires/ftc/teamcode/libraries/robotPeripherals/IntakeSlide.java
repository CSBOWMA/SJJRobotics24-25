package org.firstinspires.ftc.teamcode.libraries.robotPeripherals;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeSlide {
    private Servo slide1;
    private Servo slide2;

    final double SLIDE_ONE_FAR_POSITION = .35;
    final double SLIDE_ONE_CLOSE_POSITION = 0;
    final double SLIDE_ONE_PREPASS_POSITION = .17;
    final double SLIDE_ONE_PASS_POSITION = .05;

    final double SLIDE_TWO_FAR_POSITION = .65;
    final double SLIDE_TWO_CLOSE_POSITION = 1;
    final double SLIDE_TWO_PREPASS_POSITION = .83;
    final double SLIDE_TWO_PASS_POSITION = .95;



   public void close() {
        slide1.setPosition(SLIDE_ONE_CLOSE_POSITION);
        slide2.setPosition(SLIDE_TWO_CLOSE_POSITION);
   }

   public void far() {
       slide1.setPosition(SLIDE_ONE_FAR_POSITION);
       slide2.setPosition(SLIDE_TWO_FAR_POSITION);
   }

    public void prepass() {
        slide1.setPosition(SLIDE_ONE_PREPASS_POSITION);
        slide2.setPosition(SLIDE_TWO_PREPASS_POSITION);
    }

    public void load() {
        slide1.setPosition(SLIDE_ONE_PASS_POSITION);
        slide2.setPosition(SLIDE_TWO_PASS_POSITION);
    }

    //return what percentage the slide is out
    //as a value between 0 and 1, one being all the way out
    public double getPosition() {
        double slide1PercentOut = (slide1.getPosition()-SLIDE_ONE_CLOSE_POSITION) /
                (SLIDE_ONE_FAR_POSITION-SLIDE_ONE_CLOSE_POSITION);
        double slide2PercentOut = (slide2.getPosition()-SLIDE_TWO_CLOSE_POSITION) /
                (SLIDE_TWO_FAR_POSITION-SLIDE_ONE_CLOSE_POSITION);
        return (slide1PercentOut+slide2PercentOut)/2;
    }

    //pass in a value between 0 and 1 representing
    //what percentage the slide should be out
    public void setPosition(double percent) {
      if (percent < 0 || percent > 1) {
         return;
      }
      double slideOneTargetPosition = SLIDE_ONE_CLOSE_POSITION +
              percent*(SLIDE_ONE_FAR_POSITION-SLIDE_ONE_CLOSE_POSITION);
      double slideTwoTargetPosition = SLIDE_TWO_CLOSE_POSITION +
              percent*(SLIDE_ONE_FAR_POSITION-SLIDE_ONE_CLOSE_POSITION);
      slide1.setPosition(slideOneTargetPosition);
      slide2.setPosition(slideTwoTargetPosition);
   }

    public IntakeSlide(HardwareMap hardwareMap) {
        slide1 = hardwareMap.get(Servo.class, "intakeSlide1");
        slide2 = hardwareMap.get(Servo.class, "intakeSlide2");
    }
}
