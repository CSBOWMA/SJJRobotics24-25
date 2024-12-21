package org.firstinspires.ftc.teamcode;

//import com.qualcomm.robotcore.hardware.DcMotor;

public class Main {
//    private DcMotor motor;
    private static int F = 0;
    private static int B = 180;
    private static int R = 90;
    private static int L = -90;
    private static int TOLERANCE = 5;

    public static void main(String[] args) {
        int j = -180;
        for (int i = 0; i < 361; i++) {
            int values1 = correct(B+TOLERANCE);
            int values2 = correct(B+TOLERANCE);
            int min = Math.min(values1, values2);
            int max = Math.max(values1, values2);

            if (min <= j && max >= j){
                System.out.println("i: "+j+" notInRange: false");

            } else {
                System.out.println("i: "+j+" notInRange: true");
                if(B < j){
                    System.out.println("too right; move left to correct");
                } else if (B > j){
                    System.out.println("too left; move right to correct");
                }
            }
            j++;
        }
    }
    public static int correct(int value){
        if (value > 180) return  value-360;
        if (value <= -180) return value+360;
        return value;
    }

//    175 <= 180 ||
//    -175 >= 180  F
}
