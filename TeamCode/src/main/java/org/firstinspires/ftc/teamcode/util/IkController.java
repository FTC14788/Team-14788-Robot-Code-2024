package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.constants.PivotConstants;
import org.firstinspires.ftc.teamcode.constants.SuperstructureConstants;

public class IkController {
    public static double[] intake(double dist) {
        if (dist >= SuperstructureConstants.NORMALIZATION_DIST) {
            dist = SuperstructureConstants.NORMALIZATION_DIST;
        }
        if (dist <= 2) {
            dist = 2;
        }

        double x = dist + SuperstructureConstants.CHASSIS_FRONT_TO_PIVOT;
        double y = -SuperstructureConstants.PIVOT_HEIGHT + SuperstructureConstants.INTAKE_HEIGHT;

        y += Math.pow((dist/20), 2.2);
        
        double fixedAngle = Math.acos(x / SuperstructureConstants.NORMALIZATION_DIST);
        fixedAngle = 0.53 * Math.PI;

        if ((fixedAngle / (Math.PI * 2)) <= PivotConstants.MIN_ANGLE) {
            fixedAngle = PivotConstants.MIN_ANGLE;
        }

        double x1 = SuperstructureConstants.LINK_ONE_DIST * Math.cos(fixedAngle);
        double y1 = SuperstructureConstants.LINK_ONE_DIST * Math.sin(fixedAngle);
        
        double xt = x - x1;
        double yt = y - y1;

        double link2Dist = Math.hypot(xt, yt);
        double elbowAngle = Math.atan2(yt, xt) - fixedAngle;


        System.out.println((elbowAngle / (Math.PI * 2)));
        return new double[]{
            (fixedAngle / (Math.PI * 2)),
            (elbowAngle / (Math.PI * 2)),
            link2Dist - SuperstructureConstants.LINK_TWO_DIST
        };
    }
}
