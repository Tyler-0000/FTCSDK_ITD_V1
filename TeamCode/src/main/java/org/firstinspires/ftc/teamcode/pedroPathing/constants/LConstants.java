package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;

public class LConstants {
    static {
        ThreeWheelConstants.forwardTicksToInches = 0.0019890004972532307;
        ThreeWheelConstants.strafeTicksToInches = 0.0019720163702662274;
        ThreeWheelConstants.turnTicksToInches = 0.0022;
        ThreeWheelConstants.leftY = 3.5;
        ThreeWheelConstants.rightY = -3.5;
        ThreeWheelConstants.strafeX = -2;
        ThreeWheelConstants.leftEncoder_HardwareMapName = "leftRear";
        ThreeWheelConstants.rightEncoder_HardwareMapName = "rightRear";
        ThreeWheelConstants.strafeEncoder_HardwareMapName = "rightFront";
        ThreeWheelConstants.leftEncoderDirection = Encoder.FORWARD;
        ThreeWheelConstants.rightEncoderDirection = Encoder.FORWARD;
        ThreeWheelConstants.strafeEncoderDirection = Encoder.REVERSE;

    }
}




