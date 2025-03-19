/**
 * Phoenix Software License Agreement
 *
 * Copyright (C) Cross The Road Electronics.  All rights
 * reserved.
 * 
 * Cross The Road Electronics (CTRE) licenses to you the right to 
 * use, publish, and distribute copies of CRF (Cross The Road) firmware files (*.crf) and 
 * Phoenix Software API Libraries ONLY when in use with CTR Electronics hardware products
 * as well as the FRC roboRIO when in use in FRC Competition.
 * 
 * THE SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT
 * WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT
 * LIMITATION, ANY WARRANTY OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * CROSS THE ROAD ELECTRONICS BE LIABLE FOR ANY INCIDENTAL, SPECIAL, 
 * INDIRECT OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF
 * PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
 * BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE
 * THEREOF), ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER
 * SIMILAR COSTS, WHETHER ASSERTED ON THE BASIS OF CONTRACT, TORT
 * (INCLUDING NEGLIGENCE), BREACH OF WARRANTY, OR OTHERWISE
 */

/**
 * Description:
 * The CANdle MultiAnimation example demonstrates using multiple animations with CANdle.
 * This example has the robot using a Command Based template to control the CANdle.
 * 
 * This example uses:
 * - A CANdle wired on the CAN Bus, with a 5m led strip attached for the extra animatinos.
 * 
 * Controls (with Xbox controller):
 * Right Bumper: Increment animation
 * Left Bumper: Decrement animation
 * Start Button: Switch to setting the first 8 LEDs a unique combination of colors
 * POV Right: Configure maximum brightness for the CANdle
 * POV Down: Configure medium brightness for the CANdle
 * POV Left: Configure brightness to 0 for the CANdle
 * POV Up: Change the direction of Rainbow and Fire, must re-select the animation to take affect
 * A: Print the VBat voltage in Volts
 * B: Print the 5V voltage in Volts
 * X: Print the current in amps
 * Y: Print the temperature in degrees C
 * 
 * Supported Version:
 * 	- CANdle: 22.1.1.0
 */

 package frc.robot.subsystems;

 import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
 import com.ctre.phoenix.led.CANdle.LEDStripType;
 import com.ctre.phoenix.led.CANdle.VBatOutputMode;
 import com.ctre.phoenix.led.CANdleConfiguration;
 import com.ctre.phoenix.led.ColorFlowAnimation;
 import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
 import com.ctre.phoenix.led.LarsonAnimation;
 import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
 import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
 
 public class CANdleSystem extends SubsystemBase {
     private final int LEDS_PER_ANIMATION = 308-101;
     private final CANdle m_candle = new CANdle(Constants.CANdleID);
    // private XboxController joystick;
     private int m_candleChannel = 8;
     private boolean m_clearAllAnims = false;
     private boolean m_last5V = false;
     private boolean m_animDirection = false;
     private boolean m_setAnim = false;
 
     private Animation m_toAnimate = null;
 
     public enum AnimationTypes {
         ColorFlow,
         Larson,
         Rainbow,
         RgbFade,
         SingleFade,
         Strobe,
         Twinkle,
         TwinkleOff,
         SetAll,
         HasAlgea,
         HasCoral,
         Looking,
         Empty
     }
     private AnimationTypes m_currentAnimation;
 
     public CANdleSystem() {
        // this.joystick = joy;
         CANdleConfiguration configAll = new CANdleConfiguration();
         configAll.statusLedOffWhenActive = true;
         configAll.disableWhenLOS = false;
         configAll.stripType = LEDStripType.GRB;
         configAll.brightnessScalar = 0.8;
         configAll.vBatOutputMode = VBatOutputMode.Modulated;
         m_candle.configAllSettings(configAll, 100);
         changeAnimation(AnimationTypes.Larson);
     }
 
     public void toggle5VOverride() {
         System.out.println("State is: " + m_last5V);
         m_candle.configV5Enabled(m_last5V);
         m_last5V = !m_last5V;
     }
     public void toggleAnimDirection() {
         m_animDirection = !m_animDirection;
     }
     public int getMaximumAnimationCount() {
         return m_candle.getMaxSimultaneousAnimationCount();
     }
 

    
  


 
     /* Wrappers so we can access the CANdle from the subsystem */
     public double getVbat() { return m_candle.getBusVoltage(); }
     public double get5V() { return m_candle.get5VRailVoltage(); }
     public double getCurrent() { return m_candle.getCurrent(); }
     public double getTemperature() { return m_candle.getTemperature(); }
     public void configBrightness(double percent) { m_candle.configBrightnessScalar(percent, 0); }
     public void configLos(boolean disableWhenLos) { m_candle.configLOSBehavior(disableWhenLos, 0); }
     public void configLedType(LEDStripType type) { m_candle.configLEDType(type, 0); }
     public void configStatusLedBehavior(boolean offWhenActive) { m_candle.configStatusLedState(offWhenActive, 0); }
 
     public void changeAnimation(AnimationTypes toChange) {
         m_currentAnimation = toChange;
         
         switch(toChange)
         {
             default:
             case ColorFlow:
                    m_toAnimate = new ColorFlowAnimation(128, 20, 70, 0, 0.7, LEDS_PER_ANIMATION, Direction.Forward,  8);
                 break;

             case Larson:
                 m_toAnimate = new LarsonAnimation(0, 0, 255, 0, 0.5, LEDS_PER_ANIMATION, BounceMode.Front, 5, 8  );
                 break;

             case Rainbow:
                 m_toAnimate = new RainbowAnimation(1, 0.1, LEDS_PER_ANIMATION, m_animDirection,8);
                 break;

             case RgbFade:
                 m_toAnimate = new RgbFadeAnimation(0.7, 0.4, LEDS_PER_ANIMATION,  8 );
                 break;

             case SingleFade:
                 m_toAnimate = new SingleFadeAnimation(0, 0, 255, 0, 0.5, LEDS_PER_ANIMATION,  8);
                 break;

             case Strobe:
                 m_toAnimate = new StrobeAnimation(0, 0, 255, 0, 0.1, LEDS_PER_ANIMATION,  8);
                 break;

             case Twinkle:
                 m_toAnimate = new TwinkleAnimation(30, 70, 60, 0, 0.4, LEDS_PER_ANIMATION, TwinklePercent.Percent42, 8  );
                 break;

             case TwinkleOff:
                 m_toAnimate = new TwinkleOffAnimation(70, 90, 175, 0, 0.2, LEDS_PER_ANIMATION, TwinkleOffPercent.Percent76, 8 );
                 break;

             case Empty:
                 m_toAnimate = new RainbowAnimation(1, 0.7, LEDS_PER_ANIMATION, m_animDirection, 8 );
                 break;
            
            case HasAlgea:
                m_toAnimate = new StrobeAnimation(0, 0, 255, 0, 0.1, LEDS_PER_ANIMATION,  8);
                break;

            case HasCoral:
                m_toAnimate = new StrobeAnimation(255, 0, 0, 0, 0.1, LEDS_PER_ANIMATION,  8);
                break;

            case Looking:
            m_toAnimate = new LarsonAnimation(255, 0, 255, 0, 0.5, LEDS_PER_ANIMATION, BounceMode.Front, 7, 8  );
                break;

             case SetAll:
                 m_toAnimate = null;
                 break;
         }
         System.out.println("Changed to " + m_currentAnimation.toString());
     }
 
     public void clearAllAnims() {m_clearAllAnims = true;}
 
     @Override
     public void periodic() {
         // This method will be called once per scheduler run
         if(m_toAnimate == null) {
             if(!m_setAnim) {
                 /* Only setLEDs once, because every set will transmit a frame */
                 m_candle.setLEDs(255, 255, 255, 0, 0, 1);
                 m_candle.setLEDs(255, 255, 0, 0, 1, 1);
                 m_candle.setLEDs(255, 0, 255, 0, 2, 1);
                 m_candle.setLEDs(255, 0, 0, 0, 3, 1);
                 m_candle.setLEDs(0, 255, 255, 0, 4, 1);
                 m_candle.setLEDs(0, 255, 0, 0, 5, 1);
                 m_candle.setLEDs(0, 0, 0, 0, 6, 1);
                 m_candle.setLEDs(0, 0, 255, 0, 7, 1);
                 m_setAnim = true;
             }
         } else {
            // m_toAnimate.setSpeed((joystick.getRightY() + 1.0) / 2.0);
             m_candle.animate(m_toAnimate, m_candleChannel);
             m_setAnim = false;
         }
        // m_candle.modulateVBatOutput(joystick.getRightY());
 
         if(m_clearAllAnims) {
             m_clearAllAnims = false;
             for(int i = 0; i < 10; ++i) {
                 m_candle.clearAnimation(i);
             }
         }
     }

     public Command setLights(AnimationTypes toChange){
        return runOnce(()->{
            clearAllAnims();
            changeAnimation(toChange);
        });
     }
 
     @Override
     public void simulationPeriodic() {
         // This method will be called once per scheduler run during simulation
     }
 }