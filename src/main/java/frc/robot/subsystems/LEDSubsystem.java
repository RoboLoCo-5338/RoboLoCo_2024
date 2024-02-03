package frc.robot.subsystems;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.



import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.util.Color;
import java.util.Arrays;
import java.util.Optional;
public class LEDSubsystem extends SubsystemBase {


  public AddressableLED m_strip;
  public AddressableLEDBuffer m_Buffer;
  private Optional<Alliance> alliance;
  private boolean[] LEDPriorities;
  private Color[] ledColors;
  private int m_rainbowFirstPixelHue=1;
  /** Creates a new LED. */
  public void LED() {
    alliance = DriverStation.getAlliance();
    m_Buffer = new AddressableLEDBuffer(Constants.BUFFERSIZE);
    m_strip = new AddressableLED(Constants.PWMPORT);
    m_strip.setLength(m_Buffer.getLength());
    m_strip.start();
    ledColors = new Color[Constants.BUFFERSIZE];
    LEDPriorities = new boolean[Constants.numberOfLedActions];


  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  public void updateLEDBuffer(){
      for(int i=0;i<m_Buffer.getLength(); i++) {
          m_Buffer.setLED(i,ledColors[i]);
      }  
      m_strip.setData(m_Buffer);


  }


   public void updateLEDColors(Color[] colors){
       for(int i=0;i<m_Buffer.getLength();i++){
          ledColors[i] = colors[i];
       }
   }


  public void updateLedColor(int index, Color color){
       ledColors[index] = color;
  }


   public void changePriorityList(int index){
       LEDPriorities[index] = !LEDPriorities[index];
   }




   public void setLEDsColorArray(){
        if(LEDPriorities[0]){
            Color[] orange = new Color[m_Buffer.getLength()];
            Arrays.fill(orange, new Color(250,160,0));
            updateLEDColors(orange);
        }else if(LEDPriorities[1]){
            Color[] pink = new Color[m_Buffer.getLength()];
            Arrays.fill(pink, new Color(250,0,200));
            updateLEDColors(pink);
        }else if(LEDPriorities[2]){
             for(int i=0;i<m_Buffer.getLength();i++){
                 int hue = (m_rainbowFirstPixelHue + (i * 180 / m_Buffer.getLength())) % 180;
                 Color color = Color.fromHSV(hue,255,128);
                 updateLedColor(i, color);
             }
             m_rainbowFirstPixelHue += 3;
             m_rainbowFirstPixelHue %= 180;


        }else{
		 Color[] alliance = new Color[m_Buffer.getLength()];
            Arrays.fill(alliance, new Color(250,0,200));
            updateLEDColors(alliance);




        }
   }




}