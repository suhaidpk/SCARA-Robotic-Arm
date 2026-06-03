import processing.serial.*;
import controlP5.*;
import java.util.ArrayList;
import g4p_controls.*;

// -------------------------------------------------------------
//  Forward kinematics variables
// -------------------------------------------------------------
float upperArmLength = 100.0;   // Shoulder to elbow (mm) – adjust to your robot
float forearmLength  = 100.0;   // Elbow to gripper (mm) – adjust to your robot

// -------------------------------------------------------------
//  Existing globals
// -------------------------------------------------------------
float scalefactor;
PFont customFont;

boolean isFullscreen = false;
boolean fullscreen = false;
boolean gripperState = false; // false = release, true = pick
int leadScrewDirection = 0;   // 0 STOP, 1 UP, -1 DOWN
boolean isLeadScrewMoving = false;
int leadScrewStartTime = 0;
int leadScrewDuration = 0;

GButton minimizeButton, fullscreenButton, closeButton;
String suhaid = "S     C     A     R     A";
String popmessage = "";
ArrayList<Message> messages = new ArrayList();
int fadeOutDuration = 3000;

Serial myPort;
ControlP5 cp5;
String[] portList;
ArrayList<ArrayList<Integer>> savedmovements;
boolean play = false;
int playbackspeed = 500;
int actualmovement = 0;

// -------------------------------------------------------------
//  Forward kinematics calculation
// -------------------------------------------------------------
PVector calculateForwardKinematics(float shoulderAngle, float elbowAngle) {
  // Convert angles to radians and offset so (150°,150°) becomes (0,0)
  float shoulderRad = radians(shoulderAngle - 150);
  float elbowRad   = radians(elbowAngle - 150);
  
  // Standard SCARA forward kinematics (with two revolute joints)
  float x = upperArmLength * cos(shoulderRad) + forearmLength * cos(shoulderRad + elbowRad);
  float y = upperArmLength * sin(shoulderRad) + forearmLength * sin(shoulderRad + elbowRad);
  
  return new PVector(x, y);
}

// -------------------------------------------------------------
//  Message class (existing)
// -------------------------------------------------------------
class Message {
  String text;
  int timestamp;
  
  Message(String text) {
    this.text = text;
    this.timestamp = millis();
  }
  
  void display(float x, float y) {
    int alpha = 255;
    int elapsedTime = millis() - this.timestamp;
    if (elapsedTime < fadeOutDuration) {
      alpha = (int) map((float) elapsedTime, 0.0, (float) fadeOutDuration, 255.0, 0.0);
    }
    fill(0, alpha);
    textAlign(CENTER);
    text(this.text, x, y);
  }
  
  boolean isExpired() {
    return millis() - this.timestamp > fadeOutDuration;
  }
}

// -------------------------------------------------------------
//  Setup
// -------------------------------------------------------------
void setup() {
  size(displayWidth, displayHeight);
  scalefactor = min(width, height) / 1000.0;
  
  surface.setResizable(true);
  fullScreen();
  cp5 = new ControlP5(this);
  
  // ----- Serial port selection -----
  portList = Serial.list();
  if (portList.length > 0) {
    cp5.addDropdownList("portDropdown")
       .setPosition(0, 0)
       .setSize((int)(width * 0.05), (int)(height * 0.10))
       .setBarHeight((int)(height * 0.04))
       .setItemHeight((int)(height * 0.04))
       .setCaptionLabel("PORT")
       .setFont(createFont("Tahoma", 15))
       .setColorBackground(color(0))
       .setColorCaptionLabel(color(255))
       .setColorForeground(color(200, 35, 51))  
       .setColorActive(color(0))
       .setItems(portList);
    myPort = new Serial(this, Serial.list()[0], 9600);
  } else {
    println("No serial ports available!");
  }
  updatePort();
  
  // ----- Servo sliders (shoulder & elbow) -----
  int totalSliderHeight = (int)(6 * height * 0.01);
  int totalSpacing = (int)(6 * height * 0.05);
  int startY = (height - totalSliderHeight - totalSpacing) / 2;
  int sliderwidth = (int)(width * 0.35);
  int sliderX = (width - sliderwidth + 0) / 2;
  
  for (int i = 0; i < 2; i++) {
    Slider slider = cp5.addSlider("servo_" + (i + 1))
         .setPosition(sliderX, startY + i * ((int)(height * 0.1) + (int)(height * 0.01)))
         .setSize(sliderwidth, (int)(height * 0.05))
         .setFont(createFont("Tahoma", 20));
    slider.setColorBackground(color(0))
          .setColorForeground(color(255))     
          .setColorActive(color(200, 35, 51))
          .setColorValueLabel(color(0))
          .setColorCaptionLabel(color(255));
    if (i == 0) {
      slider.setRange(0, 270)
            .setCaptionLabel("  SHOULDER")
            .setValue(135);        // initial 135°
    }
    if (i == 1) {
      slider.setRange(0, 270)
            .setCaptionLabel("  ELBOW")
            .setValue(135);        // initial 135°
    }
    slider.onChange(new CallbackListener() {
      public void controlEvent(CallbackEvent event) {
        int servoIndex = Integer.parseInt(event.getController().getName().split("_")[1]);
        int servoValue = (int) event.getController().getValue();
        println("Sending to Arduino: " + servoIndex + " " + servoValue);
        if (!play && myPort != null) {
          myPort.write(servoIndex + " " + servoValue + "\n");
        }
      }
    });
  }
  
  // ----- Close button -----
  cp5.addButton("close")
     .setPosition((int)(width - (width * 0.05)), 0)
     .setSize((int)(width * 0.05), (int)(height * 0.05))
     .setCaptionLabel("X")
     .setColorCaptionLabel(color(255))
     .setFont(createFont("Berlin Sans FB Demi Bold", 25))
     .setColorBackground(color(0)) 
     .setColorForeground(color(255, 0, 0))  
     .setColorActive(color(200, 35, 51));
  
  savedmovements = new ArrayList<ArrayList<Integer>>();
  
  // ----- Movement control buttons -----
  cp5.addButton("saveMovements")
     .setPosition((int)(width * 0.03), (int)(height * 0.3))
     .setSize((int)(width * 0.18), (int)(height * 0.10))
     .setCaptionLabel("SAVE POSITION")
     .setFont(createFont("Tahoma", 25))
     .setColorBackground(color(0))
     .setColorCaptionLabel(color(255))
     .setColorForeground(color(200, 35, 51))  
     .setColorActive(color(0));
     
  cp5.addButton("playmovement")
     .setPosition((int)(width * 0.03), (int)(height * 0.45))
     .setSize((int)(width * 0.18), (int)(height * 0.10))
     .setCaptionLabel("PLAY MOVEMENT")
     .setFont(createFont("Tahoma", 25))
     .setColorBackground(color(0))
     .setColorCaptionLabel(color(255))
     .setColorForeground(color(200, 35, 51))  
     .setColorActive(color(0));
     
  cp5.addButton("stopMovement")
     .setPosition((int)(width * 0.03), (int)(height * 0.6))
     .setSize((int)(width * 0.18), (int)(height * 0.10))
     .setCaptionLabel("STOP MOVEMENT")
     .setFont(createFont("Tahoma", 25))
     .setColorBackground(color(0))
     .setColorCaptionLabel(color(255))
     .setColorForeground(color(200, 35, 51))  
     .setColorActive(color(0));
     
  cp5.addSlider("playbackspeed")
     .setPosition((int)(width * 0.03), (int)(height * 0.75))
     .setSize((int)(width * 0.18), (int)(height * 0.10))
     .setRange(200, 2000)
     .setValue(playbackspeed)
     .setCaptionLabel("")
     .setFont(createFont("Tahoma", 25))
     .setColorBackground(color(0)) 
     .setColorForeground(color(255))     
     .setColorActive(color(200, 35, 51))
     .setColorValueLabel(color(0));
  updateSpeed();
  
  // ----- Import / Export / Reset -----
  cp5.addButton("exportPosition")
     .setPosition((int)(width * 0.8), (int)(height * 0.3))
     .setSize((int)(width * 0.18), (int)(height * 0.10))
     .setCaptionLabel("EXPORT POSITIONS")
     .setFont(createFont("Tahoma", 25))
     .setColorBackground(color(0))
     .setColorCaptionLabel(color(255))
     .setColorForeground(color(200, 35, 51))  
     .setColorActive(color(0));
     
  cp5.addButton("importPosition")
     .setPosition((int)(width * 0.8), (int)(height * 0.45))
     .setSize((int)(width * 0.18), (int)(height * 0.10))
     .setCaptionLabel("IMPORT POSITIONS")
     .setFont(createFont("Tahoma", 25))
     .setColorBackground(color(0))
     .setColorCaptionLabel(color(255))
     .setColorForeground(color(200, 35, 51))  
     .setColorActive(color(0));
     
  cp5.addButton("resetPosition")
     .setPosition((int)(width * 0.8), (int)(height * 0.6))
     .setSize((int)(width * 0.18), (int)(height * 0.10))
     .setCaptionLabel("RESET POSITIONS")
     .setFont(createFont("Tahoma", 25))
     .setColorBackground(color(0))
     .setColorCaptionLabel(color(255))
     .setColorForeground(color(200, 35, 51))  
     .setColorActive(color(0));
  
  // ----- Gripper button -----
  cp5.addButton("toggleGripper")
     .setPosition((int)(width - 200)/2, (int)(height * 0.65))
     .setSize(200, (int)(height * 0.08))
     .setCaptionLabel(gripperState ? "RELEASE" : "PICK")
     .setFont(createFont("Tahoma", 30))
     .setColorBackground(gripperState ? color(200, 35, 51) : color(255))
     .setColorCaptionLabel(color(0))
     .setColorForeground(color(200, 35, 51))  
     .setColorActive(color(200,35,51));
  
  // ----- Lead screw buttons -----
  cp5.addButton("leadScrew_UP")
     .setPosition(sliderX, startY + 2 * ((int)(height * 0.1) + (int)(height * 0.01)))
     .setSize(sliderwidth/3, (int)(height * 0.05))
     .setCaptionLabel("UP")
     .setFont(createFont("Tahoma", 25))
     .setColorBackground(color(0))
     .setColorCaptionLabel(color(255))
     .setColorForeground(color(200, 35, 51))  
     .setColorActive(color(200,35,51));
     
  cp5.addButton("leadScrew_STOP")
     .setPosition(sliderX + (sliderwidth/3), startY + 2 * ((int)(height * 0.1) + (int)(height * 0.01)))
     .setSize(sliderwidth/3, (int)(height * 0.05))
     .setCaptionLabel("STOP")
     .setFont(createFont("Tahoma", 25))
     .setColorBackground(color(0))
     .setColorCaptionLabel(color(255))
     .setColorForeground(color(200, 35, 51))  
     .setColorActive(color(200,35,51));
     
  cp5.addButton("leadScrew_DOWN")
     .setPosition(sliderX + 2*(sliderwidth/3), startY + 2 * ((int)(height * 0.1) + (int)(height * 0.01)))
     .setSize(sliderwidth/3, (int)(height * 0.05))
     .setCaptionLabel("DOWN")
     .setFont(createFont("Tahoma", 25))
     .setColorBackground(color(0))
     .setColorCaptionLabel(color(255))
     .setColorForeground(color(200, 35, 51))  
     .setColorActive(color(200,35,51));
  
  customFont = createFont("Dune_Rise.ttf", 60 * scalefactor);
  
  // ---------------------------------------------------------
  //  IMPORTANT: Send initial positions to Arduino to sync
  // ---------------------------------------------------------
  if (myPort != null) {
    // Set shoulder and elbow to 135° (default slider position)
    myPort.write("1 135\n");
    delay(20);
    myPort.write("2 135\n");
    delay(20);
    // Force gripper to RELEASE state (0 = release)
    myPort.write("4 0\n");
  }
}

// -------------------------------------------------------------
//  Helper functions (existing)
// -------------------------------------------------------------
void updateSpeed() {
  cp5.remove("playbackspeed");
  cp5.addSlider("playbackspeed")
     .setPosition((int)(width * 0.03), (int)(height * 0.75))
     .setSize((int)(width * 0.18), (int)(height * 0.10))
     .setRange(200, 2000)
     .setValue(1000)
     .setCaptionLabel("")
     .setLabelVisible(true)
     .setLabel("")
     .setFont(createFont("Tahoma", 25))
     .setColorBackground(color(0)) 
     .setColorForeground(color(255))     
     .setColorActive(color(200, 35, 51))
     .setColorValueLabel(color(0));
}

void updateLeadScrewButtons() {
  Button upButton = (Button) cp5.getController("leadScrew_UP");
  Button stopButton = (Button) cp5.getController("leadScrew_STOP");
  Button downButton = (Button) cp5.getController("leadScrew_DOWN");
  
  upButton.setColorBackground(color(0));
  downButton.setColorBackground(color(0));
  stopButton.setColorBackground(color(0));
  
  if (leadScrewDirection == 1) {
    upButton.setColorBackground(color(200, 35, 51));
    upButton.setColorForeground(color(200, 35, 51));
    upButton.setColorActive(color(200, 35, 51));
  } else if (leadScrewDirection == -1) {
    downButton.setColorBackground(color(200, 35, 51));
    downButton.setColorForeground(color(200, 35, 51));
    downButton.setColorActive(color(200, 35, 51));
  }
  
  upButton.setCaptionLabel("UP");
  downButton.setCaptionLabel("DOWN");
  stopButton.setCaptionLabel("STOP");
}

void updatePort() {
  cp5.remove("portDropdown");
  if (portList.length > 0) {
    cp5.addDropdownList("portDropdown")
       .setPosition(0, 0)
       .setSize((int)(width * 0.05), (int)(height * 0.10))
       .setBarHeight((int)(height * 0.04))
       .setItemHeight((int)(height * 0.04))
       .setCaptionLabel("PORT")
       .setFont(createFont("Tahoma", 15))
       .setColorBackground(color(0))
       .setColorCaptionLabel(color(255))
       .setColorForeground(color(200, 35, 51))  
       .setColorActive(color(0))
       .setItems(portList);
    try {
      myPort = new Serial(this, Serial.list()[0], 9600);
      // Re-sync positions after port change
      delay(100);
      myPort.write("1 135\n");
      delay(20);
      myPort.write("2 135\n");
      delay(20);
      myPort.write("4 0\n");
    } catch (Exception e) {
      println("Error opening serial port: " + e.getMessage());
    }
  } else {
    println("No serial ports available!");
  }
}

boolean arraysEqual(String[] arr1, String[] arr2) {
  if (arr1.length != arr2.length) return false;
  for (int i = 0; i < arr1.length; i++) {
    if (!arr1[i].equals(arr2[i])) return false;
  }
  return true;
}

// -------------------------------------------------------------
//  Draw (includes forward kinematics display)
// -------------------------------------------------------------
void draw() {
  background(0);
  
  // --- Title (existing) ---
  textSize(50 * scalefactor);
  textFont(customFont);
  fill(255);
  textAlign(CENTER, CENTER);
  text(suhaid, width / 2, height * 0.1);
  
  // --- Forward Kinematics display ---
  float shoulderAngle = cp5.getController("servo_1").getValue();
  float elbowAngle   = cp5.getController("servo_2").getValue();
  PVector pos = calculateForwardKinematics(shoulderAngle, elbowAngle);
  
  fill(255);
  textSize(25);
  textAlign(CENTER);
  String coordText = String.format("Position (X,Y): (%.1f mm, %.1f mm)", pos.x, pos.y);
  text(coordText, width / 2, height * 0.2);
  
  // --- Serial port refresh (existing) ---
  String[] currentPortList = Serial.list();
  if (!arraysEqual(portList, currentPortList)) {
    portList = currentPortList;
    updatePort();
  }
  
  // --- Popup message (existing) ---
  fill(255);
  textAlign(3);
  textFont(createFont("Tahoma", 20));
  text(popmessage, (float)(width / 2 + 400), (float)(height - 50));
  
  // --- Fading messages (existing) ---
  for (int i = messages.size() - 1; i >= 0; i--) {
    Message message = messages.get(i);
    float x = (float)(width / 2);
    float y = (float)(height / 2 + (messages.size() - i - 1) * 20);
    message.display(x, y);
    if (message.isExpired()) {
      messages.remove(i);
    }
  }
  
  cp5.draw();
  
  // --- White borders around sliders and buttons (existing) ---
  noFill();
  stroke(255);
  strokeWeight(1);
  for (int i = 0; i < 2; i++) {
    Slider slider = (Slider) cp5.getController("servo_" + (i + 1));
    rect(slider.getPosition()[0] - 1, slider.getPosition()[1] - 1, slider.getWidth() + 1, slider.getHeight() + 1);
  }
  Slider velocitySlider = (Slider) cp5.getController("playbackspeed");
  rect(velocitySlider.getPosition()[0] - 1, velocitySlider.getPosition()[1] - 1, 
       velocitySlider.getWidth() + 1, velocitySlider.getHeight() + 1);
  drawButtonBorders();
  
  updateLeadScrewButtons();
}

void drawButtonBorders() {
  noFill();
  stroke(255);
  strokeWeight(2);
  String[] buttonNames = {"saveMovements", "playmovement", "stopMovement", 
                          "exportPosition", "importPosition", "resetPosition",
                          "leadScrew_UP", "leadScrew_DOWN", "leadScrew_STOP"};
  for (String name : buttonNames) {
    Button button = (Button) cp5.getController(name);
    rect(button.getPosition()[0] - 1, button.getPosition()[1] - 1, 
         button.getWidth() + 1, button.getHeight() + 1);
  }
}

// -------------------------------------------------------------
//  Lead screw functions (existing)
// -------------------------------------------------------------
void leadScrew_UP() {
  if (myPort != null) {
    myPort.write("3 1\n");
    leadScrewDirection = 1;
    isLeadScrewMoving = true;
    updateLeadScrewButtons();
    println("Lead Screw: UP");
  }
}

void leadScrew_DOWN() {
  if (myPort != null) {
    myPort.write("3 -1\n");
    leadScrewDirection = -1;
    isLeadScrewMoving = true;
    updateLeadScrewButtons();
    println("Lead Screw: DOWN");
  }
}

void leadScrew_STOP() {
  if (myPort != null) {
    myPort.write("3 0\n");
    leadScrewDirection = 0;
    isLeadScrewMoving = false;
    updateLeadScrewButtons();
    println("Lead Screw: STOP");
  }
}

// -------------------------------------------------------------
//  Control events (existing, with gripper fix)
// -------------------------------------------------------------
void controlEvent(ControlEvent theEvent) {
  if (theEvent.isFrom("portDropdown")) {
    int selection = PApplet.parseInt(theEvent.getValue());
    String selectedPort = portList[selection];
    println("Selected port: " + selectedPort);
    if (myPort != null) myPort.stop();
    myPort = new Serial(this, selectedPort, 9600);
    delay(100);
    myPort.write("1 135\n");
    delay(20);
    myPort.write("2 135\n");
    delay(20);
    myPort.write("4 0\n");
  } else if (theEvent.isFrom("stopMovement")) {
    stopMovement();
  } else if (theEvent.isFrom("toggleGripper")) {
    gripperState = !gripperState;
    Button toggleButton = (Button) cp5.getController("toggleGripper");
    toggleButton.setCaptionLabel(gripperState ? "RELEASE" : "PICK");
    toggleButton.setColorBackground(gripperState ? color(200, 35, 51) : color(255));
    if (myPort != null) {
      myPort.write("4 " + (gripperState ? 1 : 0) + "\n");
    }
  } else if (theEvent.isFrom("leadScrew_UP")) {
    leadScrew_UP();
  } else if (theEvent.isFrom("leadScrew_STOP")) {
    leadScrew_STOP();
  } else if (theEvent.isFrom("leadScrew_DOWN")) {
    leadScrew_DOWN();
  }
}

// -------------------------------------------------------------
//  Movement recording and playback (existing, with minor sync)
// -------------------------------------------------------------
void saveMovements() {
  ArrayList<Integer> movements = new ArrayList();
  for (int i = 0; i < 2; i++) {
    movements.add((int) cp5.getController("servo_" + (i+1)).getValue());
  }
  movements.add(gripperState ? 1 : 0);
  movements.add(leadScrewDirection);
  savedmovements.add(new ArrayList(movements));
  
  String leadScrewState = (leadScrewDirection == 1) ? "UP" : (leadScrewDirection == -1) ? "DOWN" : "STOP";
  println("Movement saved: " + movements + " " + leadScrewState);
  popmessage = "P  O  S  I  T  I  O  N    S  A  V  E  D  :   " + movements + " " + leadScrewState;
}

void playmovement() {
  if (!play && !savedmovements.isEmpty()) {
    play = true;
    println("Playing movements!");
    popmessage = "P  L  A  Y  I  N  G    M  O  V  E  M  E  N  T  S  !";
    
    new Thread(() -> {
      while (play) {
        int numMovements = savedmovements.size();
        int nextMovement = (actualmovement + 1) % numMovements;
        
        ArrayList<Integer> actualMovements = savedmovements.get(actualmovement);
        ArrayList<Integer> nextMovements = savedmovements.get(nextMovement);
        ArrayList<Integer> currentValues = new ArrayList<>();
        
        for (int i = 0; i < actualMovements.size() - 2; i++) {
          currentValues.add((int) cp5.getController("servo_" + (i+1)).getValue());
        }
        
        int steps = 50;
        for (int step = 0; step <= steps; step++) {
          for (int ix = 0; ix < actualMovements.size() - 2; ix++) {
            int nextVal = nextMovements.get(ix);
            int currVal = currentValues.get(ix);
            int newVal = currVal + (nextVal - currVal) * step / steps;
            cp5.getController("servo_" + (ix+1)).setValue(newVal);
            if (myPort != null) {
              myPort.write((ix+1) + " " + newVal + "\n");
            }
          }
          delay(playbackspeed / steps);
        }
        
        // Gripper
        int nextGripperState = nextMovements.get(nextMovements.size() - 2);
        if ((nextGripperState == 1) != gripperState) {
          gripperState = (nextGripperState == 1);
          Button toggleButton = (Button) cp5.getController("toggleGripper");
          toggleButton.setCaptionLabel(gripperState ? "RELEASE" : "PICK");
          toggleButton.setColorBackground(gripperState ? color(200, 35, 51) : color(255));
          if (myPort != null) {
            myPort.write("4 " + (gripperState ? 1 : 0) + "\n");
          }
        }
        
        // Lead screw
        int nextLead = nextMovements.get(nextMovements.size() - 1);
        if (nextLead != leadScrewDirection) {
          if (myPort != null) myPort.write("3 " + nextLead + "\n");
          leadScrewDirection = nextLead;
          isLeadScrewMoving = (leadScrewDirection != 0);
          updateLeadScrewButtons();
        }
        
        actualmovement = nextMovement;
      }
    }).start();
  }
}

void stopMovement() {
  for (int i = 0; i < 2; i++) {
    int currentValue = (int) cp5.getController("servo_" + (i+1)).getValue();
    if (myPort != null) {
      myPort.write((i+1) + " " + currentValue + "\n");
    }
  }
  play = false;
  println("Movements stopped.");
  popmessage = "M  O  V  E  M  E  N  T  S    S  T  O  P  P  E  D";
}

void exportPosition() {
  selectOutput("Select file to export", "saveFile");
}

void saveFile(File file) {
  if (file != null) {
    String[] lines = new String[savedmovements.size()];
    for (int i = 0; i < savedmovements.size(); i++) {
      ArrayList<Integer> moves = savedmovements.get(i);
      StringBuilder line = new StringBuilder();
      for (Integer v : moves) line.append(v).append(" ");
      lines[i] = line.toString().trim();
    }
    saveStrings(file.getAbsolutePath(), lines);
    println("Saved positions exported.");
    popmessage = "P  O  S  I  T  I  O  N  S    E  X  P  O  R  T  E  D";
  } else {
    popmessage = "N  O    F  I  L  E    S  E  L  E  C  T  E  D";
  }
}

void importPosition() {
  selectInput("Select file to import", "loadFile");
}

void loadFile(File file) {
  if (file != null) {
    String[] lines = loadStrings(file.getAbsolutePath());
    if (lines != null) {
      savedmovements.clear();
      for (String line : lines) {
        String[] parts = line.split(" ");
        ArrayList<Integer> moves = new ArrayList<>();
        for (String p : parts) moves.add(Integer.parseInt(p));
        savedmovements.add(moves);
      }
      println("Positions imported.");
      popmessage = "P  O  S  I  T  I  O  N  S    I  M  P  O  R  T  E  D";
    } else {
      popmessage = "N  O    F  I  L  E    F  O  U  N  D";
    }
  } else {
    popmessage = "N  O    F  I  L  E    S  E  L  E  C  T  E  D";
  }
}

void resetPosition() {
  stopMovement();
  savedmovements.clear();
  println("Saved positions reset.");
  popmessage = "P  O  S  I  T  I  O  N  S    R  E  S  E  T";
}

void close() {
  stopMovement();
  println("Closing application...");
  delay(500);
  exit();
}
