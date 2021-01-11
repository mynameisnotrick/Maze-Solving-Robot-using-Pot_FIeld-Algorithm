// ==========================================================================================
// ******************************************************************************************
//                    MAZE SOLFING ROBOT W/ POTENTIAL FIELD ALGORITHM
//                                    October 10, 2020
//   Made by: Glenn Bonaventura W. (2017630019), Mechatronics Engineering, Parahyangan Uni.
//                           Derived from: K04 Maze Solvong Despro
//           Mentored by: Dr. Ir. Bagus Arthaya, M.Eng (bagusart@unpar.ac.id)
// This is an open source code. Please kindly put the creator name when you're using the code
// ******************************************************************************************
// ==========================================================================================

// Pot_Field (Potential Field) merupakan algoritma maze solving dengan memberikan nilai pada
// setiap cellnya. Nilai pada setiap cell mempengaruhi robot dalam menentukan arah pergerrakan.
// Nilai dari setiap cell ditentukan dari jarak cell finish menuju cell yang hendak diberikan
// nilai tersebut. Semakin dekat dengan finish, maka semakin kecil nilai cell tersebut.
// Potential Field ini membuat robot bergerak menuju cell dengan nilai lebih kecil yang membawa
// robot menuju finish.

// --------------------------------
// Version: 1.4.0
// Last update: January 10, 2021
// --------------------------------

// =====================================
//            PIN DECLARATION
// =====================================
//               ACTUATOR
const int in1 = 9;
const int in2 = 10;
const int in3 = 5;
const int in4 = 6;
//                SENSOR
const int lecho = 13;
const int ltrigger = 12;
const int fecho = 11;
const int ftrigger = 8;
const int recho = 7;
const int rtrigger = 4; 

// =====================================
//                MEMORY
// =====================================
//       Coordinate & Orientation
int x_cor , y_cor;
int face;
int steps;
//        Unchanged Variable
// Relative position
const int left = 0;
const int front = 1;
const int right = 2;
// compass
const int north = 0;
const int east = 1;
const int south = 2;
const int west = 3;
// Maze information
const int m_width = 5;
const int m_height = 5;
const int x_fin = 2;
const int y_fin = 2;
//              Motors
int right_pwm;
int left_pwm;
int diff_pwm;
//               Ticks
int volatile unsigned long ticks_r = 0;
int volatile unsigned long ticks_l = 0;
//           Flag Array
int trail[m_width][m_height];
int junction[m_width][m_height];
int deadend[m_width][m_height];
int pot_field[m_width][m_height];
//             Others
int decision;
int way_left, way_front, way_right;


// ==========================================================================================
//                                    SET UP
// ==========================================================================================
void setup() {
// ------------------
// I/O Declaration
// ------------------
//               ACTUATORS
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
//                SENSORS
  pinMode(ltrigger,   OUTPUT);
  pinMode(lecho,      INPUT);
  pinMode(ftrigger,   OUTPUT);
  pinMode(fecho,      INPUT);
  pinMode(rtrigger,   OUTPUT);
  pinMode(recho,      INPUT);
//               INTERRUPT (TICKS COUNTING)
  attachInterrupt(digitalPinToInterrupt(3), add_left, CHANGE);
  attachInterrupt(digitalPinToInterrupt(2), add_right, CHANGE);
// ------------------
// Initial PWM
// ------------------  
  right_pwm = 90;
  left_pwm = 90;
// ------------------
// Initial position & orientation
// ------------------ 
  x_cor = 0;
  y_cor = 0;
  face = east;
// ------------------
// Nullified array
// ------------------ 
  array_null();

// ------------------
// Potential Field
// ------------------ 
// This part of code gives every cell their potential value.
// Every value saved in potfield[][] array which every coordinate have their own unique value. 
  int n = 0;
  int m = 0;
  int pot = 0;
  for (n = 0; n < m_width ; n++){
    for (m = 0; m < m_height ; m++ ){
        // The value can be counted by this formula
        pot_field[n][m] = abs(x_fin - n) + abs(y_fin - m);
    }
  }

// ------------------
// Data Print
// ------------------ 
// To ensure that the algorithm was correct, all informations regarding the cell coordinate,
// orientation, and flags printed in serial monitor 9600.
  Serial.begin(9600);
    
  Serial.print("X_cor");
  Serial.print("\tY_cor");
  Serial.print("\tOrientation");
  Serial.print("\tSteps");
  Serial.print("\t\tTrail Flag");
  Serial.print("\tDead end Flag");
  Serial.print("\tJunction Flag\n");
  
  delay(3000);
}

// ==========================================================================================
//                                        LOOP
// ==========================================================================================
void loop() {
  // This is the part when all the code looped and the algorithm try to find the finish cell
  // Unless the micromouse finished it task, the program will looped.
  finishcheck();
  wallcheck();
  decisions();
  Oop();
  dataprint();
  delay(1000);
}

// ==========================================================================================
//                                     ARRAY NULL
// ==========================================================================================
void array_null(){
  // Every data declared will have a random number value. It'll be critical when we need to compare
  // some data that haven't been checked. To make sure every data is save to compare, it's safe
  // to set all data value to 0. That's the reason for why this subroutine was needed.
  int n = 0;
  int m = 0;
  for (n = 0; n < m_width ; n++){
    for (m = 0; m < m_height ; m++ ){
      trail[m_width][m_height] = 0;
      junction[m_width][m_height] = 0;
      deadend[m_width][m_height] = 0;
      pot_field[m_width][m_height] = 0;
    }
  }
}

// ==========================================================================================
//                                    FINISH CHECK
// ==========================================================================================
void finishcheck(){
  // Micromouse needs to know wether they have finished the maze or not. This subroutine
  // was made to inform the micromouse if it already reach its finnish cell.
  if (x_cor == x_fin && y_cor == y_fin){
    Serial.println("FINISH");
    delay(500000);
  }
}

// ==========================================================================================
//                                     WALLCHECK
// ==========================================================================================
void wallcheck(){  
  // Wall check is a subroutine that saves every wall's status in their own variable. 
  // Its value will be 0 or 1 depend on absence or presence of wall in front of each sensors.
  // This value will help the algorithm to pick decisions.
  // 0 = there's no wall
  // 1 = there's a wall
  way_left = wall_present(ltrigger, lecho);
  way_front = wall_present(ftrigger, fecho);
  way_right = wall_present(rtrigger, recho);
}

// ==========================================================================================
//                                    WALL PRESENT
// ==========================================================================================
int wall_present(int trigPin, int echoPin){
  // This subroutine was made to make each sensors gives information about the presence or absence
  // of the wall in front of them. 
  // To know wether there's a wall or not, sensors will count the distance between seach sensors and wall
  int duration, distance;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  // This code bellow will count the distance in mm.
  distance = duration * 0.34 / 2;
    if (distance < 100) { 
    // The distance from wall to sensors are approx. 25mm to 275mm
    // It's safe to assume there is no wall in front of the sensor
      return 0;
    }
    else {
      return 1;
    }
}

// ==========================================================================================
//                                        DECISION
// ==========================================================================================
void decisions(){
  // Decisions is a quite important subroutine. With information from the wall bellow,
  // the algorithm now can make decision wether to move forward, rotate, or make a turn.
  decision = way_left + way_front + way_right;
  if(decision == 0){
    // When there is no way to move, it'll execute dead_end subroutine
    dead_end();
  }
  else if(decision == 1){
    // When there is one way to move, it'll execute oneway subroutine
    oneway();
  }
  else if(decision >= 2){
    // When there are more than one way to move, it'll execute jnc subroutine
    jnc();
  }
}

// ==========================================================================================
//                                        ONEWAY
// ==========================================================================================
void oneway(){
  // oneway is a subroutine that will be executed if and only if there is one possible way.
  // The micromouse will easily check the only way and move toward that way.
  if(way_left == 1){
    // When the left way opened, the micromouse will turn 90 degree ccw and move toward the
    // opened way.
    rotate_left();
  }
  else if (way_right == 1){
    // When the right way opened, the micromouse will turn 90 degree cw and move toward the
    // opened way.
    rotate_right();
  }
  else if (way_front == 1){
    // When the front way opened, the micromouse move forward toward the opened way.
    forward();
  }
}

// ==========================================================================================
//                                        DEAD END
// ==========================================================================================
void dead_end(){
  // This subroutine will executed when there is no possible way to move.
  // Dead end subroutine will be executed until the micromouse find a junction.
  half_turn();
  // The micromouse will make half turn to show the possible way.
  while(junction[x_cor][y_cor]<1){
    deadend[x_cor][y_cor] = 1;
    // The micromouse will mark dead end at that coordinate as true.
    way_left = wall_present(ltrigger, lecho);
    way_right = wall_present(rtrigger, recho);
    way_front = wall_present(ftrigger, fecho);
    oneway();
  }
}

// ==========================================================================================
//                                        JUNCTION
// ==========================================================================================
void jnc(){
  // JNC stands for junction. This subroutine triggered when there are more than one possible
  // way to move. This subroutine then will trigger one of another two subrotuine depend on
  // possible way openned. 
  if(decision == 2){
    // When there are only 2 possible ways openned, two_way subroutine will be triggered.
    // This case most likely to happened in most junction casses.
    two_way();
  }
  else if(decision == 3){
    // When there are 3 possible ways oppened, three_way subroutine will be triggered.
    // It'll never happened in any 5x5 maze. So that this three_way subroutine won't
    // be developed.
    three_way();
  }
}

// ==========================================================================================
//                                         2-WAY
// ==========================================================================================
void two_way(){
  // This subroutine will triggered when there are only two possible ways oppened.
  // Potential field will of every cell is important here in this case.

  // For every position of the robot, the potential field of every cell surrounding the robot
  // will be summoned. It'll be easier for us to determined where to go with this variable.
  int pot_north = pot_field[x_cor][y_cor+1];
  int pot_east  = pot_field[x_cor+1][y_cor];
  int pot_south = pot_field[x_cor][y_cor-1];
  int pot_west  = pot_field[x_cor-1][y_cor];

  // Every possible way in the junction need to be listed so the program will know exactly
  // where to move.
  // Here are some possible way:
  
  // ------------------
  // Left way BLOCKED
  // ------------------
  if(way_left == 0){
    
    // Facing north
    if(face == north){
      //Deadend: Front
      if(dead_end[x_cor][y_cor+1]==1){
        rotate_right();
        forward();
      }
      //Deadend: Right
      else if(deadend[x_cor+1][y_cor]==1){
        forward();
      }
      //Potential Field involved
      else if(pot_north <= pot_east){
        forward(); 
      }
      else if(pot_north > pot_east){
        rotate_right();
        forward();
      }
    }
    
    // Facing east
    else if(face == east){
      //Deadend: Front
      if(dead_end[x_cor+1][y_cor]==1){
        rotate_right();
        forward();
      }
      //Deadend: Right
      else if(deadend[x_cor][y_cor-1]==1){
        forward();
      }
      //Potential Field involved
      else if(pot_east <= pot_south){
        forward();
      }
      else if(pot_east > pot_south){
        rotate_right();
        forward();
      }
    }
    
    // Facing South
    else if(face == south){
      //Deadend: Front
      if(dead_end[x_cor][y_cor-1]==1){
        rotate_right();
        forward();
      }
      //Deadend: Right
      else if(deadend[x_cor-1][y_cor]==1){
        forward();
      }
      //Potential Field involved
      else if(pot_south <= pot_west){
        forward();
      }
      else if(pot_south < pot_west){
        rotate_right();
        forward();
      }
    }
    
    // Facing west
    else if(face == west){
      //Deadend: Front
      if(dead_end[x_cor-1][y_cor]==1){
        rotate_right();
        forward();
      }
      //Deadend: Right
      else if(deadend[x_cor][y_cor+1]==1){
        forward();
      }
      //Potential Field involved
      else if(pot_west <= pot_east){
        forward();
      }
      else if(pot_west > pot_east){
        rotate_right();
        forward();
      }
    }
  }
  
  // ------------------
  // front way BLOCKED
  // ------------------
  else if(way_left == 1 && way_right == 1){
    // Facing north
    if(face == north){
      //Deadend: Left
      if(dead_end[x_cor-1][y_cor]==1){
        rotate_right();
        forward();
      }
      //Deadend: Right
      else if(deadend[x_cor+1][y_cor]==1){
        rotate_left();
        forward();
      }
      //Potential Field involved
      else if(pot_west <= pot_east){
        rotate_left();
        forward();
      }
      else if(pot_west > pot_east){
        rotate_right();
        forward();
      }
    }
    
    // Facing east
    else if(face == east){
      //Deadend: Left
      if(dead_end[x_cor][y_cor+1]==1){
        rotate_right();
        forward();
      }
      //Deadend: Right
      else if(deadend[x_cor][y_cor-1]==1){
        rotate_left();
        forward();
      }
      //Potential Field involved
      else if(pot_north <= pot_south){
        rotate_left();
        forward();
      }
      else if(pot_north > pot_south){
        rotate_right();
        forward();
      }
    }
    
    // Facing South
    else if(face == south){
      //Deadend: Left
      if(dead_end[x_cor+1][y_cor]==1){
        rotate_right();
        forward();
      }
      //Deadend: Right
      else if(deadend[x_cor-1][y_cor]==1){
        rotate_left();
        forward();
      }
      //Potential Field involved
      else if(pot_east <= pot_west){
        rotate_left();
        forward();
      }
      else if(pot_east > pot_west){
        rotate_right();
        forward();
      }
    }
    
    // Facing west
    else if(face == west){
      //Deadend: Left
      if(dead_end[x_cor][y_cor-1]==1){
        rotate_right();
        forward();
      }
      //Deadend: Right
      else if(deadend[x_cor][y_cor+1]==1){
        rotate_left();
        forward();
      }
      //Potential Field involved
      else if(pot_south <= pot_north){
        rotate_left();
        forward();
      }
      else if(pot_south > pot_north){
        rotate_right();
        forward();
      }
    }
  }
  
  // ------------------
  // right way BLOCKED
  // ------------------
  else if(way_left == 1 && way_front == 1){
   // Facing North
   if(face == north){
    //Deadend: Left
      if(dead_end[x_cor-1][y_cor]==1){
        forward();
      }
      //Deadend: Front
      else if(deadend[x_cor][y_cor+1]==1){
        rotate_left();
        forward();
      }
      //Potential Field involved
      else if(pot_west <= pot_north){
      rotate_left();
      forward();
    }
    else if(pot_west > pot_north){
      forward();
    }
   }

   // Facing East
   else if(face == east){
    //Deadend: Left
      if(dead_end[x_cor][y_cor+1]==1){
        forward();
      }
      //Deadend: Front
      else if(deadend[x_cor+1][y_cor]==1){
        rotate_left();
        forward();
      }
      //Potential Field involved
      else if(pot_north <= pot_east){
      rotate_left();
      forward();
    }
    else if(pot_north == pot_east){
      forward();
    }
   }
   
   // Facing South
   else if(face == south){
    //Deadend: Left
      if(dead_end[x_cor+1][y_cor]==1){
        forward();
      }
      //Deadend: Front
      else if(deadend[x_cor][y_cor-1]==1){
        rotate_left();
        forward();
      }
      //Potential Field involved
      else if(pot_east <= pot_south){
      rotate_left();
      forward();
    }
    else if(pot_east > pot_south){
      forward();
    }
   }
   
   // Facing West
   else if(face == west){
    //Deadend: Left
      if(dead_end[x_cor][y_cor-1]==1){
        forward();
      }
      //Deadend: Front
      else if(deadend[x_cor-1][y_cor]==1){
        rotate_left();
        forward();
      }
      //Potential Field involved
      else if(pot_south <= pot_west){
      rotate_left();
      forward();
    }
    else if(pot_south > pot_west){
      forward();
    }
   }
  }
}

// ==========================================================================================
//                                         3-way   (THIS PROJECT WASS ABANDONED)
// ==========================================================================================
void three_way(){
  // This subroutine will triggered when there are 3 possible way oppened. 
  // Currently this subroutine was abandoned because this posibility will never happened
  // in 5x5 maze. Please kindly fill every possibility like in 2-way subroutine.
  
  // Dummy variable to make your life easier
  int pot_north = pot_field[x_cor][y_cor+1];
  int pot_east  = pot_field[x_cor+1][y_cor];
  int pot_south = pot_field[x_cor][y_cor-1];
  int pot_west  = pot_field[x_cor-1][y_cor];

  if(face == north){
    if(pot_west < pot_north && pot_west < pot_east){
      rotate_left();
    }
    else if(pot_north < pot_west && pot_north < pot_east){
      forward();
    }
    else if(pot_east < pot_north && pot_east < pot_west){
      rotate_right();
    }
  }
}

// ==========================================================================================
//                                     ROTATE LEFT
// ==========================================================================================
void rotate_left(){
  // Rotate 90 degree ccw
  // This subroutine act as one of many main program used to move the micromouse

  // The PWM need to be reset to initial pwm
  // 150 is the optimum pwm to rotate
  left_pwm = 150;
  right_pwm = 150;
  
  // Reset the ticks to 0 to count the rotation
  ticks_r = 0; 
  ticks_l = 0;

  // The micromouse will rotate 20 ticks (90 degree)
  while ((ticks_r < 20) && (ticks_l < 20)){
    // 1 Full rotation makes 80 ticks. So, 20 ticks 
    // Left wheel backward, right wheel forward.
    analogWrite(in3, 0);
    analogWrite(in4, left_pwm);
    analogWrite(in1, right_pwm); 
    analogWrite(in2, 0); 

    // If right ticks is higher than left ticks. It means that right is faster 
    // then left. So the left pwm need to be loower. It'll help stabilize
    // the movement of each wheel so that the rotation is exactly 90 degree.
    if (ticks_r > ticks_l){ 
      right_pwm = right_pwm - diff_pwm;
      if (right_pwm == 0){
        right_pwm = 120; 
      }
    }
    else if (ticks_r < ticks_l){ 
      right_pwm = right_pwm + diff_pwm;
     }
  }

  // The face will be updated.
  // It's important to update the face since it'll affect the decision.
  if(face == north){
    face = west;
  }
  else if (face == west){
    face = south;
  }
  else if (face == south){
    face = east;
  }
  else if (face == east){
    face = north;
  }
}

// ==========================================================================================
//                                   ROTATE RIGHT
// ==========================================================================================
void rotate_right(){
  // Rotate 90 degree cw
  // This subroutine act as one of many main program used to move the micromouse

  // The PWM need to be reset to initial pwm
  // 150 is the optimum pwm to rotate
  left_pwm = 150;;
  right_pwm = 150;
  
  // Reset the ticks to 0 to count the rotation
  ticks_r = 0; 
  ticks_l = 0;

  // The micromouse will rotate 20 ticks (90 degree)
  while ((ticks_r < 20) && (ticks_l < 20)){
    // 1 Full rotation makes 80 ticks. So, 20 ticks 
    // Left wheel backward, right wheel forward.
    analogWrite(in3, left_pwm);
    analogWrite(in4, 0);
    analogWrite(in1, 0); 
    analogWrite(in2, right_pwm); 

    // If right ticks is higher than left ticks. It means that right is faster 
    // then left. So the left pwm need to be loower. It'll help stabilize
    // the movement of each wheel so that the rotation is exactly 90 degree.
    if (ticks_r > ticks_l){ //case kanan lebih cepat
      right_pwm = right_pwm - diff_pwm;
      if (right_pwm == 0){
        right_pwm = 120; 
      }
    }
    else if (ticks_r < ticks_l){
      right_pwm = right_pwm + diff_pwm;
     }
  }

  // The face will be updated.
  // It's important to update the face since it'll affect the decision.
  if (face == north){
    face = east;
  }
  else if (face == east){
    face = south;
  }
  else if (face == south){
    face = west;
  }
  else if (face == west){
    face = north;
  }
}

// ==========================================================================================
//                                     HALF TURN
// ==========================================================================================
void half_turn(){
  // Rotate 90 degree cw
  // This subroutine act as one of many main program used to move the micromouse

  // The PWM need to be reset to initial pwm
  // 150 is the optimum pwm to rotate
  left_pwm = 150;
  right_pwm = 150;
  
  // Reset the ticks to 0 to count the rotation
  ticks_r = 0; 
  ticks_l = 0;

  // The micromouse will rotate 40 ticks (180 degree)
  while ((ticks_r < 40) && (ticks_l < 40)){
    // 1 Full rotation makes 80 ticks. So, 40 ticks 
    // Left wheel backward, right wheel forward.
    analogWrite(in3, 0);
    analogWrite(in4, left_pwm);
    // Roda kanan maju
    analogWrite(in1, right_pwm); 
    analogWrite(in2, 0); 

    // If right ticks is higher than left ticks. It means that right is faster 
    // then left. So the left pwm need to be loower. It'll help stabilize
    // the movement of each wheel so that the rotation is exactly 90 degree.
    if (ticks_r > ticks_l){ //case kanan lebih cepat
      right_pwm = right_pwm - diff_pwm;
      if (right_pwm == 0){
        right_pwm = 120; 
      }
    }
    else if (ticks_r < ticks_l){ //case kanan lebih lambat
      right_pwm = right_pwm + diff_pwm;
     }
  }

  // The face will be updated.
  // It's important to update the face since it'll affect the decision.
  if(face == north){
    face = south;
  }
  else if (face == west){
    face = east;
  }
  else if (face == south){
    face = north;
  }
  else if (face == east){
    face = west;
  }
}

// ==========================================================================================
//                                        FORWARD
// ==========================================================================================
void forward(){
  // This subroutine makes micromouse move 1 cell long (92 ticks)
  // Micromouse will count their initial position due to the wall and move forward.
  // Then, the micromouse count its current position dan distance to wall.
  // The difference between initial position and current position makes micromouse stabilize itself.

  // Trail Flag marker
  trail[x_cor][y_cor]++; 

  // New PWM type variable
  // This variable is quite unique since it was made for micromouse to stabilize it's forward movement.
  int normal_pwm = 95;
  int faster_pwm = 105;

  // The anchor for counting the distance is either left sensor or right sensor. It depend on wether
  // the left sensor is closer or right sensor that closer to the wall.
  int left_hole = wall_read(ltrigger, lecho);
  int right_hole = wall_read(rtrigger, recho);
  int a, b;

  // Reset ticks
  ticks_r = 0;
  ticks_l = 0;

  // While was made to make to maintain the micromouse to move 1 ticks
  while((ticks_r < 92) && (ticks_l < 92)){
    // Case 1: Left sensor is nearer to the wall.
    if(left_hole < right_hole){
      analogWrite(in3, 0);
      analogWrite(in4, 0);
      analogWrite(in1, 0);
      analogWrite(in2, 0);
      delay(200);
      a = wall_read(ltrigger, lecho);
    
      analogWrite(in3, normal_pwm);
      analogWrite(in4, 0);
      analogWrite(in1, normal_pwm);
      analogWrite(in2, 0);
      delay(200);
    
      analogWrite(in3, 0);
      analogWrite(in4, 0);
      analogWrite(in1, 0);
      analogWrite(in2, 0);
      delay(200);
      b = wall_read(ltrigger, lecho);

      if(a<b) {
        analogWrite(in3, faster_pwm);
        analogWrite(in4, 0);
        analogWrite(in1, normal_pwm);
        analogWrite(in2, 0);
        delay(200);
      }
      else if(a>b) {
        analogWrite(in3, normal_pwm);
        analogWrite(in4, 0);
        analogWrite(in1, faster_pwm);
        analogWrite(in2, 0);      
        delay(200);
      }
    }
    // Case 2: Right sensor is nearer to the wall
    if(right_hole < left_hole){
      analogWrite(in3, 0);
      analogWrite(in4, 0);
      analogWrite(in1, 0);
      analogWrite(in2, 0);
      delay(200);
      a = wall_read(rtrigger, recho);
    
      analogWrite(in3, normal_pwm);
      analogWrite(in4, 0);
      analogWrite(in1, normal_pwm);
      analogWrite(in2, 0);
      delay(200);
    
      analogWrite(in3, 0);
      analogWrite(in4, 0);
      analogWrite(in1, 0);
      analogWrite(in2, 0);
      delay(200);
      b = wall_read(rtrigger, recho);

      if(a<b) {
        analogWrite(in3, normal_pwm);
        analogWrite(in4, 0);
        analogWrite(in1, faster_pwm);
        analogWrite(in2, 0);
        delay(200);
      }
      else if(a>b) {
        analogWrite(in3, faster_pwm);
        analogWrite(in4, 0);
        analogWrite(in1, normal_pwm);
        analogWrite(in2, 0);      
        delay(200);
      }
    }
  }
  
  // Status Updates
  trail[x_cor][y_cor]++;
  
  // Coordinate Updates
  if (face == north){
    y_cor++;
  }
  else if(face == east){
    x_cor++;
  }
  else if(face == south){
    y_cor--;
  }
  else if(face == west){
    x_cor--;
  }
  
  steps++;
}

// ==========================================================================================
//                                        WALL READ
// ==========================================================================================
int wall_read (int trigPin, int echoPin){
  // This subroutine is needed to know the distance between sensors and wall
  // It's quite the same with wall_present, but it return the distance value not the bool.
  // value. 
  int duration, distance;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.34 / 2;
  // Returned value is in mm unit.
  return distance;
}

// ==========================================================================================
//                                          Oop
// ==========================================================================================
void Oop(){
  // it's a simple subroutine to stop the motors.
  analogWrite(in1, 0);
  analogWrite(in2, 0);
  analogWrite(in3, 0);
  analogWrite(in4, 0);
}

// ==========================================================================================
//                                          DATAPRINT
// ==========================================================================================
void dataprint(){
  // This subroutine was made to help troubleshoot when algorithm error occured.
  // By knowing the value of each coordinate in serial monitor (9600), it's easy to determined
  // type and place of error.
  Serial.print(x_cor);
  Serial.print("\t");
  Serial.print(y_cor);
  Serial.print("\t");
  // Print arah
  if (face == 0){
    Serial.print("North");
  }
  else if (face == 1){
    Serial.print("East");
  }
  else if (face == 2){
    Serial.print("South");
  }
  else if (face == 3){
    Serial.print("West");
  }
  Serial.print("\t\t");
  Serial.print(steps);
  Serial.print("\t\t");
  Serial.print(trail[x_cor][y_cor]);
  Serial.print("\t\t");
  Serial.print(deadend[x_cor][y_cor]);
  Serial.print("\t\t");
  Serial.print(junction[x_cor][y_cor]);
  Serial.print("\n");
}

// ==========================================================================================
//                                      TICKS COUNT
// ==========================================================================================
// These subroutine was made to count ticks every time IR sensors triggered. These ticks may contain
// many informations such as: to know which motor was faster the which; to know how far the micromouse
// has moved; and to stabilize its movement.
void add_right() {
   ticks_r++;
}

void add_left(){
  ticks_l++;
}
