/// In this file, you will have to implement seek and waypoint-following
/// The relevant locations are marked with "TODO"

class Crumb
{
  PVector position;
  Crumb(PVector position)
  {
     this.position = position;
  }
  void draw()
  {
     fill(255);
     noStroke(); 
     circle(this.position.x, this.position.y, CRUMB_SIZE);
  }
}

class Boid
{
   Crumb[] crumbs = {};
   int last_crumb;
   float acceleration;
   float rotational_acceleration;
   KinematicMovement kinematic;
   PVector target;
   
   // "private" variables
   ArrayList<PVector> waypt;
   
   float current_accel;
   float current_rotational_accel;
   
   //calcuate ratio to distance
   float distToTarget;  //original distance
   float currDistToTarget;  //current distance
   float ratioDistance;  //current / original
   boolean startPath = true;
   
   float target_rotational_velocity;
   static final float MAX_ANGLE_DEGREES = 55;
   float MAX_ANGLE = radians(MAX_ANGLE_DEGREES);
   
   Boid(PVector position, float heading, float max_speed, float max_rotational_speed, float acceleration, float rotational_acceleration)
   {
     this.kinematic = new KinematicMovement(position, heading, max_speed, max_rotational_speed);
     this.last_crumb = millis();
     this.acceleration = acceleration;
     this.rotational_acceleration = rotational_acceleration;
   }

   void update(float dt)
   {
     if (target != null)
     {  
        // TODO: Implement seek here
        float distance_y = target.y - kinematic.getPosition().y;
        float distance_x = target.x - kinematic.getPosition().x;
        
        currDistToTarget = target.dist(kinematic.getPosition());
        
        //if new path, need to initialize the original distance to target
        if (startPath)
        {
          distToTarget = currDistToTarget;
          startPath = false;
        }
        
        ratioDistance = currDistToTarget / distToTarget;
        
        //Setting base acceleration
        current_accel = acceleration;
        
        // set base rotational speed
        current_rotational_accel = rotational_acceleration;
        
        // get absolute direction of target position
        float targetRotation = atan2(distance_y, distance_x);
        
        // get direction of target position relative to Boid's front; now (hopefully) leftwards movement is a positive angle and rightwards is negative
        float angleToTarget = normalize_angle_left_right(targetRotation - kinematic.getHeading());
        
        // obtain a target rotational velocity
        // if the magnitude of our angle to the target >= constant MAX_ANGLE, we intend for our boid to turn as fast as possible
        // if the magnitude of our angle to target is 0 (I.E., boid is already facing the target), we don't want our boid to turn at all
        // but what if our angle's magnitude is between 0 and the max angle?
        // we use a linear interpolation to get a reasonable target velocity 
        target_rotational_velocity = lerp(0.0, kinematic.max_rotational_speed, min((abs(angleToTarget) / MAX_ANGLE),1.0));
        // we had to use abs value since I have no idea if lerp works with negative values
        // therefore, re-switch the sign if necessary
        if(angleToTarget < 0) {
          target_rotational_velocity *= -1;
        }
        
        //now we want to accelerate such that our actual velocity will approach our target velocity
        // obtain value of current rotational velocity
        float current_rotational_velocity = normalize_angle_left_right(kinematic.getRotationalVelocity());
        
        //Checks if current velocity is less than the target velocity (means you need to accelerate to the left)
        if (current_rotational_velocity < target_rotational_velocity)
        {
          // leftwards acceleration is positive
          current_rotational_accel = rotational_acceleration;
          //print("\nLess than\n");
        }
        //Checks if current velocity is greater than target velocity (means you need to accelerate to the right)
        else if (current_rotational_velocity > target_rotational_velocity)
        {
          // rightwards acceleration is negative
          current_rotational_accel = -rotational_acceleration;
          //print("\nGreater than\n");
        }
        //if your target velocity is equal to your current velocity, you don't need to change your velocity further
        else
        {
          current_rotational_accel = 0;
          //print("\nelse\n");
        }
        
        // now the boid's angular acceleration is towards the target angle 
        // when the target angle is close, the boid should instead start decelerating
        // make a ratio between current distance and overall distance, once halfway, start decelerating
        if (ratioDistance < 0.15 || (ratioDistance < 0.5 && distToTarget < 150) || (ratioDistance < 0.75 && distToTarget < 50))//new condition checks if its 10% more
        {
          current_accel = -acceleration * (1 - ratioDistance);  //either stops too early or doesnt stop in time 
          
          if (waypt != null && waypt.size() > 1)  //Goes to next waypt
          {
            current_accel = -acceleration * (1 - ratioDistance) / dt;
            waypt.remove(0);
            follow(waypt);
            distToTarget = currDistToTarget;
            startPath = true;
          }
          else if (kinematic.getSpeed() <= 0) //&& abs(distance_y) < 25 && abs(distance_x) < 25)  //Doesnt reach this state at the last node
          {
            print("Reach last state\n");
            current_accel = -kinematic.getSpeed();
            current_rotational_accel = -kinematic.getRotationalVelocity();
            //startPath = false;
          }
        }
        else
        {
          current_accel = acceleration;
        }
        
        kinematic.increaseSpeed(current_accel, current_rotational_accel);
        
        //DEBUGGING COMMENTS
        //print("TOTAL Y: " + abs(distance_y) + "\n");
        //print("TOTAL X: " + abs(distance_x) + "\n");
        
        //print("target: " + targetRotation + "\n");
        //print("angle to target: " + angleToTarget + "\n");
        //print("velocity: " + kinematic.getSpeed() + "\n");
        //print("rotational velocity: " + kinematic.getRotationalVelocity() + "\n");
        //print("target rotational velocity: " + target_rotational_velocity + "\n");
        //print("acceleration: " + current_accel + "\n");
        //print("rotational acceleration: " + current_rotational_accel + "\n");
        //print("t: " + abs(angleToTarget) / MAX_ANGLE + "\n");
        print("overall Distance: " + distToTarget + "\n");
        print("current Distance: " + currDistToTarget + "\n");
        print("ratio Distance: " + ratioDistance + "\n");
        //print("delta time: " + dt + "\n");
     }
     
     // place crumbs, do not change     
     if (LEAVE_CRUMBS && (millis() - this.last_crumb > CRUMB_INTERVAL))
     {
        this.last_crumb = millis();
        this.crumbs = (Crumb[])append(this.crumbs, new Crumb(this.kinematic.position));
        if (this.crumbs.length > MAX_CRUMBS)
           this.crumbs = (Crumb[])subset(this.crumbs, 1);
     }
     
     // do not change
     this.kinematic.update(dt);
     
     draw();
   }
   
   void draw()
   {
     for (Crumb c : this.crumbs)
     {
       c.draw();
     }
     
     fill(255);
     noStroke(); 
     float x = kinematic.position.x;
     float y = kinematic.position.y;
     float r = kinematic.heading;
     circle(x, y, BOID_SIZE);
     // front
     float xp = x + BOID_SIZE*cos(r);
     float yp = y + BOID_SIZE*sin(r);
     
     // left
     float x1p = x - (BOID_SIZE/2)*sin(r);
     float y1p = y + (BOID_SIZE/2)*cos(r);
     
     // right
     float x2p = x + (BOID_SIZE/2)*sin(r);
     float y2p = y - (BOID_SIZE/2)*cos(r);
     triangle(xp, yp, x1p, y1p, x2p, y2p);
   } 
   
   void seek(PVector target)
   {
      this.target = target;
   }
   
   void follow(ArrayList<PVector> waypoints)
   {
      // TODO: change to follow *all* waypoints
      /*if (target_count < waypt.size())
      {
        return;
      }*/
      //Unsure how to get the new waypoint after it finishes current one
      
      waypt = waypoints;
      
      if (waypt.size() <= 0)
      {
        return;
      }
      
      this.target = waypt.get(0);
      print("ArrayList Waypoint: " + waypoints.get(0) + "\n");
      
      
   }
}
