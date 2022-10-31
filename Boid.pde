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
   
   float target_rotational_velocity;
   float max_rotational_velocity;
   float max_angle = 55;
   
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
        
        //Setting base acceleration
        current_accel = acceleration;
        
        // set base rotational speed
        current_rotational_accel = rotational_acceleration;
        
        // get absolute direction of target position
        float targetRotation = atan2(distance_y, distance_x);
        
        // get direction of target position relative to Boid's front; now (hopefully) leftwards movement is a positive angle and rightwards is negative
        float directionRotation = normalize_angle_left_right(targetRotation - kinematic.getHeading());
        
        target_rotational_velocity = lerp(0.0, max_rotational_velocity, abs(directionRotation) / max_angle);
        
        if (abs(kinematic.getRotationalVelocity()) < abs(target_rotational_velocity))
        {
          //increase acceleration
          current_rotational_accel = -1 * abs(directionRotation) / max_angle;
        }
        else if (abs(kinematic.getRotationalVelocity()) > abs(target_rotational_velocity))
        {
          //decrease acceleration
          current_rotational_accel = -1 * abs(directionRotation) / max_angle;
        }
        
        if (directionRotation > max_angle)
        {
          current_rotational_accel = 1;
        }
        
        // orient direction of acceleration properly (I.E. if moving leftwards set the acceleration to be towards left
        // Probably need to implement the fix here
        if (directionRotation < 0)
        {
          //current_rotational_accel *= -1;  //increasing it will tighten the wiggle
        }
        
        // now the boid's angular acceleration is towards the target angle; when the target angle is close, the boid should instead start decelerating
        if (abs(distance_y) < 20 && abs(distance_x) < 20)
        //if (kinematic.getRotationalVelocity() > current_rotational_accel)
        {
          current_accel = -acceleration * 2;
          
          if (waypt != null && waypt.size() > 0)
          {
            waypt.remove(0);
            follow(waypt);
          }
          else if (kinematic.getSpeed() <= 0)
          {
            current_accel = 0;
            current_rotational_accel = -kinematic.getRotationalVelocity();
            //kinematic.increaseSpeed(current_accel, -kinematic.getRotationalVelocity());
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
        
        print("target: " + targetRotation + "\n");
        print("direction: " + directionRotation + "\n");
        print("velocity: " + kinematic.getSpeed() + "\n");
        print("rotational velocity: " + kinematic.getRotationalVelocity() + "\n");
        print("acceleration: " + current_accel + "\n");
        print("rotational acceleration: " + current_rotational_accel + "\n");
        print("t: " + (abs(directionRotation) / max_angle) + "\n");
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
