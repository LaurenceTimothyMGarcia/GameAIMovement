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
   
   ArrayList<PVector> waypt;
   
   float currentAccel;
   float currentRotAccel;
   
   float distToTarget = 0;
   
   float distRatio;
   float rotRatio;
   
   float maxSpeed;
   float maxRotSpeed;
   
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
        float distanceY = target.y - kinematic.getPosition().y;
        float distanceX = target.x - kinematic.getPosition().x;
        
        //angle between boid to target destination
        float targetRotation = atan2(distanceY, distanceX);
        
        //get direction of target position relative to Boid's front
        float angleToTarget = normalize_angle_left_right(targetRotation - kinematic.getHeading());
        
        float currDistToTarget = target.dist(kinematic.getPosition());
        
        //sets up the overall distance between the 
        if (distToTarget <= currDistToTarget)
        {
          distToTarget = currDistToTarget;
        }
        
        distRatio = currDistToTarget / distToTarget;
        rotRatio = angleToTarget / PI;
        
        if (kinematic.getSpeed() > maxSpeed)
        {
          maxSpeed = kinematic.getSpeed();
        }
        
        currentAccel = acceleration * dt * distToTarget * 10;
        
        if (distRatio < 0.5)
        {
          currentAccel *= -1;
          
          if (kinematic.getSpeed() <= maxSpeed / 2)
          {
            currentAccel = 0;
          }
        }
        
        if (angleToTarget > 0)
        {
          currentRotAccel = rotational_acceleration * dt * abs(rotRatio);
        }
        else if (angleToTarget < 0)
        {
          currentRotAccel = -rotational_acceleration * dt * abs(rotRatio);
        }
        else if (angleToTarget <= 0.05 && angleToTarget >= -0.05)
        {
          currentRotAccel = -kinematic.getRotationalVelocity();
        }
        
        if (currDistToTarget <= 5)
        {
          currentAccel = 0;
          currentRotAccel = 0;
          maxSpeed = 0;
          distToTarget = 0;
        }
        
        if (waypt != null && waypt.size() > 1 && currDistToTarget < 10)
        {
          maxSpeed = 0;
          maxRotSpeed = 0;
          waypt.remove(0);
          follow(waypt);
        }
        
        kinematic.increaseSpeed(currentAccel, currentRotAccel);
        
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
        //print("overall Distance: " + distToTarget + "\n");
        //print("current Distance: " + currDistToTarget + "\n");
        //print("ratio Distance: " + ratioDistance + "\n");
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
