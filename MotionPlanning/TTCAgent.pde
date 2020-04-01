class TTCAgent extends Agent{
  float K_goal = 20;
  float K_bounce = 20;
  float K_avoid = 30;
  float K_avoid_obs = 4;
  float foresight = 50;
  float maxForce = 100; 
  float INFLUENCE_RADIUS = 10.0;
  
  TTCAgent(PVector position, PVector goal, ArrayList<PShape> shapes, PImage texture, color colour){
    super(position, goal, shapes, texture, colour);
  }
  
  @Override
  void computeForces(PVector target, ArrayList<Agent> agents, ArrayList<Obstacle> obstacles){
    PVector goalVelocity = PVector.sub(target, position);
    goalVelocity.setMag(10);
    PVector F_goal = PVector.sub(goalVelocity, velocity);
    F_goal.mult(K_goal);
    
    PVector F_avoidance = new PVector(0,0,0);
    for(Agent agent : agents){
      if(agent.position.dist(position) == 0){continue;}
      float r = radius;
      float dist = PVector.sub(agent.position, this.position).mag();
      if(dist < 2*r){ r = dist/2.001;}
      PVector dEdx = dE(position, agent.position,
                        velocity, agent.velocity, r, r);
      PVector avoidanceF = PVector.mult(dEdx,-1);
      avoidanceF.limit(maxForce);
      F_avoidance.add(avoidanceF);
    }
    PVector F_obstacle = new PVector(0,0,0); 
    for(Obstacle obstacle: obstacles){
      float separation = obstacle.position.dist(this.position)-obstacle.radius;
      if(separation < INFLUENCE_RADIUS){
        F_obstacle.add(PVector.sub(this.position, obstacle.position).mult(2000/sq(separation)));
      }
    }
    
    this.acceleration.mult(0);
    this.acceleration.add(PVector.mult(F_goal, 1.0/mass));
    //println("Goal force:"+PVector.mult(F_goal, 1.0/mass).mag());
    this.acceleration.add(PVector.mult(F_avoidance, 1.0/mass));
    //println("Avoidance force:"+PVector.mult(F_avoidance, 1.0/mass).mag());
    this.acceleration.add(PVector.mult(F_obstacle, 1.0/mass));
    //println("Obstacle force:"+PVector.mult(F_obstacle, 1.0/mass).mag());
    println();
  }
  
  float getTimeToCollision(Agent agent){
    float maxT = foresight+1;
    float r = agent.radius + this.radius;
    PVector w = PVector.sub(agent.position, this.position);
    float c = w.magSq() - (r*r);
    if(c < 0){return 0;} // Agents are already colliding.
    
    PVector v = PVector.sub(agent.velocity, this.velocity);
    if(v.mag() == 0){ return foresight+1;}
    float a = v.dot(v);
    float b = 2*v.dot(w);
    float D = b*b - 4*a*c;
    
    float t1 = maxT, t2 = maxT;
    if(D > 0){
      t1 = (-b + sqrt(D)) / (2*a);
      t1 = (-b - sqrt(D)) / (2*a);
    }
    float tau = min(t1, t2);
    if(tau < 0 || tau > maxT){
      tau = maxT;
    }
    return tau;
  }
  
  PVector dE(PVector centerA, PVector centerB,
             PVector velocityA, PVector velocityB,
             float ra, float rb){
    PVector w = PVector.sub(centerB, centerA);
    PVector v = PVector.sub(velocityA, velocityB);
    float radius = ra+rb;
    float dist = w.mag();
    if(radius > dist){radius *= 0.99;}
    float a = v.dot(v);
    float b = w.dot(v);  
    float c = w.dot(w) - radius*radius;
    float discr = b*b - a*c;
    if((discr <= 0) || (a<0.001 && a > - 0.001)){
      return new PVector(0,0,0);
    }
    discr = sqrt(discr);
    float tau = (b - discr) / a;
    if(tau < 0 || tau > foresight){
      return new PVector(0,0,0);
    }
    PVector d = PVector.sub(PVector.mult(v,b), PVector.mult(w,a));
    d.mult(1.0/discr);
    float mag = K_avoid * exp(-tau/foresight) / (a * pow(tau, mass)) * (mass/tau + 1/foresight);
    return PVector.mult(PVector.sub(v, d), mag);
  }
}
