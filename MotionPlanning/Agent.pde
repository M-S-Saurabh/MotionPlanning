class Agent{
  PVector position;
  PVector velocity;
  PVector targetVel;
  PVector acceleration;
  GraphNode start, goal;
  float mass = 1;
  static final float radius = 5.0f;
  static final float INFLUENCE_RADIUS = 20.0;
  color colour = color(220, 71, 48);
  ArrayList<GraphNode> path;
  int currentNode = 0;
  
  Agent(PVector position, PVector goal){
    this.position = position;
    this.velocity = new PVector();
    this.acceleration = new PVector();
    this.start = new GraphNode(position);
    this.goal = new GraphNode(goal);
    this.path = new ArrayList();
  }
  
  void computeForces(PVector target, ArrayList<Agent> agents, ArrayList<Obstacle> obstacles){
    target.y = this.position.y;
    PVector targetVelocity = PVector.sub(target, this.position).normalize().mult(10);
    this.targetVel = targetVelocity.copy();
    this.velocity = targetVelocity.copy();
    PVector boidForce = new PVector(0,0,0); 
    PVector centroid = new PVector(0,0,0); int nbrCount = 0;
    for(Agent agent: agents){
      float separation = agent.position.dist(this.position);
      if(separation > 0 && separation < INFLUENCE_RADIUS){
        centroid.add(agent.position); nbrCount++;
        // collision avoidance
        boidForce.add(PVector.sub(this.position, agent.position).mult(2000/sq(separation)));
        // velocity alignment
        boidForce.add(PVector.sub(agent.velocity, this.velocity).mult(100/sq(separation)));
      }
    }
    if(nbrCount > 0){ 
      centroid.mult(1.0/nbrCount);
      boidForce.add(PVector.sub(centroid, this.position).mult(25));
    }
    
    PVector obstacleForce = new PVector(0,0,0); 
    for(Obstacle obstacle: obstacles){
      float separation = obstacle.position.dist(this.position)-obstacle.radius;
      if(separation < INFLUENCE_RADIUS){
        boidForce.add(PVector.sub(this.position, obstacle.position).mult(2000/sq(separation)));
      }
    }
    PVector totalForce = PVector.add(obstacleForce, boidForce);
    this.acceleration = PVector.mult(totalForce, 1/mass);
  }
  
  void eulerianIntegrate(float dt){
    this.velocity.add(PVector.mult(acceleration, dt));
    this.position.add(PVector.mult(velocity, dt));
  }
  
  // Taken from CCD code posted by Dr.Guy on canvas.
  boolean lineCircleIntersect(PVector start, PVector end, PVector center, float radius){
    PVector toEnd = PVector.sub(end, start);
    float lineSegmentLength = toEnd.mag();
    toEnd.normalize(); // V
    PVector toCenter = PVector.sub(center, start); // W
    
    float a = 1; // ||V||^2 :since V is normalized
    float b = -2 * toEnd.dot(toCenter); // 2 * (P0-C) 
    float c = toCenter.magSq() - sq(radius); // ||P0-C||^2 - r^2
    float D = sq(b) - 4*a*c;
    
    boolean colliding = false;
    if(D>=0){
      float t = (-b - sqrt(D))/(2*a);
      if (t > 0 && t < lineSegmentLength){
        colliding = true;
      }
    }
    return colliding;
  }
  
  int lookAhead(ArrayList<Obstacle> obstacles){
    for(int i=currentNode; i<path.size(); i++){
      PVector target = path.get(i).location;
      for(Obstacle obstacle: obstacles){
        if(lineCircleIntersect(this.position, target, obstacle.position, obstacle.radius+radius)){
          return i-1;
        }
      }
    }
    return path.size()-1;
  }
  
  
  void move(float dt, ArrayList<Agent> agents, ArrayList<Obstacle> obstacles){
    if(currentNode <= path.size()-1){
      if(currentNode < path.size()-1){
        currentNode = lookAhead(obstacles);
      }
      PVector target = path.get(currentNode).location;
      if(position.dist(target) < (radius+0.1)){
        //this.position = target.copy();
        currentNode++;
        return;
      }
      PVector oldPosition = position.copy();
      computeForces(target, agents, obstacles);
      eulerianIntegrate(dt);
      if(position.dist(oldPosition) < 0.01){
        //println(position);
      }
    }
  }
  
  void draw(float centerX, float centerY, float centerZ){
    //drawTriangles(centerX, centerY, centerZ);
    drawSpheres(centerX, centerY, centerZ);
  }
  
  void drawSpheres(float centerX, float centerY, float centerZ){
    PVector center = new PVector(centerX+position.x, centerY+position.y, centerZ+position.z);
    pushMatrix();
    fill(colour);
    stroke(colour);
    translate(center.x, center.y-5, center.z);
    specular(204,204,204); shininess(5.0);
    sphere(3);
    strokeWeight(2);
    line(0,0,0, velocity.x, velocity.y, velocity.z);
    endShape();
    popMatrix();
  }
  
  void drawTriangles(float centerX, float centerY, float centerZ){
    float size = 10; 
    PVector direction = this.velocity.copy();
    if(this.velocity.mag() == 0){
      direction = new PVector(0,0,1);
    }
    direction.setMag(size);
    PVector base = direction.cross(new PVector(0,1,0));
    base.setMag(size);
    PVector center = new PVector(centerX+position.x, centerY+position.y-1, centerZ+position.z);
    PVector a = PVector.add(center, direction);
    base.mult(0.5);
    PVector b = PVector.add(center, base);
    base.mult(-1);
    PVector c = PVector.add(center, base);
    
    pushMatrix();
    fill(colour);
    stroke(colour);
    //translate(center.x, center.y-5, center.z);
    beginShape(TRIANGLES);
    vertex(b.x, b.y, b.z);
    vertex(c.x, c.y, c.z);
    vertex(a.x, a.y, a.z);
    endShape();
    popMatrix();
  }
  
  void setPath(ArrayList<GraphNode> shortestPath){
    this.path = (ArrayList<GraphNode>) shortestPath.clone();
  }
}
