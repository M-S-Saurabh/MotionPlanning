class Agent{
  PVector position;
  PVector velocity;
  PVector targetVel;
  PVector acceleration;
  GraphNode start, goal;
  ArrayList<PShape> shapes;
  PImage texture;
  int shapeIter = 0;
  float mass = 1;
  static final float radius = 5.0f;
  float INFLUENCE_RADIUS = 20.0;
  color colour = color(220, 71, 48);
  ArrayList<GraphNode> path;
  int currentNode = 0;
  float K_goal = 20;
  
  Agent(PVector position, PVector goal, ArrayList<PShape> shapes, PImage texture){
    this.position = position;
    this.velocity = new PVector();
    this.acceleration = new PVector();
    this.start = new GraphNode(position);
    this.goal = new GraphNode(goal);
    this.path = new ArrayList();
    this.shapes = (ArrayList<PShape>) shapes.clone();
    this.texture = texture;
  }
  
  void computeForces(PVector target, ArrayList<Agent> agents, ArrayList<Obstacle> obstacles){
    //target.y = this.position.y;
    //PVector targetVelocity = PVector.sub(target, this.position).normalize().mult(10);
    //this.targetVel = targetVelocity.copy();
    //this.velocity = targetVelocity.copy();
    PVector goalVelocity = PVector.sub(target, position);
    goalVelocity.setMag(10);
    PVector F_goal = PVector.sub(goalVelocity, velocity);
    F_goal.mult(K_goal);
    
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
    totalForce.add(F_goal);
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
    for(int i=path.size()-1; i >= 0; i--){
      PVector target = path.get(i).location;
      boolean noObstacles = true;
      for(Obstacle obstacle: obstacles){ //<>//
        //float centerX = 300.0, centerY = 100.0, centerZ = 0.0;
        //strokeWeight(2); stroke(0,0,255);
        //line(centerX+position.x, centerY+position.y-5, centerZ+position.z, centerX+target.x, centerY+target.y-5, centerZ+target.z);
        boolean obstacleHit = lineCircleIntersect(this.position, target, obstacle.position, obstacle.radius+radius);
        if(obstacleHit){noObstacles = false; break;} //<>//
      }
      if(noObstacles){ return i;} //<>//
    }
    return -1;
  }
  
  
  void move(float dt, ArrayList<Agent> agents, ArrayList<Obstacle> obstacles){
    if(currentNode <= path.size()-1){
      //println("currentNode:"+currentNode);
      int tempCurrentNode = currentNode;
      if(currentNode < path.size()-1){
        currentNode = lookAhead(obstacles);
        if(currentNode == -1){
          // re-compute path.
          println("here"); //<>//
        }
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
    //drawSpheres(centerX, centerY, centerZ);
    drawShapes(centerX, centerY, centerZ);
  }
  
  void drawShapes(float centerX, float centerY, float centerZ){
    pushMatrix();
    PVector center = new PVector(centerX+position.x, centerY+position.y, centerZ+position.z);
    translate(center.x, center.y, center.z);
    if(shapeIter > shapes.size()-1){shapeIter =  shapeIter % shapes.size();}
    PShape object = shapes.get(shapeIter);
    rotateZ(PI);
    if(velocity.z > 0){
      float theta = atan2(velocity.x, velocity.z);
      rotateY(-theta);
    }
    noStroke();
    scale(2);
    //object.setFill(color(129,123,105,255));
    specular(129,123,105);
    shininess(10);
    if(texture != null){
      texture(texture);
    }
    shape(object);
    popMatrix();
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
