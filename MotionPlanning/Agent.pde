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
  static final float radius = 3.0f;
  float INFLUENCE_RADIUS = 20.0;
  color colour = color(220, 71, 48);
  ArrayList<GraphNode> path;
  int currentNode = 0;
  
  float maxSpeed = 2.5;
  
  float K_goal = 60;
  float K_obstacle = 700;
  float K_avoidance = 700;
  float K_alignment = 100;
  float K_cohesion = 25;
  
  Agent(PVector position, PVector goal, ArrayList<PShape> shapes, PImage texture, color colour){
    this.position = position;
    this.velocity = new PVector();
    this.acceleration = new PVector();
    this.start = new GraphNode(position);
    this.goal = new GraphNode(goal);
    this.path = new ArrayList();
    this.shapes = (ArrayList<PShape>) shapes.clone();
    this.texture = texture;
    this.maxSpeed += random(-1,1);
    this.colour = colour;
  }
  
  void setGoal(PVector goalPosition){
    this.goal = new GraphNode(goalPosition);
    this.replan();
  }
  
  void computeForces(PVector target, ArrayList<Agent> agents, ArrayList<Obstacle> obstacles){
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
        boidForce.add(PVector.sub(this.position, agent.position).mult(K_avoidance/sq(separation)));
        // velocity alignment
        boidForce.add(PVector.sub(agent.velocity, this.velocity).mult(K_alignment/sq(separation)));
      }
    }
    if(nbrCount > 0){ 
      centroid.mult(1.0/nbrCount);
      boidForce.add(PVector.sub(centroid, this.position).mult(K_cohesion));
    }
    
    PVector obstacleForce = new PVector(0,0,0); 
    for(Obstacle obstacle: obstacles){
      float separation = obstacle.position.dist(this.position)-obstacle.radius;
      if(separation < INFLUENCE_RADIUS){
        boidForce.add(PVector.sub(this.position, obstacle.position).mult(K_obstacle/sq(separation)));
      }
    }
    PVector totalForce = PVector.add(obstacleForce, boidForce);
    totalForce.add(F_goal);
    this.acceleration = PVector.mult(totalForce, 1/mass);
  }
  
  void printStuff(){
    println("position:"+position);
    println("velocity:"+velocity);
    println("currentNode:"+currentNode);
    println("path size:"+path.size());
  }
  
  void eulerianIntegrate(float dt){
    this.velocity.add(PVector.mult(acceleration, dt));
    this.velocity.limit(this.maxSpeed);
    this.position.add(PVector.mult(velocity, dt));
  }
  
  int lookAhead(ArrayList<Obstacle> obstacles){
    for(int i=path.size()-1; i >= 0; i--){
      PVector target = path.get(i).location;
      boolean noObstacles = true;
      for(Obstacle obstacle: obstacles){
        //float centerX = 300.0, centerY = 100.0, centerZ = 0.0;
        //strokeWeight(2); stroke(255,0,0);
        //line(centerX+position.x, centerY+position.y-5, centerZ+position.z, centerX+target.x, centerY+target.y-5, centerZ+target.z);
        boolean obstacleHit = lineCircleIntersect(this.position, target, obstacle.position, obstacle.radius+radius);
        if(obstacleHit){noObstacles = false; break;}
      }
      if(noObstacles){ return i;}
    }
    return -1;
  }
  
  void replan(){
    this.setPath(findRRT(this.position, this.goal, false));
  }
  
  void move(float dt, ArrayList<Agent> agents, ArrayList<Obstacle> obstacles){
    if(currentNode <= path.size()-1){
      //print("currentNode before:"+currentNode);
      int tempCurrentNode = currentNode;
      if(currentNode < path.size()-1){
        currentNode = lookAhead(obstacles);
      }
      //println(" after:"+currentNode+"\n");
      if(currentNode == -1){
        replan();
      }
      
      PVector target = path.get(currentNode).location;
      if(position.dist(target) < 0.1){
        this.position = target.copy();
        currentNode++;
        return;
      }
      PVector oldPosition = position.copy();
      computeForces(target, agents, obstacles);
      eulerianIntegrate(dt);
    }
  }
  
  void draw(float centerX, float centerY, float centerZ){
    //drawTriangles(centerX, centerY, centerZ);
    //drawSpheres(centerX, centerY, centerZ);
    drawShapes(centerX, centerY, centerZ);
  }
  
  PShape chooseShape(){
    if(this.velocity.mag() < 0.1){return shapes.get(0);}    
    int sz = shapes.size();
    int frame = 5;
    if(shapeIter > (sz*frame - 1)){
      shapeIter =  shapeIter % (sz*frame);
    }
    PShape object = shapes.get(shapeIter/frame);
    shapeIter++;
    return object;
  }
  
  void drawNumber(int agentNumber){
    pushMatrix();
    PVector center = new PVector(centerX+position.x, centerY+position.y, centerZ+position.z);
    translate(center.x, center.y-10, center.z);
    rotateX(-3*PI/2);
    fill(255,0,0);
    text(agentNumber,0,0);
    popMatrix();
  }
  
  void drawShapes(float centerX, float centerY, float centerZ){
    pushMatrix();
    PVector center = new PVector(centerX+position.x, centerY+position.y, centerZ+position.z);
    translate(center.x, center.y, center.z);
    PShape object = shapes.get(0);
    
    //strokeWeight(2); stroke(255,0,0);
    //line(0,0,0, this.acceleration.x, this.acceleration.y, this.acceleration.z);
    //stroke(0,255,0);
    //line(0,0,0, this.velocity.x, this.velocity.y, this.velocity.z);
    
    rotateZ(PI);
    if(velocity.mag() > 1 && this.position.dist(this.goal.location) > 1){
      object = chooseShape();
      float theta = atan2(this.velocity.x, this.velocity.z);
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
    object.setFill(this.colour);
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
    sphere(radius);
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
    this.currentNode = 0;
  }
}
