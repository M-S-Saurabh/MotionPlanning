class Bird{
  PVector position;
  PVector velocity;
  PVector acceleration;
  static final float radius = 3.0f;
  
  GraphNode start, goal;
  ArrayList<GraphNode> path;
  int currentNode = 0;
  
  ArrayList<PShape> shapes;
  int shapeIter = 0;
  color colour = color(225, 225, 225);
  
  float INFLUENCE_RADIUS = 30.0;
  float mass = 1;
  float maxSpeed = 5;
  float K_goal = 60;
  float K_obstacle = 500;
  float K_avoidance = 500;
  float K_alignment = 400;
  float K_cohesion = 30;
  
  Bird(ArrayList<PShape> shapes, ArrayList<Obstacle> obstacles){
    this.position = new PVector(
      random(-planeH/2, planeH/2),
      random(-worldH, -100),
      random(-planeW/2, planeW/2)
    );
    this.start = new GraphNode(this.position);
    Obstacle obstacle = obstacles.get(0);
    this.goal = new GraphNode(sphereSample(obstacle.position, obstacle.radius));
      
    this.velocity = new PVector(0,0,0);
    this.acceleration = new PVector(0,0,0);
    this.shapes = shapes;
    this.path = new ArrayList();
  }
  
  void setPath(ArrayList<GraphNode> shortestPath){
    this.path = (ArrayList<GraphNode>) shortestPath.clone();
    this.currentNode = 0;
  }
  
  void replan(boolean saveTree){
    this.setPath(findRRT3D(this.position, this.goal, saveTree));
    if(this.path.size() == 0){
      println("goal:"+ goal.location);
    }
  }
  
  void computeForces(PVector target, ArrayList<Bird> birds, ArrayList<Obstacle> obstacles){
    PVector goalVelocity = PVector.sub(target, position);
    goalVelocity.setMag(10);
    PVector F_goal = PVector.sub(goalVelocity, velocity);
    F_goal.mult(K_goal);
    
    PVector boidForce = new PVector(0,0,0); 
    PVector centroid = new PVector(0,0,0); int nbrCount = 0;
    for(Bird bird: birds){
      float separation = bird.position.dist(this.position);
      if(separation > 0 && separation < INFLUENCE_RADIUS){
        centroid.add(bird.position); nbrCount++;
        // collision avoidance
        boidForce.add(PVector.sub(this.position, bird.position).mult(K_avoidance/sq(separation)));
        // velocity alignment
        boidForce.add(PVector.sub(bird.velocity, this.velocity).mult(K_alignment/sq(separation)));
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
  
  PVector sphereSample(PVector center, float r){
    float u = random(0,1), v = random(0,1);
    float theta = 2*PI*u, phi = acos(2*v-1);
    PVector sample = new PVector(r*sin(theta)*cos(phi), -abs(r*sin(theta)*sin(phi)), r*cos(theta));
    //sample.mult(1.001);
    sample.add(center);
    return sample;
  }
  
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
      for(Obstacle obstacle: obstacles){
        //float centerX = 300.0, centerY = 100.0, centerZ = 0.0;
        //strokeWeight(2); stroke(0,0,255);
        //line(centerX+position.x, centerY+position.y-5, centerZ+position.z, centerX+target.x, centerY+target.y-5, centerZ+target.z);
        boolean obstacleHit = lineCircleIntersect(this.position, target, obstacle.position, obstacle.radius+radius);
        if(obstacleHit){noObstacles = false; break;}
      }
      if(noObstacles){ return i;}
    }
    return -1;
  }
  
  void changeGoal(Obstacle obstacle){
    this.goal = new GraphNode(sphereSample(obstacle.position, obstacle.radius));
    this.replan(false);
  }
  
  void setDanceMotion(){
    this.goal = new GraphNode( new PVector(
      random(-planeH/2, planeH/2),
      random(-worldH/2, -50),
      random(-planeW/2, planeW/2)
    ));
    this.replan(false);
  }
  
  void move(float dt, ArrayList<Bird> birds, ArrayList<Obstacle> obstacles){
    if(currentNode <= this.path.size()-1){
      //print("currentNode before:"+currentNode);
      int tempCurrentNode = currentNode;
      if(currentNode < path.size()-1){
        currentNode = lookAhead(obstacles);
      }
      //println(" after:"+currentNode+"\n");
      if(currentNode == -1){
        replan(false);
      }
      
      PVector target = path.get(currentNode).location;
      if(position.dist(target) < 0.1){
        this.position = target.copy();
        currentNode++;
        return;
      }
      PVector oldPosition = position.copy();
      computeForces(target, birds, obstacles);
      eulerianIntegrate(dt);
    }
    
    if(this.position.dist(this.goal.location) < 20){
      int chance = int(random(3));
      if(chance == 1){
        //int index = int(random(obstacles.size()));
        //this.changeGoal(obstacles.get(index));
        this.setDanceMotion();
      }
    }
  }
  
  void drawHighlight(){
    pushMatrix();
    PVector center = new PVector(centerX+position.x, centerY+position.y, centerZ+position.z);
    fill(255,0,0);stroke(255,0,0); strokeWeight(5);
    point(center.x, center.y-5, center.z);
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
    if(velocity.mag() > 1){
      object = chooseShape();
      float theta = atan2(this.velocity.x, this.velocity.z);
      rotateY(-theta);
    }
    
    noStroke();
    scale(0.5);
    object.setFill(color(255,255,255,255));
    specular(129,123,105);
    shininess(10);
    shape(object);
    popMatrix();
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
  
  void eulerianIntegrate(float dt){
    this.velocity.add(PVector.mult(acceleration, dt));
    this.velocity.limit(this.maxSpeed);
    this.position.add(PVector.mult(velocity, dt));
  }
}
