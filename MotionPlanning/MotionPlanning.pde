import queasycam.*;
import java.util.Queue;
import java.util.PriorityQueue;
import java.util.Stack;
import java.util.Collections;
import java.util.LinkedList;

QueasyCam cam;

float planeT = 4.0, planeW = 400.0, planeH = 400.0;
float centerX = 300.0, centerY = 100.0, centerZ = 0.0;
color landColor = color(82, 115, 84);

ArrayList<Agent> agents;
ArrayList<Obstacle> obstacles;

ArrayList<Sphere> invalidPositions;
ArrayList<PVector> milestones;
ArrayList<GraphNode> roadmap;
boolean findPath = false;
boolean displayNoPath = false;
boolean drawMap = false;
boolean isTTC = false;
int nMilestones = 200;
int minNeighbors = 5;
boolean paused = false;

void setup(){
  size(1000,1000,P3D);
  cam = new QueasyCam(this);
  cam.speed = 5; 
  cam.sensitivity = 0.5;
  lightSpecular(204,204,204);
  directionalLight(255,255,255,0,-1,-1);
  randomSeed(1);
  restart();
  // move to R keypress when interaction is added
  constructCSpace();
  sampleConfigurations();
  constructSparseGraph();
  return;
}

void spawnAgents(int numAgents, float x, float z, float lx, float lz, PVector target){
  int spawnCount = 0;
  while(spawnCount < numAgents){
    PVector start = new PVector(random(x+Agent.radius, x+lx), 0, random(z+Agent.radius, z+lz));
    if(isTTC){
      agents.add(new TTCAgent(start, target));
    }else{
      agents.add(new Agent(start, target));
    }
    spawnCount++;
  }
}

void restart(){
  findPath = false;
  displayNoPath = false;
  agents = new ArrayList();
  spawnAgents(10, -200, -100, 50, 50, new PVector(90,0,90));
  spawnAgents(10, -100, -200, 50, 50, new PVector(90,0,90));
  
  obstacles = new ArrayList();
  obstacles.add(new Obstacle(new PVector(0,0,-10), 40));
  obstacles.add(new Obstacle(new PVector(20,0,120), 40));
  obstacles.add(new Obstacle(new PVector(120,0,20), 40));
}

void constructCSpace(){
  invalidPositions = new ArrayList();
  for(Obstacle obstacle : obstacles){
    invalidPositions.add(new Sphere(obstacle.position, obstacle.radius+ Agent.radius));
  }
}

void sampleConfigurations(){
  milestones = new ArrayList();
  while(milestones.size() < nMilestones){
    PVector sample = new PVector(random(-planeW/2+5, planeW/2-5), 0, random(-planeH/2+5,planeH/2-5));
    boolean invalid = false; 
    for(Sphere s : invalidPositions){
      invalid = invalid || s.isInvalid(sample);
      if(invalid){break;}
    }
    if(!invalid){milestones.add(sample);}
  }
}

void printRoadGraph(){
  // print graph as adjacency list
  for(int i=roadmap.size()-1; i>=0; i--){
    GraphNode node = roadmap.get(i);
    print("P"+node.id+":");
    for(int j=node.neighbors.size()-1; j>=0; j--){
      GraphNode neighbor = node.neighbors.get(j);
      print(neighbor.id+" ");
    }
    print("\n");
  }
}

void constructSparseGraph(){
  roadmap =  new ArrayList();
  // Make nodes out for each milestone location.
  for(PVector milestone : milestones){
    GraphNode node = new GraphNode(milestone);
    roadmap.add(node);
  }
  // Adding startPoint and endPoint
  for(Agent agent : agents){
    roadmap.add(agent.start);
    roadmap.add(agent.goal);
  }
  
  ArrayList<ArrayList<Pair>> distance = new ArrayList();
  for(int i=0; i<roadmap.size(); i++){
    GraphNode node = roadmap.get(i);
    distance.add(new ArrayList());
    for(int j=0; j<roadmap.size(); j++){
      GraphNode neighbor = roadmap.get(j);
      float dist = node.location.dist(neighbor.location);
      Pair pair = new Pair(j, dist);
      distance.get(i).add(pair);
    }
  }
  for(int i=0; i<roadmap.size();i++){
    Collections.sort(distance.get(i));
  }
  for(int i=0; i<roadmap.size(); i++){
    GraphNode node = roadmap.get(i);
    for(int j=1; j<minNeighbors; j++){
      Pair p = distance.get(i).get(j);
      GraphNode neighbor = roadmap.get(p.index);
      node.addNeighbor(neighbor);
    }
  }
  //printRoadGraph();
}

boolean checkCollision(PVector center, float radius, PVector start, PVector end){
  PVector line = PVector.sub(end, start).normalize();
  PVector centerToStart = PVector.sub(start, center);
  PVector perpendicular = PVector.sub(centerToStart, PVector.mult(line, centerToStart.dot(line)));
  float perpendicularLength = perpendicular.mag();
  if(perpendicularLength >= radius){return false;}
  PVector perpendicularBase = PVector.add(perpendicular, center);
  float distToStart = perpendicularBase.dist(start);
  float distToEnd = perpendicularBase.dist(end);
  float lineMag = start.dist(end);
  return abs(distToStart + distToEnd - lineMag) < 0.001;
}

ArrayList<GraphNode> shortestPathBFS(GraphNode start, GraphNode end){
  Queue<ArrayList<GraphNode>> bfsPathQueue = new LinkedList();
  ArrayList<GraphNode> startPath = new ArrayList();
  startPath.add(start);
  bfsPathQueue.add(startPath);
  //print("Added "); printList(startPath);
  
  while(!bfsPathQueue.isEmpty()){
    ArrayList<GraphNode> path = bfsPathQueue.remove();
    //print("Popped "); printList(path);
    GraphNode lastNode = path.get(path.size()-1);
    if(lastNode == end){
      //println("returned"+start.location+" "+end.location);
      return path;
    }
    for(GraphNode neighbor : lastNode.neighbors){
      if(neighbor.visited){continue;}
      ArrayList<GraphNode> newPath = (ArrayList) path.clone();
      newPath.add(neighbor);
      bfsPathQueue.add(newPath);
       //print("Added "); printList(newPath);
    }
    lastNode.setVisited();
  }
  displayNoPath = true;
  return new ArrayList();
}

ArrayList<GraphNode> searchPathUCS(GraphNode start, GraphNode end){
  PriorityQueue<Path> ucsPathQueue = new PriorityQueue();
  ArrayList<GraphNode> startPath = new ArrayList();
  startPath.add(start);
  ucsPathQueue.add(new Path(startPath, 0));
  //print("Added "); printList(startPath); print("\n");
  
  while(!ucsPathQueue.isEmpty()){
    Path path = ucsPathQueue.remove();
    //print("Popped "); printList(path.list); print(":"+path.cost+"\n");
    GraphNode lastNode = path.getLastNode();
    if(lastNode == end){
      //println("returned"+start.location+" "+end.location);
      return path.list;
    }
    for(GraphNode neighbor : lastNode.neighbors){
      if(neighbor.visited){continue;}
      ArrayList<GraphNode> newPath = (ArrayList) path.list.clone();
      newPath.add(neighbor);
      float newCost = path.cost+lastNode.location.dist(neighbor.location);
      ucsPathQueue.add( new Path(newPath, newCost));
      //print("Added "); printList(newPath); print(":"+newCost+"\n");
    }
    lastNode.setVisited();
  }
  displayNoPath = true;
  return new ArrayList();
}

void printList(ArrayList<GraphNode> path){
  for(GraphNode node : path){
    print(" "+node.id);
  }
}

void drawOrigin(){
  pushMatrix();
  fill(255);
  stroke(255);
  translate(0,0,0);
  sphere(5);
  popMatrix();
}

void drawLand(){
  pushMatrix();
  fill(landColor);
  stroke(landColor);
  //center of the plane.
  translate(centerX, centerY+planeT/2, centerZ);
  box(planeW,planeT,planeH);
  popMatrix();
}

void drawEndpoint(PVector point, float h){
  pushMatrix();
  fill(77, 219, 255);
  stroke(77, 219, 255);//127,255,0); //0, 114, 233
  translate(centerX+point.x, centerY+point.y-0.1, centerZ+point.z);
  rotateX(PI/2);
  rect(-4,-4,8,8);
  popMatrix();
}

void drawObstacles(){
  if(obstacles == null || obstacles.size() <= 0){return;}
  for(Obstacle obstacle : obstacles){
    obstacle.draw(centerX, centerY, centerZ);
  }
}

void drawAgents(){
  if(agents == null || agents.size() <= 0){return;}
  for(Agent agent: agents){
    agent.draw(centerX, centerY, centerZ);
  }
}

void drawNode(GraphNode node){
  pushMatrix();
  translate(centerX+node.location.x, centerY+node.location.y-0.1, centerZ+node.location.z);
  rotateX(PI/2);
  ellipse(0,0,3,3);
  //translate(3,-10,3);
  //rotateY(-PI/2);
  //text("P:"+node.id,0,0);
  //text("p:"+node.location.x+":"+node.location.z, 0, 0);
  popMatrix();
}

void drawRoadmap(){
  if(!drawMap){return;}
  if(roadmap == null || roadmap.size() <= 0){return;}
  color yellow = color(244, 208, 63);
  fill(yellow);
  stroke(yellow);
  for(GraphNode node : roadmap){
    drawNode(node);
    strokeWeight(3);
    for(GraphNode neighbor : node.neighbors){
      line(centerX+node.location.x, centerY+node.location.y, centerZ+node.location.z,
            centerX+neighbor.location.x, centerY+neighbor.location.y, centerZ+neighbor.location.z);
    }
    strokeWeight(1);
  }
}

void update(float dt){
  if(displayNoPath){
    return;
  }
  if(!paused){
    for(Agent agent : agents){
      agent.move(dt, agents, obstacles);
    }
  }
}

void drawNoPathText(){
  pushMatrix();
  translate(centerX, centerY-100, centerZ-180);
  rotateY(-PI/2);
  text("Path from start to end does not exist with the given road-map",0,0);
  popMatrix();
}

void draw(){
  background(0);
  //drawOrigin();
  drawLand();
  for(Agent agent: agents){
    drawEndpoint(agent.start.location, 10); // start point
    drawEndpoint(agent.goal.location, 10); // stop point
  }
  drawObstacles(); // Obstacle
  drawAgents(); // Agent
  drawRoadmap();
  
  if(displayNoPath){ drawNoPathText(); }
  
  // Update Physics
  for(int i=0; i<3; i++){
    update(0.04);
  }
}

void resetGraphVisited(){
  for(GraphNode node : roadmap){
    node.resetVisited();
  }
}

void findPaths(){
  displayNoPath = false;
  for(Agent agent : agents){
    resetGraphVisited();
    //agent.setPath(shortestPathBFS(agent.start, agent.goal));
    agent.setPath(searchPathUCS(agent.start, agent.goal));
  }
}

void keyPressed(){
  if(key == 'r'){
    if(findPath){
      for(Agent agent : agents){
        agent.setPath(new ArrayList());
      }
    }else{
      //constructCSpace();
      //sampleConfigurations();
      //constructSparseGraph();
      findPaths();
    }
    findPath = !findPath;
  }
  
  if(key == '0'){
    isTTC = false;
    restart();
    constructSparseGraph();
  }
  if(key == '9'){
    isTTC = true;
    restart();
    constructSparseGraph();
  }
  if(key == 'm'){
    drawMap = !drawMap;
  }
  if(key == 'p'){
    paused = !paused;
  }
}
