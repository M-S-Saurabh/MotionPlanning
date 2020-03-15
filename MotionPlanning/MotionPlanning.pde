import queasycam.*;
import java.util.Queue;
import java.util.Stack;
import java.util.Collections;
import java.util.LinkedList;

QueasyCam cam;

float planeT = 4.0, planeW = 200.0, planeH = 200.0;
float centerX = 300.0, centerY = 100.0, centerZ = 0.0;
color landColor = color(82, 115, 84);

Agent agent;

PVector obstaclePosition;
float obstacleRadius = 20.0;
color obstacleColor = color(165, 151, 129);

PVector startPoint, endPoint;
GraphNode startNode, endNode;

ArrayList<Sphere> invalidPositions;
ArrayList<PVector> milestones;
ArrayList<GraphNode> roadmap;
ArrayList<GraphNode> shortestPath;
int currentNode = 0;
boolean displayNoPath = false;
int nMilestones = 10;

void setup(){
  size(1000,1000,P3D);
  cam = new QueasyCam(this);
  cam.speed = 5; 
  cam.sensitivity = 0.5;
  directionalLight(0,-1,-1,255,255,255);
  
  agent = new Agent(new PVector(-90.0, 0, -90.0));
  obstaclePosition = new PVector(0,0,0);
  startPoint = new PVector(-90,0,-90);
  endPoint = new PVector(90,0,90);
  randomSeed(1);
  
  invalidPositions = new ArrayList();
  constructCSpace();
  sampleConfigurations();
  //constructDenseGraph();
  constructSparseGraph();
  shortestPath = new ArrayList();
  shortestPathBFS(startNode, endNode);
  //shortestPathDFS(startNode, endNode);
  return;
}

void constructCSpace(){
  invalidPositions.add(new Sphere(obstaclePosition, obstacleRadius+agent.radius));
}

void sampleConfigurations(){
  milestones = new ArrayList();
  while(milestones.size() < nMilestones){
    PVector sample = new PVector(random(-100,100), 0, random(-100,100));
    boolean invalid = false; 
    for(Sphere s : invalidPositions){
      invalid = invalid || s.isInvalid(sample);
      if(invalid){break;}
    }
    if(!invalid){milestones.add(sample);}
  }
}

void constructDenseGraph(){
  roadmap =  new ArrayList();
  // Make nodes out for each milestone location.
  for(PVector milestone : milestones){
    GraphNode node = new GraphNode(milestone);
    roadmap.add(node);
  }
  // Adding startPoint and endPoint
  startNode = new GraphNode(startPoint);
  roadmap.add(startNode);
  endNode = new GraphNode(endPoint);
  roadmap.add(endNode);
  
  // Make a fully connected graph.
  for(GraphNode node : roadmap){
    for(GraphNode neighbor : roadmap){
      if(node == neighbor){continue;}
      node.addNeighbor(neighbor);
    }
  }
  // Prune the graph removing any edges that intersect with the obstacle.
  for(int i=roadmap.size()-1; i>=0; i--){
    GraphNode node = roadmap.get(i);
    for(int j=node.neighbors.size()-1; j>=0; j--){
      GraphNode neighbor = node.neighbors.get(j);
      boolean isCollision = checkCollision(obstaclePosition, obstacleRadius, node.location, neighbor.location);
      if(isCollision){
        node.removeNeighbor(j);
        neighbor.removeNeighbor(node);
      }
    }
  }
  //printRoadGraph(); //<>// //<>//
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
  startNode = new GraphNode(startPoint);
  roadmap.add(startNode);
  endNode = new GraphNode(endPoint);
  roadmap.add(endNode);
  
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
    for(int j=1; j<4; j++){
      Pair p = distance.get(i).get(j);
      GraphNode neighbor = roadmap.get(p.index);
      node.addNeighbor(neighbor);
    }
  }
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

void shortestPathBFS(GraphNode start, GraphNode end){
  Queue<ArrayList<GraphNode>> bfsPathQueue = new LinkedList();
  
  ArrayList<GraphNode> startPath = new ArrayList();
  startPath.add(start);
  bfsPathQueue.add(startPath);
  
  while(!bfsPathQueue.isEmpty()){
    ArrayList<GraphNode> path = bfsPathQueue.remove();
    GraphNode lastNode = path.get(path.size()-1); //<>//
    if(lastNode == end){
      shortestPath = path; 
      return;
    }
    for(GraphNode neighbor : lastNode.neighbors){
      if(neighbor.visited){continue;}
      ArrayList<GraphNode> newPath = (ArrayList) path.clone();
      newPath.add(neighbor);
      bfsPathQueue.add(newPath);
    }
    lastNode.setVisited();
  }
  shortestPath = new ArrayList();
  return;
} //<>//

void shortestPathDFS(GraphNode start, GraphNode end){
  Stack<ArrayList<GraphNode>> dfsPathStack = new Stack();
  
  ArrayList<GraphNode> startPath = new ArrayList();
  startPath.add(start);
  dfsPathStack.add(startPath);
  
  while(!dfsPathStack.isEmpty()){
    ArrayList<GraphNode> path = dfsPathStack.pop();
    GraphNode lastNode = path.get(path.size()-1);
    if(lastNode == end){
      shortestPath = path; 
      return;
    }
    for(GraphNode neighbor : lastNode.neighbors){
      if(neighbor.visited){continue;}
      ArrayList<GraphNode> newPath = (ArrayList) path.clone();
      newPath.add(neighbor);
      dfsPathStack.push(newPath);
    }
    lastNode.setVisited();
  }
  shortestPath = new ArrayList(); //<>//
  return;
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
  fill(0, 114, 233);
  stroke(0, 114, 233);
  translate(centerX+point.x, centerY+point.y-(h/2)-10, centerZ+point.z);
  box(5,h,5);
  popMatrix();
}

void drawObstacle(){
  pushMatrix();
  fill(obstacleColor);
  stroke(obstacleColor);
  translate(centerX+obstaclePosition.x, centerY+obstaclePosition.y, centerZ+obstaclePosition.z);
  sphere(obstacleRadius);
  popMatrix();
}

void drawAgent(){
  pushMatrix();
  fill(agent.colour);
  stroke(agent.colour);
  translate(centerX+agent.position.x, centerY+agent.position.y-5, centerZ+agent.position.z);
  sphere(agent.radius);
  popMatrix();
}

void drawNode(GraphNode node){
  pushMatrix();
  translate(centerX+node.location.x, centerY+node.location.y, centerZ+node.location.z);
  sphere(3);
  translate(3,-10,3);
  //rotateY(-PI/2);
  //text("P:"+node.id,0,0);
  //text("p:"+node.location.x+":"+node.location.z, 0, 0);
  popMatrix();
}

void drawRoadmap(){
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
 //<>//
void update(float dt){
  if(shortestPath.isEmpty()){
    displayNoPath = true;
    return;
  }else{
    displayNoPath = false;
  }
  
  if(currentNode < shortestPath.size()-1){
    agent.move(shortestPath.get(currentNode), shortestPath.get(currentNode+1), dt);
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
  drawEndpoint(startPoint, 10); // start point
  drawEndpoint(endPoint, 10); // stop point
  drawObstacle(); // Obstacle
  drawAgent(); // Agent
  drawRoadmap();
  
  if(displayNoPath){ drawNoPathText(); }
  
  // Update Physics
  update(0.1);
}
