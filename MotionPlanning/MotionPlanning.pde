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
ArrayList<PShape> agentShape;
PImage agentTexture;

ArrayList<TreeNode> globalTree;

void setup(){
  size(1000,1000,P3D);
  cam = new QueasyCam(this);
  cam.speed = 5; 
  cam.sensitivity = 0.5;
  pointLight(51,102,126, 0,0,-100);
  randomSeed(1);
  loadRaptor();
  restart();
  return;
}

void loadRaptor(){
  agentShape = new ArrayList();
  for(int i=1; i<=6; i++){
    PShape pose = loadShape("raptor/velo"+i+".obj");
    agentShape.add(pose);
  }
  agentTexture = loadImage("raptor/skin.jpg");
}

void spawnAgents(int numAgents, float x, float z, float lx, float lz, PVector target){
  int spawnCount = 0;
  while(spawnCount < numAgents){
    PVector start = new PVector(random(x+Agent.radius, x+lx), 0, random(z+Agent.radius, z+lz));
    if(isTTC){
      agents.add(new TTCAgent(start, target, agentShape, agentTexture));
    }else{
      agents.add(new Agent(start, target, agentShape, agentTexture));
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

void update(float dt){
  if(displayNoPath){
    return;
  }
  if(!paused){
    for(Agent agent : agents){ //<>//
      agent.move(dt, agents, obstacles);
    }
  }
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
  if(globalTree != null && globalTree.size() > 0){drawTree(globalTree);}   
  if(displayNoPath){ drawNoPathText(); }

  // Update Physics
  for(int i=0; i<3; i++){
    update(0.04);
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

void findPathsRRT(){
  displayNoPath = false;
  int iter = 0;
  for(Agent agent : agents){
    boolean saveTree = false;
    if(iter == 5){saveTree = true;} iter++;
    agent.setPath(findRRT(agent.start, agent.goal, saveTree));
  }
}

void keyPressed(){
  if(key == 'r'){
    constructCSpace();
    sampleConfigurations();
    constructSparseGraph();
    float startTime = millis(); 
    findPaths();
    float endTime = millis();
    println("PRM with UCS: "+(endTime - startTime));
    findPath = !findPath;
  }
  
  if(key == 't'){
    float startTime = millis(); 
    findPathsRRT();
    float endTime = millis();
    println("RRT: "+(endTime - startTime));
    findPath = !findPath;
  }
  
  if(key == '0'){
    isTTC = false;
    restart();
  }
  if(key == '9'){
    isTTC = true;
    restart();
  }
  if(key == 'm'){
    drawMap = !drawMap;
  }
  if(key == 'p'){
    paused = !paused;
  }
}
