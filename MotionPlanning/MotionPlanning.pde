import queasycam.*;
import java.util.Queue;
import java.util.PriorityQueue;
import java.util.Stack;
import java.util.Collections;
import java.util.LinkedList;

QueasyCam cam;

float planeT = 4.0, planeW = 400.0, planeH = 400.0;
float centerX = 300.0, centerY = 100.0, centerZ = 0.0;
float worldH = 300.0;
color landColor = color(82, 115, 84);

ArrayList<Agent> agents;
ArrayList<Bird> birds;
ArrayList<Obstacle> obstacles;
PShape obstacleModel;

ArrayList<Sphere> invalidPositions;
ArrayList<PVector> milestones;
ArrayList<GraphNode> roadmap;
boolean findPath = false;
boolean displayNoPath = false;
boolean drawMap = false;
boolean isTTC = false;

boolean paused = false;
ArrayList<PShape> agentShape;
ArrayList<PShape> birdShape;
ArrayList<PShape> treeShape;
PImage agentTexture;

ArrayList<TreeNode> globalTree;

boolean editMode = false;

void setup(){
  size(1000,1000,P3D);
  cam = new QueasyCam(this);
  cam.speed = 0.7; 
  cam.sensitivity = 1.5;
  pointLight(51,102,126, 0,0,-100);
  pointLight(centerX-200,centerY,centerZ, 255,255,255);
  lights();
  randomSeed(1);
  loadRaptor();
  loadTrees();
  loadObstacles();
  loadBirds();
  restart();
  return;
}

void loadTrees(){
  treeShape = new ArrayList();
  PShape tree = loadShape("Trees/BirchTree_1.obj");
  treeShape.add(tree);
  tree = loadShape("Trees/Willow_3.obj");
  treeShape.add(tree);
}

void loadRaptor(){
  agentShape = new ArrayList();
  for(int i=1; i<=6; i++){
    PShape pose = loadShape("raptor/velo"+i+".obj");
    agentShape.add(pose);
  }
  agentTexture = loadImage("raptor/skin.jpg");
}

void loadBirds(){
  birdShape = new ArrayList();
  for(int i=1; i<=1; i++){
    PShape pose = loadShape("Eagle.obj");
    birdShape.add(pose);
  }
  birds = new ArrayList();
  spawnBirds(50);
}

void spawnBirds(int numAgents){
  int spawnCount = 0;
  while(spawnCount < numAgents){
    birds.add(new Bird(birdShape, obstacles));
    spawnCount++;
  }
}

void spawnAgents(int numAgents, float x, float z, float lx, float lz, PVector target, color colour){
  int spawnCount = 0;
  while(spawnCount < numAgents){
    PVector start = new PVector(random(x+Agent.radius, x+lx), 0, random(z+Agent.radius, z+lz));
    if(isTTC){
      agents.add(new TTCAgent(start, target, agentShape, agentTexture, colour));
    }else{
      agents.add(new Agent(start, PVector.add(target, new PVector(random(-50,50), 0, random(-50,50))), agentShape, agentTexture, colour));
    }
    spawnCount++;
  }
}

void loadObstacles(){
  obstacleModel = loadShape("Rock_6/Rock_6.OBJ");
  obstacles = new ArrayList();
  obstacles.add(new Obstacle(new PVector(0,0,-10), 40, obstacleModel));
  obstacles.add(new Obstacle(new PVector(20,0,120), 40, obstacleModel));
  obstacles.add(new Obstacle(new PVector(120,0,20), 40, obstacleModel));
}

void restart(){
  findPath = false;
  displayNoPath = false;
  agents = new ArrayList();
  //spawnAgents(80, -200, -200, 100, 400, new PVector(190,0,0));
  spawnAgents(20, -200, -200, 100, 100, new PVector(140,0,140), color(171,170,165));
  spawnAgents(20, -200, 100, 100, 100, new PVector(140,0,-140), color(105,84,66));
  //spawnAgents(10, -200, -100, 50, 50, new PVector(90,0,90));
  //spawnAgents(10, -100, -200, 50, 50, new PVector(90,0,90));
}

void update(float dt){
  if(displayNoPath){
    return;
  }
  if(!paused){
    int iter = 0;
    for(Agent agent : agents){
      //if(iter == 17){ agent.printStuff();}
      iter++;
      agent.move(dt, agents, obstacles);
    }
    
    iter = 0;
    for(Bird bird : birds){
      iter++;
      bird.move(dt, birds, obstacles);
    }
  }
}

void draw(){
  background(90, 222, 255);
  //drawOrigin();
  drawLand();
  for(Agent agent: agents){
    drawEndpoint(agent.start.location, 10); // start point
    drawEndpoint(agent.goal.location, 10); // stop point
  }
  drawObstacles(); // Obstacle
  drawTrees();
  drawAgents(); // Agent
  drawBirds(); // Agent
  drawRoadmap();
  if(globalTree != null && globalTree.size() > 0){drawTree(globalTree);}   
  if(displayNoPath){ drawNoPathText(); }
  
  if(editMode){
    cam.controllable = false;
  }

  // Update Physics
  for(int i=0; i<3; i++){
    update(0.04);
  }
}

void findPaths(){
  displayNoPath = false;
  for(Agent agent : agents){
    resetGraphVisited();
    //agent.setPath(searchPathBFS(agent.start, agent.goal));
    agent.setPath(searchPathUCS(agent.start, agent.goal));
  }
}

void findPathsAStar(){
  displayNoPath = false;
  for(Agent agent : agents){
    resetGraphVisited();
    agent.setPath(searchPathAStar(agent.start, agent.goal));
  }
}

void findPathsRRT(){
  displayNoPath = false;
  int iter = 0;
  for(Agent agent : agents){
    boolean saveTree = false;
    //if(iter == 17){ saveTree = true;}
    iter++;
    agent.setPath(findRRT(agent.position, agent.goal, saveTree));
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
  
  if(key == 'y'){
    constructCSpace();
    sampleConfigurations();
    constructSparseGraph();
    float startTime = millis(); 
    findPathsAStar();
    float endTime = millis();
    println("PRM with A-star: "+(endTime - startTime));
    findPath = !findPath;
  }
  
  if(key == 't'){
    float startTime = millis(); 
    findPathsRRT();
    float endTime = millis();
    println("RRT: "+(endTime - startTime));
    findPath = !findPath;
  }
  
  if(key =='b'){
    int iter = 0;
    for(Bird bird : birds){
      boolean saveTree = false;
      //if(iter == 12){ 
      //  saveTree = true;
      //}
      bird.replan(saveTree);
      iter++;
    }
  }
  
  if(key == '0'){
    isTTC = false;
    globalTree = null;
    restart();
  }
  if(key == '9'){
    isTTC = true;
    globalTree = null;
    restart();
  }
  if(key == 'm'){
    drawMap = !drawMap;
  }
  if(key == 'p'){
    paused = !paused;
  }
  if(key == 'o'){
    editMode = !editMode;
    if(editMode){
      setTopCamera();
    }else{
      resetCamera();
    }
  }
  
  if(key == '='){
    if(editMode){
      for(Agent agent : agents){
        agent.setGoal(mapMouse());
      }
    }
  }
}

Agent editingAgent;
PVector editAgentStart, editAgentGoal;

Obstacle editingObstacle;

void addObstacle(PVector center){
  float radius = 10.0;
  if(obstacles == null){
    obstacles = new ArrayList();
  }
  editingObstacle = new Obstacle(center, radius, obstacleModel);
  obstacles.add(editingObstacle);
}

void mousePressed(){
  if(editMode){
    if(mouseButton == RIGHT){
      PVector center = mapMouse();
      if(center != null){
        if(editingObstacle != null){
          editingObstacle.position = center.copy();
        }else{
          addObstacle(center);
        }
      }
      findPathsRRT();
    }
    
    if(editingObstacle == null && editingAgent == null && mouseButton == LEFT){
      println("started");
      editAgentStart = mapMouse();
    }
    
    if(editingObstacle != null && mouseButton == LEFT){
      editingObstacle = null;
    }
  }
}

void mouseReleased(){
  if(editMode && (editingObstacle == null)
              && (editAgentStart != null) 
              && (mouseButton == LEFT)
  ){
    println("ended");
    editAgentGoal = mapMouse();
    editingAgent = new Agent(editAgentStart, editAgentGoal, agentShape, agentTexture, color(171,170,165));
    editingAgent.replan();
    agents.add(editingAgent);
    editingAgent = null;
    editAgentStart = null;
  }
}

void mouseWheel(MouseEvent event){
  float count = event.getCount();
  //println("mousewheel count: "+count);
  if(editingObstacle != null){
    editingObstacle.radius += (count*2);
    findPathsRRT();
  }
}
