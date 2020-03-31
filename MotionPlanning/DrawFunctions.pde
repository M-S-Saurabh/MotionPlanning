
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

void drawTree(ArrayList<TreeNode> tree){
  for(TreeNode node : tree){
    drawNode(node);
    for(TreeNode child : node.children){
      stroke(0,0,255);
      strokeWeight(2);
      line(centerX+node.location.x, centerY+node.location.y, centerZ+node.location.z,
           centerX+child.location.x, centerY+child.location.y, centerZ+child.location.z);
    }
  }
}

void drawNode(TreeNode node){
  pushMatrix();
  translate(centerX+node.location.x, centerY+node.location.y-0.1, centerZ+node.location.z);
  rotateX(PI/2);
  ellipse(0,0,3,3);
  translate(3,-10,3);
  rotateY(-PI/2);
  //text("P:"+node.id,0,0);
  //fill(255,0,0); strokeWeight(3);
  //text("p:"+ node.location.x+" : "+ node.location.z, 0, 0);
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

void drawNoPathText(){
  pushMatrix();
  translate(centerX, centerY-100, centerZ-180);
  rotateY(-PI/2);
  text("Path from start to end does not exist with the given road-map",0,0);
  popMatrix();
}
