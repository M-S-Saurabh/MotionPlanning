
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
  fill(255, 148, 77);
  stroke(255, 148, 77);
  translate(centerX+point.x, centerY+point.y-0.1, centerZ+point.z);
  rotateX(PI/2);
  //rect(-4,-4,8,8);
  ellipse(0,0,8,8);
  popMatrix();
}

void drawObstacles(){
  if(obstacles == null || obstacles.size() <= 0){return;}
  for(Obstacle obstacle : obstacles){
    obstacle.draw(centerX, centerY, centerZ);
  }
}

void drawTrees(){
  PVector center = obstacles.get(0).position.copy();
  center.add(centerX, centerY, centerZ);
  
  pushMatrix();
  translate(center.x, center.y-10, center.z+30); //<>//
  rotateX(PI);
  scale(5);
  shape(treeShape.get(0));
  translate(0, -2, -2);
  shape(treeShape.get(0));
  translate(-2, 5, 5);
  scale(2);
  shape(treeShape.get(1));
  translate(0, -3, 5.5);
  scale(0.8);
  shape(treeShape.get(0));
  popMatrix();
}

void drawAgents(){
  if(agents == null || agents.size() <= 0){return;}
  int iter = 0;
  for(Agent agent: agents){
    agent.draw(centerX, centerY, centerZ);
    //agent.drawNumber(iter);
    iter++;
  }
}

void drawBirds(){
  if(birds == null || birds.size() <= 0){return;}
  int iter = 0;
  for(Bird bird: birds){
    //if(iter == 12){
      //drawEndpoint(bird.goal.location, 10);
    //  bird.drawHighlight();
    //}
    bird.draw(centerX, centerY, centerZ);
    //agent.drawNumber(iter);
    iter++;
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
  if(node.parent == null){ fill(255,0,0); stroke(255,0,0);}
  else{ fill(0,0,255); stroke(0,0,255);}
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
