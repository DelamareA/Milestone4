import processing.core.*; 
import processing.data.*; 
import processing.event.*; 
import processing.opengl.*; 

import papaya.*; 
import processing.video.*; 
import java.util.Comparator; 
import java.util.Collections; 
import java.util.Random; 
import java.util.List; 
import processing.core.PVector; 
import papaya.*; 

import java.util.HashMap; 
import java.util.ArrayList; 
import java.io.File; 
import java.io.BufferedReader; 
import java.io.PrintWriter; 
import java.io.InputStream; 
import java.io.OutputStream; 
import java.io.IOException; 

public class TangibleGame extends PApplet {








float depth = 100;
float rotationX = 0.0f;
float rotationY = 0.0f;
float rotationZ = 0.0f;
float boardSpeed = 0.7f;

float boardSize = 50;
float ballSize = 2;

boolean addCylinderMode = false;

float cylinderBaseSize = 2;

PShape closedCylinder = new PShape();

PShape sheep_shape_alive = new PShape();
PShape sheep_shape_dead = new PShape();
ArrayList<Sheep> sheeps;
int sheeps_num = 5;
PImage background;

ArrayList<PVector> cylinderList;

float coin1; // coin du plateau 1 
float coin2; // coin du plateau 2 

PGraphics backgroundSurface;
PGraphics topViewSurface;
PGraphics scoreSurface;
PGraphics barChartSurface;
PGraphics sprotchSurface;

float score;
float totalScore;
int nbCurrentScore = 0;
int nbScoreMax;

int timeSinceLastEvent = 0;
int timeSinceLastSprotch = 0;

float[] tabScore;

Mover ball;
HScrollbar hs;

//Capture cam;
Movie cam;
PImage img;
PImage sob;
PImage back;

int hueThLow = 105;
int hueThHigh = 140;
int saturationThLow = 115;
int saturationThHigh = 255;
int brightnessThLow = 65;
int brightnessThHigh = 255;

int hueThLowP = 110; // used for a better detection of the board, used in the 'preHueTh' function
int hueThHighP = 130;
int saturationThLowP = 115;
int saturationThHighP = 255;
int brightnessThLowP = 65;
int brightnessThHighP = 255;

ArrayList<int[]> cycles = new ArrayList<int[]>();
int[][] graph;


public void setup() {
  size(1000, 700, P3D);  // size always goes first!
  if (frame != null) {
    frame.setResizable(true);
  }
  frameRate(60);
  
  back = loadImage("background.jpg");
  back.resize(width, height);

  backgroundSurface = createGraphics(width, 150, P2D);
  topViewSurface = createGraphics(backgroundSurface.height - 10, backgroundSurface.height - 10, P2D);
  scoreSurface = createGraphics(120, backgroundSurface.height - 10, P2D);
  barChartSurface = createGraphics(backgroundSurface.width - topViewSurface.width - scoreSurface.width - 70, backgroundSurface.height - 40, P2D);
  sprotchSurface = createGraphics(180, 60, P2D);
  nbScoreMax = (int)(barChartSurface.width/(pow(4.0f, 0.5f)));
  tabScore = new float[nbScoreMax];

  ball = new Mover();
  cylinderList = new ArrayList<PVector>();
  closedCylinder = loadShape("tourTextUnit.obj"); // we removed the texture of the tower for better performance
  closedCylinder.scale(0.12f * cylinderBaseSize);
  closedCylinder.rotateX(PI/2);
  
  sheeps = new ArrayList<Sheep>();
  createSheeps();
  sheep_shape_alive = loadShape("Sheep.obj");
  sheep_shape_alive.scale(10.f);
  sheep_shape_dead = loadShape("blood2.obj");
  sheep_shape_dead.scale(20.0f, 20.f, 10.0f);
  sheep_shape_dead.rotateX(PI/2);

  score = 0.0f;
  totalScore = 0.0f;

  hs = new HScrollbar(topViewSurface.width + scoreSurface.width +50, height - 40, backgroundSurface.width - topViewSurface.width - scoreSurface.width - 70, 20);
  
  /*String[] cameras = Capture.list(); // Code for webcam
  if (cameras.length == 0) {
    println("There are no cameras available for capture.");
    exit();
  } else {
    println("Available cameras:");
    for (int i = 0; i < cameras.length; i++) {
      println(cameras[i]);
    }
    cam = new Capture(this, cameras[7]);
    cam.start();
  }*/
  
  cam = new Movie(this, "testvideo.mp4");
  cam.loop();

}


public void createSheeps(){
    for (int i = 0; i < sheeps_num; i++){
      sheeps.add(new Sheep(new PVector(random(-boardSize/2, boardSize/2),random(-boardSize/2, boardSize)/2), boardSize));
      sheeps.get(i).sheep_orientation = random(-PI, PI); 
    }
    
}

public void draw() {
  pushMatrix();
  
  
  directionalLight(200, 150, 100, 0, -1, 0);
  directionalLight(130, 130, 130, 100, 1, 0);
  directionalLight(130, 130, 130, 0, 0, -1);
  ambientLight(102, 102, 102);
  image(back, 0, 0);

  noStroke();

  if (addCylinderMode == true) {
    camera(width/2, 200, 0.1f, width/2, height/2, 0, 0, 1, 0);

    translate(width/2, height/2, 0);
    pushMatrix();
    scale(1, 0.07f, 1);
    fill(0, 91, 0);
    box(boardSize);
    popMatrix();
    // on determine la positions des coins sur l'ecran
    coin1 = screenX(-boardSize/2, 0, boardSize/2);
    coin2 = screenX(boardSize/2, 0, boardSize/2);
  } else {
    ball.checkEdges();
    ball.checkCylinderCollision();


    camera(width/2, height/2 - 60, depth, width/2, height/2, 0, 0, 1, 0);

    translate(width/2, height/2, 0);

    rotateX(rotationX);
    rotateY(rotationY);
    rotateZ(rotationZ);

    pushMatrix();
    scale(1, 0.07f, 1);
    fill(0, 91, 0);
    box(boardSize);
    popMatrix();
  }

  ball.display();

  for (int i=0; i<cylinderList.size (); i++) {
    pushMatrix();
    translate(cylinderList.get(i).x, -1, cylinderList.get(i).y);
    rotateX(PI/2);
    shape(closedCylinder);
    popMatrix();
  }
  
  for (int i = 0 ; i < sheeps.size(); i ++){
    sheeps.get(i).Sheep_move();
     pushMatrix();
     if (sheeps.get(i).sheep_is_alive){
        translate(sheeps.get(i).sheep_position.x, -3.2f - sheeps.get(i).sheep_height , sheeps.get(i).sheep_position.y);
     }
     else{
         translate(sheeps.get(i).sheep_position.x, -1.65f , sheeps.get(i).sheep_position.y); 
     }
    rotateX(PI/2);
    rotateZ(sheeps.get(i).sheep_orientation);
    if (sheeps.get(i).sheep_is_alive){
      shape(sheep_shape_alive);
    }
    else{
       shape(sheep_shape_dead); 
    }
    popMatrix();
  }

  popMatrix();

  drawBackgroundSurface();
  drawScoreSurface();
  drawBarChartSurface();
  drawTopViewSurface();
  drawSprotchSurface();
  image(backgroundSurface, 0, height - backgroundSurface.height);
  image(topViewSurface, 5, height-backgroundSurface.height+5);
  image(scoreSurface, topViewSurface.width + 20, height - scoreSurface.height - 5);
  image(barChartSurface, topViewSurface.width + scoreSurface.width +50, height - scoreSurface.height - 5);
  image(sprotchSurface, width/2 - sprotchSurface.width/2, 20);

  hs.update();
  hs.display();
  
  if (cam.available() == true) {
    cam.read();
  }
  img = cam.get();
  img.resize(320, 240);
  image(img, 0, 0);
 
  sob = sobel(whiteTh(convolute(hueTh(preHueTh(convolute(img))))));
  hough(sob, 4);
  
  fill(255);
}


public void keyPressed() {
  if (key == CODED) {
    if (keyCode == SHIFT) {
      addCylinderMode = true;
    }
  }
  
}
public void keyReleased() {
  if (key == CODED) {
    if (keyCode == SHIFT) {
      addCylinderMode = false;
    }
  }
}

public void mouseClicked() {
  if (addCylinderMode == true) {

    float boardWidthOnScreen = coin2 - coin1;
    float zoom = boardSize/boardWidthOnScreen;
    float x = mouseX - width/2;
    float y = mouseY - height/2;

    if (width/2 - boardWidthOnScreen/2 <= mouseX && mouseX <= width/2 + boardWidthOnScreen/2 && height/2 - boardWidthOnScreen/2 <= mouseY && mouseY <= height/2 + boardWidthOnScreen) { // PAS CHANGER

      PVector n = new PVector(ball.location.x, 0, ball.location.z);
      n.sub(new PVector(x*zoom, 0, y*zoom));

      if (n.mag() > cylinderBaseSize + ballSize) { // cylindre pas dans ball
        cylinderList.add(new PVector(x*zoom, y*zoom));
      }
    }
  }
}

public void mouseWheel(MouseEvent event) {
  if (event.getCount() < 0.0f) {
    boardSpeed /= 1.1f;
  } else {
    boardSpeed *= 1.1f;
  }
}

class CWComparator implements Comparator<PVector> {
  PVector center;
  public CWComparator(PVector center) {
    this.center = center;
  }
  @Override
    public int compare(PVector b, PVector d) {
    if (Math.atan2(b.y-center.y, b.x-center.x)<Math.atan2(d.y-center.y, d.x-center.x))
      return -1;
    else return 1;
  }
}

public List<PVector> sortCorners(List<PVector> quad) {
  // Sort corners so that they are ordered clockwise
  PVector a = quad.get(0);
  PVector b = quad.get(2);
  PVector center = new PVector((a.x+b.x)/2,(a.y+b.y)/2);
  Collections.sort(quad, new CWComparator(center));
  // TODO:
  // Re-order the corners so that the first one is the closest to the
  // origin (0,0) of the image.
  //
  // You can use Collections.rotate to shift the corners inside the quad.
  int shift = 0;
  float distance = 3.4028234663852886E38f; // float max value
  
  for (int i = 0; i < 4; i++){
    if (quad.get(i).x * quad.get(i).x + quad.get(i).y * quad.get(i).y < distance){
      shift = i;
      distance = quad.get(i).x * quad.get(i).x + quad.get(i).y * quad.get(i).y;
    }
  }
  
  
  Collections.rotate(quad, shift);
  
  return quad;
}

class HScrollbar {
  float barWidth; //Bar's width in pixels
  float barHeight; //Bar's height in pixels
  float xPosition; //Bar's x position in pixels
  float yPosition; //Bar's y position in pixels
  float sliderPosition, newSliderPosition; //Position of slider
  float sliderPositionMin, sliderPositionMax; //Max and min values of slider
  boolean mouseOver; //Is the mouse over the slider?
  boolean locked; //Is the mouse clicking and dragging the slider now?
  /**
   * @brief Creates a new horizontal scrollbar
   *
   * @param x The x position of the top left corner of the bar in pixels
   * @param y The y position of the top left corner of the bar in pixels
   * @param w The width of the bar in pixels
   * @param h The height of the bar in pixels
   */
  HScrollbar (float x, float y, float w, float h) {
    barWidth = w;
    barHeight = h;
    xPosition = x;
    yPosition = y;
    sliderPosition = xPosition + barWidth/2 - barHeight/2;
    newSliderPosition = sliderPosition;
    sliderPositionMin = xPosition;
    sliderPositionMax = xPosition + barWidth - barHeight;
  }
  /**
   * @brief Updates the state of the scrollbar according to the mouse movement
   */
  public void update() {
    mouseOver = isMouseOver() && mouseX >= xPosition && mouseX <= xPosition + barWidth && mouseY >= yPosition && mouseY <= yPosition + barHeight;
    if (mousePressed && mouseOver) {
      locked = true;
    }
    else {
      locked = false;
    }
    if (locked) {
      newSliderPosition = constrain(mouseX - barHeight/2, sliderPositionMin, sliderPositionMax);
    }
    if (abs(newSliderPosition - sliderPosition) > 1) {
      sliderPosition = sliderPosition + (newSliderPosition - sliderPosition);
    }
  }
  /**
   * @brief Clamps the value into the interval
   *
   * @param val The value to be clamped
   * @param minVal Smallest value possible
   * @param maxVal Largest value possible
   *
   * @return val clamped into the interval [minVal, maxVal]
   */
  public float constrain(float val, float minVal, float maxVal) {
    return min(max(val, minVal), maxVal);
  }
  /**
   * @brief Gets whether the mouse is hovering the scrollbar
   *
   * @return Whether the mouse is hovering the scrollbar
   */
  public boolean isMouseOver() {
    if (mouseX > xPosition && mouseX < xPosition+barWidth &&
      mouseY > yPosition && mouseY < yPosition+barHeight) {
      return true;
    } else {
      return false;
    }
  }
  /**
   * @brief Draws the scrollbar in its current state
   */
  public void display() {
    noStroke();
    fill(204);
    rect(xPosition, yPosition, barWidth, barHeight);
    if (mouseOver || locked) {
      fill(0, 0, 0);
    } else {
      fill(102, 102, 102);
    }
    rect(sliderPosition, yPosition, barHeight, barHeight);
  }
  /**
   * @brief Gets the slider position
   *
   * @return The slider position in the interval [0,1]
   * corresponding to [leftmost position, rightmost position]
   */
  public float getPos() {
    return (sliderPosition - xPosition)/(barWidth - barHeight);
  }
}

class Mover {
  PVector location;
  PVector velocity;
  PVector gravity;
  Mover() {
    location = new PVector(0, -4, 0);
    velocity = new PVector(0, 0, 0);
    gravity = new PVector(0, 0.19f, 0);
  }
  
  public void update() {
    location.add(velocity);
  }
  public void display() {
    pushMatrix();
    fill(127, 127, 127);
    
    translate(location.x, location.y, location.z);
    sphere(ballSize);
    fill(255, 255, 255);
    popMatrix();
  }
  public void checkEdges() {
    PVector gravityForce = new PVector(sin(rotationZ) * gravity.y, 0, -sin(rotationX) * gravity.y);
    float normalForce = 1;
    float mu = 0.01f;
    float frictionMagnitude = normalForce * mu;
    PVector friction = velocity.get();
    friction.mult(-1);
    friction.normalize();
    friction.mult(frictionMagnitude);
    
    velocity.add(gravityForce);
    velocity.add(friction);
    
    location.add(velocity);
    
    if (location.x > boardSize/2){
      velocity.x = velocity.x * -1;
      location.x = boardSize/2;
      
      score = -velocity.mag();
      totalScore += score;
    }
    
    if (location.x < -boardSize/2){
      velocity.x = velocity.x * -1;
      location.x = -boardSize/2;
      
      score = -velocity.mag();
      totalScore += score;
    }
    
    if (location.z > boardSize/2){
      velocity.z = velocity.z * -1;
      location.z = boardSize/2;
      
      score = -velocity.mag();
      totalScore += score;
    }
    
    if (location.z < -boardSize/2){
      velocity.z = velocity.z * -1;
      location.z = -boardSize/2;
      
      score = -velocity.mag();
      totalScore += score;
    }
  }
  
  public void checkCylinderCollision(){
    
     for (int i=0; i<cylinderList.size(); i++){
       PVector n = new PVector(location.x, 0, location.z);
       n.sub(new PVector(cylinderList.get(i).x, 0, cylinderList.get(i).y));
       
       if (n.mag() < cylinderBaseSize + ballSize){ // collision
         
         score = velocity.mag();
         totalScore += score;
       
         n.normalize();
         PVector temp = n.get();
         temp.mult(2);
         temp.mult(velocity.dot(n));
         velocity.sub(temp);
         
         PVector n2;
         do {
           location.add(velocity); // detach the ball
           
            n2 = new PVector(location.x, 0, location.z);
            n2.sub(new PVector(cylinderList.get(i).x, 0, cylinderList.get(i).y));
         } while (n2.mag() < cylinderBaseSize + ballSize); // if still in cylinder, detach the ball
         
         cylinderList.remove(i);
       }
       
        
     }
    
  }
}

public void build(ArrayList<PVector> lines, int width, int height) {

  int n = lines.size();

  // The maximum possible number of edges is sum(0..n) = n * (n + 1)/2
  graph = new int[n * (n + 1)/2][2];

  int idx =0;

  for (int i = 0; i < lines.size (); i++) {
    for (int j = i + 1; j < lines.size (); j++) {
      if (intersect(lines.get(i), lines.get(j), width, height)) {

        // TODO
        // fill the graph using intersect() to check if two lines are
        // connected in the graph.
        
        graph[idx][0] = i;
        graph[idx][1] = j;

        idx++;
      }
    }
  }
}

/** Returns true if polar lines 1 and 2 intersect 
 * inside an area of size (width, height)
 */
public static boolean intersect(PVector line1, PVector line2, int width, int height) {

  double sin_t1 = Math.sin(line1.y);
  double sin_t2 = Math.sin(line2.y);
  double cos_t1 = Math.cos(line1.y);
  double cos_t2 = Math.cos(line2.y);
  float r1 = line1.x;
  float r2 = line2.x;

  double denom = cos_t2 * sin_t1 - cos_t1 * sin_t2;

  int x = (int) ((r2 * sin_t1 - r1 * sin_t2) / denom);
  int y = (int) ((-r2 * cos_t1 + r1 * cos_t2) / denom);

  if (0 <= x && 0 <= y && width >= x && height >= y)
    return true;
  else
    return false;
}

public ArrayList<int[]> findCycles() {

  cycles.clear();
  for (int i = 0; i < graph.length; i++) {
    for (int j = 0; j < graph[i].length; j++) {
      findNewCycles(new int[] {
        graph[i][j]
      }
      );
    }
  }
  for (int[] cy : cycles) {
    String s = "" + cy[0];
    for (int i = 1; i < cy.length; i++) {
      s += "," + cy[i];
    }
    //System.out.println(s);
  }
  return cycles;
}

public void findNewCycles(int[] path)
{
  int n = path[0];
  int x;
  int[] sub = new int[path.length + 1];

  for (int i = 0; i < graph.length; i++)
    for (int y = 0; y <= 1; y++)
      if (graph[i][y] == n)
        //  edge refers to our current node
      {
        x = graph[i][(y + 1) % 2];
        if (!visited(x, path))
          //  neighbor node not on path yet
        {
          sub[0] = x;
          System.arraycopy(path, 0, sub, 1, path.length);
          //  explore extended path
          findNewCycles(sub);
        } else if ((path.length > 2) && (x == path[path.length - 1]))
          //  cycle found
        {
          int[] p = normalize(path);
          int[] inv = invert(p);
          if (isNew(p) && isNew(inv))
          {
            cycles.add(p);
          }
        }
      }
}

//  check of both arrays have same lengths and contents
public static Boolean equals(int[] a, int[] b)
{
  Boolean ret = (a[0] == b[0]) && (a.length == b.length);

  for (int i = 1; ret && (i < a.length); i++)
  {
    if (a[i] != b[i])
    {
      ret = false;
    }
  }

  return ret;
}

//  create a path array with reversed order
public static int[] invert(int[] path)
{
  int[] p = new int[path.length];

  for (int i = 0; i < path.length; i++)
  {
    p[i] = path[path.length - 1 - i];
  }

  return normalize(p);
}

//  rotate cycle path such that it begins with the smallest node
public static int[] normalize(int[] path)
{
  int[] p = new int[path.length];
  int x = smallest(path);
  int n;

  System.arraycopy(path, 0, p, 0, path.length);

  while (p[0] != x)
  {
    n = p[0];
    System.arraycopy(p, 1, p, 0, p.length - 1);
    p[p.length - 1] = n;
  }

  return p;
}

//  compare path against known cycles
//  return true, iff path is not a known cycle
public Boolean isNew(int[] path)
{
  Boolean ret = true;

  for (int[] p : cycles)
  {
    if (equals(p, path))
    {
      ret = false;
      break;
    }
  }

  return ret;
}

//  return the int of the array which is the smallest
public static int smallest(int[] path)
{
  int min = path[0];

  for (int p : path)
  {
    if (p < min)
    {
      min = p;
    }
  }

  return min;
}

//  check if vertex n is contained in path
public static Boolean visited(int n, int[] path)
{
  Boolean ret = false;

  for (int p : path)
  {
    if (p == n)
    {
      ret = true;
      break;
    }
  }

  return ret;
}



/** Check if a quad is convex or not.
 * 
 * Algo: take two adjacent edges and compute their cross-product. 
 * The sign of the z-component of all the cross-products is the 
 * same for a convex polygon.
 * 
 * See http://debian.fmi.uni-sofia.bg/~sergei/cgsr/docs/clockwise.htm
 * for justification.
 * 
 * @param c1
 */
public static boolean isConvex(PVector c1, PVector c2, PVector c3, PVector c4) {

  PVector v21= PVector.sub(c1, c2);
  PVector v32= PVector.sub(c2, c3);
  PVector v43= PVector.sub(c3, c4);
  PVector v14= PVector.sub(c4, c1);

  float i1=v21.cross(v32).z;
  float i2=v32.cross(v43).z;
  float i3=v43.cross(v14).z;
  float i4=v14.cross(v21).z;

  if (   (i1>0 && i2>0 && i3>0 && i4>0) 
    || (i1<0 && i2<0 && i3<0 && i4<0))
    return true;
  else 
    System.out.println("Eliminating non-convex quad");
  return false;
}

/** Compute the area of a quad, and check it lays within a specific range
 */
public static boolean validArea(PVector c1, PVector c2, PVector c3, PVector c4, float max_area, float min_area) {

  PVector v21= PVector.sub(c1, c2);
  PVector v32= PVector.sub(c2, c3);
  PVector v43= PVector.sub(c3, c4);
  PVector v14= PVector.sub(c4, c1);

  float i1=v21.cross(v32).z;
  float i2=v32.cross(v43).z;
  float i3=v43.cross(v14).z;
  float i4=v14.cross(v21).z;

  float area = Math.abs(0.5f * (i1 + i2 + i3 + i4));

  boolean valid = (area < max_area && area > min_area);

  if (!valid) 
    System.out.println("Area out of range : " + area);

  return valid;
}

public static float area(PVector c1, PVector c2, PVector c3, PVector c4) {

  PVector v21= PVector.sub(c1, c2);
  PVector v32= PVector.sub(c2, c3);
  PVector v43= PVector.sub(c3, c4);
  PVector v14= PVector.sub(c4, c1);

  float i1=v21.cross(v32).z;
  float i2=v32.cross(v43).z;
  float i3=v43.cross(v14).z;
  float i4=v14.cross(v21).z;

  return Math.abs(0.5f * (i1 + i2 + i3 + i4));
}

/** Compute the (cosine) of the four angles of the quad, and check they are all large enough
 * (the quad representing our board should be close to a rectangle)
 */
public static boolean nonFlatQuad(PVector c1, PVector c2, PVector c3, PVector c4) {

  // cos(45deg) ~= 0.70
  float min_cos = 0.70f;

  PVector v21= PVector.sub(c1, c2);
  PVector v32= PVector.sub(c2, c3);
  PVector v43= PVector.sub(c3, c4);
  PVector v14= PVector.sub(c4, c1);

  float cos1=Math.abs(v21.dot(v32) / (v21.mag() * v32.mag()));
  float cos2=Math.abs(v32.dot(v43) / (v32.mag() * v43.mag()));
  float cos3=Math.abs(v43.dot(v14) / (v43.mag() * v14.mag()));
  float cos4=Math.abs(v14.dot(v21) / (v14.mag() * v21.mag()));

  if (cos1 < min_cos && cos2 < min_cos && cos3 < min_cos && cos4 < min_cos)
    return true;
  else {
    System.out.println("Flat quad");
    return false;
  }
}

public boolean isInTriangle(PVector pt, PVector p0, PVector p1, PVector p2){
  float s = p0.y * p2.x - p0.x * p2.y + (p2.y - p0.y) * pt.x + (p0.x - p2.x) * pt.y;
  float t = p0.x * p1.y - p0.y * p1.x + (p0.y - p1.y) * pt.x + (p1.x - p0.x) * pt.y;

  if ((s < 0) != (t < 0))
      return false;

  float A = -p1.y * p2.x + p0.y * (p2.x - p1.x) + p0.x * (p1.y - p2.y) + p1.x * p2.y;
  if (A < 0.0f)
  {
      s = -s;
      t = -t;
      A = -A;
  }
  return s > 0 && t > 0 && (s + t) < A;
}

public boolean isInQuad(PVector pt, PVector p0, PVector p1, PVector p2, PVector p3){
  return isInTriangle(pt, p0, p1, p2) || isInTriangle(pt, p1, p2, p3) || isInTriangle(pt, p0, p1, p3) || isInTriangle(pt, p0, p1, p3);
}


public class Sheep{
   public boolean sheep_is_alive;
   public PVector sheep_position;
   float sheep_height;
   float sheep_orientation;
   
    private  int max_jump_number = 15;
    private  float jump_duration = 500;
    private float jump_max_height = 1.2f;
    
    private float jump_orientation = 0.0f;
    
    private int jump_count = 0;
    private boolean is_jumping = false;
    private long jump_begin_time = 0;
    
    private float sheep_speed = 0.05f;
    
    private float max_x = 0.0f;
    private float max_y = 0.0f;
    private float min_x = 0.0f;
    private float min_y = 0.0f;
    
  
  public Sheep(PVector init_pos, float boardSize){
     sheep_position = init_pos;
     sheep_is_alive = true;
     sheep_height = 0.0f;
     sheep_orientation = 0.0f;
     max_x = boardSize/2;
     max_y = boardSize/2;
     min_x = -1*max_x; // Car la board est carr\u00e9e
     min_y = -1*max_y; 
     
     max_jump_number = (int)random(5, 25);
  } 
  
  public void Sheep_move(){
    checkBallCollision();
    checkCylinderCollision();
    
    if (sheep_is_alive){
       if (!is_jumping){
          is_jumping = true;
           jump_begin_time = millis();
           jump_count ++;
       }
       
       long difftime = millis() - jump_begin_time;
       if (difftime >= jump_duration){
         is_jumping = false;
         sheep_height = 0.0f;
         jump_count ++;
       }
       else{
           if (difftime <= jump_duration/2){
             sheep_height = map(difftime, 0, jump_duration/2, 0, jump_max_height);
           }
           else{
               sheep_height = map(difftime, jump_duration/2, jump_duration, jump_max_height, 0);
           } 
       }
       
       sheep_position.x -= sheep_speed * cos(sheep_orientation);
       sheep_position.y -= sheep_speed * sin(sheep_orientation);
       
       if (sheep_position.x < min_x) sheep_position.x = min_x;
       else if (sheep_position.x > max_x) sheep_position.x = max_x;
       
       if (sheep_position.y < min_y) sheep_position.y = min_y;
       else if (sheep_position.y > max_y) sheep_position.y = max_y;
       
       
       if (jump_count >= max_jump_number){
         jump_count = 0 ;
         is_jumping = false;
         sheep_height = 0.0f;
         sheep_orientation = random(0, 2*PI);
       }
    }
    else{
        sheep_height = 0;
    }
    
  }
  
  private void checkBallCollision(){
      float dist = (sheep_position.x - ball.location.x)*(sheep_position.x - ball.location.x) + (sheep_position.y - ball.location.z)*(sheep_position.y - ball.location.z);
        if (dist < ballSize*ballSize + 5*5 && sheep_is_alive){
           sheep_is_alive = false;  
           
           score = 4;
           totalScore += score;
           
           timeSinceLastSprotch = millis();
      }
  }
  
  private void checkCylinderCollision(){
      for (int i=0; i<cylinderList.size(); i++){
          float dist = sheep_position.dist(cylinderList.get(i));
         if (dist < cylinderBaseSize) {
             sheep_orientation += PI;
         }
      }
  }
}

public class RunnableSobel implements Runnable { // class used to provide sobel algorithm in parallel
  int id;
  int starting_row;
  int ending_row;
  PImage original_image;
  PImage result_image;
  int width;
  int height;
  int threshold;
  float[] buffer;

  int[][] kernelX= {{-1, 0, 1}, {-2, 0, 2}, {-1, 0, 1}};
  int[][] kernelY= {{-1, -2, -1}, {0, 0, 0}, {1, 2, 1}};

  public RunnableSobel(int id, int starting_row, int ending_row,
      PImage original_image, PImage result_image, int width,
      int height) {
    super();
    this.id = id;
    this.starting_row = starting_row;
    this.ending_row = ending_row;
    this.original_image = original_image;
    this.result_image = result_image;
    this.width = width;
    this.height = height;
    this.threshold = threshold;
    this.buffer = new float[original_image.width * original_image.height];
    
  }

  public void run() {
    float max = 0;
    for (int y = starting_row; y < ending_row; y++) {
      for (int x = 0; x < width; x++) {
        int sum_h = 0;
        int sum_v = 0;

        for (int i = 0; i <= 2; i++) {
          for (int j = 0; j <= 2; j++) {
            int clampedX = x + i - 1;
            if (x + i - 1 < 0) {
              clampedX = 0;
            } else if (x + i - 1 >= width) {
              clampedX = width - 1;
            }

            int clampedY = y + j - 1;
            if (y + j - 1 < 0) {
              clampedY = 0;
            } else if (y + j - 1 >= height) {
              clampedY = height - 1;
            }

            sum_h += original_image.pixels[clampedY * width + clampedX]
                * kernelX[i][j];
            sum_v += original_image.pixels[clampedY * width + clampedX]
                * kernelY[i][j];
          }
        }
        
        buffer[y * width + x] = sqrt(pow(sum_h, 2) + pow(sum_v, 2));
        
        if (buffer[y * width + x] > max) {
          max = buffer[y * width + x];
        }

        if (buffer[y * width + x] > max * 0.3f) {
          result_image.pixels[y * width + x] = color(255);
        } else {
          result_image.pixels[y * width + x] = color(0);
        }
      }
    }
    result_image.updatePixels();
  }
}





public class TwoDThreeD {

  // default focal length, well suited for most webcams
  float f = 700.0f;

  // intrisic camera matrix
  float [][] K = {
    {
      f, 0, 0
    }
    , 
    {
      0, f, 0
    }
    , 
    {
      0, 0, 1
    }
  };

  // Real physical coordinates of the Lego board in mm
  float boardSize = 380.f; // large Duplo board
  //float boardSize = 255.f; // smaller Lego board

  // the 3D coordinates of the physical board corners, clockwise
  float [][] physicalCorners = { // #TODO
    {
      -boardSize/2, -boardSize/2, 0, 1
    }
    , 
    { 
      boardSize/2, -boardSize/2, 0, 1
    }
    , 
    { 
      boardSize/2, boardSize/2, 0, 1
    }
    , 
    {
      -boardSize/2, boardSize/2, 0, 1
    }
  };

  public TwoDThreeD(int width, int height) {

    // set the offset to the center of the webcam image
    K[0][2] = 0.5f * width;
    K[1][2] = 0.5f * height;
  }

  public PVector get3DRotations(List<PVector> points2D) {

    // 1- Solve the extrinsic matrix from the projected 2D points
    double[][] E = solveExtrinsicMatrix(points2D);


    // 2 - Re-build a proper 3x3 rotation matrix from the camera's 
    //     extrinsic matrix E
    float[] firstColumn = {
      (float)E[0][0], 
      (float)E[1][0], 
      (float)E[2][0]
    };
    firstColumn = Mat.multiply(firstColumn, 1/Mat.norm2(firstColumn)); // normalize

    float[] secondColumn= {
      (float)E[0][1], 
      (float)E[1][1], 
      (float)E[2][1]
    };
    secondColumn = Mat.multiply(secondColumn, 1/Mat.norm2(secondColumn)); // normalize

    float[] thirdColumn = Mat.cross(firstColumn, secondColumn);

    float[][] rotationMatrix = {
      {
        firstColumn[0], secondColumn[0], thirdColumn[0]
      }
      , 
      {
        firstColumn[1], secondColumn[1], thirdColumn[1]
      }
      , 
      {
        firstColumn[2], secondColumn[2], thirdColumn[2]
      }
    };

    // 3 - Computes and returns Euler angles (rx, ry, rz) from this matrix
    return rotationFromMatrix(rotationMatrix);
  }


  private double[][] solveExtrinsicMatrix(List<PVector> points2D) {

    // p ~= K \u00b7 [R|t] \u00b7 P
    // with P the (3D) corners of the physical board, p the (2D) 
    // projected points onto the webcam image, K the intrinsic 
    // matrix and R and t the rotation and translation we want to 
    // compute.
    //
    // => We want to solve: (K^(-1) \u00b7 p) X ([R|t] \u00b7 P) = 0

    float [][] invK=Mat.inverse(K);

    float[][] projectedCorners = new float[4][3];

    for (int i=0; i<4; i++) {
      // #TODO:
      // store in projectedCorners the result of (K^(-1) \u00b7 p), for each 
      // corner p found in the webcam image.
      // You can use Mat.multiply to multiply a matrix with a vector.

      projectedCorners[i] = Mat.multiply(invK, points2D.get(i).array());
      
      //println(points2D.get(i).array()[2]);
      
    }

    // 'A' contains the cross-product (K^(-1) \u00b7 p) X P
    float[][] A= new float[12][9];

    for (int i=0; i<4; i++) {
      A[i*3][0]=0;
      A[i*3][1]=0;
      A[i*3][2]=0;

      // note that we take physicalCorners[0,1,*3*]: we drop the Z
      // coordinate and use the 2D homogenous coordinates of the physical
      // corners
      A[i*3][3]=-projectedCorners[i][2] * physicalCorners[i][0];
      A[i*3][4]=-projectedCorners[i][2] * physicalCorners[i][1];
      A[i*3][5]=-projectedCorners[i][2] * physicalCorners[i][3];

      A[i*3][6]= projectedCorners[i][1] * physicalCorners[i][0];
      A[i*3][7]= projectedCorners[i][1] * physicalCorners[i][1];
      A[i*3][8]= projectedCorners[i][1] * physicalCorners[i][3];

      A[i*3+1][0]= projectedCorners[i][2] * physicalCorners[i][0];
      A[i*3+1][1]= projectedCorners[i][2] * physicalCorners[i][1];
      A[i*3+1][2]= projectedCorners[i][2] * physicalCorners[i][3];

      A[i*3+1][3]=0;
      A[i*3+1][4]=0;
      A[i*3+1][5]=0;

      A[i*3+1][6]=-projectedCorners[i][0] * physicalCorners[i][0];
      A[i*3+1][7]=-projectedCorners[i][0] * physicalCorners[i][1];
      A[i*3+1][8]=-projectedCorners[i][0] * physicalCorners[i][3];

      A[i*3+2][0]=-projectedCorners[i][1] * physicalCorners[i][0];
      A[i*3+2][1]=-projectedCorners[i][1] * physicalCorners[i][1];
      A[i*3+2][2]=-projectedCorners[i][1] * physicalCorners[i][3];

      A[i*3+2][3]= projectedCorners[i][0] * physicalCorners[i][0];
      A[i*3+2][4]= projectedCorners[i][0] * physicalCorners[i][1];
      A[i*3+2][5]= projectedCorners[i][0] * physicalCorners[i][3];

      A[i*3+2][6]=0;
      A[i*3+2][7]=0;
      A[i*3+2][8]=0;
    }

    SVD svd=new SVD(A);

    double[][] V = svd.getV();

    double[][] E = new double[3][3];

    //E is the last column of V
    for (int i=0; i<9; i++) {
      E[i/3][i%3] = V[i][V.length-1] / V[8][V.length-1];
    }

    return E;
  }

  private PVector rotationFromMatrix(float[][]  mat) {

    // Assuming rotation order is around x,y,z
    PVector rot = new PVector();

    if (mat[1][0] > 0.998f) { // singularity at north pole
      rot.z = 0;
      float delta = (float) Math.atan2(mat[0][1], mat[0][2]);
      rot.y = -(float) Math.PI/2;
      rot.x = -rot.z + delta;
      return rot;
    }

    if (mat[1][0] < -0.998f) { // singularity at south pole
      rot.z = 0;
      float delta = (float) Math.atan2(mat[0][1], mat[0][2]);
      rot.y = (float) Math.PI/2;
      rot.x = rot.z + delta;
      return rot;
    }

    rot.y =-(float)Math.asin(mat[2][0]);
    rot.x = (float)Math.atan2(mat[2][1]/Math.cos(rot.y), mat[2][2]/Math.cos(rot.y));
    rot.z = (float)Math.atan2(mat[1][0]/Math.cos(rot.y), mat[0][0]/Math.cos(rot.y));

    return rot;
  }
}


public PImage convolute(PImage arg) {
  float[][] kernel = { 
    { 
      30, 20, 30
    }
    , 
    { 
      20, 0, 20 // slightly modified, to remove more noise
    }
    , 
    { 
      30, 20, 30
    }
  };
  float weight = 200.0f;
  PImage result = createImage(arg.width, arg.height, RGB);

  for (int y = 0; y < arg.height; y++) {
    for (int x = 0; x < arg.width; x++) {
      float r = 0.0f;
      float g = 0.0f;
      float b = 0.0f;

      for (int i = 0; i <= 2; i++) {
        for (int j = 0; j <= 2; j++) {
          int clampedX = x + i - 1;
          if (x + i - 1 < 0) {
            clampedX = 0;
          } else if (x + i - 1 >= arg.width) {
            clampedX = arg.width - 1;
          }

          int clampedY = y + j - 1;
          if (y + j - 1 < 0) {
            clampedY = 0;
          } else if (y + j - 1 >= arg.height) {
            clampedY = arg.height - 1;
          }

          r += red( arg.pixels[clampedY * arg.width + clampedX]) * kernel[i][j];
          g += green( arg.pixels[clampedY * arg.width + clampedX]) * kernel[i][j];
          b += blue( arg.pixels[clampedY * arg.width + clampedX]) * kernel[i][j];
        }
      }

      result.pixels[y * arg.width + x] = color(r / weight, g / weight, b / weight);
    }
  }

  return result;
}

public PImage sobel(PImage arg) {
  int nOfThreads = 8;
  Thread[] tabThread = new Thread[nOfThreads];
    
  PImage result = createImage(arg.width, arg.height, RGB);
  
  for (int i=0; i<nOfThreads; i++){
    tabThread[i] = new Thread(new RunnableSobel(i, i * arg.height/nOfThreads, (i+1) * arg.height/nOfThreads, arg, result, arg.width, arg.height)); // create the threads
    tabThread[i].start();  // and start them, each threads takes care of one part of the image
  }
  
  for (int i=0; i<nOfThreads; i++){
    try {
      tabThread[i].join();
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
  }
  return result;
}


public PImage preHueTh(PImage arg) { // this function replace the center of the board by a white quad, it's not perfect, but it removes most of the reflections, and only keeps the part of the image where the board is
  PImage thImg = createImage(arg.width, arg.height, RGB);
  int minX, maxX, minY, maxY;
  minX = arg.width - 1;
  maxX = 0;
  minY = arg.height - 1;
  maxY = 0;
  int count = 0;
  
  PVector p0, p1, p2, p3; // these points represent the 'guessed' position of the corners
  PVector[] p0t = new PVector[2];
  PVector[] p1t = new PVector[2];
  PVector[] p2t = new PVector[2];
  PVector[] p3t = new PVector[2];
  p0t[0] = new PVector();
  p1t[0] = new PVector();
  p2t[0] = new PVector();
  p3t[0] = new PVector();
  p0t[1] = new PVector();
  p1t[1] = new PVector();
  p2t[1] = new PVector();
  p3t[1] = new PVector();
  
  // p0t[0] represents the leftmost point of the board with the smallest y value,  p0t[1] represents the leftmost point of the board with the biggest y value, and so on

  // for each 'extreme' value (minX, maxX, minY, maxY), we obtain the other coordinates's local extreme value 
  // for instance, for minX, we try to obtain the points (minX, smallY), which is the point that, among the points with smallest coordinate X,  has the smallest coordinate Y and (minX, bigY), which is the point that, among the points with smallest coordinate X, has the biggest coordinate Y
  // once we have these 8 points we can guess the most probable quad
  // this function works fine when the quad is close to a square, but less when the board looks flat, as sometimes a corner of the board is not an extreme point
  // however, it still removes most of the reflection on the board

  for (int x = 0; x < arg.width; x++) {
    for (int y = 0; y < arg.height; y++) {
      int i = y * arg.width + x;
      thImg.pixels[i] = arg.pixels[i];
      if (hue(arg.pixels[i]) >= hueThLowP && hue(arg.pixels[i]) <= hueThHighP && saturation(arg.pixels[i]) >= saturationThLowP && saturation(arg.pixels[i]) <= saturationThHighP && brightness(arg.pixels[i]) >= brightnessThLowP && brightness(arg.pixels[i]) <= brightnessThHighP) {
        count++;
        if (x < minX){
          minX = x;
          p0t[0].x = x;
          p0t[1].x = x;
          p0t[0].y = y;
          p0t[1].y = y;
        }
        else if (x == minX){
          if (y < p0t[0].y){
            p0t[0].y = y;
          }
          else if (y > p0t[1].y){
            p0t[1].y = y;
          }
        }
        
        if (x > maxX){
          maxX = x;
          p1t[0].x = x;
          p1t[1].x = x;
          p1t[0].y = y;
          p1t[1].y = y;
        }
        else if (x == maxX){
          if (y < p1t[0].y){
            p1t[0].y = y;
          }
          else if (y > p1t[1].y){
            p1t[1].y = y;
          }
        }
        
        if (y < minY){
          minY = y;
          p2t[0].y = y;
          p2t[1].y = y;
          p2t[0].x = x;
          p2t[1].x = x;
        }
        else if (y == minY){
          if (x < p2t[0].x){
            p2t[0].x = x;
          }
          else if (x > p2t[1].x){
            p2t[1].x = x;
          }
        }

        if (y > maxY){
          maxY = y;
          p3t[0].y = y;
          p3t[1].y = y;
          p3t[0].x = x;
          p3t[1].x = x;
        }
        else if (y == maxY){
          if (x < p3t[0].x){
            p3t[0].x = x;
          }
          else if (x > p3t[1].x){
            p3t[1].x = x;
          }
        }
      }
    }
  }
  if (count < 200){ // the board is not on the picture
    return thImg;
  }
  
  float a0 = area(p0t[1], p2t[0], p1t[0], p3t[1]); // this quad corresponds to the board rotated with a negative angle
  float a1 = area(p0t[0], p2t[1], p1t[1], p3t[0]); // this quad corresponds to the board rotated with a positive angle
  
  if (a0 > a1){ // we select the most probable quad, which is the bigger one
    p0 = p0t[1];
    p1 = p1t[0];
    p2 = p2t[0];
    p3 = p3t[1];
  }
  else {
    p0 = p0t[0];
    p1 = p1t[1];
    p2 = p2t[1];
    p3 = p3t[0];
  }

  for (int x = 0; x < arg.width; x++) {
    for (int y = 0; y < arg.height; y++) {
      if (isInQuad(new PVector(x, y), p0, p1, p2, p3)){ // if the pixel is in the board with probability 100%, we paint it green
        thImg.pixels[y * arg.width + x] = color(22, 109, 85);
      }
      if (x < minX || x > maxX || y < minY || y > maxY){ // if the pixel is not in the board with probability 100%, we paint it black
        thImg.pixels[y * arg.width + x] = color(0);
      }
    }
  }
  return thImg;
}

public PImage hueTh (PImage arg) {
  PImage thImg = createImage(arg.width, arg.height, RGB);

  for (int i = 0; i < arg.width * arg.height; i++) {
    if (hue(arg.pixels[i]) >= hueThLow && hue(arg.pixels[i]) <= hueThHigh && saturation(arg.pixels[i]) >= saturationThLow && saturation(arg.pixels[i]) <= saturationThHigh && brightness(arg.pixels[i]) >= brightnessThLow && brightness(arg.pixels[i]) <= brightnessThHigh) {
      thImg.pixels[i] = color(255);
    } else {
      thImg.pixels[i] = color(0);
    }
  }

  return thImg;
}

public PImage whiteTh (PImage arg) { // intensity threshold
  PImage thImg = createImage(arg.width, arg.height, RGB);

  for (int i = 0; i < arg.width * arg.height; i++) {
    if (brightness(arg.pixels[i]) >= 200) {
      thImg.pixels[i] = color(255);
    } else {
      thImg.pixels[i] = color(0);
    }
  }

  return thImg;
}

public PImage hough(PImage edgeImg, int nLines) {
  float discretizationStepsPhi = 0.04f;
  float discretizationStepsR = 0.9f;
  // dimensions of the accumulator
  int phiDim = (int) (Math.PI / discretizationStepsPhi);
  int rDim = (int) (((edgeImg.width + edgeImg.height) * 2 + 1) / discretizationStepsR);
  double rMax = rDim;
  // our accumulator (with a 1 pix margin around)
  int[] accumulator = new int[(phiDim + 2) * (rDim + 2)];

  // pre-compute the sin and cos values, using maps, as array give bad result
  HashMap<Float, Float> mapCos = new HashMap<Float, Float>();
  HashMap<Float, Float> mapSin = new HashMap<Float, Float>();
  for (float phi = 0.0f; phi < Math.PI; phi += discretizationStepsPhi) {
    mapCos.put((Float)phi, ((Double) Math.cos(phi)).floatValue());
    mapSin.put((Float)phi, ((Double) Math.sin(phi)).floatValue());
  }

  // Fill the accumulator: on edge points (ie, white pixels of the edge
  // image), store all possible (r, phi) pairs describing lines going
  // through the point.
  for (int y = 0; y < edgeImg.height; y++) {
    for (int x = 0; x < edgeImg.width; x++) {
      // Are we on an edge?
      if (brightness(edgeImg.pixels[y * edgeImg.width + x]) != 0) {

        for (float phi = 0.0f; phi < Math.PI; phi += discretizationStepsPhi) {
          double r = (x * mapCos.get(phi) + y * mapSin.get(phi)) / discretizationStepsR;
          r += (rDim - 1) / 2;
          accumulator[(int) ((phi / discretizationStepsPhi + 1) * (rDim + 2) +r)] ++;
        }
        // ...determine here all the lines (r, phi) passing through
        // pixel (x,y), convert (r,phi) to coordinates in the
        // accumulator, and increment accordingly the accumulator.
      }  
    }
  }


  /*PImage houghImg = createImage(rDim + 2, phiDim + 2, ALPHA);
   for (int i = 0; i < accumulator.length; i++) {
   houghImg.pixels[i] = color(min(255, accumulator[i]));
   }
   houghImg.updatePixels();*/

  ArrayList<Integer> bestCandidates = new ArrayList<Integer>();
  boolean[] valid = new boolean[(phiDim + 2) * (rDim + 2)];
  int minVotes = 30;

  int neighbourhoodPhi = 10;
  int neighbourhoodR = 35;
  // only search around lines with more that this amount of votes
  // (to be adapted to your image)
  for (int accR = 0; accR < rDim; accR++) {
    for (int accPhi = 0; accPhi < phiDim; accPhi++) {
      // compute current index in the accumulator
      int idx = (accPhi + 1) * (rDim + 2) + accR + 1;
      if (accumulator[idx] > minVotes) {
        boolean bestCandidate=true;
        // iterate over the neighbourhood
        for (int dPhi=-neighbourhoodPhi/2; dPhi < neighbourhoodPhi/2+1 && bestCandidate; dPhi++) {
          int phi = accPhi+dPhi;
          if (phi < 0)
            phi = phiDim + accPhi+dPhi;
          if (phi >= phiDim)
            phi = -phiDim + accPhi+dPhi;
      
          for (int dR=-neighbourhoodR/2; dR < neighbourhoodR/2 +1; dR++) {
            // check we are not outside the image
            if (accR+dR < 0 || accR+dR >= rDim) continue;
            int neighbourIdx = (phi + 1) * (rDim + 2) + accR + dR + 1;
            if (accumulator[idx] < accumulator[neighbourIdx]) {
              // the current idx is not a local maximum!
              bestCandidate=false;
              break;
            }
            else if (accumulator[idx] == accumulator[neighbourIdx] && valid[neighbourIdx]) { // to prevent to close lines of the same value to be chosen
              bestCandidate=false;
              break;
            }
            if (accPhi * discretizationStepsPhi < 0.17f || accPhi * discretizationStepsPhi > Math.PI - 0.17f) { // if near vertical, check for opposite r
              if (rDim-accR-1+dR < 0 || rDim-accR-1+dR >= rDim) continue;
              neighbourIdx = (phi + 1) * (rDim + 2) + rDim-accR-1 + dR + 1;
              if (accumulator[idx] < accumulator[neighbourIdx]) {
                // the current idx is not a local maximum!
                bestCandidate=false;
                break;
              }
              else if (accumulator[idx] == accumulator[neighbourIdx] && valid[neighbourIdx]) {
                bestCandidate=false;
                break;
              }
            }
          }
          
        }
        if (bestCandidate) {
          // the current idx *is* a local maximum
          bestCandidates.add(idx);
          valid[idx] = true;
        }
      }
    }
  }

  Collections.sort(bestCandidates, new HoughComparator(accumulator));

  ArrayList<PVector> lines = new ArrayList<PVector>(); 


  for (int i = 0; i < bestCandidates.size() && i < nLines; i++) {
    
    int idx = bestCandidates.get(i);
    // first, compute back the (r, phi) polar coordinates:
    int accPhi = (int) (idx / (rDim + 2)) - 1;
    int accR = idx - (accPhi + 1) * (rDim + 2) - 1;
    float r = (accR - (rDim - 1) * 0.5f) * discretizationStepsR;
    float phi = accPhi * discretizationStepsPhi;

    lines.add(new PVector(r, phi));

    // Cartesian equation of a line: y = ax + b
    // in polar, y = (-cos(phi)/sin(phi))x + (r/sin(phi))
    // => y = 0 : x = r / cos(phi)
    // => x = 0 : y = r / sin(phi)
    // compute the intersection of this line with the 4 borders of
    // the image
    int x0 = 0;
    int y0 = (int) (r / sin(phi));
    int x1 = (int) (r / cos(phi));
    int y1 = 0;
    int x2 = edgeImg.width;
    int y2 = (int) (-cos(phi) / sin(phi) * x2 + r / sin(phi));
    int y3 = edgeImg.width;
    int x3 = (int) (-(y3 - r / sin(phi)) * (sin(phi) / cos(phi)));
    // Finally, plot the lines
    
    
    /*
    stroke(200, 102, 0);
    if (y0 > 0) {
      if (x1 > 0)
        line(x0, y0, x1, y1);
      else if (y2 > 0)
        line(x0, y0, x2, y2);
      else
        line(x0, y0, x3, y3);
    } else {
      if (x1 > 0) {
        if (y2 > 0)
          line(x1, y1, x2, y2);
        else
          line(x1, y1, x3, y3);
      } else
        line(x2, y2, x3, y3);
    }*/
    
  }

  getIntersections(lines);

  build(lines, edgeImg.width, edgeImg.height);

  ArrayList<int[]> oldquads = findCycles();

  ArrayList<int[]> quads = new ArrayList<int[]>();

  for (int i = 0; i < oldquads.size (); i++) {
    if (oldquads.get(i).length == 4) {
      quads.add(oldquads.get(i));
    }
  }

  for (int[] quad : quads) {
    PVector l1 = lines.get(quad[0]);
    PVector l2 = lines.get(quad[1]);
    PVector l3 = lines.get(quad[2]);
    PVector l4 = lines.get(quad[3]);
    // (intersection() is a simplified version of the
    // intersections() method you wrote last week, that simply
    // return the coordinates of the intersection between 2 lines)
    PVector c12 = intersection(l1, l2);
    PVector c23 = intersection(l2, l3);
    PVector c34 = intersection(l3, l4);
    PVector c41 = intersection(l4, l1);
    
    if (validArea(c12, c23, c34, c41, 50000, 10000) && isConvex(c12, c23, c34, c41) && nonFlatQuad(c12, c23, c34, c41)){
      fill(255, 100, 0);
      quad(c12.x, c12.y, c23.x, c23.y, c34.x, c34.y, c41.x, c41.y);
      
      List<PVector> listePoints = new ArrayList<PVector>();
      listePoints.add(c12);
      listePoints.add(c23);
      listePoints.add(c34);
      listePoints.add(c41);
      
      TwoDThreeD b = new TwoDThreeD(img.width, img.height);
      
      PVector a = b.get3DRotations(sortCorners(listePoints));
      
      rotationX = ((rotationX+a.x)/2)/2; //  first, we take the mean of the new rotation and the old one (to prevent big changes), and then we divide it by 2 (to prevent too large rotations)
      rotationZ = ((rotationZ+a.y)/2)/2;
    }
  }


  //houghImg.resize(400, 400);
  return null;
}

public PVector intersection(PVector l1, PVector l2){
  ArrayList<PVector> lines = new ArrayList<PVector>();
  lines.add(l1);
  lines.add(l2);
  return getIntersections(lines).get(0);
}

public ArrayList<PVector> getIntersections(ArrayList<PVector> lines) {
  ArrayList<PVector> intersections = new ArrayList<PVector>();
  for (int i = 0; i < lines.size () - 1; i++) {
    PVector line1 = lines.get(i);
    for (int j = i + 1; j < lines.size (); j++) {
      PVector line2 = lines.get(j);
      // compute the intersection and add it to 'intersections'
      // draw the intersection
      double d = cos(line2.y) * sin(line1.y) - cos(line1.y) * sin(line2.y);
      int x = (int) ((line2.x * sin(line1.y) - line1.x * sin(line2.y)) / d);
      int y = (int) (( - line2.x * cos(line1.y) + line1.x * cos(line2.y)) / d);

      intersections.add(new PVector(x, y, 1));
      fill(255, 128, 0);
      ellipse(x, y, 10, 10);
    }
  }
  return intersections;
}

class HoughComparator implements Comparator<Integer> {
  int[] accumulator;
  public HoughComparator(int[] accumulator) {
    this.accumulator = accumulator;
  }
  @Override
    public int compare(Integer l1, Integer l2) {
    if (accumulator[l1] > accumulator[l2]
      || (accumulator[l1] == accumulator[l2] && l1 < l2)) return -1;
    return 1;
  }
}

public void drawBackgroundSurface() { 
  backgroundSurface.noStroke();
  backgroundSurface.beginDraw();
  backgroundSurface.background(230, 225, 175);
  backgroundSurface.fill(230, 225, 175);
  backgroundSurface.rect(0, 0, backgroundSurface.width, backgroundSurface.height);
  backgroundSurface.endDraw();
}

public void drawTopViewSurface() {
  topViewSurface.noStroke();
  topViewSurface.beginDraw();
  topViewSurface.background(60, 130, 170);
  float zoom = boardSize/topViewSurface.width;
  topViewSurface.fill(255, 0, 0);
  for (int i = 0; i < cylinderList.size (); i ++) {
    float posX =  (boardSize/2 + cylinderList.get(i).x) / zoom;
    float posY = (boardSize/2 + cylinderList.get(i).y) / zoom;
    topViewSurface.ellipse(posX, posY, 2*cylinderBaseSize/zoom, 2*cylinderBaseSize/zoom);
  }
  
  topViewSurface.fill(0);
  for (int i = 0; i < sheeps.size (); i ++) {
    if (sheeps.get(i).sheep_is_alive){
      float posX = (boardSize/2 + sheeps.get(i).sheep_position.x) / zoom;
      float posY = (boardSize/2 + sheeps.get(i).sheep_position.y) / zoom;
      topViewSurface.ellipse(posX, posY, 2/zoom, 2/zoom);
    }
  }

  float ballPosX = (ball.location.x + boardSize/2)/zoom; //adding topViewSurfaceSize/2 because the ball is at position 0, 0 at the center of the plate.
  float ballPosY = (ball.location.z  + boardSize/2)/zoom;
  topViewSurface.fill(0, 240, 0);
  topViewSurface.ellipse(ballPosX, ballPosY, 2*ballSize/zoom, 2*ballSize/zoom);
  topViewSurface.endDraw();
}

public void drawScoreSurface() {

  scoreSurface.beginDraw();
  scoreSurface.stroke(255);
  scoreSurface.strokeWeight(4);
  scoreSurface.strokeJoin(ROUND);
  scoreSurface.fill(230, 225, 175);
  scoreSurface.rect(0, 0, scoreSurface.width, scoreSurface.height);
  scoreSurface.textSize(16);
  scoreSurface.fill(60, 130, 170);
  scoreSurface.text("Total Score", 5, 17);
  scoreSurface.text(totalScore, 5, 32);
  scoreSurface.text("Velocity", 5, 62);
  scoreSurface.text(ball.velocity.mag(), 5, 77);
  scoreSurface.text("Last Score", 5, 107);
  scoreSurface.text(score, 5, 122);
  scoreSurface.endDraw();
}

public void drawBarChartSurface() {  
  float rectWidth = pow(4.0f, hs.getPos() + 0.5f);
  float rectHeight = 4.0f;

  if (millis() - timeSinceLastEvent >= 400) {
    barChartSurface.beginDraw();
    barChartSurface.background(255);

    timeSinceLastEvent = millis();
    nbCurrentScore ++;

    for (int i = nbScoreMax - 1; i > 0; i--) {
      tabScore[i] = tabScore[i-1];
    }
    tabScore[0] = totalScore;

    barChartSurface.fill(23);
    barChartSurface.line(0, barChartSurface.height/2, barChartSurface.width, barChartSurface.height/2);
    
    for (int i = 0; i < nbScoreMax; i++) {
      if (tabScore[i] > 0) {
        barChartSurface.fill(0, 255, 0);
        for (int j = 0; j < tabScore[i]; j++) {
          barChartSurface.rect(i * rectWidth, barChartSurface.height - j * rectHeight - barChartSurface.height/2 - rectHeight, rectWidth, rectHeight);
        }
      }
      else {
        barChartSurface.fill(255, 0, 0);
        for (int j = 0; j > tabScore[i]; j--) {
          barChartSurface.rect(i * rectWidth, barChartSurface.height - j * rectHeight - barChartSurface.height/2, rectWidth, rectHeight);
        }
      }
    }

    barChartSurface.endDraw();
  }
}

public void drawSprotchSurface(){
  sprotchSurface.beginDraw();
  sprotchSurface.background(255, 0.0f);
  if (millis() - timeSinceLastSprotch <= 1000) {
    sprotchSurface.textSize(35);
    sprotchSurface.fill(240, 40, 40);
    sprotchSurface.text("SPROTCH", 10, 40);
  }
  
  
  sprotchSurface.endDraw();
  
}

  static public void main(String[] passedArgs) {
    String[] appletArgs = new String[] { "TangibleGame" };
    if (passedArgs != null) {
      PApplet.main(concat(appletArgs, passedArgs));
    } else {
      PApplet.main(appletArgs);
    }
  }
}
