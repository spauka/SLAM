#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"
#include "ros/ros.h"
#include <vector>
#include <set>
#include <math.h>

float intersectSquare(float x, float y, float size, float X, float Y, float rx, float ry){
  if (rx > 0){
    float dx = x - X;
    if (dx > 0){
      float dy = dx * ry / rx;
      float iy = Y + dy;
      if ((iy >= y) && (iy <= y + size))
        return dx*dx + dy*dy;
    }
  } else if (rx < 0) {
    float dx = (x + size) - X;
    if (dx < 0){
      float dy = dx * ry / rx;
      float iy = Y + dy;
      if ((iy >= y) && (iy <= y + size))
        return dx*dx + dy*dy;
    }
  }
  if (ry > 0){
    float dy = y - Y;
    if (dy > 0){
      float dx = dy * rx / ry;
      float ix = X + dx;
      if ((ix >= x) && (ix <= x + size))
        return dx*dx + dy*dy;
    }
  } else if (ry < 0) {
    float dy = (y + size) - Y;
    if (dy < 0){
      float dx = dy * rx / ry;
      float ix = X + dx;
      if ((ix >= x) && (ix <= x + size))
        return dx*dx + dy*dy;
    }
  }
  return -1.0;
}

struct Cell{
  int x;
  int y;
  int index;
  char value;
  
  void getData(char* data) const{
    data[index] = value;
  }

  Cell(int x, int y, int width, char value) 
      : x(x), y(y), index(x + y * width), value(value) {}
  bool operator==(const Cell& rhs) const{
    return index == rhs.index;
  }
  bool operator<(const Cell& rhs) const{
    return index < rhs.index;
  }
  float d(float X, float Y, float rx, float ry){
    float D = intersectSquare(x,y,1,X,Y,rx,ry);
    ROS_INFO("Intersecting cell (%d, %d, 1, %f, %f, %f, %f) D=%f",x,y,X,Y,rx,ry,D);
    if (D > 0){
      float dx = x + 0.5 - X;
      float dy = y + 0.5 - Y;
      return dx*dx + dy*dy;
    } else
      return -1.0;
  }
  
  
};

  
struct Quadtree{
  int x, y, size; //Size should be power of 2
  Quadtree* children[4];
  Cell cell;
  bool isLeaf;
  Quadtree(int x, int y, int size, Cell& cell)
      : x(x), y(y), size(size), cell(cell), isLeaf(true) {
    children[0] = children[1] = children[2] = children[3] = NULL;
  }
  
  ~Quadtree(){
    for (int i = 0; i < 4; i++)
      if (children[i])
        delete children[i];
  }
  void Insert(Cell& cell){
    if (isLeaf) {
      if (this->cell == cell)
        this->cell = cell;
      else {
        isLeaf = false;
        Insert(this->cell);
        Insert(cell);
      }
    } else {
      int dx = cell.x - x;
      int dy = cell.y - y;
      int div = size/2;
      int index = (dx > div) + 2 * (dy > div);
      if (children[index] == NULL)
        children[index] = new Quadtree(x + (dx > div)*div, y + (dy > div)*div, div, cell);
      else
        children[index]->Insert(cell);
    }
  }
  bool Delete(int cx, int cy){
    if (isLeaf)
      return ((cell.x == cx) && (cell.y == cy));
    else {
      int dx = cx - x;
      int dy = cy - y;
      int div = size/2;
      int index = (dx >= div) + 2 * (dy >= div);
      if (children[index] && children[index]->Delete(cx,cy)){
        delete children[index];
        children[index] = NULL;
      }
      return false;
    }
  }
  
    
  void getData(char* data){
    if (isLeaf)
      cell.getData(data);
    else
      for (int i = 0; i < 4; i++)
        if (children[i])
          children[i]->getData(data);
    
  }

  float intersect(float X, float Y, float rx, float ry){
    ROS_INFO("Intersecting node (%d, %d) %d",x,y,size);
    if (isLeaf)
      return cell.d(X,Y,rx,ry);
    int inside = -1;
    if ( (X >= x) && (X < x + size) && (Y >= y) && (Y < y + size)){
      int dx = X - x;
      int dy = Y - y;
      int div = size/2;
      inside = (dx >= div) + 2 * (dy >= div);
      float d = children[inside]->intersect(X,Y,rx,ry);
      ROS_INFO("Inside node %d d=%d",inside, d);
      if (d >= 0)
        return d;
    }
    //Not going to bother doing a proper sort
    float dists[4] = {-1,-1,-1,-1};
    for (int i = 0; i < 4; i++)
      if (i != inside && children[i])
        dists[i] = children[i]->intersectBoundary(X,Y,rx,ry);

    int index;
    do {
      float mind = INFINITY;
      index = -1;
      for (int i = 0; i < 4; i++)
        if (dists[i] >= 0 && dists[i] < mind){
          mind = dists[i];
          index = i;
          dists[i] = -1;
        }
      float d = children[index]->intersect(X,Y,rx,ry);
      if (d >= 0)
        return d;
    } while (index != -1);
    return -1.0;
  }

  float intersectBoundary(float X, float Y, float rx, float ry){
    return intersectSquare(x,y,size,X,Y,rx,ry);
  }
};




struct Map{
  Quadtree* cells;
  int width, height;
  int cell_count;
  std::set<Cell> unoccupied;
  Map(int width, int height) 
      : cells(NULL), width(width), height(height), cell_count(0) {}
  ~Map(){
    delete cells;
  }
  void Insert(Cell cell){
    if (cell.value == -1)
      return;
    else if (cell.value == 0)
      unoccupied.insert(cell);
    else {
      unoccupied.erase(cell);
      //ROS_INFO("Inserting Cell (%d,%d) %d",cell.x,cell.y,cell.value);
      cell_count++;
      if (cells == NULL){
        int size = 1;
        while (size < width && size < height) size <<= 1;
        //ROS_INFO("Size %d",size);
        cells = new Quadtree(0,0,size,cell);
      } else {
        cells->Insert(cell);
      }
    }
    
  }
  nav_msgs::OccupancyGrid* Grid(){
    nav_msgs::OccupancyGrid* grid = new nav_msgs::OccupancyGrid();
    grid->info.width = width;
    grid->info.height = height;
    char* data = new char[width*height];
    for (int i = 0; i < width*height; i++)
      data[i] = -1; //initialise to -1
    for (std::set<Cell>::iterator it = unoccupied.begin(); it != unoccupied.end(); it++)
      it->getData(data);
    if (cells)
      cells->getData(data);
    grid->data = std::vector<int8_t>(data, data + width*height);
    ROS_INFO("Cell Count: %d", cell_count);
    return grid;
  }
  float intersect(float X, float Y, float rx, float ry){
    if (cells && cells->intersectBoundary(X,Y,rx,ry) >= 0){
      return cells->intersect(X,Y,rx,ry);
    } else
      return -1.0;
  }
  
};


  
