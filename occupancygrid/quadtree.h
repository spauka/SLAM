#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"
#include "ros/ros.h"

#include <vector>

struct Cell{
  int x;
  int y;
  char value;

  void getData(char* data, int width){
    data[x + y*width] = value;
  }

  Cell(int x, int y, char value) 
      : x(x), y(y), value(value) {}
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
      isLeaf = false;
      Insert(this->cell);
      Insert(cell);
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
  void getData(char* data, int width){
    if (isLeaf)
      cell.getData(data, width);
    else
      for (int i = 0; i < 4; i++)
        if (children[i])
          children[i]->getData(data, width);
        
  }
  
};


struct Map{
  Quadtree* cells;
  int width, height;
  Map(int width, int height) 
      : width(width), height(height), cells(NULL) {}
  ~Map(){
    delete cells;
  }
  void Insert(Cell cell){
    if (cell.value == -1 || cell.value == 0)
      return;
    //ROS_INFO("Inserting Cell (%d,%d) %d",cell.x,cell.y,cell.value);
    if (cells == NULL){
      int size = 1;
      while (size < width && size < height) size <<= 1;
      //ROS_INFO("Size %d",size);
      cells = new Quadtree(0,0,size,cell);
    } else {
      cells->Insert(cell);
    }
  }
  nav_msgs::OccupancyGrid* Grid(){
    nav_msgs::OccupancyGrid* grid = new nav_msgs::OccupancyGrid();
    grid->info.width = width;
    grid->info.height = height;
    char* data = new char[width*height];
    for (int i = 0; i < width*height; i++)
      data[i] = 0; //initialise to 0
    if (cells)
      cells->getData(data, width);
    grid->data = std::vector<int8_t>(data, data + width*height);
    return grid;
  }  
};


  
