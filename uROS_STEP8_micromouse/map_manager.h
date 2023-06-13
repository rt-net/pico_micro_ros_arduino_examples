#ifndef MAP_MANAGER_H_
#define MAP_MANAGER_H_

#define MAZESIZE_X (16)  //迷路の大きさ(MAZESIZE_X * MAZESIZE_Y)迷路
#define MAZESIZE_Y (16)  //迷路の大きさ(MAZESIZE_Y * MAZESIZE_Y)迷路

#define _UNKNOWN 2  //壁があるかないか判らない状態の場合の値
#define NOWALL 0    //壁がないばあいの値
#define WALL 1      //壁がある場合の値
#define VWALL 3     //仮想壁の値(未使用)

typedef struct
{
  unsigned char west : 2;   //西の壁情報bit7-6
  unsigned char south : 2;  //南の壁情報 bit5-4
  unsigned char east : 2;   //東の壁情報 bit3-2
  unsigned char north : 2;  //北の壁情報 bit1-0
} t_wall;                   //壁情報を格納する構造体(ビットフィールド)

typedef enum { front, right, left, rear } t_direction;

typedef enum { north, east, south, west } t_direction_glob;

typedef struct
{
  unsigned char x;
  unsigned char y;
  t_direction_glob dir;
} t_position;

class MapManager
{
public:
  MapManager();

  void positionInit(void);
  void setMyPosDir(t_direction_glob dir);
  short getMyPosX(void);
  short getMyPosY(void);
  char getWallData(unsigned char x, unsigned char y, t_direction_glob dir);
  void setWallData(unsigned char x, unsigned char y, t_direction_glob dir, char data);
  char getGoalX(void);
  char getGoalY(void);
  void setGoalX(short data);
  void setGoalY(short data);
  void axisUpdate(void);
  void nextDir(t_direction dir);
  void setWall(bool IS_SEN_FR, bool IS_SEN_R, bool IS_SEN_L);
  t_direction getNextDir(char x, char y, t_direction_glob * dir);
  t_direction getNextDir2(short x, short y, t_direction_glob * dir);

private:
  unsigned short steps_map[MAZESIZE_X][MAZESIZE_Y];  //歩数マップ
  t_wall wall[MAZESIZE_X][MAZESIZE_Y];               //壁の情報を格納する構造体配列
  t_position mypos;
  short goal_mx, goal_my;

  void makeSearchMap(int x, int y);
  void makeMap2(int x, int y);
  int getPriority(unsigned char x, unsigned char y, t_direction_glob dir);
};

#endif  // MAP_MANAGER_H_