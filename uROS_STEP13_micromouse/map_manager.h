#ifndef map_manager_H_
#define map_manager_H_

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

typedef enum {
  front,
  right,
  left,
  rear
} t_direction;

typedef enum {
  north,
  east,
  south,
  west
} t_direction_glob;

typedef struct {
  char x;
  char y;
  t_direction_glob dir;
} t_position;


class map_manager {
private:
  unsigned short steps_map[MAZESIZE_X][MAZESIZE_Y];  //歩数マップ
  t_wall wall[MAZESIZE_X][MAZESIZE_Y];               //壁の情報を格納する構造体配列
  t_position mypos;
  short goal_mx, goal_my;
public:
  void view(char view_weight_f);
  void position_init(void);
  void set_mypos(char x, char y, t_direction_glob dir);
  void set_mypos_dir(t_direction_glob dir);
  short get_mypos_x(void);
  short get_mypos_y(void);
  t_direction_glob get_mypos_dir(void);
  char get_wall_data(char x, char y, t_direction_glob dir);
  void set_wall_data(char x, char y, t_direction_glob dir, char data);
  unsigned short get_step_map_data(char x, char y);
  char get_goal_x(void);
  char get_goal_y(void);
  void set_goal_x(short data);
  void set_goal_y(short data);

  void make_map2(int x, int y);
  void make_search_map(int x, int y);
  t_direction get_nextdir2(short x, short y, t_direction_glob *dir);
  t_direction get_nextdir(char x, char y, t_direction_glob *dir);
  void axis_update(void);
  void next_dir(t_direction dir);
  void wallch(int x, int y, int x2, int y2);
  void set_wall(bool IS_SEN_FR, bool IS_SEN_R, bool IS_SEN_L);
  int get_priority(char x, char y, t_direction_glob dir);

  map_manager();
};

#endif